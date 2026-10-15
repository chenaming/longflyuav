#!/usr/bin/env python3
"""Versioned HTTP adapter for UAV-Flow's sim checkpoint (FRU, cm).

Native mode loads the existing openvla_act.py agent once. Upstream mode reuses
an already loaded agent on localhost:5007, without allocating another model.
Neither mode commands a vehicle. The ROS bridge owns mission/world transforms.
"""
import argparse
import base64
import importlib.util
import io
import logging
import math
import os
from pathlib import Path
import sys
import threading
import time

from flask import Flask, jsonify, request
from PIL import Image
import requests

LOG = logging.getLogger("openvla_agile")
SCHEMA = "openvla-agile/v1"
CONVENTION = "mission_fru_cm_yaw_deg"


def vector4(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("expected four numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in value):
        raise ValueError("vector must contain numbers")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError("non-finite vector")
    return result


def compose_action(proprio, delta):
    yaw = math.radians(proprio[3])
    c, s = math.cos(yaw), math.sin(yaw)
    angle = (yaw + delta[3] + math.pi) % (2 * math.pi) - math.pi
    return [proprio[0] + c * delta[0] - s * delta[1],
            proprio[1] + s * delta[0] + c * delta[1],
            proprio[2] + delta[2], angle]


class UpstreamBackend:
    def __init__(self, url, timeout):
        self.url, self.timeout = url, timeout
        self.session = requests.Session()
        self.session.trust_env = False

    def ready(self):
        try:
            response = self.session.get(self.url, timeout=(1, 1))
            return response.status_code in (200, 405)
        except requests.RequestException:
            return False

    def __call__(self, data, image):
        response = self.session.post(self.url, json={
            k: data[k] for k in ("image", "instr", "proprio")
        }, timeout=(3, self.timeout))
        response.raise_for_status()
        result = response.json()
        if result.get("status") != "success":
            raise RuntimeError("upstream inference unsuccessful")
        return vector4(result["action_ori"][0])


class NativeBackend:
    def __init__(self, model_path, gpu_id, unnorm_key, agent_script):
        script = Path(agent_script).expanduser().resolve(strict=True)
        sys.path.insert(0, str(script.parent.parent))
        spec = importlib.util.spec_from_file_location("openvla_existing", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.agent = module.OpenVLAActionAgent({
            "model_path": model_path, "gpu_id": gpu_id,
            "unnorm_key": unnorm_key, "do_sample": False,
        })

    def __call__(self, data, image):
        import numpy as np
        import torch
        # Keep the original float32 state formatting and prompt exactly.
        state = np.asarray(data["proprio"], dtype=np.float32)
        state_text = ','.join(str(round(x, 1)) for x in state)
        prompt = f"In: Current State: {state_text}, What action should the uav take to {data['instr']}?\nOut:"
        with torch.inference_mode():
            raw = self.agent.act(image, prompt)
        return vector4(raw.tolist())


def create_app(backend, backend_name="test", token="", max_delta_cm=200.0):
    if not math.isfinite(max_delta_cm) or max_delta_cm <= 0:
        raise ValueError("max_delta_cm must be positive and finite")
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = 12 * 1024 * 1024
    lock = threading.Lock()

    @app.get("/health")
    def health():
        ready = not hasattr(backend, "ready") or backend.ready()
        return jsonify(status="ready" if ready else "backend_unavailable", schema=SCHEMA, backend=backend_name,
                       convention=CONVENTION, position_unit="cm",
                       input_yaw_unit="deg", output_yaw_unit="rad"), 200 if ready else 503

    @app.errorhandler(413)
    def too_large(_):
        return jsonify(status="error", message="request exceeds 12 MiB"), 413

    @app.post("/predict")
    def predict():
        if token and request.headers.get("Authorization") != "Bearer " + token:
            return jsonify(status="error", message="unauthorized"), 401
        try:
            data = request.get_json(silent=True)
            if not isinstance(data, dict) or data.get("schema") != SCHEMA:
                raise ValueError("schema must be " + SCHEMA)
            if data.get("convention") != CONVENTION:
                raise ValueError("convention must be " + CONVENTION)
            for field in ("request_id", "mission_id", "instr"):
                if not isinstance(data.get(field), str) or not data[field].strip() or len(data[field]) > 1000:
                    raise ValueError("invalid " + field)
            data["proprio"] = vector4(data.get("proprio"))
            stamp = data["observation_stamp"]
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp <= 0:
                raise ValueError("invalid observation_stamp")
            encoded = data.get("image")
            if not isinstance(encoded, str):
                raise ValueError("image must be base64")
            blob = base64.b64decode(encoded, validate=True)
            image = Image.open(io.BytesIO(blob))
            if image.width * image.height > 20_000_000:
                raise ValueError("image exceeds 20 megapixels")
            image = image.convert("RGB")
        except (ValueError, TypeError, KeyError, OSError, Image.DecompressionBombError) as exc:
            return jsonify(status="error", message=str(exc)), 400
        if not lock.acquire(blocking=False):
            return jsonify(status="error", message="inference busy; use newest observation"), 429
        start = time.monotonic()
        try:
            delta = vector4(backend(data, image))
            if math.sqrt(sum(x*x for x in delta[:3])) > max_delta_cm:
                raise ValueError("model displacement exceeds configured bound")
            if abs(delta[3]) > math.pi:
                raise ValueError("model delta yaw exceeds pi")
            action = compose_action(data["proprio"], delta)
            elapsed = time.monotonic() - start
            LOG.info("request=%s action_cm_rad=%s inference_s=%.3f", data["request_id"], action, elapsed)
            return jsonify(status="success", schema=SCHEMA,
                           convention=CONVENTION, position_unit="cm", yaw_unit="rad",
                           request_id=data["request_id"], mission_id=data["mission_id"],
                           observation_stamp=stamp, action=[action], action_ori=[delta],
                           inference_seconds=elapsed)
        except Exception:
            LOG.exception("inference failed: %s", data["request_id"])
            return jsonify(status="error", request_id=data["request_id"],
                           message="inference failed; see server log"), 502
        finally:
            lock.release()

    return app


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1",
                   help="Bind to the server LAN IP (or 0.0.0.0) for direct drone access")
    p.add_argument("--port", type=int, default=5008)
    p.add_argument("--backend", choices=("native", "upstream"), default="upstream")
    p.add_argument("--upstream", default="http://127.0.0.1:5007/predict")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--model-path", help="Checkpoint path, required for native backend")
    p.add_argument("--agent-script", default=str(Path(__file__).with_name("openvla_act.py")),
                   help="Path to existing vla-scripts/openvla_act.py")
    p.add_argument("--gpu-id", type=int, default=0)
    p.add_argument("--unnorm-key", choices=("sim",), default="sim")
    p.add_argument("--max-delta-cm", type=float, default=200.0)
    args = p.parse_args()
    if args.backend == "native" and not args.model_path:
        p.error("--model-path is required for --backend native")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        p.error("--timeout must be positive and finite")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    backend = (NativeBackend(args.model_path, args.gpu_id, args.unnorm_key, args.agent_script)
               if args.backend == "native" else UpstreamBackend(args.upstream, args.timeout))
    app = create_app(backend, args.backend, os.getenv("OPENVLA_API_TOKEN", ""), args.max_delta_cm)
    # threaded allows /health and prompt busy responses while GPU is serialized.
    app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
