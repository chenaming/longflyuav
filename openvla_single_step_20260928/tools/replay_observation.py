#!/usr/bin/env python3
"""Send one saved Odin image + simulated Agile pose; print EGO XYZ, never publish ROS."""
import argparse
import base64
import io
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

from PIL import Image
import requests

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "agile_ws/src/agile_openvla_bridge/src"))
from agile_openvla_bridge.geometry import MissionFrame


def replay(url, image_path, instruction, pose, origin, camera_offset=0.0, timeout=30):
    frame = MissionFrame(origin[:3], math.radians(origin[3]), math.radians(camera_offset))
    with Image.open(image_path) as image:
        image = image.convert("RGB")
        image.thumbnail((960, 960))
        stream = io.BytesIO()
        image.save(stream, format="JPEG", quality=85)
    payload = dict(schema="openvla-agile/v1", convention="mission_fru_cm_yaw_deg",
                   request_id=str(uuid.uuid4()), mission_id=str(uuid.uuid4()),
                   observation_stamp=time.time(), instr=instruction,
                   proprio=frame.encode_pose(pose[:3], math.radians(pose[3])),
                   image=base64.b64encode(stream.getvalue()).decode("ascii"))
    with requests.Session() as session:
        session.trust_env = False
        token = os.getenv("OPENVLA_API_TOKEN", "")
        if token:
            session.headers["Authorization"] = "Bearer " + token
        response = session.post(url, json=payload, timeout=(3, timeout))
        response.raise_for_status()
        result = response.json()
    for key in ("schema", "convention", "request_id", "mission_id", "observation_stamp"):
        if result.get(key) != payload[key]:
            raise ValueError("response mismatch: " + key)
    if result.get("status") != "success" or result.get("position_unit") != "cm" or result.get("yaw_unit") != "rad":
        raise ValueError("invalid response status or units")
    if not isinstance(result.get("action"), list) or len(result["action"]) != 1:
        raise ValueError("expected one action")
    goal, yaw = frame.decode_action(result["action"][0])
    return dict(mode="offline_replay_NO_ROS_PUBLISH", proprio_sent=payload["proprio"],
                response=result, ego_goal_message={"goal": list(goal)},
                planner_pose_unit="m", model_body_yaw_rad=yaw,
                yaw_sent_to_ego=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", required=True)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--pose", type=float, nargs=4, required=True,
                        metavar=("X_M", "Y_M", "Z_M", "YAW_DEG"))
    parser.add_argument("--origin", type=float, nargs=4,
                        help="Fixed mission origin x y z yaw_deg; defaults to --pose for one-shot replay")
    parser.add_argument("--camera-yaw-offset-deg", type=float, default=0)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    print(json.dumps(replay(args.url, args.image, args.instruction, args.pose,
                            args.origin or args.pose, args.camera_yaw_offset_deg,
                            args.timeout), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
