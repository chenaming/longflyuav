"""Real loopback HTTP, synthetic images, deterministic inference; no vehicle/GPU."""
import base64
import io
import math
from pathlib import Path
import sys
import tempfile
import threading
import unittest

from PIL import Image
from werkzeug.serving import make_server

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "server"), str(ROOT / "tools"),
               str(ROOT / "agile_ws/src/agile_openvla_bridge/src")]
from openvla_agile_server import create_app, compose_action, UpstreamBackend
from agile_openvla_bridge.geometry import MissionFrame
from replay_observation import replay
from flask import Flask, request, jsonify
from unittest.mock import Mock, patch


class HTTPReplayTest(unittest.TestCase):
    def serve(self, app):
        server = make_server("127.0.0.1", 0, app, threaded=True)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(server.server_close)
        self.addCleanup(worker.join, 3)
        self.addCleanup(server.shutdown)
        return "http://127.0.0.1:%d/predict" % server.server_port

    def test_image_pose_to_original_endpoint_to_ego_xyz(self):
        upstream = Flask("fake_existing_openvla_act")
        seen = []

        @upstream.post("/predict")
        def predict():
            data = request.get_json()
            image = Image.open(io.BytesIO(base64.b64decode(data["image"])))
            self.assertEqual(image.mode, "RGB")
            self.assertEqual(image.size, (64, 48))
            seen.append(data)
            # Forward 20 cm, right 10 cm, up 5 cm; yaw +0.1 rad.
            # Deliberately return garbage in `action`: adapter MUST use action_ori
            # and perform the composition once, as the original endpoint does.
            return jsonify(status="success", action_ori=[[20, 10, 5, .1]],
                           action=[[999999, 999999, 999999, 0]])

        upstream_url = self.serve(upstream)
        url = self.serve(create_app(UpstreamBackend(upstream_url, 3), "upstream"))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "odin_synthetic.png"
            Image.new("RGB", (64, 48), (40, 120, 200)).save(path)
            result = replay(url, path, "fly forward", [11, 22, 1.2, 90], [10, 20, 1, 30])
        goal = result["ego_goal_message"]["goal"]
        # At world yaw +90deg, body forward => +Y, body right => +X.
        for actual, expected in zip(goal, [11.1, 22.2, 1.25]):
            self.assertAlmostEqual(actual, expected, places=6)
        self.assertAlmostEqual(result["model_body_yaw_rad"], math.pi/2-.1)
        self.assertEqual(seen[0]["instr"], "fly forward")
        self.assertFalse(result["yaw_sent_to_ego"])

    def test_nonzero_origin_camera_offset_and_many_headings(self):
        for yaw in [-179, -90, -30, 0, 45, 90, 179]:
            for offset in [-20, 0, 20]:
                frame = MissionFrame((10, -4, 1), math.radians(37), math.radians(offset))
                pose = (10.5, -3.8, 1.1)
                state = frame.encode_pose(pose, math.radians(yaw))
                target = compose_action(state, [30, 10, 5, .2])
                goal, _ = frame.decode_action(target)
                heading = math.radians(yaw + offset)
                expected = [pose[0]+.3*math.cos(heading)+.1*math.sin(heading),
                            pose[1]+.3*math.sin(heading)-.1*math.cos(heading), 1.15]
                for a, b in zip(goal, expected):
                    self.assertAlmostEqual(a, b, places=7)

    def test_replay_rejects_mismatched_response_identity_and_units(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "frame.png"
            Image.new("RGB", (16, 16)).save(path)
            for field, bad in [("request_id", "other-request"), ("mission_id", "other-mission"),
                               ("observation_stamp", 0), ("convention", "wrong-frame"),
                               ("position_unit", "m"), ("yaw_unit", "deg"),
                               ("action", [[0, 0, 0, True]])]:
                def post(url, json, timeout):
                    result = {k: json[k] for k in
                              ("schema", "convention", "request_id", "mission_id", "observation_stamp")}
                    result.update(status="success", position_unit="cm", yaw_unit="rad", action=[[0, 0, 0, 0]])
                    result[field] = bad
                    response = Mock()
                    response.json.return_value = result
                    return response
                with patch("requests.Session.post", side_effect=post):
                    with self.assertRaises(ValueError):
                        replay("http://unused/predict", path, "hover", [0, 0, 1, 0], [0, 0, 1, 0])


if __name__ == "__main__":
    unittest.main()
