import base64
import io
import math
import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
from contextlib import nullcontext
from unittest.mock import patch
from PIL import Image
from openvla_agile_server import create_app, NativeBackend, SCHEMA, CONVENTION


class ServerTest(unittest.TestCase):
    def setUp(self):
        stream = io.BytesIO()
        Image.new("RGB", (16, 16)).save(stream, format="JPEG")
        self.payload = dict(schema=SCHEMA, convention=CONVENTION, request_id="request-1",
                            mission_id="mission-1", observation_stamp=123.456,
                            instr="fly forward", proprio=[100, 200, 300, 90],
                            image=base64.b64encode(stream.getvalue()).decode())
        self.app = create_app(lambda data, image: [20, 0, 0, .1])
        self.client = self.app.test_client()

    def test_compose_target_and_metadata(self):
        response = self.client.post("/predict", json=self.payload)
        self.assertEqual(response.status_code, 200)
        result = response.get_json()
        self.assertEqual(result["request_id"], "request-1")
        self.assertEqual(result["mission_id"], "mission-1")
        self.assertEqual(result["observation_stamp"], 123.456)
        self.assertEqual(result["position_unit"], "cm")
        for a, b in zip(result["action"][0], [100, 220, 300, math.pi/2+.1]):
            self.assertAlmostEqual(a, b)

    def test_invalid_units_shape_image_and_nonfinite(self):
        for key, value in [("convention", "meters"), ("proprio", [0, 0, 0]),
                           ("proprio", [0, 0, 0, float("nan")]), ("image", "not-base64"),
                           ("observation_stamp", -1), ("instr", "")]:
            data = dict(self.payload)
            data[key] = value
            self.assertEqual(self.client.post("/predict", json=data).status_code, 400)

    def test_failure_and_unbounded_action(self):
        for delta in ([300, 0, 0, 0], [float("nan"), 0, 0, 0]):
            app = create_app(lambda data, image: delta)
            self.assertEqual(app.test_client().post("/predict", json=self.payload).status_code, 502)

    def test_busy_request_not_queued(self):
        def backend(data, image):
            nested = app.test_client().post("/predict", json=self.payload)
            self.assertEqual(nested.status_code, 429)
            return [0, 0, 0, 0]
        app = create_app(backend)
        self.assertEqual(app.test_client().post("/predict", json=self.payload).status_code, 200)

    def test_optional_auth(self):
        app = create_app(lambda d, i: [0, 0, 0, 0], token="test-only")
        client = app.test_client()
        self.assertEqual(client.post("/predict", json=self.payload).status_code, 401)
        self.assertEqual(client.post("/predict", json=self.payload,
                                    headers={"Authorization": "Bearer test-only"}).status_code, 200)

    def test_native_uses_explicit_agent_path_and_original_prompt(self):
        # Real dynamic import, numpy state formatting; fake GPU class only.
        with tempfile.TemporaryDirectory() as tmp:
            script = Path(tmp) / "openvla_act.py"
            script.write_text(
                "import numpy as np\n"
                "class OpenVLAActionAgent:\n"
                "    def __init__(self, cfg): self.cfg = cfg\n"
                "    def act(self, image, prompt):\n"
                "        self.prompt = prompt\n"
                "        self.mode = image.mode\n"
                "        return np.array([20., 0., 0., .1])\n")
            with patch.dict("sys.modules", {"torch": SimpleNamespace(inference_mode=nullcontext)}):
                backend = NativeBackend("/example/checkpoint", 2, "sim", script)
                response = create_app(backend, "native").test_client().post("/predict", json=self.payload)
            self.assertEqual(response.status_code, 200)
            self.assertEqual(backend.agent.cfg["gpu_id"], 2)
            self.assertEqual(backend.agent.mode, "RGB")
            self.assertEqual(backend.agent.prompt,
                "In: Current State: 100.0,200.0,300.0,90.0, What action should the uav take to fly forward?\nOut:")

    def test_server_bound_must_be_finite_positive(self):
        for bound in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                create_app(lambda d, i: [0, 0, 0, 0], max_delta_cm=bound)


if __name__ == "__main__":
    unittest.main()
