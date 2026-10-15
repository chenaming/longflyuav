import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / "agile_ws/src/agile_openvla_bridge/src/agile_openvla_bridge/step_cycle.py"
spec = importlib.util.spec_from_file_location("step_cycle", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StepCycleTest(unittest.TestCase):
    def test_only_explicit_trigger_can_start_one_request(self):
        c = module.StepCycle()
        self.assertFalse(c.begin())
        c.request(False)
        with self.assertRaises(ValueError):
            c.request(False)
        self.assertTrue(c.begin())
        self.assertFalse(c.begin())
        with self.assertRaises(ValueError):
            c.request(False)
        c.inflight = False
        with self.assertRaises(ValueError):
            c.request(True)
        c.cancel()
        self.assertFalse(c.begin())
        c.request(False)
        self.assertTrue(c.begin())

    def test_arrival_requires_continuous_low_speed_and_distance(self):
        c = module.StepCycle(dwell=1)
        self.assertFalse(c.arrived(.02, .01, 10))
        self.assertFalse(c.arrived(.02, .2, 10.9))
        self.assertFalse(c.arrived(.02, .01, 11))
        self.assertFalse(c.arrived(.2, .01, 11.9))
        self.assertFalse(c.arrived(.02, .01, 12))
        self.assertFalse(c.arrived(.02, .01, 12.9))
        self.assertTrue(c.arrived(.02, .01, 13.1))
        self.assertFalse(c.arrived(.02, float("nan"), 14))

    def test_invalid_dwell_rejected(self):
        for value in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                module.StepCycle(dwell=value)
