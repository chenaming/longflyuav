#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from agile_openvla_bridge.geometry import MissionFrame, check_goal, quaternion_rpy, wrap


class CoordinatesTest(unittest.TestCase):
    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_nonzero_origin_and_yaw_roundtrip(self):
        frame = MissionFrame((10, 20, 2), math.pi/2)
        state = frame.encode_pose((11, 22, 2.5), 2*math.pi/3)
        self.assertVector(state, (200, 100, 50, -30))
        position, yaw = frame.decode_action(state[:3] + [math.radians(state[3])])
        self.assertVector(position, (11, 22, 2.5))
        self.assertAlmostEqual(yaw, 2*math.pi/3)

    def test_forward_right_up_and_yaw_reflection(self):
        frame = MissionFrame((0, 0, 1), 0)
        pos, yaw = frame.decode_action([100, 200, 50, math.pi/4])
        self.assertVector(pos, (1, -2, 1.5))
        self.assertAlmostEqual(yaw, -math.pi/4)

    def test_absolute_target_not_added_to_current_pose(self):
        frame = MissionFrame((10, 0, 1), 0)
        frame.encode_pose((12, 0, 1), 0)
        pos, _ = frame.decode_action([300, 0, 0, 0])
        self.assertVector(pos, (13, 0, 1))

    def test_camera_offset_and_yaw_wrap(self):
        frame = MissionFrame((1, 2, 3), math.radians(170), math.radians(20))
        state = frame.encode_pose((1, 2, 3), math.radians(-170))
        self.assertAlmostEqual(state[3], -20)
        _, yaw = frame.decode_action([0, 0, 0, math.radians(-20)])
        self.assertAlmostEqual(wrap(yaw-math.radians(-170)), 0)

    def test_body_forward_after_90_degree_turn(self):
        frame = MissionFrame((0, 0, 1), 0)
        self.assertAlmostEqual(frame.encode_pose((0, 0, 1), math.pi/2)[3], -90)
        pos, yaw = frame.decode_action([0, -100, 0, -math.pi/2])
        self.assertVector(pos, (0, 1, 1))
        self.assertAlmostEqual(yaw, math.pi/2)

    def test_goal_limits_and_motion_during_inference(self):
        options = dict(max_step=1., max_drift=.25, max_radius=3.,
                       min_z=.2, max_z=2.5, max_vertical_step=.3)
        check_goal((.2, 0, 1), (0, 0, 1), (.02, 0, 1), (0, 0, 1), **options)
        for goal, current in [((100, 0, 1), (0, 0, 1)), ((.2, 0, 1), (.8, 0, 1)),
                              ((.2, 0, .1), (0, 0, 1)), ((.2, 0, 1.5), (0, 0, 1)),
                              ((float("nan"), 0, 1), (0, 0, 1))]:
            with self.assertRaises(ValueError):
                check_goal(goal, (0, 0, 1), current, (0, 0, 1), **options)

    def test_quaternion_validation(self):
        self.assertVector(quaternion_rpy((0, 0, math.sin(.4), math.cos(.4))), (0, 0, .8))
        with self.assertRaises(ValueError):
            quaternion_rpy((0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
