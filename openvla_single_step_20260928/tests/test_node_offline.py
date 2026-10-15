"""Exercise real bridge guards with minimal ROS message/transport stand-ins.

These tests do NOT validate a running ROS graph or a catkin build.
"""
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy  # Import outside the temporary sys.modules patch; avoid reloading it.

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "agile_ws/src/agile_openvla_bridge"
sys.path.insert(0, str(PACKAGE / "src"))


class Stamp:
    def __init__(self, value=100.):
        self.value = value

    def to_sec(self):
        return self.value

    def __sub__(self, other):
        return Stamp(self.value - other.value)


def message(**kwargs):
    return NS(**kwargs)


def load_bridge():
    modules = {}
    for name, attrs in {
        "cv2": {}, "message_filters": {},
        "rospy": {"Time": NS(now=lambda: Stamp(100.))},
        "cv_bridge": {"CvBridge": object},
        "geometry_msgs.msg": {"PoseStamped": message},
        "mavros_msgs.msg": {"State": message},
        "nav_msgs.msg": {"Odometry": message},
        "quadrotor_msgs.msg": {"GoalSet": message},
        "sensor_msgs.msg": {"CompressedImage": type("CompressedImage", (), {}), "Image": message},
        "std_msgs.msg": {"Empty": message, "String": message, "Bool": message},
        "std_srvs.srv": {"SetBool": object, "SetBoolResponse": message,
                         "Trigger": object, "TriggerResponse": message},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        modules[name] = module
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location("bridge_offline", PACKAGE / "scripts/openvla_ego_bridge.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


bridge = load_bridge()


class NodeGuardsTest(unittest.TestCase):
    def node(self):
        node = bridge.Bridge.__new__(bridge.Bridge)
        node.lock = threading.RLock()
        node.cycle = bridge.StepCycle()
        node.hover_pub = Mock()
        node.enabled, node.start_enabled, node.dry_run = True, False, False
        node.origin, node.generation, node.goal = object(), 5, (1, 0, 1)
        node.stop_pub, node.publish_status = Mock(), Mock()
        node.frame, node.body_frame = "map", "base_link"
        node.max_tilt = math.radians(20)
        node.require_offboard = True
        node.state = NS(connected=True, armed=True, mode="OFFBOARD")
        node.state_arrival = time.monotonic()
        node.max_sensor_age, node.sync_slop = .75, .15
        node.latest_pair = None
        node.alignment_confirmed, node.convention_confirmed = True, True
        return node

    def odom(self, stamp=100, frame="map", yaw=0):
        return NS(header=NS(stamp=Stamp(stamp), frame_id=frame), child_frame_id="base_link",
                  pose=NS(pose=NS(position=NS(x=10, y=20, z=1),
                      orientation=NS(x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2)))))

    def test_stop_cancels_inflight_request_and_publishes_once(self):
        node = self.node()
        generation = node.generation
        node.disable("stop")
        self.assertFalse(node.enabled)
        self.assertNotEqual(node.generation, generation)
        self.assertIsNone(node.goal)
        self.assertIsNone(node.origin)
        node.stop_pub.publish.assert_called_once()
        node.on_stop(None)
        node.stop_pub.publish.assert_called_once()

    def test_offboard_loss_during_http_invalidates_request_immediately(self):
        node = self.node()
        node.on_state(NS(connected=True, armed=True, mode="POSCTL"))
        self.assertFalse(node.enabled)
        self.assertEqual(node.generation, 6)
        node.on_state(NS(connected=True, armed=True, mode="OFFBOARD"))
        self.assertFalse(node.enabled)  # no automatic resume

    def test_instruction_change_invalidates_old_response(self):
        node = self.node()
        node.on_instruction(NS(data=" fly left "))
        self.assertEqual(node.instruction, "fly left")
        self.assertEqual(node.generation, 6)
        self.assertFalse(node.enabled)

    def test_pose_frame_and_body_frame_checked(self):
        node = self.node()
        pose, yaw = node.pose(self.odom(yaw=math.pi/2))
        self.assertEqual(pose, (10, 20, 1))
        self.assertAlmostEqual(yaw, math.pi/2)
        with self.assertRaises(ValueError):
            node.pose(self.odom(frame="odom"))
        odom = self.odom()
        odom.child_frame_id = "camera"
        with self.assertRaises(ValueError):
            node.pose(odom)

    def test_stale_future_and_unsynchronized_sensors_rejected(self):
        node = self.node()
        for stamp in (0, 98, 101):
            with self.assertRaises(ValueError):
                node.check_stamp(Stamp(stamp), .75)
        image = NS(header=NS(stamp=Stamp(100)))
        with self.assertRaises(ValueError):
            node.check_observation(image, self.odom(99.5), time.monotonic())
        with self.assertRaises(ValueError):
            node.check_observation(image, self.odom(), time.monotonic()-2)
        node.check_observation(image, self.odom(99.9), time.monotonic())

    def test_tilt_and_invalid_pose_rejected(self):
        node = self.node()
        odom = self.odom()
        odom.pose.pose.orientation.x = math.sin(math.pi/6)
        odom.pose.pose.orientation.w = math.cos(math.pi/6)
        with self.assertRaises(ValueError):
            node.pose(odom)
        odom = self.odom()
        odom.pose.pose.position.x = float("nan")
        with self.assertRaises(ValueError):
            node.pose(odom)

    def test_no_enable_with_preview_uncalibrated_or_stale_state(self):
        for field, value in (("dry_run", True), ("convention_confirmed", False),
                             ("alignment_confirmed", False), ("state_arrival", time.monotonic()-2)):
            node = self.node()
            node.enabled = False
            setattr(node, field, value)
            self.assertFalse(node.on_enable(NS(data=True)).success)

    def test_repeated_enable_does_not_reset_inflight_mission(self):
        node = self.node()
        self.assertFalse(node.on_enable(NS(data=True)).success)
        self.assertEqual(node.generation, 5)


if __name__ == "__main__":
    unittest.main()
