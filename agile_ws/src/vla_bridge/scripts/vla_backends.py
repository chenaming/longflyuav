#!/usr/bin/env python3
"""Pluggable control backends. Step 5: DirectFsmBackend; EgoPlanner is a stub."""

import os
import sys

import rospy
from std_msgs.msg import Bool


def load_uav_command():
    try:
        from uav_control_fsm.msg import uav_command
        return uav_command
    except ImportError:
        vendor = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor")
        if vendor not in sys.path:
            sys.path.insert(0, vendor)
        from uav_control_fsm.msg import uav_command
        return uav_command


UavCommand = load_uav_command()


class ControlBackend(object):
    def publish_pose(self, x, y, z, yaw_deg, stamp):
        raise NotImplementedError

    def hover(self):
        pass

    def land(self):
        pass


class DirectFsmBackend(ControlBackend):
    """Writes /uav1/move_command only when the caller asks. Preview is separate."""

    def __init__(
        self,
        fsm_topic="/uav1/move_command",
        enable_fsm_pub=False,
        move_mode=0,
        hover_topic="/uav1/hover_command",
        land_topic="/uav1/land_command",
        enable_failsafe_pub=False,
    ):
        self._move_mode = int(move_mode)
        self._fsm_pub = None
        self._hover_pub = None
        self._land_pub = None
        if enable_fsm_pub:
            self._fsm_pub = rospy.Publisher(fsm_topic, UavCommand, queue_size=1)
            rospy.logwarn("DirectFsmBackend advertising %s", fsm_topic)
        if enable_failsafe_pub:
            self._hover_pub = rospy.Publisher(hover_topic, Bool, queue_size=1, latch=True)
            self._land_pub = rospy.Publisher(land_topic, Bool, queue_size=1, latch=True)
            rospy.logwarn("DirectFsmBackend advertising failsafe %s %s", hover_topic, land_topic)

    def build(self, x, y, z, yaw_deg, stamp):
        cmd = UavCommand()
        cmd.header.stamp = stamp
        cmd.header.frame_id = "uav1_map"
        cmd.Move_mode = self._move_mode  # 0 = ENU pos + yaw deg
        cmd.pos_ref.x = float(x)
        cmd.pos_ref.y = float(y)
        cmd.pos_ref.z = float(z)
        cmd.vel_ref.x = 0.0
        cmd.vel_ref.y = 0.0
        cmd.vel_ref.z = 0.0
        cmd.acc_ref.x = 0.0
        cmd.acc_ref.y = 0.0
        cmd.acc_ref.z = 0.0
        cmd.yaw_ref = float(yaw_deg)  # VLA rad -> FSM deg (converted by caller)
        cmd.yaw_rate_ref = 0.0
        return cmd

    def publish_pose(self, x, y, z, yaw_deg, stamp):
        if self._fsm_pub is None:
            return
        self._fsm_pub.publish(self.build(x, y, z, yaw_deg, stamp))

    def hover(self):
        if self._hover_pub is None:
            return
        self._hover_pub.publish(Bool(data=True))

    def land(self):
        if self._land_pub is None:
            return
        self._land_pub.publish(Bool(data=True))


class EgoPlannerBackend(ControlBackend):
    def publish_pose(self, x, y, z, yaw_deg, stamp):
        rospy.logwarn_throttle(5.0, "EgoPlannerBackend is a stub; not publishing goal_point")
