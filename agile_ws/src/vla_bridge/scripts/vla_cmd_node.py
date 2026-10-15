#!/usr/bin/env python3
"""Convert VLA targets to control-frame position commands."""

import math
import os
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from vla_bridge.msg import VLAAction

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vla_geometry import (
    clamp_step,
    first_frame_to_enu,
    lerp_pose,
    wrap_pi,
    yaw_from_quat,
)
from vla_backends import DirectFsmBackend, UavCommand


class VlaCmdNode:
    def __init__(self):
        self.lock = threading.RLock()

        self.enable_publish = bool(
            rospy.get_param("/control/enable_publish", False)
        )
        self.backend = rospy.get_param(
            "/control/backend", "direct_fsm"
        )
        self.move_mode = int(
            rospy.get_param("/control/move_mode", 0)
        )
        self.control_rate = float(
            rospy.get_param("/control/control_rate_hz", 25.0)
        )
        self.stale_sec = float(
            rospy.get_param("/control/stale_action_sec", 1.0)
        )
        self.odom_timeout = float(
            rospy.get_param("/watchdog/odom_timeout_sec", 0.5)
        )
        self.max_xy = float(
            rospy.get_param("/control/max_step_xy_m", 0.25)
        )
        self.max_z = float(
            rospy.get_param("/control/max_step_z_m", 0.05)
        )
        self.max_yaw = math.radians(float(
            rospy.get_param("/control/max_yaw_step_deg", 8.0)
        ))
        self.lock_altitude = bool(
            rospy.get_param("/control/lock_altitude", True)
        )
        self.fixed_altitude = float(
            rospy.get_param("/control/fixed_altitude", 1.2)
        )
        self.use_vla_yaw = bool(
            rospy.get_param("/control/use_vla_yaw", False)
        )
        self.segment_duration = max(
            float(rospy.get_param(
                "/control/interpolation_sec", 0.5
            )),
            0.05,
        )

        # 必须先核实 ODIN -> 控制坐标系变换，才允许实际输出。
        self.transform_verified = bool(rospy.get_param(
            "/frames/control_transform_verified", False
        ))

        self.enabled = False
        self.origin_locked = False
        self.origin = None
        self.pose = None
        self.segment = None
        self.last_action_stamp = None
        self.enable_stamp = None
        self.last_observation_stamp = None

        self.safe_pub = rospy.Publisher(
            "action_safe", VLAAction, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "debug/move_command", UavCommand, queue_size=1
        )

        self.fsm = DirectFsmBackend(
            fsm_topic="/uav1/move_command",
            enable_fsm_pub=(
                self.enable_publish
                and self.transform_verified
                and self.backend == "direct_fsm"
                and self.move_mode == 0
            ),
            move_mode=self.move_mode,
        )

        rospy.Subscriber(
            "action_raw", VLAAction, self.on_action, queue_size=1
        )
        rospy.Subscriber(
            "enable_control", Bool, self.on_enable, queue_size=1
        )
        rospy.Subscriber(
            "origin_locked", Bool, self.on_origin_locked, queue_size=1
        )
        rospy.Subscriber(
            "first_frame_origin",
            PoseStamped,
            self.on_origin,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param(
                "/frames/pose_for_control",
                "/uav1/mavros/local_position/odom",
            ),
            Odometry,
            self.on_pose,
            queue_size=1,
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.control_rate, 1.0)),
            self.on_timer,
        )

        rospy.loginfo(
            "cmd ready: backend=%s publish=%s transform_verified=%s",
            self.backend,
            self.enable_publish,
            self.transform_verified,
        )

    def clear_target(self):
        # 调用者持有 self.lock。
        self.segment = None
        self.last_action_stamp = None

    @staticmethod
    def get_yaw(q):
        norm = math.sqrt(
            q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        )
        if not math.isfinite(norm) or norm < 1e-8:
            raise ValueError("invalid quaternion")
        return yaw_from_quat(
            q.x / norm, q.y / norm, q.z / norm, q.w / norm
        )

    def on_enable(self, msg):
        value = bool(msg.data)
        with self.lock:
            if value != self.enabled:
                self.clear_target()
                self.enabled = value
                self.enable_stamp = (
                    rospy.Time.now().to_sec() if value else None
                )
                rospy.loginfo(
                    "control permission changed: %s; old target cleared",
                    value,
                )

    def on_origin_locked(self, msg):
        with self.lock:
            if not msg.data:
                self.origin_locked = False
                self.origin = None
                self.last_observation_stamp = None
                self.clear_target()
            elif self.origin is not None:
                self.origin_locked = True

    def on_origin(self, msg):
        try:
            p = msg.pose.position
            origin_p = np.array([p.x, p.y, p.z], dtype=float)
            yaw = self.get_yaw(msg.pose.orientation)
            stamp = msg.header.stamp.to_sec()

            if not np.all(np.isfinite(origin_p)) or stamp <= 0:
                raise ValueError("invalid origin position or timestamp")

            with self.lock:
                self.origin = (origin_p, yaw, stamp)
                self.origin_locked = True
                self.last_observation_stamp = None
                self.clear_target()

            rospy.loginfo("new origin received; old target cleared")

        except (ValueError, TypeError, OverflowError) as exc:
            rospy.logwarn("invalid origin: %s", exc)

    def on_pose(self, msg):
        try:
            p = msg.pose.pose.position
            position = np.array([p.x, p.y, p.z], dtype=float)
            yaw = self.get_yaw(msg.pose.pose.orientation)
            stamp = msg.header.stamp.to_sec()

            if not np.all(np.isfinite(position)) or stamp <= 0:
                raise ValueError("invalid control odometry")

            with self.lock:
                self.pose = (position, yaw, stamp)

        except (ValueError, TypeError, OverflowError) as exc:
            rospy.logwarn_throttle(
                1.0, "invalid control odometry: %s", exc
            )

    def current_pose(self, now):
        # 调用者持有 self.lock。
        if self.pose is None:
            return None
        age = now - self.pose[2]
        if not 0.0 <= age <= self.odom_timeout:
            return None
        return self.pose

    def on_action(self, msg):
        try:
            values = np.asarray(msg.action, dtype=float)
            if values.shape != (4,) or not np.all(np.isfinite(values)):
                raise ValueError("action must contain 4 finite values")

            if not msg.is_absolute:
                raise ValueError("expected absolute target, not delta")

            frame = (msg.frame_id or "").strip().lower()
            if frame not in ("first_frame", "enu", "uav1_map"):
                raise ValueError("unsupported action frame: " + frame)

            yaw_unit = (msg.yaw_unit or "").strip().lower()
            if yaw_unit in ("rad", "radian", "radians"):
                target_yaw = float(values[3])
            elif yaw_unit in ("deg", "degree", "degrees"):
                target_yaw = math.radians(float(values[3]))
            else:
                raise ValueError("unsupported yaw unit: " + yaw_unit)

            now = rospy.Time.now().to_sec()
            observation_stamp = msg.header.stamp.to_sec()
            age = now - observation_stamp

            if observation_stamp <= 0 or not 0.0 <= age <= self.stale_sec:
                raise ValueError("stale observation age=%.3fs" % age)

            with self.lock:
                if not self.origin_locked or self.origin is None:
                    raise ValueError("origin not locked")

                pose = self.current_pose(now)
                if pose is None:
                    raise ValueError("control odometry unavailable")

                p_now, yaw_now, _ = pose
                p0, yaw0, origin_stamp = self.origin

                if observation_stamp < origin_stamp:
                    raise ValueError("observation predates current origin")

                if (
                    self.last_observation_stamp is not None
                    and observation_stamp <= self.last_observation_stamp
                ):
                    raise ValueError("duplicate or out-of-order observation")

                if (
                    self.enabled
                    and self.enable_stamp is not None
                    and observation_stamp < self.enable_stamp
                ):
                    raise ValueError("waiting for observation after enable")

                # 输入前三维已经由 HTTP 客户端转换为米。
                if frame == "first_frame":
                    target, target_yaw = first_frame_to_enu(
                        values[:3], target_yaw, p0, yaw0
                    )
                else:
                    target = values[:3].copy()
                    target_yaw = wrap_pi(target_yaw)

                if self.lock_altitude:
                    target[2] = self.fixed_altitude
                if not self.use_vla_yaw:
                    target_yaw = yaw_now

                target, target_yaw = clamp_step(
                    target,
                    target_yaw,
                    p_now,
                    yaw_now,
                    self.max_xy,
                    self.max_z,
                    self.max_yaw,
                )

                self.segment = (
                    p_now.copy(),
                    float(yaw_now),
                    target.copy(),
                    float(target_yaw),
                    now,
                )
                self.last_action_stamp = observation_stamp
                self.last_observation_stamp = observation_stamp

                out = VLAAction()
                out.header.stamp = msg.header.stamp
                out.header.seq = msg.header.seq
                out.header.frame_id = "enu"
                out.request_id = msg.request_id
                out.frame_id = "enu"
                out.is_absolute = True
                out.yaw_unit = "rad"
                out.action = [
                    float(target[0]),
                    float(target[1]),
                    float(target[2]),
                    float(target_yaw),
                ]
                out.action_ori = list(msg.action_ori)
                out.infer_ms = msg.infer_ms
                out.instruction = msg.instruction
                self.safe_pub.publish(out)

            rospy.loginfo(
                "target preview id=%d xyz_m=%s yaw_rad=%.3f",
                msg.request_id,
                target.tolist(),
                target_yaw,
            )

        except (ValueError, TypeError, OverflowError) as exc:
            rospy.logwarn_throttle(
                1.0, "drop action_raw: %s", exc
            )

    def on_timer(self, _event):
        now = rospy.Time.now()
        now_sec = now.to_sec()

        with self.lock:
            if self.segment is None:
                return

            if not self.origin_locked or self.origin is None:
                self.clear_target()
                return

            pose = self.current_pose(now_sec)
            if pose is None:
                self.clear_target()
                rospy.logwarn_throttle(
                    1.0, "control odometry stale; target cleared"
                )
                return

            age = now_sec - self.last_action_stamp
            if not 0.0 <= age <= self.stale_sec:
                self.clear_target()
                rospy.logwarn_throttle(
                    1.0, "target expired; stop publishing old command"
                )
                return

            p0, yaw0, p1, yaw1, start = self.segment
            alpha = (now_sec - start) / self.segment_duration
            p, yaw = lerp_pose(p0, yaw0, p1, yaw1, alpha)

            if not self.use_vla_yaw:
                yaw = pose[1]

            yaw_deg = math.degrees(float(yaw))
            command = self.fsm.build(
                p[0], p[1], p[2], yaw_deg, now
            )
            self.debug_pub.publish(command)

            actual_publish = bool(
                self.enable_publish
                and self.transform_verified
                and self.enabled
                and self.backend == "direct_fsm"
                and self.move_mode == 0
            )

            if actual_publish:
                self.fsm.publish_pose(
                    p[0], p[1], p[2], yaw_deg, now
                )

        rospy.loginfo_throttle(
            2.0,
            "command xyz_m=[%.3f, %.3f, %.3f] "
            "yaw_deg=%.2f publish=%s",
            p[0], p[1], p[2], yaw_deg, actual_publish,
        )


def main():
    rospy.init_node("vla_cmd_node")
    VlaCmdNode()
    rospy.spin()


if __name__ == "__main__":
    main()
