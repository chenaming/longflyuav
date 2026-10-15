#!/usr/bin/env python3
"""
ROS1 OpenVLA bridge.

Input:
  compressed image + Odin odometry + MAVROS odometry

Model:
  proprio: first-frame position in cm, yaw in degrees
  action:  first-frame target position in cm, yaw in radians

Output:
  preview PoseStamped in MAVROS odometry frame
  optional uav_control_fsm/uav_command

Does not arm, take off, switch flight modes, or plan around obstacles.
"""

import base64
import io
import math
import threading
import time
import uuid

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / "fly"))
from latest_observation import LatestObservation
import numpy as np
import requests
import rospy
import tf2_ros

from PIL import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Empty, String
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_matrix,
    quaternion_multiply,
)
from uav_control_fsm.msg import uav_command


def wrap_pi(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_values(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)

    if not np.all(np.isfinite(values)) or norm < 1e-8:
        raise ValueError("invalid quaternion")

    return values / norm


def pose_values(odom):
    pose = odom.pose.pose
    position = np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=float,
    )

    if not np.all(np.isfinite(position)):
        raise ValueError("invalid position")

    yaw = euler_from_quaternion(
        quaternion_values(pose.orientation)
    )[2]

    return position, float(yaw)


def vec4(value, name):
    if (
        isinstance(value, (list, tuple))
        and len(value) == 1
        and isinstance(value[0], (list, tuple))
    ):
        value = value[0]

    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("%s must have exactly 4 values" % name)

    result = np.asarray(value, dtype=float)

    if not np.all(np.isfinite(result)):
        raise ValueError("%s contains NaN or Inf" % name)

    return result


def rotation_z(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)

    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


class VLABridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.run_id = rospy.get_param("~run_id", uuid.uuid4().hex)
        rospy.set_param("~run_id", self.run_id)

        self.url = rospy.get_param(
            "~url", "http://10.62.25.215:5007/predict"
        )
        self.instruction = rospy.get_param(
            "~instruction",
            "Move forward and then return to the starting point",
        ).strip()
        rospy.set_param("~active_instruction", self.instruction)

        # 网络等待时限与动作有效期是两个不同参数。
        self.http_timeout = float(
            rospy.get_param("~http_timeout", 60.0)
        )
        self.max_observation_age = float(
            rospy.get_param("~max_observation_age", 1.0)
        )
        self.sensor_timeout = float(
            rospy.get_param("~sensor_timeout", 0.5)
        )
        self.max_target_hold = float(
            rospy.get_param("~max_target_hold", 2.0)
        )

        self.max_xy = float(rospy.get_param("~max_step_xy", 0.35))
        self.max_z = float(rospy.get_param("~max_step_z", 0.05))
        self.max_yaw = math.radians(
            float(rospy.get_param("~max_yaw_step_deg", 8.0))
        )
        self.command_speed = float(
            rospy.get_param("~command_speed", 0.2)
        )
        self.command_yaw_rate = math.radians(
            float(rospy.get_param("~command_yaw_rate_deg", 20.0))
        )

        self.enable_publish = bool(
            rospy.get_param("~enable_publish", False)
        )

        # 仅在确认 FSM 的坐标系、位置单位和 yaw 单位后才能设为 true。
        self.fsm_contract_verified = bool(
            rospy.get_param("~fsm_contract_verified", False)
        )
        self.fsm_frame = rospy.get_param(
            "~fsm_frame", "uav1_map"
        )

        if (
            self.http_timeout <= 0
            or self.max_observation_age <= 0
            or self.sensor_timeout <= 0
            or self.max_target_hold <= 0
            or self.max_xy <= 0
            or self.max_z <= 0
            or self.max_yaw <= 0
            or self.command_speed <= 0
            or self.command_yaw_rate <= 0
        ):
            raise ValueError("timing and movement limits must be positive")

        self.preview_pair_alignment = bool(
            rospy.get_param("~preview_pair_alignment", False)
        )
        if self.preview_pair_alignment and self.enable_publish:
            raise ValueError(
                "preview_pair_alignment requires enable_publish=false"
            )
        self.candidate_pub = None
        if self.preview_pair_alignment:
            self.candidate_pub = rospy.Publisher(
                "/vla/target_candidate", PoseStamped, queue_size=1
            )
            rospy.logwarn(
                "PAIR PREVIEW ONLY: assumes matching body axes and "
                "reference points; alignment is not calibrated."
            )

        self.session = requests.Session()
        self.session.trust_env = False

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(120.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.epoch = 0
        self.origin = None
        self.latest = None
        self.last_sent_key = None
        self.in_flight = False
        self.request_id = int(rospy.get_param("~request_id_start", 0))
        self.request_started = 0.0

        self.operator_enable = False
        self.enable_after = None
        self.estop = False

        self.control_pose = None
        self.flight_state = None

        self.target = None
        self.active_control = False
        self.last_reason = "waiting for observations"

        self.preview_pub = rospy.Publisher(
            "/vla/target_preview", PoseStamped, queue_size=1
        )
        self.validated_pub = rospy.Publisher(
            "/vla/validated_prediction", String, queue_size=1
        )
        self.audit_pub = rospy.Publisher(
            "/vla/coordinate_audit", String, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/vla/bridge_status", String, queue_size=1
        )

        self.command_pub = None
        self.hover_pub = None

        if self.enable_publish and self.fsm_contract_verified:
            self.command_pub = rospy.Publisher(
                "/uav1/move_command", uav_command, queue_size=1
            )
            self.hover_pub = rospy.Publisher(
                "/uav1/hover_command", Bool, queue_size=1
            )

        rospy.Subscriber(
            "/vla/instruction", String, self.on_instruction,
            queue_size=1,
        )
        rospy.Subscriber(
            "/vla/reset_origin", Empty, self.on_reset,
            queue_size=1,
        )
        rospy.Subscriber(
            "/vla/operator_enable", Bool, self.on_enable,
            queue_size=1,
        )
        rospy.Subscriber(
            "/vla/emergency_stop", Bool, self.on_estop,
            queue_size=1,
        )
        rospy.Subscriber(
            "/uav1/mavros/state", State, self.on_state,
            queue_size=1,
        )
        rospy.Subscriber(
            "/uav1/mavros/local_position/odom",
            Odometry,
            self.on_control_pose,
            queue_size=1,
        )

        self.latest_images = LatestObservation(slop=0.05)
        self.image_sub = rospy.Subscriber(
            "/uav1/odin1/image/compressed", CompressedImage,
            self.latest_images.put_image, queue_size=1, buff_size=8*1024*1024)
        self.odin_sub = rospy.Subscriber(
            "/uav1/odin1/odometry", Odometry,
            lambda m: self.latest_images.put_pose("odin", m), queue_size=1)
        self.mavros_sub = rospy.Subscriber(
            "/uav1/mavros/local_position/odom", Odometry,
            lambda m: self.latest_images.put_pose("mavros", m), queue_size=1)
        self.inference_hz = float(rospy.get_param("~inference_hz", 5.0))
        if not math.isfinite(self.inference_hz) or not 0.1 <= self.inference_hz <= 20:
            raise ValueError("inference_hz must be between 0.1 and 20")
        self.request_timer = rospy.Timer(
            rospy.Duration(1.0 / self.inference_hz), self.on_request_timer)
        self.control_timer = rospy.Timer(
            rospy.Duration(0.04), self.on_control_timer
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0), self.on_status_timer
        )

        rospy.loginfo(
            "bridge started: preview=%s url=%s",
            self.command_pub is None,
            self.url,
        )

    def flight_ready(self):
        if self.flight_state is None:
            return False

        received, armed, mode = self.flight_state

        return (
            time.monotonic() - received <= 2.0
            and armed
            and mode == "OFFBOARD"
        )

    def stop_control(self, reason):
        # 调用者持有 self.lock。
        if self.active_control:
            # 人工已切出 Offboard 时，不再插入自动控制命令。
            if self.hover_pub is not None and self.flight_ready():
                self.hover_pub.publish(Bool(data=True))

        self.active_control = False
        self.operator_enable = False
        self.enable_after = None
        self.target = None
        self.last_reason = reason

    def invalidate(self, reason, reset_origin=False):
        # 使正在执行的 HTTP 请求失效，但不强行中断网络线程。
        self.epoch += 1
        self.latest = None
        self.last_sent_key = None
        self.stop_control(reason)

        if reset_origin:
            self.origin = None

    def on_instruction(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return

        with self.lock:
            if text != self.instruction:
                self.instruction = text
                rospy.set_param("~active_instruction", text)
                self.invalidate(
                    "instruction changed; waiting for new origin",
                    reset_origin=True,
                )

    def on_reset(self, _msg):
        with self.lock:
            self.invalidate(
                "origin reset; waiting for new observation",
                reset_origin=True,
            )

    def on_enable(self, msg):
        with self.lock:
            if not msg.data:
                self.invalidate("operator disabled")
                return

            if self.estop or not self.flight_ready():
                self.last_reason = "enable rejected: estop or flight state"
                return

            if self.command_pub is None:
                self.last_reason = "enable rejected: preview mode"
                return

            if self.operator_enable:
                return

            self.invalidate("waiting for observation after enable")
            self.operator_enable = True
            self.enable_after = rospy.Time.now().to_sec()

    def on_estop(self, msg):
        with self.lock:
            self.estop = bool(msg.data)

            if self.estop:
                self.invalidate("emergency stop")
            # 解除急停不会自动恢复人工使能。

    def on_state(self, msg):
        with self.lock:
            self.flight_state = (
                time.monotonic(),
                bool(msg.armed),
                (msg.mode or "").strip(),
            )

    def on_control_pose(self, msg):
        try:
            p, yaw = pose_values(msg)
            stamp = msg.header.stamp.to_sec()

            if stamp <= 0 or not msg.header.frame_id:
                return

            with self.lock:
                self.control_pose = (
                    p, yaw, stamp, time.monotonic(),
                    msg.header.frame_id,
                )

        except (ValueError, TypeError, OverflowError):
            pass

    def current_control(self):
        # 调用者持有 self.lock。
        if self.control_pose is None:
            return None

        p, yaw, stamp, received, frame = self.control_pose
        age = rospy.Time.now().to_sec() - stamp

        if (
            not 0.0 <= age <= self.sensor_timeout
            or time.monotonic() - received > self.sensor_timeout
        ):
            return None

        return p, yaw, frame

    def on_observation(self, image, odin, mavros):
        try:
            p_odin, yaw_odin = pose_values(odin)
            p_control, yaw_control = pose_values(mavros)

            stamp = image.header.stamp.to_sec()
            age = rospy.Time.now().to_sec() - stamp

            if (
                stamp <= 0
                or not 0.0 <= age <= self.sensor_timeout
                or not odin.header.frame_id
                or not mavros.header.frame_id
                or not odin.child_frame_id
                or not mavros.child_frame_id
            ):
                return

            frame_identity = (
                odin.header.frame_id,
                odin.child_frame_id,
                mavros.header.frame_id,
                mavros.child_frame_id,
            )

            with self.lock:
                if self.estop or not self.instruction:
                    return

                if self.origin is None:
                    self.origin = (
                        p_odin.copy(),
                        yaw_odin,
                        frame_identity,
                    )
                    rospy.loginfo(
                        "origin locked: position_m=%s yaw_deg=%.2f",
                        p_odin.tolist(),
                        math.degrees(yaw_odin),
                    )

                p0, yaw0, origin_frames = self.origin

                if frame_identity != origin_frames:
                    self.invalidate(
                        "odometry frame changed; reset required",
                        reset_origin=True,
                    )
                    return

                local = rotation_z(yaw0).T.dot(p_odin - p0)

                # 模型输入：cm、deg。
                proprio = [
                    float(local[0] * 100.0),
                    float(local[1] * 100.0),
                    float(local[2] * 100.0),
                    math.degrees(wrap_pi(yaw_odin - yaw0)),
                ]

                self.latest = {
                    "epoch": self.epoch,
                    "instruction": self.instruction,
                    "image": image,
                    "proprio": proprio,
                    "origin_p": p0.copy(),
                    "origin_yaw": yaw0,
                    "source_frame": odin.header.frame_id,
                    "target_frame": mavros.header.frame_id,
                    "control_p": p_control.copy(),
                    "control_yaw": yaw_control,
                    "received": time.monotonic(),
                }

        except (ValueError, TypeError, OverflowError) as exc:
            rospy.logwarn_throttle(
                1.0, "observation rejected: %s", exc
            )

    def on_request_timer(self, _event):
        with self.lock:
            if self.in_flight or self.estop:
                return
            observation = self.latest_images.snapshot(rospy.Time.now().to_sec())
            if observation is None:
                self.last_reason = "waiting for newest image and time-matched poses"
                return
            self.latest = None
            self.on_observation(*observation)
            if self.latest is None:
                return
            sample = self.latest
            image = sample["image"]
            stamp = image.header.stamp.to_sec()
            age = rospy.Time.now().to_sec() - stamp

            if (
                not 0.0 <= age <= self.sensor_timeout
                or time.monotonic() - sample["received"] > self.sensor_timeout
            ):
                self.last_reason = "waiting for fresh synchronized observation"
                return

            key = (
                sample["epoch"],
                image.header.stamp.secs,
                image.header.stamp.nsecs,
                image.header.seq,
            )

            if key == self.last_sent_key:
                return

            self.last_sent_key = key
            self.request_id += 1
            request_id = self.request_id
            self.in_flight = True
            self.request_started = time.monotonic()

        threading.Thread(
            target=self.request_worker,
            args=(request_id, sample),
            daemon=True,
        ).start()

    def transform_target(self, sample, action):
        # action 已是 first_frame 下的 m、rad。
        p_odom = (
            sample["origin_p"]
            + rotation_z(sample["origin_yaw"]).dot(action[:3])
        )
        yaw_odom = wrap_pi(sample["origin_yaw"] + action[3])

        if self.preview_pair_alignment:
            # 仅估算预览：假设同一机体参考点、相同机体轴向和竖直方向。
            observed = np.asarray(sample["proprio"], dtype=float)
            p_odin_observed = (
                sample["origin_p"]
                + rotation_z(sample["origin_yaw"]).dot(observed[:3] / 100.0)
            )
            yaw_odin_observed = wrap_pi(
                sample["origin_yaw"] + math.radians(observed[3])
            )
            alignment_yaw = wrap_pi(
                sample["control_yaw"] - yaw_odin_observed
            )

            target_p = (
                sample["control_p"]
                + rotation_z(alignment_yaw).dot(p_odom - p_odin_observed)
            )
            target_yaw = wrap_pi(alignment_yaw + yaw_odom)

            # 候选结果不代表已通过后续动作范围检查。
            candidate = PoseStamped()
            candidate.header.stamp = sample["image"].header.stamp
            candidate.header.frame_id = sample["target_frame"]
            candidate.pose.position.x = float(target_p[0])
            candidate.pose.position.y = float(target_p[1])
            candidate.pose.position.z = float(target_p[2])

            q = quaternion_from_euler(0.0, 0.0, target_yaw)
            candidate.pose.orientation.x = q[0]
            candidate.pose.orientation.y = q[1]
            candidate.pose.orientation.z = q[2]
            candidate.pose.orientation.w = q[3]

            self.candidate_pub.publish(candidate)
            rospy.loginfo(
                "PAIR CANDIDATE ONLY frame=%s xyz_m=%s yaw_deg=%.2f "
                "estimated_alignment_yaw_deg=%.2f",
                sample["target_frame"], target_p.tolist(),
                math.degrees(target_yaw), math.degrees(alignment_yaw),
            )
            return target_p, target_yaw

        source = sample["source_frame"]
        target = sample["target_frame"]

        if source == target:
            return p_odom, yaw_odom

        # 使用观测时刻的变换，不能用响应返回时刻的变换。
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            sample["image"].header.stamp,
            rospy.Duration(0.1),
        )

        t = transform.transform.translation
        q = quaternion_values(transform.transform.rotation)
        rotation = quaternion_matrix(q)[:3, :3]

        p_target = rotation.dot(p_odom) + np.array([t.x, t.y, t.z])
        target_q = quaternion_multiply(
            q, quaternion_from_euler(0.0, 0.0, yaw_odom)
        )
        yaw_target = euler_from_quaternion(target_q)[2]

        return p_target, float(yaw_target)

    def request_worker(self, request_id, sample):
        started = time.monotonic()

        try:
            # 减少传输量；图像预处理需与训练保持一致。
            rgb = Image.open(
                io.BytesIO(bytes(sample["image"].data))
            ).convert("RGB")
            rgb = rgb.resize((224, 224))

            buffer = io.BytesIO()
            rgb.save(buffer, format="JPEG", quality=85)

            payload = {
                "image": base64.b64encode(
                    buffer.getvalue()
                ).decode("ascii"),
                "instr": sample["instruction"],
                "proprio": sample["proprio"],
                "request_id": request_id,
            }

            response = self.session.post(
                self.url,
                json=payload,
                timeout=(5.0, self.http_timeout),
            )
            response.raise_for_status()
            result = response.json()

            if result.get("status") != "success":
                raise ValueError(
                    "server error: %s" % result.get("message")
                )

            action = vec4(result.get("action"), "action")
            raw_delta = vec4(result.get("action_ori"), "action_ori")
            proprio = np.asarray(sample["proprio"], dtype=float)
            input_yaw = math.radians(proprio[3])
            expected = np.r_[
                proprio[:3] + rotation_z(input_yaw).dot(raw_delta[:3]),
                wrap_pi(input_yaw + raw_delta[3]),
            ]
            error = action - expected
            error[3] = wrap_pi(error[3])
            if np.max(np.abs(error)) > 1e-3:
                raise ValueError("server action/action_ori coordinate contract mismatch")
            self.audit_pub.publish(String(data=__import__("json").dumps({
                "request_id": request_id,
                "run_id": self.run_id,
                "instruction": sample["instruction"],
                "server_composition_verified": True,
                "input_position_unit": "cm", "input_yaw_unit": "deg",
                "output_position_unit": "cm", "output_yaw_unit": "rad",
                "max_composition_error": float(np.max(np.abs(error))),
                "physical_axes_calibrated": False,
                "coordinate_mode": "paired_estimate" if self.preview_pair_alignment else "tf",
                "action_ori": raw_delta.tolist(),
            })))
            action[:3] /= 100.0

            elapsed = time.monotonic() - started
            observation_age = (
                rospy.Time.now().to_sec()
                - sample["image"].header.stamp.to_sec()
            )

            rospy.loginfo(
                "HTTP success id=%d elapsed=%.2fs age=%.2fs "
                "action_firstframe_m_rad=%s",
                request_id,
                elapsed,
                observation_age,
                action.tolist(),
            )

            with self.lock:
                if sample["epoch"] != self.epoch or self.estop:
                    raise ValueError("obsolete request: task or control changed")

            # 同时用 ROS 观测时间和单调时钟检查旧结果。
            if (
                not 0.0 <= observation_age <= self.max_observation_age
                or elapsed > self.max_observation_age
            ):
                raise ValueError(
                    "stale prediction: age=%.2fs limit=%.2fs"
                    % (observation_age, self.max_observation_age)
                )

            target_p, target_yaw = self.transform_target(sample, action)

            # COORD_TRACE_V1: 同一次请求的坐标记录，位于步长检查之前。
            trace = {
                "request_id": request_id,
                "epoch": sample["epoch"],
                "run_id": self.run_id,
                "instruction": sample["instruction"],
                "image_stamp": sample["image"].header.stamp.to_sec(),
                "elapsed_s": elapsed,
                "observation_age_s": observation_age,
                "preview_pair_alignment": self.preview_pair_alignment,
                "control_publishing_configured": self.command_pub is not None,
                "source_frame": sample["source_frame"],
                "target_frame": sample["target_frame"],
                "origin_odin_xyz_m": sample["origin_p"].tolist(),
                "origin_odin_yaw_deg": math.degrees(sample["origin_yaw"]),
                "input_proprio_cm_deg": list(sample["proprio"]),
                "server_action_ori_cm_rad": raw_delta.tolist(),
                "server_composition_max_error": float(np.max(np.abs(error))),
                "server_action_cm_rad": vec4(
                    result.get("action"), "action"
                ).tolist(),
                "converted_action_m_rad": action.tolist(),
                "paired_mavros_xyz_m": sample["control_p"].tolist(),
                "paired_mavros_yaw_deg": math.degrees(sample["control_yaw"]),
                "target_mavros_xyz_m": target_p.tolist(),
                "target_mavros_yaw_deg": math.degrees(target_yaw),
                "target_minus_paired_mavros_m": (
                    target_p - sample["control_p"]
                ).tolist(),
                "target_minus_paired_mavros_yaw_deg": math.degrees(
                    wrap_pi(target_yaw - sample["control_yaw"])
                ),
            }
            rospy.loginfo(
                "COORD_TRACE " + __import__("json").dumps(
                    trace, ensure_ascii=False, allow_nan=False
                )
            )

            if not np.all(np.isfinite(target_p)):
                raise ValueError("invalid transformed target")

            with self.lock:
                if sample["epoch"] != self.epoch or self.estop:
                    raise ValueError("request invalidated during transform")

                current = self.current_control()
                if current is None:
                    raise ValueError("no fresh MAVROS odometry")

                p_now, yaw_now, frame = current
                if frame != sample["target_frame"]:
                    raise ValueError("control frame changed")

                # 限制相对观测时刻与当前时刻的目标偏移。
                for position, yaw in (
                    (sample["control_p"], sample["control_yaw"]),
                    (p_now, yaw_now),
                ):
                    delta = target_p - position
                    if np.linalg.norm(delta[:2]) > self.max_xy:
                        raise ValueError("target exceeds XY step limit")
                    if abs(delta[2]) > self.max_z:
                        raise ValueError("target exceeds Z step limit")
                    if abs(wrap_pi(target_yaw - yaw)) > self.max_yaw:
                        raise ValueError("target exceeds yaw step limit")

                self.validated_pub.publish(String(data=__import__("json").dumps({
                    "run_id": self.run_id, "request_id": request_id,
                    "instruction": sample["instruction"],
                    "observation_stamp": sample["image"].header.stamp.to_sec(),
                    "validated_stamp": rospy.Time.now().to_sec(),
                    "target_frame": frame, "target_xyz_m": target_p.tolist(),
                    "target_yaw_rad": target_yaw,
                    "coordinate_mode": "paired_estimate" if self.preview_pair_alignment else "tf",
                    "physical_axes_calibrated": False,
                    "epoch": sample["epoch"], "latest_only": True,
                    "action_ori_cm_rad": raw_delta.tolist(),
                    "observed_xyz_m": sample["control_p"].tolist(),
                    "observed_yaw_rad": sample["control_yaw"],
                }, ensure_ascii=False)))

                preview = PoseStamped()
                preview.header.stamp = sample["image"].header.stamp
                preview.header.frame_id = frame
                preview.pose.position.x = float(target_p[0])
                preview.pose.position.y = float(target_p[1])
                preview.pose.position.z = float(target_p[2])

                q = quaternion_from_euler(0.0, 0.0, target_yaw)
                preview.pose.orientation.x = q[0]
                preview.pose.orientation.y = q[1]
                preview.pose.orientation.z = q[2]
                preview.pose.orientation.w = q[3]
                self.preview_pub.publish(preview)

                if (
                    self.operator_enable
                    and self.enable_after is not None
                    and sample["image"].header.stamp.to_sec() < self.enable_after
                ):
                    raise ValueError("observation predates operator enable")

                duration = max(
                    0.2,
                    float(np.linalg.norm(target_p - p_now))
                    / self.command_speed,
                    abs(wrap_pi(target_yaw - yaw_now))
                    / self.command_yaw_rate,
                )

                self.target = {
                    "epoch": self.epoch,
                    "p0": p_now.copy(),
                    "yaw0": yaw_now,
                    "p1": target_p.copy(),
                    "yaw1": target_yaw,
                    "frame": frame,
                    "start": time.monotonic(),
                    "duration": duration,
                }
                self.last_reason = "valid target preview"

                rospy.loginfo(
                    "target preview frame=%s xyz_m=%s yaw_deg=%.2f",
                    frame,
                    target_p.tolist(),
                    math.degrees(target_yaw),
                )

        except Exception as exc:
            with self.lock:
                if sample["epoch"] == self.epoch:
                    if self.operator_enable or self.active_control:
                        self.invalidate("prediction rejected: " + str(exc))
                    else:
                        self.last_reason = "prediction rejected: " + str(exc)

            rospy.logwarn("request id=%d: %s", request_id, exc)

        finally:
            with self.lock:
                self.in_flight = False

    def on_control_timer(self, _event):
        with self.lock:
            if not self.operator_enable:
                return

            if self.estop or not self.flight_ready():
                self.invalidate("control stopped: flight state or estop")
                return

            current = self.current_control()
            if current is None:
                self.invalidate("control stopped: stale MAVROS odometry")
                return

            if (
                self.latest is None
                or time.monotonic() - self.latest["received"] > self.sensor_timeout
            ):
                self.invalidate("control stopped: synchronized sensors stale")
                return

            if self.target is None:
                return

            target = self.target

            if target["epoch"] != self.epoch:
                self.invalidate("control stopped: obsolete target")
                return

            if current[2] != target["frame"]:
                self.invalidate("control stopped: frame changed")
                return

            elapsed = time.monotonic() - target["start"]
            if elapsed > self.max_target_hold:
                self.invalidate("control stopped: target hold expired")
                return

            if self.command_pub is None:
                return

            alpha = min(max(elapsed / target["duration"], 0.0), 1.0)
            position = (
                target["p0"] + alpha * (target["p1"] - target["p0"])
            )
            yaw = wrap_pi(
                target["yaw0"]
                + alpha * wrap_pi(target["yaw1"] - target["yaw0"])
            )

            command = uav_command()
            command.header.stamp = rospy.Time.now()
            command.header.frame_id = self.fsm_frame
            command.Move_mode = 0
            command.pos_ref.x = float(position[0])
            command.pos_ref.y = float(position[1])
            command.pos_ref.z = float(position[2])
            command.yaw_ref = math.degrees(yaw)
            command.yaw_rate_ref = 0.0

            self.command_pub.publish(command)
            self.active_control = True
            self.last_reason = "publishing position command"

    def on_status_timer(self, _event):
        with self.lock:
            elapsed = (
                time.monotonic() - self.request_started
                if self.in_flight else 0.0
            )
            text = (
                "preview=%s operator=%s origin=%s "
                "in_flight=%s elapsed=%.1fs run_id=%s instruction=%s detail=%s"
                % (
                    self.command_pub is None,
                    self.operator_enable,
                    self.origin is not None,
                    self.in_flight,
                    elapsed,
                    self.run_id,
                    self.instruction,
                    self.last_reason,
                )
            )

        self.status_pub.publish(String(data=text))
        rospy.loginfo_throttle(2.0, text)


if __name__ == "__main__":
    rospy.init_node("vla_direct_bridge")
    VLABridge()
    rospy.spin()
