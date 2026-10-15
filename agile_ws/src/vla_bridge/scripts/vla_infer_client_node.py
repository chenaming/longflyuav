#!/usr/bin/env python3
"""HTTP client: paired observation -> POST /predict -> action_raw."""

import base64
import json
import math
import threading

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from vla_bridge.msg import VLAAction, VLAHeartbeat

try:
    import requests
except ImportError:
    requests = None


class VlaInferClientNode(object):
    def __init__(self):
        if requests is None:
            raise rospy.ROSException(
                "python3-requests is required: "
                "sudo apt install python3-requests"
            )

        host = rospy.get_param(
            "~server_host",
            rospy.get_param("/ground/host", "127.0.0.1"),
        )
        port = int(
            rospy.get_param(
                "~server_port",
                rospy.get_param("/ground/port", 5007),
            )
        )
        endpoint = rospy.get_param(
            "~endpoint",
            rospy.get_param("/ground/endpoint", "/predict"),
        )
        self._url = "http://%s:%s%s" % (host, port, endpoint)

        self._timeout = float(
            rospy.get_param(
                "~infer_timeout_sec",
                rospy.get_param("/ground/infer_timeout_sec", 1.5),
            )
        )

        rate_hz = float(
            rospy.get_param("/image/inference_rate_hz", 2.0)
        )
        self._default_instruction = rospy.get_param(
            "~default_instruction", "move forward"
        )
        image_timeout = float(
            rospy.get_param("/watchdog/image_timeout_sec", 0.5)
        )

        self._image_stale_sec = max(
            image_timeout * 2.0,
            2.0 / max(rate_hz, 0.1) + 0.15,
        )

        self._lock = threading.Lock()
        self._latest_image = None
        self._latest_proprio = None
        self._instruction = self._default_instruction
        self._in_flight = False
        self._allow_infer = False
        self._request_id = 0
        self._last_error = "waiting for image/proprio"
        self._last_alive = False
        self._last_rtt = -1.0

        self.action_pub = rospy.Publisher(
            "action_raw", VLAAction, queue_size=1
        )
        self.heartbeat_pub = rospy.Publisher(
            "heartbeat", VLAHeartbeat, queue_size=1
        )
        self.latency_pub = rospy.Publisher(
            "latency", Float32, queue_size=1
        )

        rospy.Subscriber(
            "observation",
            String,
            self._on_observation,
            queue_size=1,
        )
        rospy.Subscriber(
            "instruction",
            String,
            self._on_instruction,
            queue_size=1,
        )
        rospy.Subscriber(
            "allow_infer",
            Bool,
            self._on_allow_infer,
            queue_size=1,
        )

        period = 1.0 / max(rate_hz, 0.1)
        self._timer = rospy.Timer(
            rospy.Duration(period), self._on_timer
        )

        rospy.loginfo(
            "vla_infer_client_node ready "
            "url=%s timeout=%.2fs rate=%.1fHz",
            self._url,
            self._timeout,
            rate_hz,
        )

    def _on_observation(self, msg):
        try:
            data = json.loads(msg.data)
            values = [float(v) for v in data["proprio"]]

            if len(values) != 4 or not all(
                math.isfinite(v) for v in values
            ):
                raise ValueError("invalid proprio")

            image = CompressedImage()
            image.header.stamp = rospy.Time(
                int(data["stamp_secs"]),
                int(data["stamp_nsecs"]),
            )
            image.header.seq = int(data["seq"])
            image.header.frame_id = data["image_frame"]
            image.format = "jpeg"
            image.data = base64.b64decode(
                data["image"], validate=True
            )

            if image.header.stamp.to_sec() <= 0 or not image.data:
                raise ValueError(
                    "invalid image timestamp or empty image"
                )

            proprio = Float32MultiArray()
            proprio.data = values

            # 同时更新，保证图片与位姿来自同一个观测。
            with self._lock:
                self._latest_image = image
                self._latest_proprio = proprio

        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            rospy.logwarn_throttle(
                1.0,
                "drop invalid observation: %s",
                exc,
            )

    def _on_instruction(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return

        with self._lock:
            self._instruction = text

    def _on_allow_infer(self, msg):
        with self._lock:
            self._allow_infer = bool(msg.data)

    def _on_timer(self, _event):
        with self._lock:
            if self._in_flight:
                return

            if not self._allow_infer:
                self._publish_heartbeat_locked(
                    request_id=self._request_id,
                    alive=False,
                    rtt_sec=-1.0,
                    error="infer gated by watchdog",
                )
                return

            image = self._latest_image
            proprio_msg = self._latest_proprio
            instruction = self._instruction

            if image is None or proprio_msg is None:
                self._publish_heartbeat_locked(
                    request_id=self._request_id,
                    alive=False,
                    rtt_sec=-1.0,
                    error="waiting for image/proprio",
                )
                return

            stamp = image.header.stamp.to_sec()
            now = rospy.Time.now().to_sec()

            if stamp > 0.0 and (now - stamp) > self._image_stale_sec:
                self._publish_heartbeat_locked(
                    request_id=self._request_id,
                    alive=False,
                    rtt_sec=-1.0,
                    error="stale image age=%.2fs" % (now - stamp),
                )
                return

            proprio = [float(x) for x in proprio_msg.data]

            if len(proprio) != 4:
                self._publish_heartbeat_locked(
                    request_id=self._request_id,
                    alive=False,
                    rtt_sec=-1.0,
                    error="proprio length %d != 4" % len(proprio),
                )
                return

            self._request_id += 1
            request_id = self._request_id
            self._in_flight = True

        worker = threading.Thread(
            target=self._do_request,
            args=(request_id, image, proprio, instruction),
            name="vla-http-%d" % request_id,
        )
        worker.daemon = True
        worker.start()

    def _do_request(self, request_id, image, proprio, instruction):
        error = ""
        alive = False
        rtt = -1.0

        try:
            payload = {
                "image": base64.b64encode(
                    bytes(bytearray(image.data))
                ).decode("ascii"),

                # 桥接内部位置单位：米。
                # 模型输入位置单位：厘米。
                # yaw 保持为度。
                "proprio": [
                    proprio[0] * 100.0,
                    proprio[1] * 100.0,
                    proprio[2] * 100.0,
                    proprio[3],
                ],
                "instr": instruction,
                "request_id": request_id,
            }

            started = rospy.Time.now()

            response = requests.post(
                self._url,
                json=payload,
                timeout=self._timeout,
            )

            rtt = (rospy.Time.now() - started).to_sec()
            response.raise_for_status()
            body = response.json()

            if body.get("status") != "success":
                raise RuntimeError(
                    "server status=%s message=%s"
                    % (
                        body.get("status"),
                        body.get("message"),
                    )
                )

            action = self._as_vec4(
                body.get("action"), "action"
            )
            action_ori = self._as_vec4(
                body.get("action_ori"), "action_ori"
            )

            # 模型输出：位置 cm，角度 rad。
            # 桥接内部：位置 m，角度 rad。
            action = [
                action[0] / 100.0,
                action[1] / 100.0,
                action[2] / 100.0,
                action[3],
            ]

            action_ori = [
                action_ori[0] / 100.0,
                action_ori[1] / 100.0,
                action_ori[2] / 100.0,
                action_ori[3],
            ]

            self._publish_action(
                request_id,
                image.header,
                action,
                action_ori,
                rtt,
                instruction,
            )

            self.latency_pub.publish(Float32(data=rtt))
            alive = True

            rospy.loginfo_throttle(
                2.0,
                "predict ok id=%d rtt=%.3fs action=%s",
                request_id,
                rtt,
                action,
            )

        except Exception as exc:
            error = str(exc)
            rospy.logwarn_throttle(
                1.0,
                "predict failed id=%d: %s",
                request_id,
                error,
            )

        finally:
            with self._lock:
                self._in_flight = False
                self._last_alive = alive
                self._last_rtt = rtt
                self._last_error = error
                self._publish_heartbeat_locked(
                    request_id,
                    alive,
                    rtt,
                    error,
                )

    def _publish_action(
        self,
        request_id,
        image_header,
        action,
        action_ori,
        rtt,
        instruction,
    ):
        msg = VLAAction()

        # 保留拍照时间，不把旧观测标记成新观测。
        msg.header.stamp = image_header.stamp
        msg.header.seq = image_header.seq
        msg.header.frame_id = "first_frame"

        msg.request_id = request_id
        msg.frame_id = "first_frame"
        msg.is_absolute = True
        msg.yaw_unit = "rad"

        # 前三维为米，第四维为弧度。
        msg.action = action
        msg.action_ori = action_ori

        msg.infer_ms = float(rtt) * 1000.0
        msg.instruction = instruction

        self.action_pub.publish(msg)

    def _publish_heartbeat_locked(
        self,
        request_id,
        alive,
        rtt_sec,
        error,
    ):
        hb = VLAHeartbeat()
        hb.header.stamp = rospy.Time.now()
        hb.request_id = request_id
        hb.alive = bool(alive)
        hb.rtt_sec = float(rtt_sec)
        hb.last_error = error or ""
        self.heartbeat_pub.publish(hb)

    @staticmethod
    def _as_vec4(value, name):
        if (
            isinstance(value, (list, tuple))
            and value
            and isinstance(value[0], (list, tuple))
        ):
            value = value[0]

        if not isinstance(value, (list, tuple)) or len(value) < 4:
            raise ValueError(
                "%s must be length-4, got %r" % (name, value)
            )

        return [
            float(value[0]),
            float(value[1]),
            float(value[2]),
            float(value[3]),
        ]


def main():
    rospy.init_node("vla_infer_client_node")
    VlaInferClientNode()
    rospy.spin()


if __name__ == "__main__":
    main()
