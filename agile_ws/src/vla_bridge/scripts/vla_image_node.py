#!/usr/bin/env python3
"""Decode Odin JPEG, letterbox 224 RGB, publish first-frame proprio. No HTTP, no control."""

from collections import deque
import math
import os
import sys
import threading
import base64
import json

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, MultiArrayDimension, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vla_geometry import apply_odom_to_enu, first_frame_proprio, yaw_to_quat
from vla_image_proc import decode_jpeg_to_rgb, encode_jpeg_rgb, resize_rgb


class VlaImageNode(object):
    def __init__(self):
        self._image_topic = rospy.get_param("~image_topic", rospy.get_param("/image/source", "/uav1/odin1/image/compressed"))
        self._odom_topic = rospy.get_param("~odom_topic", rospy.get_param("/image/odometry", "/uav1/odin1/odometry"))
        self._rate_hz = float(rospy.get_param("~inference_rate_hz", rospy.get_param("/image/inference_rate_hz", 2.0)))
        self._size = int(rospy.get_param("~resize", rospy.get_param("/image/resize", 224)))
        self._resize_mode = rospy.get_param("~resize_mode", rospy.get_param("/image/resize_mode", "letterbox"))
        self._jpeg_quality = int(rospy.get_param("~jpeg_quality", rospy.get_param("/image/jpeg_quality", 80)))
        self._max_jpeg_bytes = int(rospy.get_param("~max_jpeg_bytes", rospy.get_param("/image/max_jpeg_bytes", 40000)))
        self._slop = float(rospy.get_param("~sync_slop_sec", rospy.get_param("/image/sync_slop_sec", 0.15)))

        rot = rospy.get_param("/frames/odom_to_enu/R", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        trans = rospy.get_param("/frames/odom_to_enu/t", [0.0, 0.0, 0.0])
        self._odom_R = np.asarray(rot, dtype=np.float64).reshape(3, 3)
        self._odom_t = np.asarray(trans, dtype=np.float64).reshape(3)

        self._lock = threading.Lock()
        self._latest_image = None
        self._odom_buf = deque(maxlen=40)
        self._origin = None
        self._origin_armed = False
        self._last_instruction = None
        self._drop_oversize = 0
        self._drop_desync = 0
        self.observation_pub = rospy.Publisher(
            "observation", String, queue_size=1
            )
        self.infer_image_pub = rospy.Publisher("image_for_infer", CompressedImage, queue_size=1)
        self.proprio_pub = rospy.Publisher("proprio", Float32MultiArray, queue_size=1)
        self.debug_image_pub = rospy.Publisher("debug/image", Image, queue_size=1)
        self.sync_dt_pub = rospy.Publisher("debug/image_odom_dt", Float32, queue_size=1)
        self.origin_pub = rospy.Publisher("first_frame_origin", PoseStamped, queue_size=1, latch=True)
        self.origin_locked_pub = rospy.Publisher("origin_locked", Bool, queue_size=1, latch=True)
        self.origin_locked_pub.publish(Bool(data=False))

        rospy.Subscriber(self._image_topic, CompressedImage, self._on_image, queue_size=1)
        rospy.Subscriber(self._odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber("instruction", String, self._on_instruction, queue_size=1)
        rospy.Subscriber("reset_origin", Empty, self._on_reset_origin, queue_size=1)

        period = 1.0 / max(self._rate_hz, 0.1)
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)
        rospy.loginfo(
            "vla_image_node ready src=%s odom=%s rate=%.1fHz size=%d mode=%s slop=%.3fs",
            self._image_topic,
            self._odom_topic,
            self._rate_hz,
            self._size,
            self._resize_mode,
            self._slop,
        )

    def _on_image(self, msg):
        with self._lock:
            self._latest_image = msg

    def _on_odom(self, msg):
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()
        with self._lock:
            self._odom_buf.append((stamp, msg))

    def _arm_origin_lock(self, reason):
        self._origin = None
        self._origin_armed = True
        self.origin_locked_pub.publish(Bool(data=False))
        rospy.loginfo("first_frame origin will lock (%s)", reason)

    def _publish_origin(self, p0, yaw0, frame_id, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id or "enu"
        msg.pose.position.x = float(p0[0])
        msg.pose.position.y = float(p0[1])
        msg.pose.position.z = float(p0[2])
        qx, qy, qz, qw = yaw_to_quat(yaw0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.origin_pub.publish(msg)
        self.origin_locked_pub.publish(Bool(data=True))

    def _on_instruction(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return
        with self._lock:
            if text == self._last_instruction:
                return
            prev = self._last_instruction
            self._last_instruction = text
            self._arm_origin_lock("instruction %r -> %r" % (prev, text))

    def _on_reset_origin(self, _msg):
        with self._lock:
            self._arm_origin_lock("reset_origin")

    def _nearest_odom(self, image_stamp, buf):
        if not buf:
            return None, None
        best_msg = None
        best_dt = None
        for stamp, msg in buf:
            dt = abs(stamp - image_stamp)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_msg = msg
        return best_msg, best_dt

    def _on_timer(self, _event):
        with self._lock:
            image_msg = self._latest_image
            odom_buf = list(self._odom_buf)
            origin = self._origin
            origin_armed = self._origin_armed

        if not origin_armed and origin is None:
            rospy.logwarn_throttle(2.0, "waiting for /vla/instruction or /vla/reset_origin to lock first_frame")
            return

        if image_msg is None:
            rospy.logwarn_throttle(2.0, "waiting for image on %s", self._image_topic)
            return

        image_stamp = image_msg.header.stamp.to_sec()
        if image_stamp <= 0.0:
            image_stamp = rospy.Time.now().to_sec()

        odom_msg, dt = self._nearest_odom(image_stamp, odom_buf)
        if dt is not None:
            self.sync_dt_pub.publish(Float32(data=float(dt)))
        if odom_msg is None or dt is None or dt > self._slop:
            self._drop_desync += 1
            rospy.logwarn_throttle(
                1.0,
                "IMAGE_ODOM_DESYNC dt=%s slop=%.3f drops=%d",
                "none" if dt is None else ("%.3f" % dt),
                self._slop,
                self._drop_desync,
            )
            return

        rgb = decode_jpeg_to_rgb(image_msg.data)
        if rgb is None:
            rospy.logwarn_throttle(1.0, "JPEG decode failed")
            return

        rgb224 = resize_rgb(rgb, self._size, self._resize_mode)
        jpeg = encode_jpeg_rgb(rgb224, self._jpeg_quality)
        if jpeg is None:
            rospy.logwarn_throttle(1.0, "JPEG encode failed")
            return
        if len(jpeg) > self._max_jpeg_bytes:
            self._drop_oversize += 1
            rospy.logwarn_throttle(
                1.0,
                "drop oversize JPEG %d > %d drops=%d",
                len(jpeg),
                self._max_jpeg_bytes,
                self._drop_oversize,
            )
            return

        pose = odom_msg.pose.pose
        p_enu, yaw_enu = apply_odom_to_enu(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            self._odom_R,
            self._odom_t,
        )
        if origin is None:
            origin = (p_enu.copy(), float(yaw_enu), odom_msg.header.frame_id or "")
            with self._lock:
                if self._origin is None:
                    self._origin = origin
                    rospy.loginfo(
                        "locked first_frame origin p0=[%.3f, %.3f, %.3f] yaw0_deg=%.2f odom_frame=%s",
                        p_enu[0],
                        p_enu[1],
                        p_enu[2],
                        math.degrees(yaw_enu),
                        origin[2],
                    )
            self._publish_origin(p_enu, yaw_enu, origin[2], odom_msg.header.stamp)
            p0, yaw0, _frame = origin
        else:
            p0, yaw0, _frame = origin

        proprio = first_frame_proprio(p_enu, yaw_enu, p0, yaw0)

        out = CompressedImage()
        out.header.stamp = image_msg.header.stamp
        out.header.frame_id = image_msg.header.frame_id
        out.header.seq = image_msg.header.seq
        out.format = "jpeg"
        out.data = jpeg
        self.infer_image_pub.publish(out)

        arr = Float32MultiArray()
        arr.layout.dim = [
            MultiArrayDimension(label="proprio", size=4, stride=4)
        ]
        arr.data = proprio
        self.proprio_pub.publish(arr)

        observation = {
            "stamp_secs": image_msg.header.stamp.secs,
            "stamp_nsecs": image_msg.header.stamp.nsecs,
            "seq": image_msg.header.seq,
            "image_frame": image_msg.header.frame_id,
            "image": base64.b64encode(bytes(jpeg)).decode("ascii"),
            "proprio": [float(v) for v in proprio],
        }

        self.observation_pub.publish(
            String(data=json.dumps(observation, allow_nan=False))
        )
        debug = Image()
        debug.header = out.header
        debug.header.frame_id = "vla_letterbox"
        debug.height = int(rgb224.shape[0])
        debug.width = int(rgb224.shape[1])
        debug.encoding = "rgb8"
        debug.is_bigendian = 0
        debug.step = int(rgb224.shape[1] * 3)
        debug.data = np.ascontiguousarray(rgb224).tobytes()
        self.debug_image_pub.publish(debug)

        rospy.loginfo_throttle(
            2.0,
            "image_for_infer %dx%d jpeg=%dB dt=%.3fs proprio=[%.3f, %.3f, %.3f, %.2f]",
            rgb224.shape[1],
            rgb224.shape[0],
            len(jpeg),
            dt,
            proprio[0],
            proprio[1],
            proprio[2],
            proprio[3],
        )


def main():
    rospy.init_node("vla_image_node")
    VlaImageNode()
    rospy.spin()


if __name__ == "__main__":
    main()
