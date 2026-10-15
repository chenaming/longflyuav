import base64
import math
import threading
import time

import message_filters
import requests
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage


class CameraSender:
    def __init__(self):
        self.url = rospy.get_param(
            "~url", "http://10.62.25.215:5007/predict"
        )
        self.instruction = rospy.get_param(
            "~instruction",
            "Move forward and then return to the starting point",
        )

        self.lock = threading.Lock()
        self.origin = None
        self.latest = None

        self.session = requests.Session()
        self.session.trust_env = False

        self.image_sub = message_filters.Subscriber(
            "/uav1/odin1/image/compressed",
            CompressedImage,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )
        self.odom_sub = message_filters.Subscriber(
            "/uav1/odin1/odometry",
            Odometry,
            queue_size=50,
        )

        # 图像和里程计时间戳相差不超过 0.05 秒
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.odom_sub],
            queue_size=30,
            slop=0.05,
        )
        self.sync.registerCallback(self.on_pair)

    def on_pair(self, image_msg, odom_msg):
        pose = odom_msg.pose.pose
        p = pose.position
        q = pose.orientation

        # 归一化四元数
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-8:
            rospy.logwarn_throttle(5, "无效四元数，跳过此帧")
            return

        qx, qy, qz, qw = (
            q.x / norm, q.y / norm, q.z / norm, q.w / norm
        )

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, yaw)):
            return

        with self.lock:
            # 首个成功配对的图像和位姿确定本次任务原点
            if self.origin is None:
                self.origin = (
                    p.x, p.y, p.z, yaw,
                    odom_msg.header.frame_id,
                    odom_msg.child_frame_id,
                )
                rospy.loginfo(
                    "原点已固定：位置(m)=[%.4f, %.4f, %.4f], yaw=%.2f度",
                    p.x, p.y, p.z, math.degrees(yaw),
                )

            x0, y0, z0, yaw0, frame0, child0 = self.origin

            if (
                odom_msg.header.frame_id != frame0
                or odom_msg.child_frame_id != child0
            ):
                rospy.logerr_throttle(5, "里程计坐标系发生变化，停止接收该帧")
                return

            dx, dy, dz = p.x - x0, p.y - y0, p.z - z0
            c, s = math.cos(yaw0), math.sin(yaw0)

            # 旋转到首帧朝向的坐标系，并从米转为厘米
            relative_x = (c * dx + s * dy) * 100.0
            relative_y = (-s * dx + c * dy) * 100.0
            relative_z = dz * 100.0

            relative_yaw = (yaw - yaw0 + math.pi) % (2 * math.pi) - math.pi

            proprio = [
                relative_x,
                relative_y,
                relative_z,
                math.degrees(relative_yaw),
            ]

            stamp_gap = abs(
                (image_msg.header.stamp - odom_msg.header.stamp).to_sec()
            )

            # 只保留最新配对，不排队发送旧图像
            self.latest = (
                image_msg, proprio, stamp_gap, time.monotonic()
            )

    def run(self):
        rospy.loginfo("等待相机和里程计配对，接口：%s", self.url)

        while not rospy.is_shutdown():
            with self.lock:
                sample = self.latest
                self.latest = None

            if sample is None:
                rospy.sleep(0.02)
                continue

            image_msg, proprio, stamp_gap, received_at = sample

            # 传感器停止更新后，不发送缓存中的旧帧
            if time.monotonic() - received_at > 1.0:
                continue

            payload = {
                "image": base64.b64encode(
                    bytes(image_msg.data)
                ).decode("ascii"),
                "instr": self.instruction,
                "proprio": proprio,
            }

            rospy.loginfo(
                "发送：proprio=%s，图像/位姿时间差=%.3f秒",
                [round(v, 3) for v in proprio],
                stamp_gap,
            )

            start = time.monotonic()
            try:
                response = self.session.post(
                    self.url,
                    json=payload,
                    timeout=(30, 120),
                )
                response.raise_for_status()
                result = response.json()

                rospy.loginfo(
                    "图像时间=%.3f，总耗时=%.2f秒，返回=%s",
                    image_msg.header.stamp.to_sec(),
                    time.monotonic() - start,
                    result,
                )
            except (requests.RequestException, ValueError) as exc:
                rospy.logwarn("请求失败：%s", exc)
                rospy.sleep(1.0)


if __name__ == "__main__":
    rospy.init_node("camera_to_openvla")
    CameraSender().run()
