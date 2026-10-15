#!/usr/bin/env python3
"""Fake sensor feed for vla_bridge without Odin.

source_mode=odin (default): publish /uav1/odin1 compressed JPEG + odometry.
source_mode=processed: bypass image node and publish /vla/image_for_infer + /vla/proprio.
"""

import math

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String


def _color_test_jpeg(width, height, quality):
    """Non-square RGB pattern: red left, green right, blue bottom, white center."""
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    mid = width // 2
    rgb[:, :mid] = (255, 0, 0)
    rgb[:, mid:] = (0, 255, 0)
    band = max(8, height // 6)
    rgb[height - band :, :] = (0, 0, 255)
    side = max(8, min(height, width) // 5)
    y0 = (height - side) // 2
    x0 = (width - side) // 2
    rgb[y0 : y0 + side, x0 : x0 + side] = (255, 255, 255)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("failed to encode fake JPEG")
    return buf.tobytes()


def _yaw_to_quat(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main():
    rospy.init_node("vla_fake_observation")
    source_mode = rospy.get_param("~source_mode", "odin")
    rate_hz = float(rospy.get_param("~rate_hz", 10.0 if source_mode == "odin" else 2.0))
    instruction = rospy.get_param("~instruction", "move forward")
    proprio = rospy.get_param("~proprio", [0.0, 0.0, 0.0, 0.0])
    width = int(rospy.get_param("~width", 320))
    height = int(rospy.get_param("~height", 240))
    quality = int(rospy.get_param("~jpeg_quality", 80))
    start_pos = rospy.get_param("~start_pos", [1.0, 2.0, 1.2])
    vx = float(rospy.get_param("~vx", 0.25))
    yaw_rate_deg = float(rospy.get_param("~yaw_rate_deg", 10.0))

    instr_pub = rospy.Publisher("/vla/instruction", String, queue_size=1, latch=True)
    instr_pub.publish(String(data=instruction))

    jpeg = _color_test_jpeg(width, height, quality)
    rate = rospy.Rate(rate_hz)
    start = rospy.Time.now()
    seq = 0

    if source_mode == "processed":
        image_pub = rospy.Publisher("/vla/image_for_infer", CompressedImage, queue_size=1)
        proprio_pub = rospy.Publisher("/vla/proprio", Float32MultiArray, queue_size=1)
        rospy.loginfo(
            "publishing processed fake obs at %.1f Hz jpeg=%dB instr=%r",
            rate_hz,
            len(jpeg),
            instruction,
        )
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            img = CompressedImage()
            img.header.stamp = now
            img.header.seq = seq
            img.header.frame_id = "fake_camera"
            img.format = "jpeg"
            img.data = jpeg
            image_pub.publish(img)

            arr = Float32MultiArray()
            arr.layout.dim = [MultiArrayDimension(label="proprio", size=4, stride=4)]
            arr.data = [float(x) for x in proprio]
            proprio_pub.publish(arr)
            seq += 1
            rate.sleep()
        return

    image_topic = rospy.get_param("~image_topic", rospy.get_param("/image/source", "/uav1/odin1/image/compressed"))
    odom_topic = rospy.get_param("~odom_topic", rospy.get_param("/image/odometry", "/uav1/odin1/odometry"))
    image_pub = rospy.Publisher(image_topic, CompressedImage, queue_size=1)
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    rospy.loginfo(
        "publishing fake Odin at %.1f Hz image=%s odom=%s jpeg=%dx%d/%dB instr=%r",
        rate_hz,
        image_topic,
        odom_topic,
        width,
        height,
        len(jpeg),
        instruction,
    )

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = (now - start).to_sec()
        if t < 0.0:
            t = 0.0

        img = CompressedImage()
        img.header.stamp = now
        img.header.seq = seq
        img.header.frame_id = "odin1_optical_frame"
        img.format = "jpeg"
        img.data = jpeg
        image_pub.publish(img)

        yaw = math.radians(yaw_rate_deg * t)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.seq = seq
        odom.header.frame_id = "odom"
        odom.child_frame_id = "odin1_base_link"
        odom.pose.pose.position.x = float(start_pos[0]) + vx * t
        odom.pose.pose.position.y = float(start_pos[1])
        odom.pose.pose.position.z = float(start_pos[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom_pub.publish(odom)

        seq += 1
        rate.sleep()


if __name__ == "__main__":
    main()
