import base64
import threading

import requests
import rospy
from sensor_msgs.msg import CompressedImage

lock = threading.Lock()
latest = None


def camera_callback(msg):
    global latest
    with lock:
        latest = msg


def main():
    global latest

    rospy.init_node("camera_to_openvla")

    # 启动时通过 _url 参数填写主机地址
    url = rospy.get_param("~url")
    instruction = "Move forward and then return to the starting point"

    rospy.Subscriber(
        "/uav1/odin1/image/compressed",
        CompressedImage,
        camera_callback,
        queue_size=1,
        buff_size=8 * 1024 * 1024,
    )

    session = requests.Session()
    session.trust_env = False  # 直连主机，避免环境代理影响
    rospy.loginfo("等待相机图像，推理接口：%s", url)

    while not rospy.is_shutdown():
        with lock:
            msg = latest
            latest = None

        if msg is None:
            rospy.sleep(0.02)
            continue

        payload = {
            "image": base64.b64encode(bytes(msg.data)).decode("ascii"),
            "instr": instruction,
            # 仅用于联通测试，不能作为真实位姿使用
            "proprio": [0.0, 0.0, 0.0, 0.0],
        }

        try:
            rospy.loginfo("开始发送图像：%d 字节", len(msg.data))
            response = session.post(url, json=payload, timeout=(30, 120))
            response.raise_for_status()
            result = response.json()

            rospy.loginfo(
                "图像时间=%.3f，返回=%s",
                msg.header.stamp.to_sec(),
                result,
            )
        except (requests.RequestException, ValueError) as exc:
            rospy.logwarn("请求失败：%s", exc)
            rospy.sleep(1.0)


if __name__ == "__main__":
    main()
