#!/usr/bin/env python3
"""Trigger one inference; print its result (arrival is reported on ~status)."""
import argparse
import json
import threading
import time
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable", action="store_true", help="enable live output before stepping; requires armed OFFBOARD")
    parser.add_argument("--stop", action="store_true", help="cancel and stop this bridge's current goal")
    a = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("openvla_step_client", anonymous=True)
    prefix = "/openvla_ego_bridge"
    rospy.wait_for_service(prefix + "/step", timeout=5)
    enable = rospy.ServiceProxy(prefix + "/enable", SetBool)
    if a.stop:
        print(enable(False).message)
        return
    event = threading.Event()
    output = []
    active = [False]
    def result(msg):
        if active[0]:
            output.append(json.loads(msg.data))
            event.set()
    def status(msg):
        data = json.loads(msg.data)
        if active[0] and data["status"] in ("step_error", "disabled"):
            output.append(data)
            event.set()
    rospy.Subscriber(prefix + "/result", String, result, queue_size=5)
    rospy.Subscriber(prefix + "/status", String, status, queue_size=5)
    time.sleep(.3)
    if a.enable:
        response = enable(True)
        if not response.success:
            raise RuntimeError(response.message)
    active[0] = True
    response = rospy.ServiceProxy(prefix + "/step", Trigger)()
    if not response.success:
        raise RuntimeError(response.message)
    if not event.wait(20):
        raise RuntimeError("no inference result in 20s; inspect bridge status/log")
    print(json.dumps(output[-1], indent=2, ensure_ascii=False))
    print("Arrival/hover status: rostopic echo /openvla_ego_bridge/status")


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as exc:
        raise SystemExit(str(exc))
