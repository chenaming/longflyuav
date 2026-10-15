#!/usr/bin/env python3
"""Real ROS + real GPU, simulated FCU/odometry on a separate ROS master.

Never connects to the hardware ROS master. No actuator interfaces exist here.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import xmlrpc.client


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--image", required=True)
    p.add_argument("--url", default="http://127.0.0.1:15008/predict")
    p.add_argument("--report", required=True)
    a = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    uri = "http://127.0.0.1:11421"
    env = dict(os.environ, ROS_MASTER_URI=uri, ROS_HOSTNAME="127.0.0.1")
    env.pop("ROS_IP", None)
    # Refuse to reuse a possibly non-test master.
    import socket
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 11421))
    master_log = open(str(a.report) + ".master.log", "w")
    node_log = open(str(a.report) + ".node.log", "w")
    master = subprocess.Popen(["roscore", "-p", "11421"], env=env, stdout=master_log, stderr=subprocess.STDOUT)
    node = None
    quit_event = threading.Event()
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(uri).getPid("/bench")[0] == 1:
                    break
            except OSError:
                time.sleep(.1)
        else:
            raise RuntimeError("isolated master did not start")
        os.environ.update(ROS_MASTER_URI=uri, ROS_HOSTNAME="127.0.0.1")
        os.environ.pop("ROS_IP", None)
        import rospy
        from sensor_msgs.msg import CompressedImage
        from nav_msgs.msg import Odometry
        from mavros_msgs.msg import State
        from quadrotor_msgs.msg import GoalSet
        from std_msgs.msg import Empty, Bool, String
        from std_srvs.srv import Trigger, SetBool
        rospy.init_node("uav1_planner_node", disable_signals=True)
        rospy.set_param("/uav1_planner_node/grid_map/virtual_ground", -2.)
        rospy.set_param("/uav1_planner_node/grid_map/virtual_ceil", 3.)
        poses, goals, results, hover, stops, statuses = [[10., 20., 1.]], [], [], [], [], []
        lock = threading.Lock()
        image = Path(a.image).read_bytes()
        odom_pub = rospy.Publisher("/uav1/mavros/local_position/odom", Odometry, queue_size=1)
        img_pub = rospy.Publisher("/uav1/odin1/image/compressed", CompressedImage, queue_size=1)
        state_pub = rospy.Publisher("/uav1/mavros/state", State, queue_size=1)
        rospy.Subscriber("/uav1/goal_point", GoalSet, lambda msg: goals.append(list(msg.goal)))
        rospy.Subscriber("/uav1/hover_command", Bool, lambda msg: hover.append(msg.data))
        rospy.Subscriber("/uav1/mandatory_stop_to_planner", Empty, lambda msg: stops.append(True))
        rospy.Subscriber("/openvla_ego_bridge/result", String, lambda msg: results.append(json.loads(msg.data)))
        rospy.Subscriber("/openvla_ego_bridge/status", String, lambda msg: statuses.append(json.loads(msg.data)))

        def sensors():
            while not quit_event.wait(.05):
                msg = Odometry()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id, msg.child_frame_id = "map", "base_link"
                with lock:
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = poses[0]
                msg.pose.pose.orientation.z = math.sin(math.pi/4)
                msg.pose.pose.orientation.w = math.cos(math.pi/4)
                odom_pub.publish(msg)
                rgb = CompressedImage()
                rgb.header.stamp, rgb.format, rgb.data = msg.header.stamp, "jpeg", image
                img_pub.publish(rgb)
                state_pub.publish(State(connected=True, armed=True, mode="OFFBOARD"))

        threading.Thread(target=sensors, daemon=True).start()
        node = subprocess.Popen([
            sys.executable, str(root / "agile_ws/src/agile_openvla_bridge/scripts/openvla_ego_bridge.py"),
            "_server_url:=" + a.url, "_dry_run:=false", "_enabled:=false",
            "_camera_alignment_confirmed:=true", "_model_convention_confirmed:=true",
            "_instruction:=fly forward", "_record_dir:=" + str(Path(a.report).parent / "bench_observations"),
        ], env=env, stdout=node_log, stderr=subprocess.STDOUT)
        rospy.wait_for_service("/openvla_ego_bridge/enable", timeout=15)
        step = rospy.ServiceProxy("/openvla_ego_bridge/step", Trigger)
        enable = rospy.ServiceProxy("/openvla_ego_bridge/enable", SetBool)
        time.sleep(1)
        assert enable(True).success
        for index in range(2):
            assert step().success
            assert not step().success, "duplicate step was accepted"
            deadline = time.monotonic() + 15
            while len(goals) <= index and time.monotonic() < deadline:
                time.sleep(.05)
            assert len(goals) == index + 1, statuses[-5:]
            goal = goals[index]
            assert goal[1] > poses[0][1], "forward at 90 degrees must move in map +Y"
            assert not step().success, "step during motion was accepted"
            with lock:
                poses[0] = goal
            began = time.monotonic()
            deadline = began + 6
            while len(hover) <= index and time.monotonic() < deadline:
                time.sleep(.05)
            assert len(hover) == index + 1, statuses[-5:]
            assert time.monotonic() - began >= .95, "hover dwell skipped"
            time.sleep(1)
            assert len(goals) == index + 1, "unrequested automatic goal"
        assert results[0]["mission_id"] == results[1]["mission_id"], "origin reset at arrival"
        assert results[1]["proprio_sent"][0] > 0, "new pose not sent on second step"
        assert len(stops) == 2
        report = dict(passed=True, isolated_ros_master=uri,
                      real_gpu=True, real_ros_transport=True, simulated_pose_and_fcu=True,
                      hardware_goal_published=False, actual_flight_tested=False,
                      goals=goals, inference_results=results, hover_count=len(hover),
                      checks=["one goal per trigger", "duplicate/busy trigger rejected",
                              "90-degree heading transform", "1s arrival dwell",
                              "stop+hover on completion", "fixed origin across steps",
                              "next step uses updated pose", "no automatic extra goal"])
        Path(a.report).write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps({k:v for k,v in report.items() if k != "inference_results"}, indent=2))
    finally:
        quit_event.set()
        for process in (node, master):
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
        master_log.close()
        node_log.close()


if __name__ == "__main__":
    main()
