#!/usr/bin/env python3
"""Record a VLA demo bag (plan 4.3). Never enables FCU publish."""

from __future__ import print_function

import json
import os
import re
import signal
import subprocess
from datetime import datetime

import rospy
from std_msgs.msg import String

try:
    import rospkg
    import yaml
except ImportError:
    rospkg = None
    yaml = None


DEFAULT_TOPICS = [
    "/vla/instruction",
    "/uav1/odin1/image/compressed",
    "/uav1/odin1/odometry",
    "/uav1/odin1/odometry_highfreq",
    "/uav1/odin1/imu",
    "/uav1/mavros/local_position/odom",
    "/uav1/mavros/local_position/pose",
    "/uav1/mavros/state",
    "/uav1/mavros/rc/in",
    "/uav1/move_command",
    "/uav1/mavros/setpoint_raw/local",
    "/tf",
    "/tf_static",
]

DEFAULT_FORBID = ("cloud", "points", "undistorted", "render")


def _sanitize(text, fallback="episode"):
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", (text or "").strip())
    cleaned = cleaned.strip("_")
    return cleaned or fallback


def _load_topic_config():
    topics = list(DEFAULT_TOPICS)
    forbid = list(DEFAULT_FORBID)
    if rospkg is None or yaml is None:
        return topics, forbid
    try:
        pkg = rospkg.RosPack().get_path("vla_bridge")
        path = os.path.join(pkg, "config", "vla_record_topics.yaml")
        with open(path, "r") as handle:
            data = yaml.safe_load(handle) or {}
        loaded = data.get("topics") or topics
        forbid = data.get("forbid_substrings") or forbid
        topics = [str(item) for item in loaded]
    except Exception as exc:
        rospy.logwarn("using built-in record topic list: %s", exc)
    return topics, [str(item).lower() for item in forbid]


def _filter_topics(topics, forbid):
    kept = []
    for topic in topics:
        lower = topic.lower()
        blocked = [token for token in forbid if token in lower]
        if blocked:
            rospy.logwarn("skip record topic %s (forbid=%s)", topic, blocked)
            continue
        kept.append(topic)
    return kept


def _write_json(path, payload):
    tmp = path + ".tmp"
    with open(tmp, "w") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    os.replace(tmp, path)


class VlaRecordSession(object):
    def __init__(self):
        if bool(rospy.get_param("/control/enable_publish", False)):
            raise rospy.ROSException("vla_record_session refuses to run with enable_publish=true")

        instruction = rospy.get_param("~instruction", "move forward").strip()
        if not instruction:
            raise rospy.ROSException("~instruction must be a non-empty English command")
        instruction_zh = rospy.get_param("~instruction_zh", "").strip()
        instr_id = _sanitize(rospy.get_param("~instr_id", "episode"))
        bag_dir = os.path.expanduser(rospy.get_param("~bag_dir", os.path.join(os.path.expanduser("~"), "vla_bags")))
        site = rospy.get_param("~site", "unknown")
        lighting = rospy.get_param("~lighting", "unknown")
        pilot = rospy.get_param("~pilot", "unknown")
        notes = rospy.get_param("~notes", "")

        topics, forbid = _load_topic_config()
        extra = rospy.get_param("~extra_topics", [])
        if extra:
            topics.extend([str(item) for item in extra])
        self._topics = _filter_topics(topics, forbid)
        if "/vla/instruction" not in self._topics:
            self._topics.insert(0, "/vla/instruction")
        if "/uav1/odin1/image/compressed" not in self._topics:
            raise rospy.ROSException("record list must include /uav1/odin1/image/compressed")

        os.makedirs(bag_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        episode_id = "%s_%s" % (stamp, instr_id)
        self._prefix = os.path.join(bag_dir, episode_id)
        self._bag_path = self._prefix + ".bag"
        self._meta_path = self._prefix + ".meta.json"

        self._meta = {
            "id": episode_id,
            "instruction": instruction,
            "instruction_zh": instruction_zh,
            "instruction_unified": instruction,
            "site": site,
            "lighting": lighting,
            "pilot": pilot,
            "success": None,
            "safety_events": [],
            "enable_publish": False,
            "bag": self._bag_path,
            "topics": list(self._topics),
            "notes": notes,
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "finished_at": None,
        }
        _write_json(self._meta_path, self._meta)

        self._instr_pub = rospy.Publisher("/vla/instruction", String, queue_size=1, latch=True)
        rospy.sleep(0.2)
        self._instr_pub.publish(String(data=instruction))

        cmd = ["rosbag", "record", "-O", self._prefix] + self._topics
        rospy.loginfo("recording %s topics=%s", self._bag_path, self._topics)
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        rospy.on_shutdown(self._stop)

    def _stop(self):
        proc = getattr(self, "_proc", None)
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except OSError:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except Exception:
                proc.terminate()
        self._meta["finished_at"] = datetime.now().isoformat(timespec="seconds")
        try:
            _write_json(self._meta_path, self._meta)
        except Exception as exc:
            rospy.logwarn("failed to update meta: %s", exc)
        rospy.loginfo("record session stopped bag=%s meta=%s", self._bag_path, self._meta_path)


def main():
    rospy.init_node("vla_record_session")
    VlaRecordSession()
    rospy.spin()


if __name__ == "__main__":
    main()
