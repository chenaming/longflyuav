#!/usr/bin/env python3
"""Degrade-only watchdog: hover/land and cut enable_control. Never arm, takeoff, or offboard."""

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from vla_bridge.msg import VLAAction, VLAHeartbeat, VLAStatus

try:
    from mavros_msgs.msg import State as MavrosState
except ImportError:
    MavrosState = None


def _is_finite_vec4(values):
    if values is None or len(values) < 4:
        return False
    for item in values[:4]:
        try:
            number = float(item)
        except (TypeError, ValueError):
            return False
        if math.isnan(number) or math.isinf(number):
            return False
    return True


def _is_wait_error(error):
    text = (error or "").lower()
    tokens = (
        "waiting",
        "stale image",
        "stale proprio",
        "infer gated",
        "proprio length",
    )
    return any(token in text for token in tokens)


class VlaWatchdogNode(object):
    def __init__(self):
        self._enable_publish = bool(rospy.get_param("/control/enable_publish", False))
        self._backend = rospy.get_param("/control/backend", "direct_fsm")
        self._hb_timeout = float(rospy.get_param("/watchdog/heartbeat_timeout_sec", 2.0))
        self._fail_to_hover = int(rospy.get_param("/watchdog/fail_to_hover_count", 3))
        self._fail_to_land = float(rospy.get_param("/watchdog/fail_to_land_sec", 8.0))
        self._image_timeout = float(rospy.get_param("/watchdog/image_timeout_sec", 0.5))
        self._odom_timeout = float(rospy.get_param("/watchdog/odom_timeout_sec", 0.5))
        self._disagree_m = float(rospy.get_param("/frames/odom_disagree_m", 0.3))
        self._auto_land = bool(rospy.get_param("/watchdog/enable_auto_land", True))
        image_topic = rospy.get_param("/image/source", "/uav1/odin1/image/compressed")
        model_odom_topic = rospy.get_param(
            "/frames/pose_for_model",
            rospy.get_param("/image/odometry", "/uav1/odin1/odometry"),
        )
        control_odom_topic = rospy.get_param("/frames/pose_for_control", "/uav1/mavros/local_position/odom")
        hover_topic = rospy.get_param("/watchdog/hover_topic", "/uav1/hover_command")
        land_topic = rospy.get_param("/watchdog/land_topic", "/uav1/land_command")

        self._lock = threading.Lock()
        self._operator_enable = False
        self._estop = False
        self._nan_latched = False
        self._ever_infer_ok = False
        self._ever_heartbeat = False
        self._hover_sent = False
        self._land_sent = False
        self._fail_count = 0
        self._fail_since = None
        self._last_hb_recv = 0.0
        self._last_hb_alive = False
        self._last_hb_error = ""
        self._last_hb_id = 0
        self._last_ok_time = 0.0
        self._last_image_recv = 0.0
        self._last_model_odom_recv = 0.0
        self._last_control_odom_recv = 0.0
        self._model_xy = None
        self._control_xy = None
        self._mavros_mode = ""
        self._saw_offboard = False
        self._left_offboard = False
        self._detail = "INIT"
        self._state = "INIT"

        self.status_pub = rospy.Publisher("status", VLAStatus, queue_size=1)
        self.enable_pub = rospy.Publisher("enable_control", Bool, queue_size=1, latch=True)
        self.allow_pub = rospy.Publisher("allow_infer", Bool, queue_size=1, latch=True)
        self.debug_hover_pub = rospy.Publisher("debug/hover_command", Bool, queue_size=1, latch=True)
        self.debug_land_pub = rospy.Publisher("debug/land_command", Bool, queue_size=1, latch=True)
        self._hover_pub = None
        self._land_pub = None
        if self._enable_publish:
            self._hover_pub = rospy.Publisher(hover_topic, Bool, queue_size=1, latch=True)
            self._land_pub = rospy.Publisher(land_topic, Bool, queue_size=1, latch=True)
            rospy.logwarn("watchdog will publish FSM failsafe on %s and %s", hover_topic, land_topic)
        else:
            rospy.loginfo("enable_publish=false: hover/land preview on /vla/debug/* only")

        rospy.Subscriber("heartbeat", VLAHeartbeat, self._on_heartbeat, queue_size=1)
        rospy.Subscriber("emergency_stop", Bool, self._on_estop, queue_size=1)
        rospy.Subscriber("enable_control", Bool, self._on_enable, queue_size=1)
        rospy.Subscriber("action_raw", VLAAction, self._on_action, queue_size=1)
        rospy.Subscriber(image_topic, CompressedImage, self._on_image, queue_size=1)
        rospy.Subscriber(model_odom_topic, Odometry, self._on_model_odom, queue_size=1)
        rospy.Subscriber(control_odom_topic, Odometry, self._on_control_odom, queue_size=1)
        if MavrosState is not None:
            rospy.Subscriber("/uav1/mavros/state", MavrosState, self._on_mavros_state, queue_size=1)
        else:
            rospy.loginfo("mavros_msgs not installed; skipping /uav1/mavros/state")

        self.enable_pub.publish(Bool(data=False))
        self.allow_pub.publish(Bool(data=False))
        self._timer = rospy.Timer(rospy.Duration(0.2), self._on_timer)
        rospy.loginfo(
            "vla_watchdog_node ready hb_timeout=%.1fs fail_hover=%d fail_land=%.1fs auto_land=%s",
            self._hb_timeout,
            self._fail_to_hover,
            self._fail_to_land,
            self._auto_land,
        )

    def _on_enable(self, msg):
        with self._lock:
            self._operator_enable = bool(msg.data)

    def _on_estop(self, msg):
        with self._lock:
            self._estop = bool(msg.data)

    def _on_heartbeat(self, msg):
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._ever_heartbeat = True
            self._last_hb_recv = now
            self._last_hb_alive = bool(msg.alive)
            self._last_hb_error = msg.last_error or ""
            self._last_hb_id = int(msg.request_id)
            if msg.alive:
                self._fail_count = 0
                self._fail_since = None
                self._ever_infer_ok = True
                self._last_ok_time = now
                self._nan_latched = False
            elif not _is_wait_error(msg.last_error):
                self._fail_count += 1
                if self._fail_since is None:
                    self._fail_since = now

    def _on_action(self, msg):
        if _is_finite_vec4(msg.action):
            return
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._nan_latched = True
            if self._fail_since is None:
                self._fail_since = now
        rospy.logwarn_throttle(1.0, "watchdog: NaN/Inf action_raw id=%s", msg.request_id)

    def _on_image(self, _msg):
        with self._lock:
            self._last_image_recv = rospy.Time.now().to_sec()

    def _on_model_odom(self, msg):
        with self._lock:
            self._last_model_odom_recv = rospy.Time.now().to_sec()
            self._model_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            )

    def _on_control_odom(self, msg):
        with self._lock:
            self._last_control_odom_recv = rospy.Time.now().to_sec()
            self._control_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            )

    def _on_mavros_state(self, msg):
        mode = (msg.mode or "").strip()
        with self._lock:
            self._mavros_mode = mode
            if mode == "OFFBOARD":
                self._saw_offboard = True
                self._left_offboard = False
            elif self._saw_offboard and mode and mode != "OFFBOARD":
                self._left_offboard = True

    def _cut_enable(self, reason):
        if self._operator_enable:
            rospy.logwarn("watchdog cut enable_control: %s", reason)
            self.enable_pub.publish(Bool(data=False))

    def _send_hover(self, reason):
        if not self._hover_sent:
            rospy.logwarn("watchdog HOVER: %s", reason)
            self.debug_hover_pub.publish(Bool(data=True))
            if self._hover_pub is not None:
                self._hover_pub.publish(Bool(data=True))
            self._hover_sent = True
        self._cut_enable(reason)

    def _send_land(self, reason):
        if not self._land_sent:
            rospy.logwarn("watchdog LAND: %s", reason)
            self.debug_land_pub.publish(Bool(data=True))
            if self._land_pub is not None:
                self._land_pub.publish(Bool(data=True))
            self._land_sent = True
        self._cut_enable(reason)

    def _fresh(self, last_recv, timeout, now):
        return last_recv > 0.0 and (now - last_recv) <= timeout

    def _on_timer(self, _event):
        now = rospy.Time.now().to_sec()
        with self._lock:
            image_ok = self._fresh(self._last_image_recv, self._image_timeout, now)
            odom_ok = self._fresh(self._last_model_odom_recv, self._odom_timeout, now)
            control_ok = self._fresh(self._last_control_odom_recv, self._odom_timeout, now)
            hb_fresh = self._fresh(self._last_hb_recv, self._hb_timeout, now)
            infer_ok = bool(self._last_hb_alive and hb_fresh)
            disagree = False
            if control_ok and odom_ok and self._control_xy is not None and self._model_xy is not None:
                dx = self._control_xy[0] - self._model_xy[0]
                dy = self._control_xy[1] - self._model_xy[1]
                dz = self._control_xy[2] - self._model_xy[2]
                delta = math.sqrt(dx * dx + dy * dy + dz * dz)
                disagree = delta > self._disagree_m
            estop = self._estop
            nan_latched = self._nan_latched
            fail_count = self._fail_count
            fail_since = self._fail_since
            ever_infer_ok = self._ever_infer_ok
            ever_heartbeat = self._ever_heartbeat
            left_offboard = self._left_offboard
            operator_enable = self._operator_enable
            land_sent = self._land_sent
            last_ok = self._last_ok_time
            last_hb_recv = self._last_hb_recv
            last_error = self._last_hb_error

        sensors_ok = image_ok and odom_ok
        if ever_infer_ok and not sensors_ok and fail_since is None:
            fail_since = now
            with self._lock:
                if self._fail_since is None:
                    self._fail_since = now
        if disagree and fail_since is None:
            fail_since = now
            with self._lock:
                if self._fail_since is None:
                    self._fail_since = now
        if ever_heartbeat and not hb_fresh:
            if fail_since is None:
                fail_since = now
                with self._lock:
                    if self._fail_since is None:
                        self._fail_since = now

        need_hover = False
        reasons = []
        if land_sent:
            reasons.append("already landed")
        if estop:
            need_hover = True
            reasons.append("emergency_stop")
        if nan_latched:
            need_hover = True
            reasons.append("NaN action")
        if disagree:
            need_hover = True
            reasons.append("odom disagree")
        if fail_count >= self._fail_to_hover:
            need_hover = True
            reasons.append("infer fail_count=%d" % fail_count)
        if ever_heartbeat and not hb_fresh:
            need_hover = True
            reasons.append("heartbeat timeout")
        if ever_infer_ok and not sensors_ok:
            need_hover = True
            reasons.append("image/odom stale")

        fail_age = (now - fail_since) if fail_since is not None else -1.0
        need_land = bool(
            self._auto_land
            and not estop
            and fail_since is not None
            and fail_age >= self._fail_to_land
            and (need_hover or land_sent)
        )
        if left_offboard:
            reasons.append("mavros left OFFBOARD")

        detail = "; ".join(reasons) if reasons else (
            last_error if last_error else "ok"
        )

        if land_sent or need_land:
            state = "LAND"
            self._send_hover(detail)
            self._send_land(detail)
            allow_infer = False
        elif need_hover:
            state = "HOVER"
            self._send_hover(detail)
            allow_infer = False
        elif not sensors_ok:
            state = "WAIT_SENSORS"
            detail = "waiting for image/odom"
            allow_infer = False
        elif left_offboard:
            state = "FAILSAFE"
            self._cut_enable(detail)
            allow_infer = False
        elif infer_ok:
            state = "TRACKING"
            allow_infer = True
            with self._lock:
                if operator_enable and not self._estop:
                    self._hover_sent = False
        elif hb_fresh:
            state = "REQUESTING"
            allow_infer = True
        else:
            state = "READY"
            allow_infer = sensors_ok

        if left_offboard or need_hover or need_land or disagree or nan_latched or estop:
            self._cut_enable(detail)

        self.allow_pub.publish(Bool(data=bool(allow_infer)))

        last_infer_age = -1.0
        if last_ok > 0.0:
            last_infer_age = now - last_ok
        elif last_hb_recv > 0.0:
            last_infer_age = now - last_hb_recv

        publish_enabled = bool(
            self._enable_publish
            and operator_enable
            and allow_infer
            and state == "TRACKING"
            and not left_offboard
        )

        msg = VLAStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        msg.image_ok = bool(image_ok)
        msg.odom_ok = bool(odom_ok)
        msg.infer_ok = bool(infer_ok)
        msg.publish_enabled = bool(publish_enabled)
        msg.backend = self._backend
        msg.last_infer_age_sec = float(last_infer_age)
        msg.detail = detail[:200]
        self.status_pub.publish(msg)
        with self._lock:
            self._state = state
            self._detail = detail

        rospy.loginfo_throttle(
            2.0,
            "watchdog state=%s image=%s odom=%s infer=%s enable_pub=%s allow_infer=%s fail=%d detail=%s",
            state,
            image_ok,
            odom_ok,
            infer_ok,
            publish_enabled,
            allow_infer,
            fail_count,
            detail,
        )


def main():
    rospy.init_node("vla_watchdog_node")
    VlaWatchdogNode()
    rospy.spin()


if __name__ == "__main__":
    main()
