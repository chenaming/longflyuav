#!/usr/bin/env python3
"""Bounded real-topic commissioning. Requires the explicit disarmed FSM mode.

Never calls arming or offboard_command. Restores the original FCU mode.
These diagnostic targets are not model predictions.
"""
import json
import math
import threading
import time
from pathlib import Path

import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from uav_control_fsm.msg import uav_command


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


class Bench:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = None
        self.odom = None
        self.setpoints = []
        self.commands = []
        self.armed_ever = False
        self.pub = rospy.Publisher('/uav1/move_command', uav_command, queue_size=1)
        self.report_pub = rospy.Publisher('/vla/bench_report', String, queue_size=1, latch=True)
        rospy.Subscriber('/uav1/mavros/state', State, self.on_state)
        rospy.Subscriber('/uav1/mavros/local_position/odom', Odometry, self.on_odom)
        rospy.Subscriber('/uav1/mavros/setpoint_raw/local', PositionTarget, self.on_setpoint)
        rospy.Subscriber('/uav1/move_command', uav_command, self.on_command)
        rospy.wait_for_service('/uav1/mavros/set_mode', timeout=10)
        self.mode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        self.report = {'kind': 'disarmed_diagnostic_targets', 'status': 'running',
                       'model_predictions_used': False, 'cases': [], 'armed_ever': False,
                       'run_id': rospy.get_param('/vla_direct_bridge/run_id', ''),
                       'instruction': rospy.get_param('/vla_direct_bridge/active_instruction', '')}
        self.path = Path(rospy.get_param('~report_path', '/home/nvidia/.local/state/vla-commissioning/bench_report.json'))

    def on_state(self, m):
        with self.lock:
            self.state = (m, time.monotonic())
            self.armed_ever |= m.armed

    def on_odom(self, m):
        with self.lock:
            self.odom = (m, time.monotonic())

    def on_setpoint(self, m):
        with self.lock:
            self.setpoints.append((time.monotonic(), m))
            self.setpoints = self.setpoints[-500:]

    def on_command(self, m):
        with self.lock:
            self.commands.append((time.monotonic(), m))
            self.commands = self.commands[-500:]

    def check(self):
        with self.lock:
            now = time.monotonic()
            if self.armed_ever or not self.state or now - self.state[1] > 1.5:
                raise RuntimeError('armed observed or stale flight state')
            if not self.state[0].connected or self.state[0].armed:
                raise RuntimeError('FCU must be connected and disarmed')
            if not self.odom or now - self.odom[1] > .5:
                raise RuntimeError('fresh odometry required')
            return self.state[0], self.odom[0]

    def persist(self):
        self.report['armed_ever'] = self.armed_ever
        self.report['updated_unix'] = time.time()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2))
        self.report_pub.publish(String(data=json.dumps(self.report, ensure_ascii=False)))

    def hold(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.check()
            time.sleep(.04)

    def command(self, dx=0., dy=0., dz=0., yaw_deg=0., frame='uav1_map', stale=False):
        _, odom = self.check()
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        c = uav_command()
        c.header.stamp = rospy.Time.now() - rospy.Duration(2. if stale else 0.)
        c.header.frame_id = frame
        c.Move_mode = 0
        c.pos_ref.x, c.pos_ref.y, c.pos_ref.z = p.x + dx, p.y + dy, p.z + dz
        c.yaw_ref = math.degrees(yaw) + yaw_deg
        return c

    def case(self, name, accepted=True, **kw):
        start = time.monotonic()
        sent = self.command(**kw)
        for _ in range(18):
            state, _ = self.check()
            if state.mode != 'OFFBOARD':
                raise RuntimeError('FCU left OFFBOARD')
            sent.header.stamp = rospy.Time.now() - rospy.Duration(2. if kw.get('stale') else 0.)
            self.pub.publish(sent)
            time.sleep(.04)
        with self.lock:
            samples = [m for t, m in self.setpoints if t > start + .12]
            commands_seen = len([1 for t, m in self.commands if t > start])
        if not samples or commands_seen < 5:
            raise RuntimeError('missing actual command or setpoint messages')
        _, odom = self.check()
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        expected = [sent.pos_ref.x, sent.pos_ref.y, sent.pos_ref.z] if accepted else [p.x, p.y, p.z]
        expected_yaw = math.radians(sent.yaw_ref) if accepted else euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        last = samples[-1]
        got = [last.position.x, last.position.y, last.position.z]
        error = max(abs(a-b) for a,b in zip(expected,got))
        yaw_error = abs(wrap(last.yaw - expected_yaw))
        passed = error < .012 and yaw_error < math.radians(1.5) and last.coordinate_frame == 1
        result = {'name': name, 'expected_acceptance': accepted, 'passed': passed,
                  'command_count': commands_seen, 'setpoint_count': len(samples),
                  'expected_xyz_m': expected, 'actual_xyz_m': got,
                  'position_max_error_m': error, 'yaw_error_deg': math.degrees(yaw_error),
                  'setpoint_source': last._connection_header.get('callerid'),
                  'coordinate_frame': last.coordinate_frame, 'type_mask': last.type_mask}
        self.report['cases'].append(result)
        self.persist()
        if not passed:
            raise RuntimeError('case failed: ' + name)
        # Discontinue commands long enough to exercise stale-command hold.
        self.hold(.7)

    def run(self):
        if rospy.get_param('/uav1/uav_control_fsm/bench_disarmed', False) is not True:
            raise RuntimeError('explicit FSM bench_disarmed protection is required')
        self.hold(2.5)
        state, _ = self.check()
        original = state.mode
        self.report['original_mode'] = original
        self.persist()
        try:
            if original != 'OFFBOARD':
                response = self.mode(base_mode=0, custom_mode='OFFBOARD')
                self.report['offboard_mode_sent'] = response.mode_sent
            deadline = time.monotonic() + 8
            while self.check()[0].mode != 'OFFBOARD':
                if time.monotonic() > deadline:
                    raise RuntimeError('OFFBOARD was not accepted')
                time.sleep(.1)
            for name, kw in (
                ('world_X_plus_5cm', {'dx': .05}),
                ('world_Y_plus_5cm', {'dy': .05}),
                ('world_Z_plus_1cm', {'dz': .01}),
                ('yaw_plus_3deg', {'yaw_deg': 3.}),
                ('reject_wrong_frame', {'frame': 'wrong', 'dx': .05, 'accepted': False}),
                ('reject_stale_stamp', {'stale': True, 'dx': .05, 'accepted': False}),
                ('reject_large_XY', {'dx': .3, 'accepted': False}),
                ('reject_large_Z', {'dz': .1, 'accepted': False}),
                ('reject_large_yaw', {'yaw_deg': 15., 'accepted': False}),
            ):
                self.case(name, **kw)
            self.report['status'] = 'passed'
        except Exception as exc:
            self.report['status'] = 'failed'
            self.report['error'] = str(exc)
        finally:
            # Stop diagnostics, allow command timeout to hold the current pose.
            time.sleep(.7)
            if original != 'OFFBOARD':
                try:
                    self.mode(base_mode=0, custom_mode=original)
                    deadline = time.monotonic()+5
                    while self.check()[0].mode != original and time.monotonic()<deadline:
                        time.sleep(.1)
                    self.report['restored_mode'] = self.check()[0].mode
                    if self.report['restored_mode'] != original:
                        raise RuntimeError('original mode not restored')
                except Exception as exc:
                    self.report['status'] = 'failed'
                    self.report['restore_error'] = str(exc)
            self.persist()
        print(json.dumps(self.report, ensure_ascii=False, indent=2), flush=True)
        return self.report['status'] == 'passed'


if __name__ == '__main__':
    rospy.init_node('vla_bench_validation')
    bench = Bench()
    # Give subscriptions time to receive the initial state and odometry.
    time.sleep(1)
    ok = bench.run()
    # Keep the finite test's evidence latched for the monitor; no commands sent.
    if ok:
        rospy.spin()
    else:
        raise SystemExit(1)
