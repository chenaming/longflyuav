#!/usr/bin/env python3
"""Real model outputs to disarmed controller, with strict bench-only bounds.

This validates message propagation; it cannot validate flight feedback on a fixed UAV.
No arming services or normal FSM offboard_command are used.
"""
import json
import math
import time
from pathlib import Path

import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from bench_validate import Bench, wrap


class ModelBench(Bench):
    def __init__(self):
        super().__init__()
        self.report_pub.unregister()
        self.report_pub = rospy.Publisher('/vla/model_bench_report', String, queue_size=1, latch=True)
        self.path = Path('/home/nvidia/.local/state/vla-commissioning/chair_round/model_bench_report.json')
        self.report.update(kind='disarmed_bounded_model_targets', model_predictions_used=True,
                           physical_navigation_verified=False,
                           limits={'xy_m': .05, 'z_m': .01, 'yaw_deg': 3.})
        self.prediction = None
        self.last_used = None
        rospy.Subscriber('/vla/validated_prediction', String, self.on_prediction, queue_size=1)

    def on_prediction(self, msg):
        try:
            p = json.loads(msg.data)
            if p.get('run_id') != self.report['run_id'] or p.get('instruction') != self.report['instruction']:
                return
            if p.get('target_frame') != 'map':
                return
            v = p['target_xyz_m'] + [p['target_yaw_rad'], p['observation_stamp'], p['validated_stamp']]
            if len(v) != 6 or not all(math.isfinite(x) for x in v):
                return
            with self.lock:
                self.prediction = p
        except (ValueError, TypeError, KeyError):
            pass

    def next_prediction(self):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            self.check()
            if rospy.get_param('/vla_direct_bridge/active_instruction') != self.report['instruction']:
                raise RuntimeError('instruction changed')
            with self.lock:
                p = self.prediction
                if p and p['request_id'] != self.last_used:
                    now = rospy.Time.now().to_sec()
                    if 0 <= now-p['observation_stamp'] <= 2 and 0 <= now-p['validated_stamp'] <= 1:
                        self.last_used = p['request_id']
                        return dict(p)
            time.sleep(.04)
        raise RuntimeError('no fresh validated model prediction within 20s')

    def run(self):
        assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed', False) is True
        assert rospy.get_param('/vla_direct_bridge/enable_publish', False) is False
        self.hold(2.5)
        original = self.check()[0].mode
        self.report['original_mode'] = original
        self.persist()
        try:
            if original != 'OFFBOARD':
                self.report['offboard_mode_sent'] = self.mode(base_mode=0, custom_mode='OFFBOARD').mode_sent
            end = time.monotonic()+8
            while self.check()[0].mode != 'OFFBOARD':
                if time.monotonic()>end:
                    raise RuntimeError('OFFBOARD not accepted')
                time.sleep(.1)
            for index in range(8):
                p = self.next_prediction()
                _, odom = self.check()
                pos, q = odom.pose.pose.position, odom.pose.pose.orientation
                yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
                raw = [a-b for a,b in zip(p['target_xyz_m'], [pos.x,pos.y,pos.z])]
                scale = min(1., .05/max(math.hypot(raw[0],raw[1]), 1e-9))
                step = [raw[0]*scale, raw[1]*scale, max(-.01,min(.01,raw[2]))]
                dyaw = max(-3.,min(3.,math.degrees(wrap(p['target_yaw_rad']-yaw))))
                self.case('model_request_%s' % p['request_id'], dx=step[0], dy=step[1], dz=step[2], yaw_deg=dyaw)
                self.report['cases'][-1].update(prediction=p, raw_model_delta_xyz_m=raw,
                    bounded_delta_xyz_m=step, bounded_delta_yaw_deg=dyaw,
                    model_target_was_limited=math.hypot(raw[0],raw[1])>.05 or abs(raw[2])>.01 or abs(math.degrees(wrap(p['target_yaw_rad']-yaw)))>3,
                    command_source='/vla_model_bench_validation')
                self.persist()
            self.report['status'] = 'passed'
        except Exception as exc:
            self.report.update(status='failed',error=str(exc))
        finally:
            time.sleep(.7)
            if original != 'OFFBOARD':
                try:
                    self.mode(base_mode=0, custom_mode=original)
                    end=time.monotonic()+5
                    while self.check()[0].mode != original and time.monotonic()<end:
                        time.sleep(.1)
                    self.report['restored_mode']=self.check()[0].mode
                    if self.report['restored_mode'] != original:
                        raise RuntimeError('original mode not restored')
                except Exception as exc:
                    self.report.update(status='failed',restore_error=str(exc))
            self.persist()
        print(json.dumps(self.report, ensure_ascii=False, indent=2), flush=True)
        return self.report['status']=='passed'


if __name__=='__main__':
    rospy.init_node('vla_model_bench_validation')
    bench=ModelBench()
    time.sleep(1)
    if bench.run():
        rospy.spin()  # Keeps evidence only; sends no further commands.
    else:
        raise SystemExit(1)
