"""Run on ROS-equipped NX. All mode/arming endpoints below are Python fakes."""
import threading
import time
import unittest
from types import SimpleNamespace as NS
from closed_loop_flight import Runtime
from mavros_msgs.msg import PositionTarget
import rospy


class AdapterTests(unittest.TestCase):
    def runtime(self):
        r=Runtime.__new__(Runtime)
        r.lock=threading.RLock();r.closed=False;r.rpc_busy=False;r.rpc_finished=0.;r.rpc_log=[]
        r.stop=False;r.mission=NS(phase='ARM');r.last_publish=time.monotonic();r.publish_count=60
        return r

    def telemetry_data(self,throttle=1500,ch6=2000,ch9=999):
        stamp=rospy.Time.now();now=time.monotonic()
        rc=[1500]*16;rc[2]=throttle;rc[5]=ch6;rc[8]=ch9
        vector=NS(x=0.,y=0.,z=0.)
        odom=NS(header=NS(stamp=stamp,frame_id='map'),
            pose=NS(pose=NS(position=vector,orientation=NS(x=0,y=0,z=0,w=1))),
            twist=NS(twist=NS(linear=vector,angular=vector)))
        return {'state':(NS(connected=True,armed=True,mode='OFFBOARD'),now),
                'odom':(odom,now),'extended':(NS(landed_state=2),now),
                'rc':(NS(channels=rc,rssi=255),now)}

    def test_throttle_and_raw_switch_changes_do_not_request_takeover(self):
        r=self.runtime();r.rc_reference=[999]*16
        for throttle in [999,1237,1500,2000]:
            self.assertIsNone(r.telemetry(self.telemetry_data(throttle))[4])

    def test_rc_signal_loss_is_still_a_fault(self):
        r=self.runtime();r.rc_reference=[999]*16
        d=self.telemetry_data();d['rc']=(d['rc'][0],time.monotonic()-2)
        self.assertEqual(r.telemetry(d)[4],'RC signal unavailable')

    def test_default_run_is_read_only(self):
        r=self.runtime();r.args=NS(execute=False);r.preflight=lambda:[0,0,0,0]
        r.claim=lambda:self.fail('read-only acquired flight ownership')
        r.submit=lambda _:self.fail('read-only submitted RPC')
        r.run()

    def test_stream_preserves_yaw_and_masks_yaw_rate(self):
        r=self.runtime();messages=[]
        r.sp=[.1,.2,1.,.3];r.policy_at=time.monotonic()
        r.publisher=NS(publish=lambda m:messages.append(m))
        thread=threading.Thread(target=r.stream);thread.start();time.sleep(.15);r.closed=True;thread.join(1)
        self.assertGreaterEqual(len(messages),3)
        m=messages[-1]
        self.assertEqual((m.position.x,m.position.y,m.position.z),(.1,.2,1.))
        self.assertEqual(m.yaw,.3)
        self.assertFalse(m.type_mask & PositionTarget.IGNORE_YAW)
        self.assertTrue(m.type_mask & PositionTarget.IGNORE_YAW_RATE)

    def test_policy_freeze_stops_old_setpoint_stream(self):
        r=self.runtime();messages=[];r.sp=[0,0,1,0];r.policy_at=time.monotonic()-1
        r.publisher=NS(publish=lambda m:messages.append(m))
        thread=threading.Thread(target=r.stream);thread.start();time.sleep(.12);r.closed=True;thread.join(1)
        self.assertEqual(messages,[])

    def test_stop_blocks_pending_arm_submission(self):
        r=self.runtime();r.stop=True
        r.arm=lambda **_:self.fail('arm after stop')
        r.submit(('arm',True));self.assertFalse(r.rpc_busy)

    def test_inflight_rpc_does_not_queue_more_requests(self):
        r=self.runtime();event=threading.Event();calls=[]
        def fake_arm(**kw):calls.append(kw);event.wait(.5);return NS(success=True)
        r.arm=fake_arm;r.submit(('arm',True));r.submit(('arm',True))
        event.set()
        end=time.monotonic()+1
        while r.rpc_busy and time.monotonic()<end:time.sleep(.01)
        self.assertEqual(calls,[{'value':True}])


if __name__=='__main__':
    rospy.init_node('test_flight_adapter_no_fcu_commands',anonymous=True,disable_signals=True)
    unittest.main(verbosity=2)
