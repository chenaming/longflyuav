"""Run on ROS-equipped NX. All mode/arming endpoints below are Python fakes."""
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
from closed_loop_flight import Runtime
from mavros_msgs.msg import PositionTarget
import rospy


class AdapterTests(unittest.TestCase):
    def test_preflight_reacquires_odometry_that_pauses_during_parameter_reads(self):
        r=self.runtime();r.args=NS(allow_paired_estimate=True)
        def refresh():
            r.data=self.telemetry_data();r.data['state'][0].armed=False;r.data['state'][0].mode='MANUAL'
            r.data['state'][0].system_status=3;r.data['extended'][0].landed_state=1
            r.data['prediction']=({'latest_only':True,'coordinate_mode':'paired_estimate'},time.monotonic())
        refresh()
        r.master=NS(getSystemState=lambda:([('/uav1/mavros/setpoint_raw/local',['/uav1/uav_control_fsm'])],[],[]))
        params={'/uav1/uav_control_fsm/bench_disarmed':True,'/uav1/uav_control_fsm/bench_allow_armed':False,
                '/vla_direct_bridge/enable_publish':False,'/uav1/uav_control_fsm/external_setpoint_control_active':False}
        integers={'COM_OF_LOSS_T':0,'COM_OBL_ACT':0,'COM_OBL_RC_ACT':0,'COM_RC_OVERRIDE':1,
                  'RC_MAP_FLTMODE':6,'RC_MAP_ARM_SW':9,'RC_MAP_KILL_SW':10}
        calls=[]
        def get(param_id):
            calls.append(param_id)
            if len(calls)==1:
                odom,_=r.data['odom'];r.data['odom']=(odom,time.monotonic()-.7)
            return NS(success=True,value=NS(integer=integers[param_id],real=1. if param_id=='COM_OF_LOSS_T' else 0.))
        r.arm=lambda **_:self.fail('preflight armed/disarmed FCU')
        r.mode=lambda **_:self.fail('preflight changed mode')
        with patch('closed_loop_flight.rospy.get_param',side_effect=lambda n,d=None:params.get(n,d)),\
             patch('closed_loop_flight.rospy.wait_for_service'),\
             patch('closed_loop_flight.rospy.ServiceProxy',return_value=get),\
             patch('closed_loop_flight.time.sleep',side_effect=lambda _:refresh()):
            r.preflight()
        self.assertEqual(len(calls),7)

    def test_preflight_retries_parameter_cache_not_ready(self):
        r=self.runtime();r.args=NS(allow_paired_estimate=True);r.data=self.telemetry_data()
        r.data['state'][0].armed=False;r.data['state'][0].mode='MANUAL'
        r.data['state'][0].system_status=3;r.data['extended'][0].landed_state=1
        r.data['prediction']=({'latest_only':True,'coordinate_mode':'paired_estimate'},time.monotonic())
        r.master=NS(getSystemState=lambda:([('/uav1/mavros/setpoint_raw/local',['/uav1/uav_control_fsm'])],[],[]))
        params={'/uav1/uav_control_fsm/bench_disarmed':True,'/uav1/uav_control_fsm/bench_allow_armed':False,
                '/vla_direct_bridge/enable_publish':False,'/uav1/uav_control_fsm/external_setpoint_control_active':False}
        integers={'COM_OF_LOSS_T':0,'COM_OBL_ACT':0,'COM_OBL_RC_ACT':0,'COM_RC_OVERRIDE':1,
                  'RC_MAP_FLTMODE':6,'RC_MAP_ARM_SW':9,'RC_MAP_KILL_SW':10}
        calls=[]
        def get(param_id):
            calls.append(param_id)
            return NS(success=not (param_id=='COM_OF_LOSS_T' and calls.count(param_id)==1),
                      value=NS(integer=integers[param_id],real=1. if param_id=='COM_OF_LOSS_T' else 0.))
        r.arm=lambda **_:self.fail('readiness check armed/disarmed FCU')
        r.mode=lambda **_:self.fail('readiness check changed mode')
        with patch('closed_loop_flight.rospy.get_param',side_effect=lambda n,d=None:params.get(n,d)),\
             patch('closed_loop_flight.rospy.wait_for_service'),\
             patch('closed_loop_flight.rospy.ServiceProxy',return_value=get),\
             patch('closed_loop_flight.time.sleep'):
            r.preflight()
        self.assertGreaterEqual(calls.count('COM_OF_LOSS_T'),2)

    def runtime(self):
        r=Runtime.__new__(Runtime)
        r.lock=threading.RLock();r.closed=False;r.rpc_busy=False;r.rpc_finished=0.;r.rpc_log=[]
        r.stop=False;r.mission=NS(phase='ARM');r.last_publish=time.monotonic();r.publish_count=60
        r.control_released=False;r.offboard_seen=False;r.release_mode=None
        r.owner=True;r.events=[];r.data={};r.sp=None
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

    def test_disarm_worker_rechecks_ground_and_accepts_middle_throttle_only_when_landed(self):
        for landed in [2,1]:
            r=self.runtime();r.data=self.telemetry_data(1500)
            r.data['state'][0].mode='AUTO.LAND';r.data['extended'][0].landed_state=landed
            calls=[];r.arm=lambda **kw:calls.append(kw) or NS(success=True)
            r.submit(('arm',False))
            deadline=time.monotonic()+1
            while r.rpc_busy and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(calls,[{'value':False}] if landed==1 else [])

    def test_preflight_accepts_middle_throttle_without_fc_parameters_changed(self):
        r=self.runtime();r.rc_reference=None;r.args=NS(allow_paired_estimate=True)
        r.data=self.telemetry_data(1500)
        r.data['state'][0].armed=False;r.data['state'][0].mode='MANUAL'
        r.data['state'][0].system_status=3;r.data['extended'][0].landed_state=1
        r.data['prediction']=({'latest_only':True,'coordinate_mode':'paired_estimate'},time.monotonic())
        r.master=NS(getSystemState=lambda:([('/uav1/mavros/setpoint_raw/local',['/uav1/uav_control_fsm'])],[],[]))
        params={'/uav1/uav_control_fsm/bench_disarmed':True,'/uav1/uav_control_fsm/bench_allow_armed':False,
                '/vla_direct_bridge/enable_publish':False,'/uav1/uav_control_fsm/external_setpoint_control_active':False}
        integers={'COM_OF_LOSS_T':0,'COM_OBL_ACT':0,'COM_OBL_RC_ACT':0,'COM_RC_OVERRIDE':1,
                  'RC_MAP_FLTMODE':6,'RC_MAP_ARM_SW':9,'RC_MAP_KILL_SW':10}
        def get(param_id):return NS(success=True,value=NS(integer=integers[param_id],real=1. if param_id=='COM_OF_LOSS_T' else 0.))
        with patch('closed_loop_flight.rospy.get_param',side_effect=lambda n,d=None:params.get(n,d)),\
             patch('closed_loop_flight.rospy.wait_for_service'),\
             patch('closed_loop_flight.rospy.ServiceProxy',return_value=get):
            r.preflight()
        self.assertEqual(r.rc_reference[2],1500)

    def test_actual_mode_callback_latches_takeover_and_stops_stream(self):
        r=self.runtime();r.sp=[0,0,1,.2];r.policy_at=time.monotonic();messages=[]
        r.publisher=NS(publish=lambda m:messages.append(m));r.mission.phase='NAVIGATE'
        r.receive('state',NS(mode='OFFBOARD'))
        r.receive('state',NS(mode='POSCTL'))
        self.assertTrue(r.control_released);self.assertIsNone(r.sp)
        r.sp=[0,0,1,.2]  # Even a stale policy write cannot restart publication.
        thread=threading.Thread(target=r.stream);thread.start();time.sleep(.12);r.closed=True;thread.join(1)
        self.assertEqual(messages,[])
        r.arm=lambda **_:self.fail('arm/disarm after takeover')
        r.mode=lambda **_:self.fail('mode override after takeover')
        for action in [('arm',True),('arm',False),('mode','OFFBOARD'),('mode','AUTO.LAND')]:r.submit(action)
        r.abort_and_land('simultaneous fault')
        self.assertEqual(r.rpc_log,[])

    def test_own_auto_land_is_not_latched_as_pilot_mode(self):
        r=self.runtime();r.mission.phase='LAND_REQUEST'
        r.receive('state',NS(mode='OFFBOARD'));r.receive('state',NS(mode='AUTO.LAND'))
        self.assertFalse(r.control_released)

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
