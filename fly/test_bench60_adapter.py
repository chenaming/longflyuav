"""No FCU RPCs: test abort paths using fakes only."""
import unittest,time,threading
from types import SimpleNamespace as NS
from bench60 import TimedBench


class Tests(unittest.TestCase):
    def test_release_wait_never_releases_while_armed_then_releases_after_disarm(self):
        r=TimedBench.__new__(TimedBench);r.owner=True;r.rpc_busy=False;r.report_data={}
        clock=[100.];calls=[]
        def snapshot():
            return {'state':(NS(connected=True,armed=clock[0]<100.2,mode='POSCTL',system_status=3),clock[0]),
                    'extended':(NS(landed_state=1),clock[0])}
        r.snapshot=snapshot
        r.master=NS(getSystemState=lambda:([('/uav1/mavros/setpoint_raw/local',['/uav1/uav_control_fsm'])],[],[]))
        def release(**kw):
            self.assertFalse(snapshot()['state'][0].armed);calls.append(kw);return NS(success=True)
        r.ownership=release
        self.assertTrue(r.wait_for_disarmed_release(clock=lambda:clock[0],sleep=lambda d:clock.__setitem__(0,clock[0]+d)))
        self.assertEqual(calls,[{'value':False}]);self.assertFalse(r.owner)

    def test_release_timeout_keeps_airborne_owner_and_never_calls_service(self):
        r=TimedBench.__new__(TimedBench);r.owner=True;r.rpc_busy=False;r.report_data={}
        clock=[100.]
        r.snapshot=lambda:{'state':(NS(connected=True,armed=True,mode='POSCTL',system_status=4),clock[0])}
        r.master=NS(getSystemState=lambda:([],[],[]))
        r.ownership=lambda **_:self.fail('release while armed')
        self.assertFalse(r.wait_for_disarmed_release(timeout=.3,clock=lambda:clock[0],sleep=lambda d:clock.__setitem__(0,clock[0]+d)))
        self.assertTrue(r.owner)

    def test_takeover_already_disarmed_releases_original_controller(self):
        r=TimedBench.__new__(TimedBench)
        r.control_released=True;r.publisher=None;r.watchdog=None;r.owner=True
        r.report_data={};r.release_mode='POSCTL';r.rpc_busy=False
        r.actual_armed=lambda:False;r.save=lambda:None;r.lease=lambda:None
        now=time.monotonic()
        r.snapshot=lambda:{'state':(NS(connected=True,armed=False,mode='POSCTL',system_status=3),now),
                           'extended':(NS(landed_state=1),now)}
        r.master=NS(getSystemState=lambda:([('/uav1/mavros/setpoint_raw/local',['/uav1/uav_control_fsm'])],[],[]))
        calls=[];r.ownership=lambda **kw:calls.append(kw) or NS(success=True)
        r.arm=lambda **_:self.fail('takeover cleanup disarmed aircraft')
        r.mode=lambda **_:self.fail('takeover cleanup changed flight mode')
        r.handoff_to_pilot()
        self.assertEqual(calls,[{'value':False}])
        self.assertFalse(r.owner)

    def test_normal_cleanup_keeps_heartbeat_lands_then_disarms_without_rc_low(self):
        r=TimedBench.__new__(TimedBench);clock=[0.];vehicle={'mode':'OFFBOARD','armed':True}
        r.watchdog=None;r.control_released=False;r.mission=NS(phase='ARM')
        r.lock=threading.RLock();r.sp=[0,0,.03,0];r.rpc_busy=False;r.rpc_log=[]
        r.report_data={};r.save=lambda:None;r.policy_at=0.
        pulses=[];actions=[]
        r.lease=lambda:pulses.append((clock[0],r.policy_at,r.sp is not None))
        r.cleanup_state=lambda now:dict(fresh=True,connected=True,mode=vehicle['mode'],armed=vehicle['armed'],
                                       landed=vehicle['mode']=='AUTO.LAND' and now>=.4,land_fresh=True)
        def submit(action):
            actions.append((clock[0],action));r.rpc_log.append({'t':clock[0],'action':action})
            if action==('mode','AUTO.LAND'):vehicle['mode']='AUTO.LAND'
            elif action==('arm',False):
                self.assertGreaterEqual(clock[0],2.4);vehicle['armed']=False
            else:self.fail('unexpected cleanup action '+str(action))
        r.submit=submit
        result=r.run_normal_shutdown(clock=lambda:clock[0],sleep=lambda s:clock.__setitem__(0,clock[0]+s))
        self.assertTrue(result['disarmed_confirmed']);self.assertFalse(result['force_disarm_used'])
        self.assertEqual([x[1] for x in actions],[('mode','AUTO.LAND'),('arm',False)])
        self.assertTrue(pulses[0][2]);self.assertFalse(pulses[-1][2])
        self.assertTrue(all(abs(t-heartbeat)<1e-9 for t,heartbeat,_ in pulses))

    def test_preflight_failure_never_disarms_or_changes_mode(self):
        r=TimedBench.__new__(TimedBench)
        r.owner=False;r.rpc_log=[];r.rpc_busy=False;r.closed=False;r.report_data={}
        r.control_released=False
        r.actual_armed=lambda:True;r.save=lambda:None
        r.arm=lambda **_:self.fail('preflight rejection called arm/disarm')
        r.mode=lambda **_:self.fail('preflight rejection changed mode')
        self.assertFalse(r.stop_and_verify())
        self.assertTrue(r.closed);self.assertTrue(r.report_data['no_control_acquired'])
    def test_rc_fault_keeps_the_trigger_channels(self):
        r=TimedBench.__new__(TimedBench);r.stop=False;r.stopping=False;r.report_data={}
        r.rc_reference=[999]*16
        channels=[999]*16;channels[2]=1237
        r.snapshot=lambda:{'rc':(NS(channels=channels),time.monotonic())}
        r.telemetry=lambda _:({'armed':True,'mode':'OFFBOARD'},[0,0,0,0],0,0,'RC takeover requested')
        with self.assertRaisesRegex(RuntimeError,'RC takeover'):r.pulse()
        self.assertEqual(r.report_data['fault_snapshot']['rc_channels'][2],1237)
        self.assertEqual(r.report_data['fault_snapshot']['rc_reference'][2],999)

    def test_pilot_mode_exit_never_disarms_or_overrides_mode(self):
        r=TimedBench.__new__(TimedBench)
        r.control_released=True;r.stopping=False;r.publisher=None;r.watchdog=None
        r.report_data={};r.release_mode='POSCTL';r.actual_armed=lambda:True
        r.save=lambda:None;r.lease=lambda:None
        r.arm=lambda **_:self.fail('disarm after pilot takeover')
        r.mode=lambda **_:self.fail('mode override after pilot takeover')
        self.assertFalse(r.stop_and_verify())
        self.assertTrue(r.report_data['no_disarm_on_takeover']);self.assertTrue(r.closed)


if __name__=='__main__':unittest.main(verbosity=2)
