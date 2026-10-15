"""No FCU RPCs: test abort paths using fakes only."""
import unittest,time,threading
from types import SimpleNamespace as NS
from bench60 import TimedBench


class Tests(unittest.TestCase):
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
