"""No FCU RPCs: test abort paths using fakes only."""
import unittest,time
from types import SimpleNamespace as NS
from bench60 import TimedBench


class Tests(unittest.TestCase):
    def test_preflight_failure_never_disarms_or_changes_mode(self):
        r=TimedBench.__new__(TimedBench)
        r.owner=False;r.rpc_log=[];r.rpc_busy=False;r.closed=False;r.report_data={}
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


if __name__=='__main__':unittest.main(verbosity=2)
