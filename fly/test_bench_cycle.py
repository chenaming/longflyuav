import unittest
from types import SimpleNamespace as NS
from bench_cycle import run_cycle
from stack_ground_guard import ground_safe


class Fake:
    def __init__(self,fail=None):self.calls=[];self.fail=fail;self.stops=0
    def __getattr__(self,name):
        def call(*args):
            self.calls.append(name)
            if name=='stop_stack':self.stops+=1
            if name==self.fail:raise RuntimeError(name)
        return call


class Tests(unittest.TestCase):
    def test_each_motor_run_starts_with_stop_and_always_stops_afterwards(self):
        b=Fake();run_cycle(b)
        self.assertEqual(b.calls,['stop_stack','start_stack','wait_ready','infer','model_chain','precheck','motor','stop_stack'])
    def test_check_only_never_runs_model_control_or_motor(self):
        b=Fake();run_cycle(b,True)
        self.assertEqual(b.calls,['stop_stack','start_stack','wait_ready','infer','stop_stack'])
    def test_failure_before_arming_still_closes_stack(self):
        b=Fake('infer')
        with self.assertRaisesRegex(RuntimeError,'infer'):run_cycle(b)
        self.assertEqual(b.calls[-1],'stop_stack');self.assertNotIn('motor',b.calls)
    def test_failed_initial_ground_guard_never_starts_or_retries_shutdown(self):
        b=Fake('stop_stack')
        with self.assertRaises(RuntimeError):run_cycle(b)
        self.assertEqual(b.calls,['stop_stack'])
    def test_no_kill_fallback_when_postflight_stop_guard_refuses(self):
        b=Fake()
        def stop():
            b.calls.append('stop_stack');b.stops+=1
            if b.stops==2:raise RuntimeError('still airborne')
        b.stop_stack=stop
        with self.assertRaisesRegex(RuntimeError,'still airborne'):run_cycle(b)
        self.assertEqual(b.calls[-1],'keep_for_takeover')
    def test_stale_armed_disconnected_or_airborne_never_permits_shutdown(self):
        s=NS(connected=True,armed=False,system_status=3);e=NS(landed_state=1)
        self.assertTrue(ground_safe(s,10,e,10,10))
        for k,v in [('armed',True),('connected',False),('system_status',4)]:
            old=getattr(s,k);setattr(s,k,v);self.assertFalse(ground_safe(s,10,e,10,10));setattr(s,k,old)
        self.assertFalse(ground_safe(s,8,e,10,10));self.assertFalse(ground_safe(s,10,e,8,10))
        e.landed_state=2;self.assertFalse(ground_safe(s,10,e,10,10))


if __name__=='__main__':unittest.main(verbosity=2)
