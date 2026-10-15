"""Exercise the deployed cleanup helper with asynchronous FCU state delivery."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import unittest

spec=importlib.util.spec_from_file_location('motor_stop_under_test',Path(os.environ.get('FLY_STOP_MODULE','outputs/fly/motor_stop.py')))
stop=importlib.util.module_from_spec(spec);spec.loader.exec_module(stop)

class Tests(unittest.TestCase):
    def test_accepted_normal_ack_is_not_repeated_while_waiting_for_state(self):
        clock=[0.];calls=[]
        def sleep(seconds):clock[0]+=seconds
        def normal(**kw):
            self.assertIs(kw['value'],False)
            calls.append(clock[0]);return SimpleNamespace(success=True,result=0)
        def force(**kw):raise AssertionError('ordinary stop state arrives within one heartbeat')
        result=stop.bench_disarm(normal,force,lambda:clock[0]<1.,propellers_removed_bench=True,
            prepare_manual_stop=lambda:{'mode_sent':True},sleep=sleep)
        self.assertEqual(len(calls),1)
        self.assertTrue(result['disarmed_confirmed'])
        self.assertFalse(result['force_disarm_used'])

    def test_last_normal_retry_ack_waits_for_next_heartbeat(self):
        clock=[0.];normal_calls=[];force_calls=[];accepted_at=[None]
        def sleep(seconds):clock[0]+=seconds
        def state():
            return not (accepted_at[0] is not None and clock[0]>=accepted_at[0]+.8)
        def normal(**kw):
            self.assertIs(kw['value'],False)
            normal_calls.append(clock[0])
            # MANUAL/landed settling: only the last ordinary retry succeeds.
            if len(normal_calls)>=4:
                if accepted_at[0] is None:accepted_at[0]=clock[0]
                return SimpleNamespace(success=True,result=0)
            return SimpleNamespace(success=False,result=1)
        def force(**kw):
            force_calls.append(clock[0])
            return SimpleNamespace(success=True,result=0)
        result=stop.bench_disarm(normal,force,state,
            propellers_removed_bench=True,prepare_manual_stop=lambda:{'mode_sent':True},sleep=sleep)
        self.assertTrue(result['disarmed_confirmed'])
        self.assertEqual(force_calls,[], 'ordinary disarm accepted; wait for its state heartbeat before force')
        self.assertFalse(result['force_disarm_used'])

    def test_accepted_ack_without_state_confirmation_still_uses_bounded_fallback(self):
        clock=[0.];forced=[]
        def sleep(seconds):clock[0]+=seconds
        def force(**kw):
            self.assertEqual(kw['param1'],0.)
            forced.append(clock[0]);return SimpleNamespace(success=True,result=0)
        result=stop.bench_disarm(lambda **kw:SimpleNamespace(success=True,result=0),force,
            lambda:False if forced else True,propellers_removed_bench=True,sleep=sleep)
        self.assertTrue(result['disarmed_confirmed'])
        self.assertTrue(result['force_disarm_used'])
        self.assertTrue(forced)
        self.assertLessEqual(forced[0],3.1)

if __name__=='__main__':unittest.main()
