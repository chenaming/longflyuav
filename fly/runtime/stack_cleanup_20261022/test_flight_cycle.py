import unittest
from flight_cycle import run_flight_cycle


class Fake:
    def __init__(self,fail=None):self.calls=[];self.fail=fail
    def __getattr__(self,name):
        def action(*args):
            self.calls.append((name,args))
            if name==self.fail:raise RuntimeError(name)
        return action


class Tests(unittest.TestCase):
    def test_default_lifecycle_checks_only_and_stops_after_ground_confirmation(self):
        b=Fake();run_flight_cycle(b)
        self.assertEqual([c[0] for c in b.calls],['stop_stack','start_stack','wait_ready','infer','flight','wait_ground','stop_stack'])
        self.assertEqual(b.calls[4],('flight',(True,)))
    def test_execute_waits_for_ground_before_shutdown(self):
        b=Fake();run_flight_cycle(b,False)
        self.assertEqual(b.calls[4],('flight',(False,)));self.assertEqual(b.calls[-2],('wait_ground',(False,)))
    def test_airborne_takeover_never_closes_positioning_nodes(self):
        b=Fake('wait_ground')
        with self.assertRaises(RuntimeError):run_flight_cycle(b,False)
        self.assertEqual([c[0] for c in b.calls].count('stop_stack'),1)
        self.assertEqual(b.calls[-1][0],'keep_for_takeover')
    def test_preflight_error_still_checks_ground_and_closes(self):
        b=Fake('flight')
        with self.assertRaises(RuntimeError):run_flight_cycle(b,True)
        self.assertEqual([c[0] for c in b.calls][-2:],['wait_ground','stop_stack'])
    def test_failed_initial_ground_check_never_restarts_stack(self):
        b=Fake('stop_stack')
        with self.assertRaises(RuntimeError):run_flight_cycle(b)
        self.assertEqual(b.calls,[('stop_stack',())])


if __name__=='__main__':unittest.main(verbosity=2)
