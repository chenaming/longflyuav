"""Exercise live release guards with fakes; no arm or mode endpoints."""
import unittest
from types import SimpleNamespace as NS
from ownership_release import release_if_safe

FSM='/uav1/uav_control_fsm'

class Tests(unittest.TestCase):
    def check(self,armed=False,connected=True,mode='POSCTL',landed=1,age=0,
              extra=False,watchdog=False,success=True):
        now=100.;calls=[];data={
            'state':(NS(connected=connected,armed=armed,mode=mode,system_status=3),now-age),
            'extended':(NS(landed_state=landed),now-age)}
        publishers=[('/uav1/mavros/setpoint_raw/local',[FSM]+(['other_controller'] if extra else []))]
        subscribers=[('/uav1/mavros/state',['/fly_bench60_watchdog'])] if watchdog else []
        def release(**kw):calls.append(kw);return NS(success=success)
        result=release_if_safe(data,(publishers,subscribers,[]),release,now=now)
        return result,calls

    def test_disarmed_ground_with_only_original_controller_releases(self):
        ok,calls=self.check();self.assertTrue(ok);self.assertEqual(calls,[{'value':False}])

    def test_airborne_armed_stale_disconnected_or_offboard_never_releases(self):
        for kw in [dict(armed=True),dict(landed=2),dict(age=2),dict(connected=False),dict(mode='OFFBOARD')]:
            with self.subTest(kw=kw):self.assertEqual(self.check(**kw),(False,[]))

    def test_other_controller_or_live_watchdog_never_releases(self):
        self.assertEqual(self.check(extra=True),(False,[]))
        self.assertEqual(self.check(watchdog=True),(False,[]))

    def test_service_rejection_is_not_success(self):
        ok,calls=self.check(success=False);self.assertFalse(ok);self.assertEqual(calls,[{'value':False}])

if __name__=='__main__':unittest.main(verbosity=2)
