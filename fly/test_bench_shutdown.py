"""Normal cleanup must land, observe ground, then ordinary-disarm without RC-low."""
import unittest
from bench_shutdown import NormalShutdown


def state(mode='OFFBOARD',armed=True,landed=False,land_fresh=True):
    return dict(fresh=True,connected=True,mode=mode,armed=armed,landed=landed,land_fresh=land_fresh)


class Tests(unittest.TestCase):
    def test_keep_stream_until_land_mode_confirmed_not_just_ack(self):
        p=NormalShutdown(0)
        keep,action=p.step(0,state())
        self.assertTrue(keep);self.assertEqual(action,('mode','AUTO.LAND'))
        keep,action=p.step(.2,state(),rpc_ready=False)
        self.assertTrue(keep);self.assertIsNone(action)
        keep,action=p.step(.3,state('AUTO.LAND'))
        self.assertFalse(keep);self.assertIsNone(action)

    def test_no_disarm_before_continuous_fresh_ground(self):
        p=NormalShutdown(0)
        p.step(0,state());p.step(.2,state('AUTO.LAND'))
        for t in [.3,1,2]:
            self.assertIsNone(p.step(t,state('AUTO.LAND'))[1])
        p.step(2.1,state('AUTO.LAND',landed=True))
        self.assertIsNone(p.step(3.9,state('AUTO.LAND',landed=True))[1])
        self.assertEqual(p.step(4.2,state('AUTO.LAND',landed=True))[1],('arm',False))
        self.assertNotEqual(p.phase,'DONE')  # ACK/request is not a confirmed disarm.
        p.step(4.3,state('AUTO.LAND',armed=False,landed=True))
        self.assertEqual(p.phase,'DONE')

    def test_stale_or_bouncing_land_state_resets_ground_timer(self):
        p=NormalShutdown(0);p.step(0,state('AUTO.LAND',landed=True))
        p.step(1.5,state('AUTO.LAND',landed=True,land_fresh=False))
        self.assertIsNone(p.step(2.1,state('AUTO.LAND',landed=True))[1])
        p.step(3,state('AUTO.LAND',landed=False))
        self.assertIsNone(p.step(3.1,state('AUTO.LAND',landed=True))[1])

    def test_pilot_mode_always_stops_automatic_cleanup(self):
        for mode in ['POSCTL','MANUAL','ALTCTL']:
            p=NormalShutdown(0);p.step(0,state())
            self.assertEqual(p.step(.1,state(mode)),(False,None))
            self.assertEqual(p.phase,'TAKEOVER')
            self.assertEqual(p.step(.2,state('OFFBOARD')),(False,None))

    def test_timeout_does_not_issue_force_or_ordinary_disarm_in_air(self):
        p=NormalShutdown(0);p.step(0,state())
        self.assertEqual(p.step(12.1,state('AUTO.LAND')),(False,None))
        self.assertEqual(p.phase,'FAILED')

    def test_auto_disarm_on_ground_is_normal(self):
        p=NormalShutdown(0);p.step(0,state())
        p.step(1,state('AUTO.LAND',armed=False,landed=True))
        self.assertEqual(p.phase,'DONE');self.assertTrue(p.auto_disarmed)

    def test_disarm_rpc_busy_does_not_drop_ground_request(self):
        p=NormalShutdown(0);p.step(0,state('AUTO.LAND',landed=True))
        self.assertIsNone(p.step(2.1,state('AUTO.LAND',landed=True),rpc_ready=False)[1])
        self.assertEqual(p.step(2.2,state('AUTO.LAND',landed=True))[1],('arm',False))


if __name__=='__main__':unittest.main(verbosity=2)
