import math,unittest
from bench60_policy import Bench60,watchdog_reasons,pilot_mode_selected
from types import SimpleNamespace as NS


class Tests(unittest.TestCase):
    def setUp(self):
        self.b=Bench60([1,2,.03,.4],('r',1,'chair'));self.b.begin(0)
    def p(self,t,n=1):
        return dict(run_id='r',epoch=1,instruction='chair',observation_stamp=1000+t,
                    request_id=n,latest_only=True,target_frame='map',
                    target_xyz_m=[100,-100,100],target_yaw_rad=2,
                    action_ori_cm_rad=[25,0,0,0])
    def test_stages_are_timed_and_never_1m(self):
        for t,phase in [(0,'SIMULATED_TAKEOFF'),(9.99,'SIMULATED_TAKEOFF'),(10,'SIMULATED_HOVER'),(12,'MODEL_LOOP'),(29.99,'MODEL_LOOP'),(30,'STOPPING')]:
            self.assertEqual(self.b.step(t,1000+t),phase)
            self.assertEqual(self.b.sp,[1,2,.03,.4])
    def test_only_image_after_hover_updates_target(self):
        self.b.step(12,1012)
        self.assertFalse(self.b.accept(self.p(11),12,1012))
        self.assertEqual(self.b.sp,[1,2,.03,.4])
        self.assertTrue(self.b.accept(self.p(12.1,2),12.2,1012.2))
        d=[self.b.sp[i]-self.b.home[i] for i in range(3)]
        self.assertLessEqual(math.hypot(*d[:2]),.0200000001)
        self.assertLessEqual(abs(d[2]),.003000001)
        self.assertLessEqual(abs(self.b.sp[3]-.4),math.radians(1)+1e-9)
    def test_duplicates_never_count(self):
        self.b.step(12,1012);p=self.p(12.1)
        self.assertTrue(self.b.accept(p,12.2,1012.2))
        self.assertFalse(self.b.accept(p,12.3,1012.3));self.assertEqual(len(self.b.samples),1)
    def test_old_and_wrong_task_rejected(self):
        self.b.step(12,1012)
        with self.assertRaises(ValueError):self.b.accept(self.p(10),13,1013)
        p=self.p(12.1);p['epoch']=2
        with self.assertRaises(ValueError):self.b.accept(p,12.2,1012.2)
    def test_nonfinite_rejected(self):
        self.b.step(12,1012);p=self.p(12.1);p['target_xyz_m'][0]=float('nan')
        with self.assertRaises(ValueError):self.b.accept(p,12.2,1012.2)
    def test_no_accumulation(self):
        self.b.step(12,1012)
        for i in range(100):self.b.accept(self.p(12.1+i*.1,i+1),12.2+i*.1,1012.2+i*.1)
        self.assertLessEqual(math.dist(self.b.sp[:2],self.b.home[:2]),.02000000001)
    def test_old_initial_model_does_not_abort_timed_hold(self):
        self.b.step(9,1009)
        self.assertFalse(self.b.offer(self.p(0),9,1009,0))
        self.assertEqual(self.b.sp,self.b.home)
    def test_waiting_new_image_does_not_reuse_pre_hover_or_stale_result(self):
        self.b.step(12,1012)
        self.assertFalse(self.b.offer(self.p(11),12,1012,12))
        self.assertFalse(self.b.offer(self.p(12.1),15,1015,15))
        self.assertEqual(self.b.samples,[])
        self.assertTrue(self.b.offer(self.p(15,2),15.1,1015.1,15.1))


class WatchdogTests(unittest.TestCase):
    def reasons(self,**kw):
        channels=[1500]*16;channels[2]=channels[8]=999
        d=dict(now=20,start=0,armed_since=10,alive=True,lease={},lease_at=20,
               state=NS(connected=True,armed=True,mode='OFFBOARD'),state_at=20,
               rc=NS(channels=channels,rssi=100),rc_at=20,initial_ch6=1500)
        d.update(kw);return watchdog_reasons(**d)
    def test_healthy_not_stopped(self):self.assertEqual(self.reasons(),[])
    def test_thirty_second_window_allowed_but_hard_45_stops(self):
        self.assertEqual(self.reasons(armed_since=-10),[])
        self.assertIn('armed 45s limit',self.reasons(armed_since=-26))
    def test_dead_policy_stops_even_with_healthy_fcu(self):
        self.assertIn('policy heartbeat stale',self.reasons(lease_at=19))
        self.assertIn('parent exited',self.reasons(alive=False))
    def test_rc_signal_failsafe_preserved_but_sticks_do_not_stop(self):
        channels=[1500]*16;channels[2]=999;channels[8]=2000
        self.assertEqual(self.reasons(rc=NS(channels=channels,rssi=100)),[])
        channels[2]=1500
        self.assertEqual(self.reasons(rc=NS(channels=channels,rssi=100)),[])
        self.assertIn('RC stale',self.reasons(rc_at=19))
    def test_state_and_mode_failures_stop(self):
        self.assertIn('FCU state stale',self.reasons(state_at=18))
        self.assertEqual(self.reasons(state=NS(connected=True,armed=True,mode='MANUAL')),[])
    def test_actual_mode_exit_hands_over_without_watchdog_disarm(self):
        state=NS(connected=True,armed=True,mode='POSCTL')
        self.assertTrue(pilot_mode_selected(20,10,state,20,{}))
        self.assertFalse(pilot_mode_selected(20,10,state,20,{'stop':True}))
        self.assertFalse(pilot_mode_selected(20,10,state,18,{}))
        state.mode='AUTO.LAND'
        self.assertFalse(pilot_mode_selected(20,10,state,20,{'normal_cleanup':True}))
        state.mode='POSCTL'
        self.assertTrue(pilot_mode_selected(20,10,state,20,{'normal_cleanup':True}))
    def test_normal_parent_stop_and_absolute_limit(self):
        self.assertIn('parent requested stop',self.reasons(lease={'stop':True}))
        self.assertIn('overall 65s limit',self.reasons(start=-46))
        self.assertEqual(self.reasons(lease={'normal_cleanup':True}),[])


if __name__=='__main__':unittest.main(verbosity=2)
