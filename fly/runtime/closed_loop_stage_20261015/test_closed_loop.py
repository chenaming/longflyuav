"""Deterministic policy tests; does not import ROS or send flight commands."""
import math
import sys
import unittest
from types import SimpleNamespace as NS
from flight_core import Config, Mission, norm
from latest_observation import LatestObservation

EPOCH=1792036000.


def msg(stamp, value):
    return NS(header=NS(stamp=NS(to_sec=lambda:stamp)), value=value)


class LatestTests(unittest.TestCase):
    def test_busy_frames_are_overwritten(self):
        buffer=LatestObservation()
        for i in range(100):
            stamp=100+i*.01
            buffer.put_pose('odin',msg(stamp,i));buffer.put_pose('mavros',msg(stamp,i))
            buffer.put_image(msg(stamp,i))
        self.assertEqual(buffer.snapshot(101)[0].value,99)
        self.assertEqual(buffer.overwritten,99)

    def test_never_falls_back_to_older_pairable_image(self):
        b=LatestObservation()
        b.put_pose('odin',msg(100,1));b.put_pose('mavros',msg(100,1))
        b.put_image(msg(100,1));b.put_image(msg(100.2,2))
        self.assertIsNone(b.snapshot(100.21))
        b.put_pose('odin',msg(100.2,2));b.put_pose('mavros',msg(100.2,2))
        self.assertEqual(b.snapshot(100.21)[0].value,2)

    def test_stale_and_future_image_rejected(self):
        b=LatestObservation();b.put_image(msg(100,1))
        for kind in ('odin','mavros'):b.put_pose(kind,msg(100,1))
        self.assertIsNone(b.snapshot(101));self.assertIsNone(b.snapshot(99))


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.c=Config(allow_paired=True)
        self.m=Mission(self.c,'chair',0,EPOCH,[3,4,.4,0])
        self.p=[3,4,.4,0]
        self.s={'fresh':True,'connected':True,'armed':False,'mode':'MANUAL','landed':True}
        self.n=0

    def prediction(self,t,raw=(25,0,0,0),pose=None):
        self.n+=1;pose=pose or self.p
        return dict(run_id='run',epoch=1,instruction='chair',target_frame='map',request_id=self.n,
                    observation_stamp=EPOCH+t,latest_only=True,coordinate_mode='paired_estimate',
                    target_xyz_m=[pose[i]+raw[i]/100 for i in range(3)],target_yaw_rad=pose[3]+raw[3],
                    action_ori_cm_rad=list(raw),observed_xyz_m=pose[:3],observed_yaw_rad=pose[3])

    def tick(self,t,pred=None,**kw):
        return self.m.step(t,EPOCH+t,self.p,kw.pop('speed',0),kw.pop('yaw_rate',0),self.s,pred,**kw)

    def nav(self,t=10):
        self.s.update(armed=True,mode='OFFBOARD',landed=False)
        self.p=[3,4,1.4,0];self.m.arm_requested=True
        self.m.sp=list(self.p);self.m.target=list(self.p)
        self.m.phase='WAIT_NEW_IMAGE';self.m.navigation_stamp=EPOCH+t
        self.m.navigation_start=t;self.m.last_tick=t;self.m.last_ros=EPOCH+t
        self.m.identity=('run',1,'chair','map')

    def test_first_target_only_gates_takeoff(self):
        self.tick(.1,self.prediction(.1))
        self.assertEqual(self.m.phase,'PRESTREAM')
        self.assertEqual(self.m.target,self.p)
        self.tick(2.2);_,action=self.tick(2.3)
        self.assertEqual(action,('mode','OFFBOARD'))
        self.s['mode']='OFFBOARD';self.tick(2.4)
        _,action=self.tick(2.5,self.prediction(2.5));self.assertEqual(action,('arm',True))
        self.assertIsNone(self.tick(2.6)[1])
        self.s['armed']=True;self.tick(2.7)
        self.assertAlmostEqual(self.m.target[2],1.4)

    def test_takeoff_settle_discards_old_results(self):
        self.m.arm_requested=True;self.s.update(armed=True,mode='OFFBOARD')
        self.m.phase='TAKEOFF';self.p=[3,4,1.4,0];self.m.target=list(self.p)
        for i in range(1,26):self.tick(i*.1)
        self.assertEqual(self.m.phase,'WAIT_NEW_IMAGE')
        old=self.prediction(.5,pose=[3,4,.4,0]);self.tick(2.6,old)
        self.assertEqual(self.m.phase,'WAIT_NEW_IMAGE')
        self.tick(2.7,self.prediction(2.7))
        self.assertEqual(self.m.phase,'NAVIGATE')

    def test_repeated_frame_does_not_count_as_arrival(self):
        self.nav();pred=self.prediction(10.1,(0,0,0,0))
        for i in range(1,19):self.tick(10+i*.1,pred)
        self.assertLessEqual(len(self.m.small),1);self.assertFalse(self.m.arrival_inferred)

    def test_arrival_needs_fresh_distinct_predictions_and_stability(self):
        self.nav()
        actions=[]
        for i in range(1,26):
            t=10+i*.21;actions.append(self.tick(t,self.prediction(t,(0,0,0,0)))[1])
        self.assertTrue(self.m.arrival_inferred)
        self.assertEqual(self.m.phase,'LAND_REQUEST')
        self.assertIn(('mode','AUTO.LAND'),actions)
        self.s['mode']='AUTO.LAND';sp,_=self.tick(15.6)
        self.assertIsNone(sp);self.assertEqual(self.m.phase,'LANDING')
        self.s.update(armed=False,landed=True);self.tick(15.7)
        self.assertEqual(self.m.phase,'DONE')

    def test_high_actual_speed_blocks_arrival(self):
        self.nav()
        for i in range(1,30):
            t=10+i*.2;self.tick(t,self.prediction(t,(0,0,0,0)),speed=.15)
        self.assertFalse(self.m.arrival_inferred);self.assertEqual(self.m.phase,'NAVIGATE')

    def test_stale_inference_holds_then_abort_lands_not_arrival(self):
        self.nav();self.tick(10.1,self.prediction(10.1))
        for i in range(2,54):self.tick(10+i*.1)
        self.assertEqual(self.m.phase,'LAND_REQUEST');self.assertFalse(self.m.arrival_inferred)
        self.assertIn('inference unavailable',self.m.reason)

    def test_rc_mode_exit_never_reenters_or_rearms(self):
        self.nav();self.s['mode']='POSCTL'
        self.assertEqual(self.tick(10.1),(None,None))
        self.assertEqual(self.m.phase,'TAKEOVER')
        self.s['mode']='OFFBOARD';self.assertEqual(self.tick(10.2),(None,None))

    def test_nonfinite_geofence_epoch_old_prediction_rejected(self):
        self.nav()
        for mutate in (lambda p:p.update(epoch=2),lambda p:p.update(target_xyz_m=[float('nan'),4,1.4]),
                       lambda p:p.update(observation_stamp=EPOCH-1),lambda p:p.update(target_xyz_m=[9,4,1.4]),
                       lambda p:p.update(latest_only=False)):
            pred=self.prediction(10.1);mutate(pred)
            self.assertFalse(self.m.accept(pred,10.1,EPOCH+10.1,self.p))
        self.assertEqual(self.m.accepted_predictions,0)

    def test_model_target_outside_height_not_clamped_into_arrival(self):
        self.nav();self.p[2]=.91
        self.assertFalse(self.m.accept(self.prediction(10.1,(0,0,-2,0)),10.1,EPOCH+10.1,self.p))
        self.assertFalse(self.m.arrival_inferred)

    def test_ground_confirmation_required_before_normal_disarm(self):
        self.nav();self.m.phase='LANDING';self.s['mode']='AUTO.LAND'
        for i in range(1,35):self.assertIsNone(self.tick(10+i*.1)[1])
        self.s['landed']=True
        actions=[self.tick(14+i*.1)[1] for i in range(25)]
        self.assertEqual([a for a in actions if a],[('arm',False)])

    def test_unexpected_arm_before_gate_is_not_adopted(self):
        self.s['armed']=True
        self.assertEqual(self.tick(.1),(None,None));self.assertEqual(self.m.phase,'TAKEOVER')

    def test_clock_jump_aborts_airborne_mission(self):
        self.nav()
        self.m.step(10.1,EPOCH+110.1,self.p,0,0,self.s)
        self.assertEqual(self.m.phase,'LAND_REQUEST');self.assertFalse(self.m.arrival_inferred)

    def test_command_speed_bounded(self):
        self.nav();before=list(self.m.sp)
        sp,_=self.tick(10.1,self.prediction(10.1))
        self.assertLessEqual(norm([a-b for a,b in zip(sp[:3],before[:3])]),.020001)

    def test_pair_estimate_requires_explicit_choice(self):
        self.m.cfg.allow_paired=False
        self.assertFalse(self.m.accept(self.prediction(.1),.1,EPOCH+.1,self.p))

    def test_stale_initial_gate_cannot_arm_later(self):
        self.tick(.1,self.prediction(.1))
        self.tick(2.2);self.tick(2.3)
        self.s['mode']='OFFBOARD';self.tick(2.4)
        self.assertIsNone(self.tick(2.5)[1]);self.assertFalse(self.m.arm_requested)

    def test_moving_aircraft_before_arm_cancels_start(self):
        self.tick(.1,self.prediction(.1));self.p[0]+=.15
        self.assertEqual(self.tick(.2),(None,None));self.assertEqual(self.m.phase,'ABORTED')

    def test_land_ack_without_landed_and_disarmed_is_not_done(self):
        self.nav();self.m.phase='LAND_REQUEST';self.s['mode']='AUTO.LAND'
        self.tick(10.1);self.tick(11.)
        self.assertEqual(self.m.phase,'LANDING')
        self.s['landed']=True;self.tick(11.1)
        self.assertEqual(self.m.phase,'LANDING')

    def test_large_raw_action_cannot_count_as_arrival_when_aircraft_caught_up(self):
        self.nav();pred=self.prediction(10.1,(25,0,0,0))
        self.p[0]+=.25
        self.assertTrue(self.m.accept(pred,10.1,EPOCH+10.1,self.p))
        self.assertEqual(self.m.small,[])


if __name__=='__main__':unittest.main(verbosity=2)
