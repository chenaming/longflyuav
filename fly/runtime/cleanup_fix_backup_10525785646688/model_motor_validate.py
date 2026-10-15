#!/usr/bin/env python3
"""Five-second armed, propeller-removed model-point test through the original FSM.
No motor-test command, force-arm, parameter bypass, or normal AUTO_TAKEOFF path.
"""
import json,math,time
from pathlib import Path
import rospy
from std_msgs.msg import Bool,String
from mavros_msgs.msg import RCOut,ActuatorControl,ESCStatus,RCIn,ExtendedState,StatusText
from mavros_msgs.srv import CommandBool,CommandLong
from motor_stop import bench_disarm
from tf.transformations import euler_from_quaternion
from model_bench_validate import ModelBench
from bench_validate import wrap
from motor_task_guard import same_task, rc_is_ready

class MotorBench(ModelBench):
 def __init__(self):
  self.allow_armed=False;self.arm_deadline=0.;self.lease=False
  super().__init__()
  self.report_pub.unregister()
  self.report_pub=rospy.Publisher('/vla/model_motor_report',String,queue_size=1,latch=True)
  self.path=Path('/home/nvidia/.local/state/vla-commissioning/motor_model_test/motor_report.json')
  self.report.update(kind='five_second_armed_model_point_bench',status='preparing',cases=[],motor_rotation_confirmed=False,limits={'xy_m':.02,'z_m':.003,'yaw_deg':1.},model_targets=[],physical_navigation_verified=False)
  self.lease_pub=rospy.Publisher('/vla/motor_bench_lease',Bool,queue_size=1)
  self.lease_timer=rospy.Timer(rospy.Duration(.04),lambda _:self.lease_pub.publish(Bool(data=self.lease)))
  self.arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool)
  self.force_stop=rospy.ServiceProxy('/uav1/mavros/cmd/command',CommandLong)
  self.outputs=[];self.actuators=[];self.rpms=[]
  self.landed_states=[];self.fcu_statustext=[]
  rospy.Subscriber('/uav1/mavros/extended_state',ExtendedState,lambda m:self.landed_states.append({'t':time.monotonic(),'landed_state':m.landed_state}))
  rospy.Subscriber('/uav1/mavros/statustext/recv',StatusText,lambda m:self.fcu_statustext.append({'t':time.monotonic(),'severity':m.severity,'text':m.text}))
  rospy.Subscriber('/uav1/mavros/rc/out',RCOut,lambda m:self.outputs.append({'t':time.monotonic(),'channels':list(m.channels)}))
  rospy.Subscriber('/uav1/mavros/target_actuator_control',ActuatorControl,lambda m:self.actuators.append({'t':time.monotonic(),'controls':list(m.controls)}))
  rospy.Subscriber('/uav1/mavros/esc_status',ESCStatus,lambda m:self.rpms.append({'t':time.monotonic(),'rpm':[x.rpm for x in m.esc_status]}))

 def check(self):
  with self.lock:
   now=time.monotonic()
   if not self.state or now-self.state[1]>1.5 or not self.state[0].connected:raise RuntimeError('stale/disconnected FCU')
   if not self.odom or now-self.odom[1]>.5:raise RuntimeError('stale odometry')
   if self.state[0].armed and (not self.allow_armed or now>self.arm_deadline):raise RuntimeError('armed outside bounded window')
   return self.state[0],self.odom[0]

 def bounded_command(self,p):
  _,odom=self.check();pos,q=odom.pose.pose.position,odom.pose.pose.orientation
  yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
  raw=[a-b for a,b in zip(p['target_xyz_m'],[pos.x,pos.y,pos.z])]
  scale=min(1.,.02/max(math.hypot(raw[0],raw[1]),1e-9))
  step=[raw[0]*scale,raw[1]*scale,max(-.003,min(.003,raw[2]))]
  dyaw=max(-1.,min(1.,math.degrees(wrap(p['target_yaw_rad']-yaw))))
  c=self.command(dx=step[0],dy=step[1],dz=step[2],yaw_deg=dyaw)
  return c,{'prediction':p,'raw_delta_xyz_m':raw,'limited_delta_xyz_m':step,'limited_delta_yaw_deg':dyaw,'sent_xyz_m':[c.pos_ref.x,c.pos_ref.y,c.pos_ref.z],'sent_yaw_deg':c.yaw_ref}

 def run(self):
  initial_mode=self.check()[0].mode
  assert initial_mode in ('MANUAL','OFFBOARD')
  original='MANUAL'
  self.report['initial_mode']=initial_mode
  assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed') is True
  assert rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed') is True
  assert rospy.get_param('/vla_direct_bridge/enable_publish') is False
  assert rospy.wait_for_message('/vla/motor_watchdog_ready',Bool,timeout=5).data is True
  self.report['original_mode']=original;self.persist()
  try:
   self.hold(2.)
   self.report.update(status='entering_offboard',offboard_entry='mavros_set_mode');self.persist()
   self.report['offboard_mode_sent']=self.mode(base_mode=0,custom_mode='OFFBOARD').mode_sent
   end=time.monotonic()+5
   while self.check()[0].mode!='OFFBOARD':
    if time.monotonic()>end:raise RuntimeError('OFFBOARD not observed')
    time.sleep(.05)
   self.report['offboard_start_monotonic']=time.monotonic()
   self.hold(5.)
   p=self.next_prediction()
   if not same_task(p,self.report,rospy.get_param('/vla_direct_bridge/active_instruction',''),rospy.get_param('/vla_direct_bridge/run_id','')):raise RuntimeError('task changed before arming')
   c,entry=self.bounded_command(p)
   # Verify the bounded real model point in the disarmed controller first.
   for _ in range(12):
    c.header.stamp=rospy.Time.now();self.pub.publish(c);time.sleep(.04)
   with self.lock:
    sp=self.setpoints[-1][1]
   if sp._connection_header.get('callerid')!='/uav1/uav_control_fsm':raise RuntimeError('unexpected setpoint source')
   if max(abs(a-b) for a,b in zip(entry['sent_xyz_m'],[sp.position.x,sp.position.y,sp.position.z]))>1e-4:raise RuntimeError('model point not propagated before arming')
   self.report['disarmed_point_verified']=entry
   if not same_task(p,self.report,rospy.get_param('/vla_direct_bridge/active_instruction',''),rospy.get_param('/vla_direct_bridge/run_id','')):raise RuntimeError('task changed before arming')
   if not 0<=rospy.Time.now().to_sec()-p['observation_stamp']<=2:raise RuntimeError('model prediction stale before arming')
   self.allow_armed=True;self.arm_deadline=time.monotonic()+8.5;self.lease=True
   self.lease_pub.publish(Bool(data=True));time.sleep(.12)
   rc=rospy.wait_for_message('/uav1/mavros/rc/in',RCIn,timeout=2)
   if not rc_is_ready(rc.channels):raise RuntimeError('RC throttle/CH9 not low before arming')
   if not same_task(p,self.report,rospy.get_param('/vla_direct_bridge/active_instruction',''),rospy.get_param('/vla_direct_bridge/run_id','')):raise RuntimeError('task changed before arming')
   if not 0<=rospy.Time.now().to_sec()-p['observation_stamp']<=2:raise RuntimeError('model prediction stale before arming')
   state,_=self.check()
   if state.armed or state.mode!='OFFBOARD' or state.system_status!=3:raise RuntimeError('expected disarmed OFFBOARD Standby before arming')
   result=self.arm(value=True)
   self.report['arming_response']={'success':result.success,'result':result.result}
   if not result.success:raise RuntimeError('normal arming rejected')
   end=time.monotonic()+1.5
   while not self.check()[0].armed:
    if time.monotonic()>end:raise RuntimeError('armed state not observed')
    time.sleep(.02)
   start=time.monotonic();self.report['armed_start_monotonic']=start;end=start+5.
   used=set();sent=[]
   while time.monotonic()<end:
    state,_=self.check()
    if not state.armed or state.mode!='OFFBOARD':raise RuntimeError('left armed OFFBOARD')
    with self.lock:p=dict(self.prediction) if self.prediction else None
    if not p or not 0<=rospy.Time.now().to_sec()-p['observation_stamp']<=2:raise RuntimeError('model prediction stale')
    if not same_task(p,self.report,rospy.get_param('/vla_direct_bridge/active_instruction',''),rospy.get_param('/vla_direct_bridge/run_id','')):raise RuntimeError('task changed')
    c,entry=self.bounded_command(p);c.header.stamp=rospy.Time.now();self.pub.publish(c)
    sent.append((time.monotonic(),entry))
    if p['request_id'] not in used:
     used.add(p['request_id']);self.report['model_targets'].append(entry)
    time.sleep(.04)
   self.report['armed_command_duration_s']=time.monotonic()-start
   with self.lock:pts=[(t,m) for t,m in self.setpoints if start+.15<t<end]
   matched=0
   for t,m in pts:
    candidates=[e for ts,e in sent if t-.2<=ts<=t+.02]
    if m._connection_header.get('callerid')!='/uav1/uav_control_fsm':continue
    if any(max(abs(a-b) for a,b in zip(e['sent_xyz_m'],[m.position.x,m.position.y,m.position.z]))<1e-4 and abs(wrap(math.radians(e['sent_yaw_deg'])-m.yaw))<1e-4 for e in candidates):matched+=1
   self.report.update(armed_setpoints=len(pts),matched_model_setpoints=matched,distinct_model_predictions=len(used),model_target_transport_verified=bool(pts) and matched/len(pts)>.8)
   if not self.report['model_target_transport_verified']:raise RuntimeError('armed setpoints did not track bounded model commands')
   self.report['status']='passed_transport_pending_motor_observation';self.report['model_command_window_completed']=True
  except Exception as exc:
   self.report.update(status='failed',error=str(exc),failure_monotonic=time.monotonic(),failure_ros_s=rospy.Time.now().to_sec(),distinct_model_predictions=len(self.report.get('model_targets',[])))
   if self.report.get('armed_start_monotonic') is not None:
    self.report['armed_command_duration_s']=time.monotonic()-self.report['armed_start_monotonic']
   with self.lock:last_prediction=dict(self.prediction) if self.prediction else None
   if last_prediction:
    self.report['last_model_request_id']=last_prediction['request_id']
    self.report['last_model_observation_age_s']=rospy.Time.now().to_sec()-last_prediction['observation_stamp']
   self.report['model_command_window_completed']=False
  finally:
   try:
    self.report['stop_result']=bench_disarm(self.arm,self.force_stop,lambda:self.state[0].armed if self.state else None,propellers_removed_bench=rospy.get_param('~propellers_removed_bench',False))
   except Exception as exc:self.report['disarm_error']=str(exc)
   self.lease=False;self.lease_pub.publish(Bool(data=False));self.allow_armed=False
   self.report['final_armed']=self.state[0].armed if self.state else None
   if self.report['final_armed'] is not False:self.report.update(status='failed',error='disarm not confirmed')
   if self.report['final_armed'] is False:
    try:
     if self.report.get('model_target_transport_verified'):
      self.report.update(status='disarmed_holding_offboard');self.persist()
      until=self.report['offboard_start_monotonic']+30.
      while time.monotonic()<until:
       state,_=self.check()
       if state.armed or state.mode!='OFFBOARD':raise RuntimeError('post-test state/mode changed')
       time.sleep(.05)
      self.report['offboard_duration_s']=time.monotonic()-self.report['offboard_start_monotonic']
      self.report['status']='passed_transport_pending_motor_observation'
     self.mode(base_mode=0,custom_mode=original);time.sleep(.5)
     self.report['restored_mode']=self.state[0].mode
    except Exception as exc:self.report.update(status='failed',restore_error=str(exc))
    finally:
     self.arm(value=False);self.lease=False
     self.mode(base_mode=0,custom_mode=original);time.sleep(.5)
     self.report['final_armed']=self.state[0].armed
     self.report['restored_mode']=self.state[0].mode
   self.report.update(rc_output_samples=self.outputs,actuator_samples=self.actuators,esc_rpm_samples=self.rpms,landed_state_samples=self.landed_states,fcu_statustext=self.fcu_statustext)
   self.persist()
  print(json.dumps(self.report,ensure_ascii=False,indent=2),flush=True)
  return self.report['final_armed'] is False

if __name__=='__main__':
 raise SystemExit('Use test_motor_chain.sh --props-removed --fixed; this is an internal module.')
