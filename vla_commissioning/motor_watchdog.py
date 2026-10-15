#!/usr/bin/env python3
"""Independent one-shot disarm watchdog for the specifically authorized bench test."""
import json,time
from pathlib import Path
import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import State,RCIn
from mavros_msgs.srv import CommandBool,SetMode,CommandLong
from motor_stop import bench_disarm
rospy.init_node('vla_motor_watchdog')
state=[None];heartbeat=[0.];seen=[None];rc=[None,0.];initial_arm_switch=[None]
def on_rc(m):
 rc[:]=[m,time.monotonic()]
 if len(m.channels)>=9 and initial_arm_switch[0] is None:initial_arm_switch[0]=m.channels[8]>1500
def on_state(m):
 state[0]=m
 if m.armed and seen[0] is None:seen[0]=time.monotonic()
rospy.Subscriber('/uav1/mavros/state',State,on_state)
rospy.Subscriber('/vla/motor_bench_lease',Bool,lambda m:heartbeat.__setitem__(0,time.monotonic() if m.data else 0.))
rospy.Subscriber('/uav1/mavros/rc/in',RCIn,on_rc)
rospy.wait_for_message('/uav1/mavros/rc/in',RCIn,timeout=5)
rospy.wait_for_service('/uav1/mavros/cmd/arming',timeout=5)
arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool)
mode=rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
force_stop=rospy.ServiceProxy('/uav1/mavros/cmd/command',CommandLong)
ready=rospy.Publisher('/vla/motor_watchdog_ready',Bool,queue_size=1,latch=True)
ready.publish(Bool(data=True))
report={'armed_seen':False,'disarm_triggered':False};deadline=time.monotonic()+65
while time.monotonic()<deadline and not rospy.is_shutdown():
 s=state[0];now=time.monotonic()
 if seen[0] is not None:
  report['armed_seen']=True
  if s and not s.armed:
   time.sleep(.02)
   continue
  rc_bad=rc[0] is None or now-rc[1]>.6 or len(rc[0].channels)<9 or rc[0].channels[2]>=1100 or (rc[0].channels[8]>1500)!=initial_arm_switch[0]
  if now-seen[0]>8. or now-heartbeat[0]>.4 or rc_bad:
   report['disarm_triggered']=True
   report['rc_guard_triggered']=rc_bad
   try:report['stop_result']=bench_disarm(arm,force_stop,lambda:state[0].armed if state[0] else None,propellers_removed_bench=rospy.get_param('~propellers_removed_bench',False))
   except Exception as e:report['error']=str(e)
   if state[0] and not state[0].armed:
    try:mode(base_mode=0,custom_mode='MANUAL')
    except Exception:pass
   time.sleep(.2)
 time.sleep(.02)
report.update(final_armed=state[0].armed if state[0] else None,updated_unix=time.time())
Path(rospy.get_param('~report_path','/home/nvidia/agile/runtime/vla/watchdog_report.json')).write_text(json.dumps(report,indent=2))
