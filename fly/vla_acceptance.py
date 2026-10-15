#!/usr/bin/env python3
"""Finite validation. Motor mode requires explicit propeller-removed/fixed flags."""
import argparse,fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path
BASE=Path('/home/nvidia/agile');RUNTIME=BASE/'runtime/vla'
sys.path.insert(0,str(BASE/'vla_commissioning'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy
from mavros_msgs.msg import State,RCIn
from mavros_msgs.srv import CommandBool,CommandLong,SetMode,MessageInterval
from motor_stop import bench_disarm

def atomic(path,data):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2));tmp.replace(path)

def stage(name):
 rospy.init_node('vla_'+name+'_validation')
 if name=='diagnostic':
  from bench_validate import Bench
  test=Bench()
 elif name=='model':
  from model_bench_validate import ModelBench
  test=ModelBench()
 else:
  from model_motor_validate import MotorBench
  rospy.set_param('~propellers_removed_bench',True)
  test=MotorBench()
  test.report['session_token']=os.environ.get('FLY_MOTOR_STAGE_TOKEN','')
 test.path=RUNTIME/(name+'_report.json')
 time.sleep(1)
 ok=test.run()
 if name=='motor':
  test.report['normal_cleanup_passed']=bool(test.report.get('stop_result',{}).get('disarmed_confirmed')) and not test.report.get('stop_result',{}).get('force_disarm_used',False) and not test.report.get('stop_result',{}).get('normal_error') and bool((test.report.get('stop_result',{}).get('normal_response') or {}).get('success'))
  test.persist()
 if name=='motor':ok=test.report.get('model_target_transport_verified') and test.report.get('final_armed') is False and test.report.get('status')=='passed_transport_pending_motor_observation' and test.report.get('normal_cleanup_passed') is True
 raise SystemExit(0 if ok else 1)

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--motor',action='store_true');parser.add_argument('--props-removed',action='store_true');parser.add_argument('--fixed',action='store_true');parser.add_argument('--stage',choices=['diagnostic','model','motor']);a=parser.parse_args()
 if a.stage:
  marker_path=RUNTIME/'motor_owner.json'
  marker_data=json.loads(marker_path.read_text()) if marker_path.exists() else {}
  token=os.environ.get('FLY_MOTOR_STAGE_TOKEN','')
  if a.stage!='motor' or not token or marker_data.get('stage_token')!=token or marker_data.get('pid')!=os.getppid():
   raise SystemExit('Internal stage requires the guarded parent. Use test_motor_chain.sh --props-removed --fixed.')
  stage(a.stage)
 if not a.motor:raise SystemExit('This fly copy only runs the explicit motor bench.')
 if a.motor and not (a.props_removed and a.fixed):raise SystemExit('Motor bench requires --props-removed --fixed. Never use with propellers attached.')
 lock=(RUNTIME/'test.lock').open('w')
 try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit('Another validation is running.')
 rospy.init_node('vla_acceptance_supervisor')
 states=[None,0.]
 rospy.Subscriber('/uav1/mavros/state',State,lambda s:states.__setitem__(slice(None),[s,time.monotonic()]))
 def current():
  if states[0] is None or time.monotonic()-states[1]>2:raise RuntimeError('FCU state stale')
  return states[0]
 deadline=time.monotonic()+20
 while states[0] is None or time.monotonic()-states[1]>2:
  if time.monotonic()>deadline:raise RuntimeError("FCU state did not become fresh within 20s")
  time.sleep(.1)
 s=current()
 assert s.connected and not s.armed and (s.mode=='MANUAL' or (a.motor and s.mode=='OFFBOARD')),'Require connected, disarmed MANUAL (motor bench also accepts OFFBOARD)'
 assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed',False) is True
 assert rospy.get_param('/vla_direct_bridge/enable_publish',True) is False
 deadline=time.monotonic()+30
 while True:
  health=json.loads((RUNTIME/'status.json').read_text())
  if health.get('ready') and time.time()-health.get('updated_unix',0)<3:break
  if time.monotonic()>deadline:raise RuntimeError('No fresh validated prediction within 30s; model/network/guards are not ready')
  time.sleep(.2)
 if not a.motor:
  subprocess.run(['python3',str(BASE/'vla_commissioning/test_coordinates.py')],check=True)
  subprocess.run(['python3',str(BASE/'vla_commissioning/test_motor_stop.py')],check=True)
  atomic(RUNTIME/'acceptance.json',{'run_id':health['run_id'],'status':'running','updated_unix':time.time()})
  for name in ['diagnostic','model']:
   try:
    with (RUNTIME/(name+'_test.log')).open('ab') as log:
     subprocess.run(['python3',__file__,'--stage',name],check=True,timeout=100,stdout=log,stderr=log)
   except (subprocess.CalledProcessError,subprocess.TimeoutExpired) as exc:
    atomic(RUNTIME/'acceptance.json',{'run_id':health['run_id'],'status':'failed','stage':name,'error':str(exc),'updated_unix':time.time()})
    raise
   print(name+': passed',flush=True)
  atomic(RUNTIME/'acceptance.json',{'run_id':health['run_id'],'status':'passed_disarmed_bench','updated_unix':time.time(),'physical_navigation_verified':False})
  print('PASS: coordinates, stop helper, disarmed guards and eight real model targets. No arming was requested.')
  return
 rc=rospy.wait_for_message('/uav1/mavros/rc/in',RCIn,timeout=5)
 if not (len(rc.channels)>=9 and rc.channels[8]<1300 and rc.channels[2]<1100):
  atomic(RUNTIME/'motor_report.json',{'run_id':health['run_id'],'instruction':health['instruction'],'status':'blocked_preflight','error':'Requires CH9 low and throttle low','rc_channels':list(rc.channels),'armed_ever':False,'final_armed':current().armed,'restored_mode':current().mode,'updated_unix':time.time(),'physical_navigation_verified':False})
  raise SystemExit('Motor bench requires CH9 low and throttle low; no arming requested')
 atomic(RUNTIME/'motor_rc_context.json',{'rc_arm_switch_override':False,'normal_mavros_arming':True,'initial_rc_channels':list(rc.channels),'updated_unix':time.time(),'px4_parameters_changed':False})
 interval=rospy.ServiceProxy('/uav1/mavros/set_message_interval',MessageInterval)
 for msg_id in (140,36,291):
  try:interval(message_id=msg_id,message_rate=20.)
  except rospy.ServiceException:pass
 processes=[];logs=[];marker=RUNTIME/'motor_owner.json';ok=False
 def start(args,name):
  log=(RUNTIME/(name+'.log')).open('ab');logs.append(log)
  p=subprocess.Popen(args,stdout=log,stderr=log,start_new_session=True);processes.append(p);return p
 try:
  import uuid
  os.environ['FLY_MOTOR_STAGE_TOKEN']=uuid.uuid4().hex
  atomic(marker,{'pid':os.getpid(),'created':time.time(),'stage_token':os.environ['FLY_MOTOR_STAGE_TOKEN']})
  deadline=time.monotonic()+15
  while True:
   h=json.loads((RUNTIME/'status.json').read_text())
   if h.get('motor_test_owns_fsm') and not h.get('processes',{}).get('fsm',{}).get('running'):break
   if time.monotonic()>deadline:raise RuntimeError('Protected FSM did not yield ownership')
   time.sleep(.2)
  fsm=start([str(BASE/'agile_ws/devel/lib/uav_control_fsm/uav_control_fsm_node'),'__ns:=/uav1','__name:=uav_control_fsm','_bench_disarmed:=true','_bench_allow_armed:=true','fast_lio_odometry:=odin1/odometry'],'motor_fsm')
  start(['python3',str(Path(__file__).resolve().parent/'motor_watchdog.py'),'_propellers_removed_bench:=true','_session_token:='+os.environ['FLY_MOTOR_STAGE_TOKEN'],'_expected_run_id:='+rospy.get_param('/vla_direct_bridge/run_id'),'_expected_instruction:='+rospy.get_param('/vla_direct_bridge/active_instruction')],'motor_watchdog')
  time.sleep(3)
  assert fsm.poll() is None,'Motor bench FSM exited'
  print('Starting finite five-second real model motor bench.',flush=True)
  result=subprocess.run(['python3',__file__,'--stage','motor'],timeout=100)
  ok=result.returncode==0
 finally:
  # Keep both FSM and independent watchdog alive until disarmed is confirmed.
  arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool)
  force=rospy.ServiceProxy('/uav1/mavros/cmd/command',CommandLong)
  stopped=bench_disarm(arm,force,lambda:current().armed,propellers_removed_bench=True)
  atomic(RUNTIME/'motor_wrapper_cleanup.json',stopped)
  if not stopped.get('disarmed_confirmed'):
   raise RuntimeError('DISARM NOT CONFIRMED; watchdog and controller left running. Use transmitter to stop.')
  try:
   rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)(base_mode=0,custom_mode='MANUAL');time.sleep(1)
  except Exception as exc:
   stopped['mode_restore_error']=str(exc);atomic(RUNTIME/'motor_wrapper_cleanup.json',stopped);ok=False
  for p in reversed(processes):
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in processes:
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
  marker.unlink(missing_ok=True)
  for log in logs:log.close()
  deadline=time.monotonic()+12
  while time.monotonic()<deadline:
   if rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed',True) is False:break
   time.sleep(.2)
  assert current().armed is False and current().mode=='MANUAL','Final disarmed MANUAL not confirmed'
  assert rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed',True) is False,'Default protected FSM not restored'
 if not ok:raise SystemExit('Motor transport acceptance failed; inspect runtime/vla/motor_report.json')
 print('PASS: model-to-controller motor transport and final disarm. Visual motor rotation still requires human observation.')

if __name__=='__main__':main()
