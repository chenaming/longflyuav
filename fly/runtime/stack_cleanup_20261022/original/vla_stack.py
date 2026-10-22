#!/usr/bin/env python3
"""Headless bench stack. Startup never requests OFFBOARD or arming."""
import fcntl, json, os, signal, socket, subprocess, sys, time, uuid
from pathlib import Path

BASE=Path('/home/nvidia/agile'); STATE=BASE/'runtime/vla'
STATE.mkdir(parents=True,exist_ok=True)

def read(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return {}

def write(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2));tmp.replace(path)

def owned(pid):
    try:return b'vla_stack.py\0daemon' in Path('/proc/%s/cmdline'%pid).read_bytes()
    except OSError:return False

def master():
    try:
        with socket.create_connection(('127.0.0.1',11311),timeout=.3):return True
    except OSError:return False

def main():
    command=sys.argv[1] if len(sys.argv)>1 else 'status'
    manifest=read(STATE/'status.json');pid=manifest.get('pid')
    if command=='start':
        if pid and owned(pid):print('Already running; PID',pid);return
        if master():raise SystemExit('An existing ROS master is active. Stop the previous launcher before starting this stack.')
        with (STATE/'supervisor.log').open('ab') as log:
            p=subprocess.Popen([sys.executable,__file__,'daemon'],stdout=log,stderr=log,stdin=subprocess.DEVNULL,start_new_session=True)
        time.sleep(1)
        if p.poll() is not None:raise SystemExit('Startup failed; see runtime/vla/supervisor.log')
        print('Started PID',p.pid,'; readiness: ./vla_status.sh')
    elif command=='status':
        manifest['supervisor_alive']=bool(pid and owned(pid));print(json.dumps(manifest,ensure_ascii=False,indent=2))
        if not manifest['supervisor_alive']:raise SystemExit(1)
    elif command=='stop':
        if not pid or not owned(pid):print('Supervisor is not running.');return
        # Live FCU state is mandatory before stopping setpoint/sensor services.
        import rospy
        from mavros_msgs.msg import State
        rospy.init_node('vla_stop_precheck',anonymous=True)
        s=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
        if not s.connected or s.armed:raise SystemExit('Stop refused: require connected/disarmed FCU.')
        os.kill(pid,signal.SIGTERM)
        end=time.monotonic()+25
        while owned(pid) and time.monotonic()<end:time.sleep(.2)
        if owned(pid):raise SystemExit('Stop still pending; inspect supervisor log.')
        print('Stopped owned stack. No arming/mode command was sent.')
    elif command=='daemon':daemon()
    else:raise SystemExit('Usage: vla_stack.py start|status|stop')

def daemon():
    lock=(STATE/'supervisor.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another supervisor is active.')
    if master():raise SystemExit('ROS master already active; refusing duplicate stack.')
    config=read(BASE/'vla_config.json')
    run_id='chair-'+uuid.uuid4().hex[:12];children={};logs={};last={};stopping=[False]
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
    commands={
      'master':['roscore'],
      'odin':['/bin/bash','-c','source /home/nvidia/agile/odin_ws/devel/setup.bash; exec roslaunch odin_ros_driver odin1_ros1.launch ns:=uav1'],
      'mavros':['roslaunch','uav_control_fsm','px4.launch','fcu_url:='+config['fcu_url']],
      'fsm':['roslaunch',str(BASE/'vla_bench.launch')],
      'relay':['python3','-u',str(BASE/'fly/relay.py')],
      'bridge':['python3',str(BASE/'vla_direct_bridge.py'),'_url:='+config['inference_url'],'_instruction:='+config['instruction'],'_run_id:='+run_id,'_request_id_start:=0','_enable_publish:=false','_preview_pair_alignment:=true','_http_timeout:=15.0','_max_observation_age:=2.0'],
      'monitor':['python3',str(BASE/'vla_monitor.py')],
      'evidence':['python3',str(BASE/'vla_report_publisher.py')]
    }
    def start(name):
        nonlocal run_id
        if name in children and children[name].poll() is None:return
        if time.monotonic()-last.get(name,0)<5:return
        last[name]=time.monotonic()
        if name=='bridge' and name in children:
            run_id='chair-'+uuid.uuid4().hex[:12]
            commands['bridge']=[('_run_id:='+run_id) if a.startswith('_run_id:=') else a for a in commands['bridge']]
        if name in logs:logs[name].close()
        logs[name]=(STATE/(name+'.log')).open('ab')
        children[name]=subprocess.Popen(commands[name],stdout=logs[name],stderr=logs[name],stdin=subprocess.DEVNULL,start_new_session=True)
    def stop(name):
        p=children.get(name)
        if not p or p.poll() is not None:return
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    start('master')
    deadline=time.monotonic()+15
    while not master():
        if time.monotonic()>deadline:raise SystemExit('ROS master failed.')
        time.sleep(.2)
    import rospy
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    rospy.init_node('vla_stack_supervisor',disable_signals=True)
    state=[None,0.];odom=[0.];image=[0.];pred=[{},0.]
    def on_state(m):state[:]=[m,time.monotonic()]
    def on_prediction(m):
        try:pred[:]=[json.loads(m.data),time.monotonic()]
        except ValueError:pass
    rospy.Subscriber('/uav1/mavros/state',State,on_state)
    rospy.Subscriber('/uav1/odin1/odometry',Odometry,lambda _:odom.__setitem__(0,time.monotonic()))
    rospy.Subscriber('/uav1/odin1/image/compressed',CompressedImage,lambda _:image.__setitem__(0,time.monotonic()))
    rospy.Subscriber('/vla/validated_prediction',String,on_prediction)
    try:
        while not stopping[0] and not rospy.is_shutdown():
            now=time.monotonic();s=state[0];fresh=now-state[1]<2
            start('odin');start('mavros');start('monitor');start('evidence');start('relay')
            marker=read(STATE/'motor_owner.json');pause=False
            if marker:
                try:
                    cmd=Path('/proc/%s/cmdline'%marker['pid']).read_bytes()
                    pause=b'vla_acceptance.py' in cmd and b'--motor' in cmd and time.time()-marker['created']<150
                except (OSError,KeyError):pass
            if pause and fresh and s and not s.armed:stop('fsm')
            if not pause and fresh and s and s.connected and not s.armed and now-odom[0]<1:start('fsm')
            if fresh and s and s.connected and now-odom[0]<1:start('bridge')
            p=pred[0]
            ready=bool(fresh and s and s.connected and now-odom[0]<1 and p.get('run_id')==run_id and now-pred[1]<2 and 0<=rospy.Time.now().to_sec()-p.get('observation_stamp',0)<2)
            write(STATE/'status.json',{'pid':os.getpid(),'run_id':run_id,'updated_unix':time.time(),'ready':ready,'instruction':config['instruction'],'inference_url':config['inference_url'],'fcu':{'connected':s.connected,'armed':s.armed,'mode':s.mode,'fresh':fresh} if s else None,'odom_age_s':now-odom[0],'validated_prediction_age_s':now-pred[1],'last_request_id':p.get('request_id'),'motor_test_owns_fsm':pause,'processes':{k:{'pid':v.pid,'running':v.poll() is None} for k,v in children.items()},'scope':'protected bench; physical navigation not verified'})
            time.sleep(1)
    finally:
        # The CLI verifies disarmed before signaling; no automatic restart or flight action.
        for name in reversed(list(commands)):stop(name)
        write(STATE/'status.json',{'pid':os.getpid(),'stopped':True,'updated_unix':time.time(),'run_id':run_id})

if __name__=='__main__':main()
