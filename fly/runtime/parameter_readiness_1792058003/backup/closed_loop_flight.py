#!/usr/bin/env python3
"""Explicit opt-in live mission. Default is a read-only preflight check."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import rosgraph
import rospy
from mavros_msgs.msg import State, ExtendedState, RCIn, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamGet
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from tf.transformations import euler_from_quaternion
from flight_core import Config, Mission, norm

BASE=Path(__file__).resolve().parent
FSM='/uav1/uav_control_fsm'
TOPIC='/uav1/mavros/setpoint_raw/local'


class Runtime:
    def __init__(self, args):
        self.args=args
        self.lock=threading.RLock();self.data={};self.stop=False
        self.publisher=None;self.owner=False;self.sp=None;self.policy_at=0.
        self.last_publish=0.;self.publish_count=0
        self.rpc_busy=False;self.rpc_finished=0.;self.rpc_log=[]
        self.events=[];self.mission=None;self.closed=False
        self.control_released=False;self.offboard_seen=False;self.release_mode=None
        self.initial_mode=None;self.rc_reference=None
        self.cfg=Config(allow_paired=args.allow_paired_estimate)
        rospy.init_node('vla_closed_loop_flight',disable_signals=True)
        self.master=rosgraph.Master(rospy.get_name())
        for key,topic,typ in (
            ('state','/uav1/mavros/state',State),
            ('extended','/uav1/mavros/extended_state',ExtendedState),
            ('rc','/uav1/mavros/rc/in',RCIn),
            ('odom','/uav1/mavros/local_position/odom',Odometry),
            ('prediction','/vla/validated_prediction',String)):
            rospy.Subscriber(topic,typ,lambda m,k=key:self.receive(k,m),queue_size=1)
        self.mode=rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
        self.arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool)
        self.ownership=rospy.ServiceProxy(FSM+'/external_setpoint_control',CommandBool)
        self.status=rospy.Publisher('/vla/flight_status',String,queue_size=1,latch=True)
        self.task_pub=rospy.Publisher('/vla/instruction',String,queue_size=1)
        self.reset_pub=rospy.Publisher('/vla/reset_origin',Empty,queue_size=1)
        signal.signal(signal.SIGINT,lambda *_:setattr(self,'stop',True))
        signal.signal(signal.SIGTERM,lambda *_:setattr(self,'stop',True))
        threading.Thread(target=self.stream,daemon=True).start()

    def receive(self,key,message):
        if key=='prediction':
            try: message=json.loads(message.data)
            except (ValueError,TypeError): return
        with self.lock:
            self.data[key]=(message,time.monotonic())  # Overwrite, never queue results.
            if key=='state' and self.owner:
                if message.mode=='OFFBOARD':self.offboard_seen=True
                elif self.offboard_seen and self.mission is not None:
                    own_landing=(message.mode=='AUTO.LAND' and
                        self.mission.phase in ('LAND_REQUEST','LANDING','HOLD_LAND_FAILED'))
                    if not own_landing and not self.control_released:
                        self.control_released=True;self.release_mode=message.mode;self.sp=None
                        self.events.append({'t':time.monotonic(),'event':'pilot_or_px4_mode_takeover',
                                            'mode':message.mode})

    def snapshot(self):
        with self.lock: return dict(self.data)

    def telemetry(self,data):
        now=time.monotonic();ros_now=rospy.Time.now().to_sec()
        s,st=data.get('state',(None,0));o,ot=data.get('odom',(None,0))
        e,et=data.get('extended',(None,0));r,rt=data.get('rc',(None,0))
        state={'fresh':bool(s and now-st<1.5),'connected':bool(s and s.connected),
               'armed':bool(s and s.armed),'mode':s.mode if s else '',
               'landed':bool(e and now-et<2 and e.landed_state==ExtendedState.LANDED_STATE_ON_GROUND)}
        fault=None
        if not state['fresh'] or not state['connected']:fault='FCU heartbeat stale/disconnected'
        if o is None:raise RuntimeError('尚未收到 MAVROS 里程计')
        p,q=o.pose.pose.position,o.pose.pose.orientation
        quat=[q.x,q.y,q.z,q.w]
        if not all(math.isfinite(x) for x in quat) or norm(quat)<.5:
            raise RuntimeError('invalid odometry quaternion')
        quat=[x/norm(quat) for x in quat]
        pose=[p.x,p.y,p.z,euler_from_quaternion(quat)[2]]
        v=o.twist.twist.linear;w=o.twist.twist.angular
        speed=norm([v.x,v.y,v.z]);yaw_rate=w.z
        if not all(math.isfinite(x) for x in pose+[speed,yaw_rate]):raise RuntimeError('nonfinite odometry')
        if now-ot>.5 or not 0<=ros_now-o.header.stamp.to_sec()<=.5 or o.header.frame_id!='map':
            fault='odometry stale or frame changed'
        if r is None or now-rt>.75 or len(r.channels)<9 or r.rssi==0:
            fault='RC signal unavailable'
        # RC stick/switch values are not flight-mode authority. PX4's actual
        # State.mode selects takeover; arm/kill switches remain owned by PX4.
        if ros_now<1700000000:fault='NX wall clock is not synchronized'
        return state,pose,speed,yaw_rate,fault

    def preflight(self):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not self.stop:
            d=self.snapshot()
            if all(k in d for k in ('state','odom','rc','extended','prediction')):break
            time.sleep(.05)
        d=self.snapshot();s,p,v,w,fault=self.telemetry(d)
        if fault:raise RuntimeError(fault)
        if s['armed'] or s['mode'] not in ('MANUAL','POSCTL') or not s['landed']:
            raise RuntimeError('要求已连接、未解锁、在地面，模式 MANUAL 或 POSCTL')
        if d['state'][0].system_status!=3:raise RuntimeError('飞控尚未 Standby')
        rc=list(d['rc'][0].channels)
        if v>.08 or abs(w)>math.radians(3):raise RuntimeError('起飞前机体需要保持静止')
        if rospy.get_param(FSM+'/bench_disarmed',None) is not True or rospy.get_param(FSM+'/bench_allow_armed',None) is not False:
            raise RuntimeError('需要默认台架控制器作为定位转发和独占控制权交接端')
        if rospy.get_param('/vla_direct_bridge/enable_publish',True):raise RuntimeError('桥接必须保持预览，避免两个目标发布者')
        if rospy.get_param(FSM+'/external_setpoint_control_active',False):raise RuntimeError('已有外部控制权，请先处理前一任务')
        pubs=dict(self.master.getSystemState()[0])
        if pubs.get('/uav1/move_command'):raise RuntimeError('已有模型/电机台架测试发布控制目标')
        if pubs.get(TOPIC)!=[FSM]:raise RuntimeError('setpoint 发布者不是单份原控制器: '+str(pubs.get(TOPIC)))
        rospy.wait_for_service(FSM+'/external_setpoint_control',timeout=3)
        prediction,received=d.get('prediction',({},0))
        if time.monotonic()-received>2 or not prediction.get('latest_only'):
            raise RuntimeError('缺少最新单帧桥接的有效结果')
        if prediction.get('coordinate_mode')=='paired_estimate' and not self.args.allow_paired_estimate:
            raise RuntimeError('当前仍为配对坐标估算；只有明确接受这一限制时才添加 --allow-paired-estimate')
        # Read failsafes; do not weaken them or change any PX4 parameter.
        get=rospy.ServiceProxy('/uav1/mavros/param/get',ParamGet)
        failsafes={}
        for name in ('COM_OF_LOSS_T','COM_OBL_ACT','COM_OBL_RC_ACT','COM_RC_OVERRIDE',
                     'RC_MAP_FLTMODE','RC_MAP_ARM_SW','RC_MAP_KILL_SW'):
            reply=get(param_id=name)
            if not reply.success:raise RuntimeError('无法读取飞控失联策略 '+name)
            failsafes[name]={'integer':reply.value.integer,'real':reply.value.real}
        loss=failsafes['COM_OF_LOSS_T'];loss_s=loss['real'] or loss['integer']
        if not 0<=loss_s<=2:raise RuntimeError('COM_OF_LOSS_T 需要核实，当前超过 2 秒或非法')
        if failsafes['COM_OBL_RC_ACT']['integer'] not in (0,3,4):
            raise RuntimeError('当前 Offboard 失联策略不是 Position/Return/Land，需要先核实')
        if failsafes['COM_OBL_ACT']['integer'] not in (0,1,2):raise RuntimeError('未知 Offboard 无遥控失联策略')
        if failsafes['COM_RC_OVERRIDE']['integer'] & 2:
            raise RuntimeError('PX4 已启用 OFFBOARD 摇杆接管，和仅模式开关接管的要求不符；未修改参数')
        self.initial_mode=s['mode'];self.rc_reference=rc
        print('只读检查通过：未解锁、地面静止、遥控器有效、单份控制器、最新图像推理。',flush=True)
        print('PX4 失联参数（未修改）：'+json.dumps(failsafes),flush=True)
        return p

    def claim(self):
        # Called only while disarmed after the first valid post-start prediction.
        s,_,_,_,fault=self.telemetry(self.snapshot())
        if fault or s['armed']:raise RuntimeError('交接前状态变化: '+str(fault))
        response=self.ownership(value=True)
        if not response.success:raise RuntimeError('控制器拒绝交接 setpoint 发布权')
        self.owner=True
        if not rospy.get_param(FSM+'/external_setpoint_control_active',False):raise RuntimeError('交接未确认')
        time.sleep(.15)  # Let a previously queued bench hold drain before creating our publisher.
        self.publisher=rospy.Publisher(TOPIC,PositionTarget,queue_size=1)
        self.events.append({'t':time.monotonic(),'event':'exclusive_setpoint_handoff'})

    def stream(self):
        deadline=time.monotonic()
        while not self.closed:
            now=time.monotonic()
            with self.lock:
                sp=list(self.sp) if self.sp is not None else None
                age=now-self.policy_at
                publisher=self.publisher
                released=self.control_released
            # A wedged policy/ROS call must not keep an obsolete destination alive forever.
            # Stop the stream and let configured PX4 Offboard-loss protection handle a crash.
            if publisher is not None and sp is not None and age<.30 and not released:
                msg=PositionTarget();msg.header.stamp=rospy.Time.now();msg.header.frame_id='map'
                msg.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                msg.type_mask=(PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|
                    PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE)
                msg.position.x,msg.position.y,msg.position.z=sp[:3];msg.yaw=sp[3]
                try:
                    publisher.publish(msg)
                    self.last_publish=now;self.publish_count+=1
                except Exception:pass
            deadline+=1/30
            time.sleep(max(0,min(1/30,deadline-time.monotonic())))
            if time.monotonic()-deadline>.1:deadline=time.monotonic()

    def submit(self, action):
        if action is None or self.rpc_busy:return
        if self.control_released:return
        if action==('arm',True) and (self.stop or self.mission.phase!='ARM'):return
        if action in (('arm',True),('mode','OFFBOARD')) and (time.monotonic()-self.last_publish>.15 or self.publish_count<50):
            self.mission.transition('ABORTED',time.monotonic(),'setpoint stream not established')
            return
        self.rpc_busy=True
        def call():
            result={'t':time.monotonic(),'action':action}
            try:
                # A flight-mode callback can arrive after submit but before the
                # worker is scheduled. Never issue late autonomy RPCs to a pilot.
                if self.control_released or (action==('arm',True) and self.stop):
                    result['skipped']='control released or stopped';return
                if action==('arm',False):
                    # Recheck a possibly changed ground observation just before
                    # an ordinary disarm RPC; no RC-low condition is added.
                    latest=self.snapshot();s,st=latest.get('state',(None,0));e,et=latest.get('extended',(None,0))
                    now=time.monotonic()
                    ground=bool(e and now-et<.75 and e.landed_state==ExtendedState.LANDED_STATE_ON_GROUND)
                    if (not s or not s.connected or now-st>=1.5 or s.mode!='AUTO.LAND' or not ground):
                        result['skipped']='fresh ground in AUTO.LAND not confirmed';return
                # No force-arm and no force-disarm service exists in this program.
                response=self.mode(base_mode=0,custom_mode=action[1]) if action[0]=='mode' else self.arm(value=action[1])
                result['response']=str(response)
            except Exception as exc:result['error']=str(exc)
            finally:
                with self.lock:
                    self.rpc_log.append(result);self.rpc_finished=time.monotonic();self.rpc_busy=False
        threading.Thread(target=call,daemon=True).start()

    def report(self, state):
        m=self.mission
        data={'phase':m.phase,'reason':m.reason,'instruction':m.instruction,
              'arrival_inferred':m.arrival_inferred,'semantic_goal_verified':False,
              'physical_flight_validated':False,'run_and_epoch':m.identity,
              'last_request_id':m.last_request,'accepted_predictions':m.accepted_predictions,
              'rejected_predictions':m.rejected_predictions,'small_action_count':len(m.small),
              'home_xyz_m':m.home,'setpoint_xyz_yaw':m.sp,'target_xyz_yaw':m.target,
              'fcu':state,'setpoint_count':self.publish_count,'rpc':self.rpc_log,
              'events':self.events,'updated_unix':time.time()}
        data.update(rc_takeover_policy='PX4_actual_mode_only',release_mode=self.release_mode,
                    control_released=self.control_released)
        raw=json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)
        self.status.publish(String(data=raw))
        path=BASE/'runtime'/'flight_report.json';tmp=path.with_suffix('.tmp')
        tmp.write_text(raw,encoding='utf-8');tmp.replace(path)

    def run(self):
        pose=self.preflight()
        if not self.args.execute:
            print('本次没有发布控制目标、切换模式或解锁。',flush=True);return
        print('执行正式飞行流程：相对起飞点上升 1 m，实时推理导航，启发式到达后 AUTO.LAND。',flush=True)
        print('Ctrl+C 请求降落；遥控器切出 OFFBOARD 后不会自动重新进入。',flush=True)
        baseline=self.snapshot().get('prediction',({},0))[0]
        baseline_run=baseline.get('run_id');baseline_epoch=baseline.get('epoch',-1)
        self.task_pub.publish(String(data=self.args.instruction));self.reset_pub.publish(Empty())
        start=time.monotonic()
        self.mission=Mission(self.cfg,self.args.instruction,start,rospy.Time.now().to_sec(),pose)
        last_phase=None;last_report=0.;last_audit=0.;fault_latch=None
        while not rospy.is_shutdown():
            now=time.monotonic();d=self.snapshot()
            try:s,p,v,w,fault=self.telemetry(d)
            except Exception as exc:
                if self.mission is None:raise
                s={'fresh':False,'connected':False,'armed':bool(d.get('state',(None,))[0] and d['state'][0].armed),'mode':d['state'][0].mode if 'state' in d else '', 'landed':False}
                p=self.mission.sp;v=0.;w=0.;fault=str(exc)
            if fault_latch:fault=fault_latch
            if self.owner and now-last_audit>=1:
                pubs=dict(self.master.getSystemState()[0]);allowed={FSM,rospy.get_name()}
                if set(pubs.get(TOPIC,[]))-allowed or pubs.get('/uav1/move_command'):
                    fault_latch='another control publisher appeared'
                if not rospy.get_param(FSM+'/external_setpoint_control_active',False):
                    fault_latch='controller ownership lost'
                active=(rospy.get_param('/vla_direct_bridge/run_id',''),rospy.get_param('/vla_direct_bridge/active_instruction',''))
                if self.mission.identity and active!=(self.mission.identity[0],self.args.instruction):
                    fault_latch='active instruction/bridge run changed'
                last_audit=now
            if self.control_released:
                self.stop=True;sp=None
                self.mission.transition('TAKEOVER',now,'PX4 confirmed '+str(self.release_mode)+'; no automatic re-entry or disarm')
            else:
                pred=d.get('prediction',({},0))[0]
                if self.mission.identity is None and pred.get('run_id')==baseline_run and pred.get('epoch',-1)<=baseline_epoch:
                    pred=None  # Wait for acknowledgement of a new observation origin.
                elif self.mission.identity and pred:
                    identity=(pred.get('run_id'),pred.get('epoch'),pred.get('instruction'),pred.get('target_frame'))
                    if identity!=self.mission.identity:
                        fault_latch=fault='model task/reference identity changed'
                sp,action=self.mission.step(now,rospy.Time.now().to_sec(),p,v,w,s,pred,fault,self.stop)
                if self.mission.phase=='PRESTREAM' and not self.owner:
                    self.claim();self.mission.entered=time.monotonic()
                self.submit(action)
            with self.lock:self.sp=sp if self.owner else None;self.policy_at=time.monotonic()
            if self.mission.phase!=last_phase:
                print(self.mission.phase+': '+self.mission.reason,flush=True)
                self.events.append({'t':now,'phase':self.mission.phase,'reason':self.mission.reason})
                last_phase=self.mission.phase
            if now-last_report>=1:
                self.report(s);last_report=now
            if self.mission.phase in ('DONE','ABORTED','TAKEOVER'):
                # Drain a normal arm RPC and observe FCU afterwards; never exit on ACK alone.
                if self.rpc_busy or (self.mission.arm_requested and now-self.rpc_finished<2):
                    time.sleep(.03);continue
                if self.mission.phase=='ABORTED' and self.mission.arm_requested and s['armed'] and s['mode']=='OFFBOARD':
                    self.mission.transition('LAND_REQUEST',now,'late arm observed after abort');continue
                self.report(s);break
            time.sleep(.03)

    def cleanup(self):
        # Never use disarm as an airborne shutdown method.
        self.sp=None;self.closed=True
        if self.publisher:self.publisher.unregister()
        if self.owner:
            d=self.snapshot();s,t=d.get('state',(None,0))
            if s and time.monotonic()-t<1.5 and s.connected and not s.armed and not self.rpc_busy:
                if s.mode=='OFFBOARD' and not self.control_released:
                    try:self.mode(base_mode=0,custom_mode=self.initial_mode)
                    except Exception:pass
                try:
                    result=self.ownership(value=False)
                    print('原控制器发布权恢复：'+str(result.success),flush=True)
                except Exception as exc:print('交接释放失败：'+str(exc),flush=True)
            else:
                print('尚未确认未解锁，保留外部交接状态；飞控/遥控器接管，不强制停机。',flush=True)

    def abort_and_land(self, reason):
        """Drain any in-flight arm call, hold, and hand an airborne failure to PX4 LAND."""
        if not self.owner or self.control_released:return
        self.stop=True
        print('异常恢复：等待飞控真实状态；不执行强制上锁。原因：'+reason,flush=True)
        attempted=-1e9
        while not rospy.is_shutdown():
            if self.control_released:return
            now=time.monotonic();d=self.snapshot();state,received=d.get('state',(None,0))
            fresh=state is not None and now-received<1.5 and state.connected
            try:_,pose,_,_,_=self.telemetry(d)
            except Exception:pose=self.mission.sp if self.mission else None
            if fresh and not state.armed and not self.rpc_busy and now-self.rpc_finished>=2:
                return
            if fresh and state.mode!='OFFBOARD' and not self.rpc_busy and now-self.rpc_finished>=2:
                return  # AUTO.LAND, pilot mode, or another PX4 failsafe owns the aircraft.
            with self.lock:
                self.sp=list(pose) if pose is not None else None
                self.policy_at=now
            if fresh and state.armed and state.mode=='OFFBOARD' and now-attempted>=1:
                self.submit(('mode','AUTO.LAND'));attempted=now
            time.sleep(.05)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true',help='执行真实起飞/导航/降落；默认只读检查')
    mode.add_argument('--check-only',action='store_true')
    parser.add_argument('--instruction',default='fly to the chair')
    parser.add_argument('--allow-paired-estimate',action='store_true',help='明确接受当前未完整标定的配对坐标估算')
    args=parser.parse_args()
    if not args.instruction.strip():parser.error('instruction must not be empty')
    (BASE/'runtime').mkdir(exist_ok=True)
    with (BASE/'runtime'/'closed_loop_flight.lock').open('w') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有闭环飞行进程')
        runtime=Runtime(args)
        try:runtime.run()
        except Exception as exc:
            print('流程停止：'+str(exc),flush=True)
            runtime.abort_and_land(str(exc))
            raise
        finally:runtime.cleanup()


if __name__=='__main__':main()
