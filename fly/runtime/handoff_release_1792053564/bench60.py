#!/usr/bin/env python3
"""Explicit 30s ground-only motor/phase rehearsal. No one-metre climb command."""
import argparse,bisect,fcntl,json,math,os,subprocess,time,uuid
from pathlib import Path
from types import SimpleNamespace
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandLong
from closed_loop_flight import Runtime,FSM,TOPIC
from motor_stop import bench_disarm
from bench60_policy import Bench60,wrap
from bench_shutdown import NormalShutdown
from ownership_release import release_if_safe

BASE=Path(__file__).resolve().parent


class TimedBench(Runtime):
    def receive(self,key,message):
        if key=='rc' and hasattr(self,'rc_history'):
            row={'t':time.monotonic(),'channels':list(message.channels),'rssi':message.rssi}
            previous=self.rc_history[-1] if self.rc_history else None
            if previous is None or row['channels']!=previous['channels'] or row['t']-previous['t']>.5:
                self.rc_history.append(row)
                if len(self.rc_history)>3000:self.rc_history.pop(0)
        super().receive(key,message)

    def __init__(self):
        self.rc_history=[]
        super().__init__(SimpleNamespace(execute=True,allow_paired_estimate=True,instruction=''))
        self.token=uuid.uuid4().hex;self.done=False;self.stopping=False
        self.pilot_takeover=False
        self.normal_cleanup=False;self.emergency_stop=False;self.cleanup_started_at=None
        self.watchdog=None;self.ready_token=None;self.points=[];self.last_pulse=0.
        self.lease_pub=rospy.Publisher('/vla/bench60/lease',String,queue_size=1,latch=True)
        rospy.Subscriber('/vla/bench60/watchdog_ready',String,lambda m:setattr(self,'ready_token',m.data),queue_size=1)
        rospy.Subscriber(TOPIC,PositionTarget,self.record_point,queue_size=100)
        self.report_data={'kind':'30s_ground_timed_motor_bench','token':self.token,'duration_s':30.,
            'started_unix':time.time(),'status':'preparing','height_1m_verified':False,
            'takeoff_is_simulated':True,'hover_is_simulated':True,
            'real_flight_entry_exercised':False,'physical_navigation_verified':False,
            'arrival_verified':False,'motor_rotation_observed_by_user':False,
            'limits':{'xy_m':.02,'z_m':.003,'yaw_deg':1},'events':self.events}

    def record_point(self,m):
        if self.owner:
            self.points.append({'t':time.monotonic(),'xyz_yaw':[m.position.x,m.position.y,m.position.z,m.yaw],
                                'source':m._connection_header.get('callerid')})

    def lease(self):
        self.lease_pub.publish(String(data=json.dumps({'token':self.token,'stop':self.emergency_stop,
            'normal_cleanup':self.normal_cleanup,'cleanup_started_at':self.cleanup_started_at,
            'done':self.done,'rpc_busy':self.rpc_busy,'pilot_takeover':self.pilot_takeover})))

    def pulse(self,check=True):
        if self.stop and not self.stopping:raise RuntimeError('现场请求停止')
        data=self.snapshot();s,p,_,_,fault=self.telemetry(data)
        if check and fault:
            r,received=data.get('rc',(None,0))
            self.report_data['fault_snapshot']={'fault':fault,'t':time.monotonic(),
                'rc_channels':list(r.channels) if r else None,'rc_age_s':time.monotonic()-received,
                'rc_reference':self.rc_reference,'state':s}
            raise RuntimeError(fault)
        if check and self.watchdog is not None and self.watchdog.poll() is not None:
            raise RuntimeError('独立看门狗已退出')
        if self.owner:
            with self.lock:self.policy_at=time.monotonic()
        self.lease();self.last_pulse=time.monotonic()
        return data,s,p

    def wait(self,condition,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not rospy.is_shutdown():
            data,s,p=self.pulse()
            if condition(data,s,p):return data,s,p
            time.sleep(.025)
        raise RuntimeError('等待状态超时')

    def prepare_stop(self):
        data=self.snapshot();s,t=data.get('state',(None,0));r,rt=data.get('rc',(None,0))
        if not r or time.monotonic()-rt>.75 or len(r.channels)<9 or r.channels[2]>=1100 or r.channels[8]>=1300:
            return {'mode_sent':False,'reason':'RC not fresh and low'}
        if not s or not s.connected or time.monotonic()-t>1.5:return {'mode_sent':False,'reason':'state stale'}
        return {'mode_sent':self.mode(base_mode=0,custom_mode='MANUAL').mode_sent,
                'reason':'explicit fixed propeller-removed emergency bench shutdown'}

    def actual_armed(self):
        s,t=self.snapshot().get('state',(None,0))
        return s.armed if s and s.connected and time.monotonic()-t<1.5 else None

    def save(self):
        self.report_data['rc_history']=list(self.rc_history)
        self.report_data['updated_unix']=time.time()
        p=BASE/'runtime'/'bench30_report.json';tmp=p.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.report_data,ensure_ascii=False,indent=2,allow_nan=False));tmp.replace(p)

    def run_bench(self):
        home=self.preflight()
        self.report_data['initial_rc_channels']=list(self.rc_reference)
        p=self.snapshot()['prediction'][0]
        self.args.instruction=p['instruction']
        identity=(p['run_id'],p['epoch'],p['instruction'])
        plan=Bench60(home,identity);self.plan=plan
        plan.accept(p,time.monotonic(),rospy.Time.now().to_sec())
        self.report_data.update(instruction=p['instruction'],identity=identity,home_xyz_yaw=home)
        self.claim();self.sp=list(home);self.policy_at=time.monotonic()
        self.mission=SimpleNamespace(phase='ARM')  # Reuse the guarded single RPC dispatcher.
        self.mission.transition=lambda phase,*_:setattr(self.mission,'phase',phase)
        log=(BASE/'runtime'/'bench60_watchdog.log').open('ab')
        self.watchdog_log=log
        self.watchdog=subprocess.Popen(['python3',str(BASE/'bench60_watchdog.py'),
            '_token:='+self.token,'_parent_pid:='+str(os.getpid())],stdout=log,stderr=log,start_new_session=True)
        self.wait(lambda *_:self.ready_token==self.token,6)
        begin=time.monotonic()
        self.wait(lambda *_:time.monotonic()-begin>=2.2 and self.publish_count>=60,5)
        self.submit(('mode','OFFBOARD'))
        self.wait(lambda d,s,p:s['mode']=='OFFBOARD' and not self.rpc_busy,5)
        # Recheck live prediction/RC immediately before the single normal arm RPC.
        d,s,p=self.pulse();plan.accept(d['prediction'][0],time.monotonic(),rospy.Time.now().to_sec())
        if s['armed']:raise RuntimeError('解锁前状态发生变化')
        self.submit(('arm',True))
        self.wait(lambda d,s,p:s['armed'] and s['mode']=='OFFBOARD' and not self.rpc_busy,4)
        start=time.monotonic();plan.begin(start)
        self.command_updates=[{'t':start,'sent_xyz_yaw':list(home)}]
        self.report_data.update(armed_start_monotonic=start,status='running')
        print('已正常解锁；30 秒台架开始：0–10 秒模拟起飞，10–12 秒模拟悬停，随后循环更新受限目标。',flush=True)
        last_phase=None;last_audit=0.;last_saved=0.;last_new=None;last_print=0.;holding_stale=False
        while not rospy.is_shutdown():
            now=time.monotonic();d,s,pose=self.pulse()
            if not s['armed'] or s['mode']!='OFFBOARD':raise RuntimeError('30 秒窗口内退出已解锁 OFFBOARD')
            if math.dist(pose[:3],home[:3])>.10 or abs(wrap(pose[3]-home[3]))>math.radians(10):
                raise RuntimeError('机体位姿超出台架固定范围')
            phase=plan.step(now,rospy.Time.now().to_sec())
            if phase!=last_phase:
                print('%5.1fs  %s'%(now-start,phase),flush=True)
                self.events.append({'t':now,'elapsed_s':now-start,'phase':phase})
                if phase=='MODEL_LOOP':last_new=now
                last_phase=phase
            if phase=='STOPPING':break
            pred,received=d.get('prediction',({},0))
            accepted=plan.offer(pred,now,rospy.Time.now().to_sec(),received)
            if accepted:
                last_new=now;holding_stale=False
                self.command_updates.append({'t':now,'sent_xyz_yaw':list(plan.sp)})
            if phase=='MODEL_LOOP':
                since=now-last_new
                if since>5:raise RuntimeError('循环推理连续 5 秒无有效新图像；中止而非到达')
                image_stale=(bool(plan.samples) and rospy.Time.now().to_sec()-plan.samples[-1]['image_stamp']>2)
                if (since>2 or image_stale) and not holding_stale:
                    plan.sp=list(home);holding_stale=True
                    self.command_updates.append({'t':now,'sent_xyz_yaw':list(home)})
                    self.events.append({'t':now,'elapsed_s':now-start,'phase':'HOLD_STALE'})
                    print('新图像暂时中断：保持台架起点，继续等待；不使用过期模型目标。',flush=True)
            with self.lock:self.sp=list(plan.sp)
            if now-last_audit>=1:
                pubs=dict(self.master.getSystemState()[0]);allowed={FSM,rospy.get_name()}
                if set(pubs.get(TOPIC,[]))-allowed or pubs.get('/uav1/move_command'):
                    raise RuntimeError('出现其他控制发布者')
                if not rospy.get_param(FSM+'/external_setpoint_control_active',False):raise RuntimeError('控制权交接失效')
                active=(rospy.get_param('/vla_direct_bridge/run_id',''),rospy.get_param('/vla_direct_bridge/active_instruction',''))
                if active!=(identity[0],identity[2]):raise RuntimeError('活动指令改变')
                last_audit=now
            if now-last_print>=10:
                print('  已运行 %.1fs；新图像目标=%d；setpoint累计=%d'%(now-start,len(plan.samples),self.publish_count),flush=True)
                last_print=now
            if now-start>=25 and not self.report_data.get('stop_notice_given'):
                print('本轮约剩 5 秒；结束后自动请求降落，确认地面后普通上锁，不要求油门回最低。',flush=True)
                self.report_data['stop_notice_given']=True
            if now-last_saved>=1:
                self.report_data.update(elapsed_s=now-start,phase=phase,model_samples=plan.samples)
                self.save();last_saved=now
            time.sleep(.025)
        if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
        self.report_data.update(elapsed_s=time.monotonic()-start,window_completed=True,phase='STOPPING',
                                status='completed_pending_cleanup',model_samples=plan.samples)

    def cleanup_state(self,now):
        d=self.snapshot();s,t=d.get('state',(None,0));e,et=d.get('extended',(None,0))
        ros_now=rospy.Time.now().to_sec()
        land_fresh=bool(e and now-et<.75 and 0<=ros_now-e.header.stamp.to_sec()<.75)
        return dict(fresh=bool(s and now-t<1.5),connected=bool(s and s.connected),
            armed=bool(s and s.armed),mode=s.mode if s else '',land_fresh=land_fresh,
            landed=bool(e and e.landed_state==1))

    def run_normal_shutdown(self,clock=time.monotonic,sleep=time.sleep):
        self.normal_cleanup=True;self.cleanup_started_at=clock()
        if self.mission is None:self.mission=SimpleNamespace(phase='LAND_REQUEST')
        self.mission.phase='LAND_REQUEST'
        policy=NormalShutdown(self.cleanup_started_at)
        trace=[];last_phase=None;last_saved=clock()
        # Acknowledge cleanup over the lease channel before AUTO.LAND appears
        # on the independent FCU state channel.
        while self.watchdog is not None and self.ready_token!=self.token+':normal_cleanup':
            now=clock()
            if self.control_released:return dict(takeover=True,disarmed_confirmed=False,force_disarm_used=False)
            with self.lock:self.policy_at=now
            self.lease()
            if now-self.cleanup_started_at>2:
                return dict(disarmed_confirmed=False,force_disarm_used=False,normal_error='watchdog cleanup handshake timed out')
            sleep(.03)
        while True:
            now=clock()
            if self.control_released:
                return dict(disarmed_confirmed=False,force_disarm_used=False,takeover=True,
                            reason='actual pilot/PX4 mode selected',trace=trace)
            s=self.cleanup_state(now)
            keep,action=policy.step(now,s,rpc_ready=not self.rpc_busy)
            with self.lock:
                self.policy_at=now
                if not keep:self.sp=None
            self.lease()
            if policy.land_confirmed:self.mission.phase='LANDING'
            if action:self.submit(action)
            trace.append({'t':now,'phase':policy.phase,'state':s,'keep_setpoints':keep,
                          'rpc_busy':self.rpc_busy,'action':action})
            if policy.phase!=last_phase:
                print('正常收尾 '+policy.phase+': '+policy.reason,flush=True)
                last_phase=policy.phase
            if now-last_saved>=1:
                self.report_data['normal_shutdown_trace']=trace;self.save();last_saved=now
            if policy.phase=='TAKEOVER':
                self.control_released=True;self.release_mode=s['mode']
                return dict(disarmed_confirmed=False,force_disarm_used=False,takeover=True,
                            reason=policy.reason,trace=trace)
            if policy.phase=='FAILED':
                return dict(disarmed_confirmed=False,force_disarm_used=False,
                            normal_error=policy.reason,trace=trace)
            if policy.phase=='DONE' and not self.rpc_busy:
                responses=[x for x in self.rpc_log if x['t']>=self.cleanup_started_at]
                return dict(disarmed_confirmed=True,force_disarm_used=False,
                    normal_landing_confirmed=policy.land_confirmed,auto_disarmed=policy.auto_disarmed,
                    ordinary_disarm_requests=policy.disarm_requests,rpc=responses,trace=trace)
            if now-self.cleanup_started_at>14:
                return dict(disarmed_confirmed=False,force_disarm_used=False,
                            normal_error='ordinary cleanup RPC did not drain',trace=trace)
            sleep(.03)

    def wait_for_disarmed_release(self,timeout=30.,clock=time.monotonic,sleep=time.sleep):
        if not getattr(self,'owner',False):return True
        deadline=clock()+timeout
        while not rospy.is_shutdown():
            try:
                if not self.rpc_busy and release_if_safe(self.snapshot(),self.master.getSystemState(),
                                                         self.ownership,now=clock()):
                    self.owner=False
                    print('已确认上锁并落地，原控制器发布权已释放；没有切换模式或发送上锁指令。',flush=True)
                    return True
            except Exception as exc:
                self.report_data['ownership_release_error']=str(exc)
            if clock()>=deadline:return False
            sleep(.1)
        return False

    def handoff_to_pilot(self):
        self.stop=True;self.pilot_takeover=True;self.sp=None;self.closed=True
        if self.publisher:self.publisher.unregister();self.publisher=None
        self.done=True;self.lease()
        if self.watchdog:
            try:self.watchdog.wait(timeout=4)
            except subprocess.TimeoutExpired:pass
        self.report_data.update(status='pilot_takeover',final_armed=self.actual_armed(),
            release_mode=self.release_mode,no_disarm_on_takeover=True,
            error='测试由实际模式接管终止，未自动上锁或切回OFFBOARD')
        self.save()
        print('已交还 '+str(self.release_mode)+'：停止发送目标，不自动上锁、不切回 OFFBOARD。',flush=True)
        print('等待确认上锁后回收发布权（最多30秒）；超时后下次台架启动会先检查并回收。',flush=True)
        released=self.wait_for_disarmed_release()
        self.report_data.update(ownership_restored=released,ownership_release_pending=not released,
                                final_armed=self.actual_armed())
        self.save()
        return False

    def stop_and_verify(self):
        if self.control_released:
            # User mode selection has precedence over timed test cleanup.
            return self.handoff_to_pilot()
        if not self.owner and not self.rpc_log and not self.rpc_busy:
            self.closed=True
            self.report_data.update(status='failed',no_control_acquired=True,final_armed=self.actual_armed())
            self.save();return False
        self.stopping=True;self.stop=True;self.lease()
        # Keep the independent watchdog alive across a possibly late arm RPC.
        wait_start=time.monotonic()
        while self.rpc_busy or (self.rpc_log and time.monotonic()-self.rpc_finished<1.5):
            with self.lock:self.policy_at=time.monotonic()
            self.lease()
            if time.monotonic()-wait_start>8:raise RuntimeError('服务仍未返回；保留独立看门狗，请现场停止')
            time.sleep(.04)
        result=self.run_normal_shutdown()
        if result.get('takeover'):return self.handoff_to_pilot()
        self.report_data['normal_shutdown_result']=result
        if not result.get('disarmed_confirmed'):
            # The force path remains an explicit propeller-removed emergency,
            # never a normal end-of-window disarm and never a passing result.
            if self.control_released:return self.handoff_to_pilot()
            self.emergency_stop=True;self.lease()
            force=rospy.ServiceProxy('/uav1/mavros/cmd/command',CommandLong)
            fallback=bench_disarm(self.arm,force,self.actual_armed,propellers_removed_bench=True,
                                 prepare_manual_stop=self.prepare_stop)
            fallback['normal_error']=result.get('normal_error','ordinary shutdown failed')
            result=fallback
        self.report_data['stop_result']=result
        if not result.get('disarmed_confirmed'):raise RuntimeError('未确认上锁；独立看门狗继续运行，请现场停止')
        if self.actual_armed() is not False:raise RuntimeError('恢复模式前未确认未解锁')
        response=self.mode(base_mode=0,custom_mode='MANUAL')
        end=time.monotonic()+3
        while time.monotonic()<end:
            self.lease();s,st=self.snapshot().get('state',(None,0))
            if s and s.connected and time.monotonic()-st<1.5 and not s.armed and s.mode=='MANUAL':break
            time.sleep(.04)
        else:raise RuntimeError('上锁后未确认 MANUAL')
        self.sp=None;self.closed=True
        if self.publisher:self.publisher.unregister();self.publisher=None
        if self.owner:
            released=self.ownership(value=False)
            if not released.success:raise RuntimeError('原控制器发布权恢复失败')
            self.owner=False
        self.done=True;self.lease();time.sleep(.15)
        if self.watchdog:
            try:self.watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:raise RuntimeError('看门狗尚未确认结束')
        self.report_data.update(final_armed=self.actual_armed(),final_mode=s.mode,
            ownership_restored=not bool(rospy.get_param(FSM+'/external_setpoint_control_active',True)),
            bench_allow_armed=rospy.get_param(FSM+'/bench_allow_armed',None))
        start=self.report_data.get('armed_start_monotonic',float('inf'))
        elapsed=self.report_data.get('elapsed_s',0)
        points=[p for p in self.points if start<=p['t']<=start+elapsed]
        gaps=[b['t']-a['t'] for a,b in zip(points,points[1:])]
        self.report_data.update(setpoint_trace=points,setpoint_count=len(points),
            setpoint_hz=(len(points)-1)/(points[-1]['t']-points[0]['t']) if len(points)>1 else 0,
            max_setpoint_gap_s=max(gaps) if gaps else None,rpc=self.rpc_log)
        home=self.report_data.get('home_xyz_yaw',[0]*4)
        safe=all(math.dist(p['xyz_yaw'][:2],home[:2])<=.020001
            and abs(p['xyz_yaw'][2]-home[2])<=.003001
            and abs(wrap(p['xyz_yaw'][3]-home[3]))<=math.radians(1)+1e-6 for p in points)
        source_ok=bool(points) and all(p['source']==rospy.get_name() for p in points)
        samples=self.report_data.get('model_samples',[])
        commands=getattr(self,'command_updates',[{'t':start,'sent_xyz_yaw':home}])
        command_times=[x['t'] for x in commands]
        matched=0
        for point in points:
            index=bisect.bisect_right(command_times,point['t'])-1
            candidates=commands[max(0,index-2):index+1]
            if any(max(abs(a-b) for a,b in zip(point['xyz_yaw'],c['sent_xyz_yaw']))<1e-6 for c in candidates):
                matched+=1
        normal=not result.get('force_disarm_used') and not result.get('normal_error')
        watchdog_path=BASE/'runtime'/'bench30_watchdog.json'
        watchdog=json.loads(watchdog_path.read_text()) if watchdog_path.exists() else {}
        watchdog_ok=watchdog.get('token')==self.token and watchdog.get('final_armed') is False and not watchdog.get('release_error')
        watchdog_ok=watchdog_ok and watchdog.get('normal_cleanup_acknowledged') is True
        watchdog_ok=watchdog_ok and all(not r.get('force_disarm_used') and not r.get('normal_error') for r in watchdog.get('stop_results',[]))
        self.report_data.update(all_setpoints_bounded=safe,setpoint_source_verified=source_ok,
            matched_setpoints=matched,command_updates=commands,normal_stop=normal,watchdog=watchdog,watchdog_ok=watchdog_ok)
        ok=(self.report_data.get('window_completed') and len(points)>750 and max(gaps,default=99)<.15
            and len(samples)>=10 and matched/len(points)>.98 and safe and source_ok and normal and watchdog_ok
            and self.report_data['ownership_restored'] and self.report_data['bench_allow_armed'] is False
            and self.report_data['final_armed'] is False and not self.report_data.get('error'))
        self.report_data['status']='passed_timed_bench' if ok else 'failed'
        self.save()
        print('结果：%s；模型新图像=%d，setpoint=%d，频率=%.2fHz，最大间隔=%s；最终未解锁 MANUAL。'%(
            self.report_data['status'],len(samples),len(points),self.report_data['setpoint_hz'],self.report_data['max_setpoint_gap_s']),flush=True)
        print('报告：'+str(BASE/'runtime'/'bench30_report.json'),flush=True)
        return ok


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--props-removed',action='store_true');parser.add_argument('--fixed',action='store_true')
    args=parser.parse_args()
    if not args.props_removed or not args.fixed:parser.error('requires --props-removed --fixed')
    locks=[]
    for path in [BASE/'runtime'/'closed_loop_flight.lock',Path('/home/nvidia/agile/runtime/vla/test.lock')]:
        lock=path.open('w');locks.append(lock)
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有控制/测试任务运行')
    runtime=TimedBench()
    try:runtime.run_bench()
    except BaseException as exc:
        runtime.report_data.update(status='failed',error=str(exc))
        if hasattr(runtime,'plan') and runtime.plan.started is not None:
            runtime.report_data.update(elapsed_s=time.monotonic()-runtime.plan.started,model_samples=runtime.plan.samples)
        print('台架停止：'+str(exc),flush=True)
    finally:
        try:ok=runtime.stop_and_verify()
        except BaseException as exc:
            runtime.report_data['cleanup_error']=str(exc);runtime.save();raise
    raise SystemExit(0 if ok else 1)


if __name__=='__main__':main()
