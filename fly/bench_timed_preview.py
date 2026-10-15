#!/usr/bin/env python3
"""Live, DISARMED 10-second phase rehearsal. No flight/mode/control publishers."""
import json
from pathlib import Path
import threading
import time
import rospy
import rosgraph
from mavros_msgs.msg import State,RCIn
from std_msgs.msg import String


def main():
    rospy.init_node('fly_bench_timed_preview',anonymous=True)
    lock=threading.Lock();latest={}
    report={'kind':'disarmed_timed_phase_rehearsal','takeoff_is_simulated':True,
            'height_1m_verified':False,'real_flight_entry_exercised':False,
            'arming_requested':False,'mode_requested':False,'samples':[],
            'started_unix':time.time(),'status':'waiting_for_initial_prediction'}
    path=Path(__file__).resolve().parent/'runtime'/'bench_timed_preview.json'
    def receive(key,msg):
        if key=='prediction':
            try:msg=json.loads(msg.data)
            except ValueError:return
        with lock:latest[key]=(msg,time.monotonic())
    for key,topic,typ in [('state','/uav1/mavros/state',State),('rc','/uav1/mavros/rc/in',RCIn),
                          ('prediction','/vla/validated_prediction',String)]:
        rospy.Subscriber(topic,typ,lambda m,k=key:receive(k,m),queue_size=1)
    def snapshot():
        with lock:return dict(latest)
    def guard(data):
        now=time.monotonic();s,st=data['state'];rc,rt=data['rc']
        if now-st>1.5 or not s.connected or s.armed or s.mode!='MANUAL':
            raise RuntimeError('台架计时阶段要求新鲜状态、未解锁、MANUAL')
        if now-rt>.75 or len(rc.channels)<9 or rc.rssi==0 or rc.channels[2]>=1100 or rc.channels[8]>=1300:
            raise RuntimeError('遥控器新鲜度或油门/CH9条件不满足')
    def prediction(data,cutoff,identity=None):
        p,received=data.get('prediction',({},0));now=rospy.Time.now().to_sec()
        if time.monotonic()-received>2 or not p.get('latest_only'):return None
        if not 0<=now-p.get('observation_stamp',0)<=2 or p['observation_stamp']<=cutoff:return None
        key=(p.get('run_id'),p.get('epoch'),p.get('instruction'))
        if identity is not None and key!=identity:raise RuntimeError('任务/参考原点改变，取消本轮台架测试')
        return p
    started=rospy.Time.now().to_sec();deadline=time.monotonic()+15
    try:
        while not rospy.is_shutdown():
            data=snapshot()
            if 'state' in data and 'rc' in data:
                guard(data);p=prediction(data,started)
                if p:break
            if time.monotonic()>deadline:raise RuntimeError('没有本次测试开始后的有效模型结果')
            time.sleep(.05)
        if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
        assert rospy.get_param('/vla_direct_bridge/enable_publish') is False
        assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed') is True
        assert rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed') is False
        assert not rospy.get_param('/uav1/uav_control_fsm/external_setpoint_control_active',False)
        master=rosgraph.Master(rospy.get_name())
        assert not dict(master.getSystemState()[0]).get('/uav1/move_command')
        identity=(p['run_id'],p['epoch'],p['instruction'])
        report.update(run_id=p['run_id'],epoch=p['epoch'],instruction=p['instruction'],status='simulated_takeoff_phase')
        print('台架模拟起飞阶段：计时 10 秒，保持未解锁，不发送 1 米爬升目标。',flush=True)
        begin=time.monotonic();last=None;last_valid=begin
        while time.monotonic()-begin<10 and not rospy.is_shutdown():
            data=snapshot();guard(data);p=prediction(data,started,identity)
            if p and p['request_id']!=last:
                last=p['request_id'];last_valid=time.monotonic()
                report['samples'].append({'request_id':last,'image_stamp':p['observation_stamp'],
                                          'target_xyz_m':p['target_xyz_m'],'target_yaw_rad':p['target_yaw_rad']})
            if time.monotonic()-last_valid>2:raise RuntimeError('连续有效推理中断超过 2 秒')
            time.sleep(.03)
        if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
        if len(report['samples'])<3:raise RuntimeError('10 秒阶段的新图像样本不足')
        report['simulated_takeoff_seconds']=time.monotonic()-begin
        cutoff=rospy.Time.now().to_sec();report['post_hover_image_cutoff']=cutoff
        print('模拟阶段结束；丢弃此前所有结果，等待新图像。高度仍未验证。',flush=True)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not rospy.is_shutdown():
            data=snapshot();guard(data);p=prediction(data,cutoff,identity)
            if p:
                report['fresh_post_phase_prediction']=p;break
            time.sleep(.03)
        else:raise RuntimeError('未收到模拟悬停结束后拍摄的新图像结果')
        report['status']='passed_disarmed_timed_rehearsal'
        print('通过：计时阶段后收到新图像 request=%s；准备未解锁模型传递测试。'%p['request_id'],flush=True)
    except BaseException as exc:
        report['status']='failed';report['error']=str(exc)
        raise
    finally:
        report['updated_unix']=time.time()
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
