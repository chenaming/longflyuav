#!/usr/bin/env python3
"""Read-only, operator-driven translation measurement. No publishers/services."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import uuid


def rotation(q):
    if len(q)!=4 or not all(math.isfinite(x) for x in q):raise ValueError('invalid quaternion')
    norm=math.sqrt(sum(x*x for x in q))
    if norm<1e-8:raise ValueError('zero quaternion')
    x,y,z,w=[v/norm for v in q]
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def compare(before,after):
    if (before['frame_id'],before['child_frame_id'])!=(after['frame_id'],after['child_frame_id']):
        raise ValueError('coordinate frame changed during measurement')
    delta=[b-a for a,b in zip(before['position_m'],after['position_m'])]
    r=rotation(before['quaternion_xyzw'])
    local=[sum(r[i][j]*delta[i] for i in range(3)) for j in range(3)]
    r_after=rotation(after['quaternion_xyzw'])
    yaw_before=math.atan2(r[1][0],r[0][0]);yaw_after=math.atan2(r_after[1][0],r_after[0][0])
    yaw_delta=math.atan2(math.sin(yaw_after-yaw_before),math.cos(yaw_after-yaw_before))
    return {'delta_world_m':delta,'delta_initial_child_m':local,
            'distance_m':math.sqrt(sum(x*x for x in delta)),
            'yaw_change_deg':math.degrees(yaw_delta)}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--motion',choices=['forward','left','up'],default='forward')
    parser.add_argument('--distance',type=float,default=.20)
    args=parser.parse_args()
    if not math.isfinite(args.distance) or not 0<args.distance<=.5:raise SystemExit('手动测量距离要求0至0.5m。')
    import rospy,rosgraph,message_filters
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    rospy.init_node('fly_check_axes',anonymous=True)
    lock=threading.Lock();states=[];pairs=[];armed_seen=[False]
    def on_state(m):
        with lock:
            armed_seen[0]=armed_seen[0] or m.armed
            states.append((time.monotonic(),m));states[:]=states[-20:]
    def paired(a,b):
        with lock:pairs.append((time.monotonic(),a,b));pairs[:]=pairs[-1000:]
    rospy.Subscriber('/uav1/mavros/state',State,on_state,queue_size=10)
    a=message_filters.Subscriber('/uav1/odin1/odometry',Odometry,queue_size=50)
    b=message_filters.Subscriber('/uav1/mavros/local_position/odom',Odometry,queue_size=50)
    synchronizer=message_filters.ApproximateTimeSynchronizer([a,b],50,.05)
    synchronizer.registerCallback(paired)
    def state_ok():
        with lock:
            last=states[-1] if states else None
            ever_armed=armed_seen[0]
        if ever_armed:raise RuntimeError('本次记录期间观察到解锁，测量已取消。')
        if last is None or time.monotonic()-last[0]>2:raise RuntimeError('飞控状态不新鲜。')
        m=last[1]
        if not m.connected or m.armed or m.mode!='MANUAL' or m.system_status!=3:
            raise RuntimeError('要求已连接、未解锁、MANUAL、Standby。')
    def protected():
        if time.time()<1700000000:raise RuntimeError('NX系统时间异常，请先校时，再做位移采样。')
        state_ok()
        for name,expected in [('/vla_direct_bridge/enable_publish',False),
                              ('/uav1/uav_control_fsm/bench_disarmed',True),
                              ('/uav1/uav_control_fsm/bench_allow_armed',False)]:
            if rospy.get_param(name,None) is not expected:raise RuntimeError('保护参数不符合测量要求：'+name)
        pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        if pubs.get('/uav1/move_command'):raise RuntimeError('另一个控制测试正在发布目标。')
    def sample():
        protected();start=time.monotonic()
        while time.monotonic()-start<2:
            state_ok();time.sleep(.05)
        protected()
        with lock:window=[p for p in pairs if p[0]>=start]
        if len(window)<5 or time.monotonic()-window[-1][0]>.5:raise RuntimeError('两路配对里程计不足或已过期。')
        result={}
        for index,label in [(1,'odin'),(2,'mavros')]:
            msgs=[p[index] for p in window];last=msgs[-1]
            frames={(m.header.frame_id,m.child_frame_id) for m in msgs}
            if len(frames)!=1:raise RuntimeError('采样期间坐标系发生变化。')
            points=[[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z] for m in msgs]
            if not all(math.isfinite(x) for p in points for x in p):raise RuntimeError('位姿不是有限数值。')
            q=last.pose.pose.orientation
            result[label]={'frame_id':last.header.frame_id,'child_frame_id':last.child_frame_id,
                'position_m':[sum(p[j] for p in points)/len(points) for j in range(3)],
                'quaternion_xyzw':[q.x,q.y,q.z,q.w],
                'position_range_m':[max(p[j] for p in points)-min(p[j] for p in points) for j in range(3)],
                'sample_count':len(points),'first_stamp':msgs[0].header.stamp.to_sec(),'last_stamp':last.header.stamp.to_sec()}
        return result
    time.sleep(1)
    protected()
    folder=Path(__file__).resolve().parent/'runtime/axis_checks'/uuid.uuid4().hex
    folder.mkdir(parents=True)
    direction={'forward':'机头前方','left':'机体左侧','up':'正上方'}[args.motion]
    report={'motion':args.motion,'operator_distance_m':args.distance,'scope':'read-only translation experiment; not complete extrinsic calibration'}
    try:
        input('保持拆桨、未解锁。先将机体放稳、保持不动，按回车采集起点：')
        report['before']=sample()
        (folder/'measurement.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        input('起点已保存。将整个机体向%s平移%.1fcm，保持朝向和其他方向不变；放稳后按回车：' % (direction,args.distance*100))
        report['after']=sample()
        report['results']={label:compare(report['before'][label],report['after'][label]) for label in ['odin','mavros']}
        report['status']='captured'
        for label,r in report['results'].items():
            print('%s 世界坐标变化(m)：%s' % (label,[round(v,6) for v in r['delta_world_m']]))
            print('%s 起始child坐标变化(m)：%s；位移=%.2fcm；yaw变化=%.2f°' %
                  (label,[round(v,6) for v in r['delta_initial_child_m']],r['distance_m']*100,r['yaw_change_deg']))
        print('这是位移记录，需要按实际安装轴解释；不会自动判定外参已标定。')
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='aborted',error=str(exc));raise
    finally:
        (folder/'measurement.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print('记录：'+str(folder/'measurement.json'))

if __name__=='__main__':
    try:main()
    except (RuntimeError,ValueError,KeyboardInterrupt) as exc:
        raise SystemExit('测量已取消：'+str(exc)) from None
