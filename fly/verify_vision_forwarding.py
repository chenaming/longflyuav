"""Passive Odin->vision_pose forwarding verification; no flight services/publishers."""
import json,math,time
from pathlib import Path
import rospy,rosgraph
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus,State


def main():
    rospy.init_node('fly_verify_vision_forwarding',anonymous=True)
    data={}
    def receive(k,m):data.setdefault(k,[]).append((time.monotonic(),m))
    for k,topic,cls in [('odin','/uav1/odin1/odometry_highfreq',Odometry),
                        ('vision','/uav1/mavros/vision_pose/pose',PoseStamped),
                        ('fcu','/uav1/mavros/local_position/odom',Odometry),
                        ('estimator','/uav1/mavros/estimator_status',EstimatorStatus),
                        ('state','/uav1/mavros/state',State)]:
        rospy.Subscriber(topic,cls,lambda m,k=k:receive(k,m),queue_size=2000,tcp_nodelay=True)
    time.sleep(3)
    for key in ('odin','vision','fcu','estimator','state'):
        if key not in data:raise SystemExit('位置链路缺少消息：'+key)
    rows=list(data['vision']);hz=(len(rows)-1)/(rows[-1][0]-rows[0][0]) if len(rows)>1 else 0
    stamp_set={m.header.stamp.to_nsec() for _,m in rows}
    odin={m.header.stamp.to_nsec():m.pose.pose for _,m in data['odin']}
    errors=[]
    for _,m in rows:
        p=odin.get(m.header.stamp.to_nsec())
        if p:errors.append(math.dist([p.position.x,p.position.y,p.position.z],
                                    [m.pose.position.x,m.pose.position.y,m.pose.position.z]))
    age=rospy.Time.now().to_sec()-rows[-1][1].header.stamp.to_sec()
    graph=rosgraph.Master(rospy.get_name()).getSystemState()
    pubs=dict(graph[0]).get('/uav1/mavros/vision_pose/pose',[])
    est=data['estimator'][-1][1]
    state=data['state'][-1][1]
    fcu=data['fcu'][-1][1].pose.pose.position
    op=data['odin'][-1][1].pose.pose.position
    report=dict(hz=hz,count=len(rows),unique_stamps=len(stamp_set),paired_count=len(errors),
                max_copy_error_m=max(errors,default=None),source_age_s=age,publishers=pubs,
                relative_position_valid=est.pos_horiz_rel_status_flag,vertical_position_valid=est.pos_vert_abs_status_flag,
                armed=state.armed,mode=state.mode,fcu_odin_difference_m=math.dist([fcu.x,fcu.y,fcu.z],[op.x,op.y,op.z]),
                actual_position_mode_switch_tested=False,updated_unix=time.time())
    report['passed']=bool(35<=hz<=65 and len(stamp_set)==len(rows) and len(errors)>=len(rows)*.8
        and max(errors,default=99)<1e-7 and 0<=age<.2 and pubs==['/uav1/uav_control_fsm']
        and est.pos_horiz_rel_status_flag and est.pos_vert_abs_status_flag and not state.armed)
    p=Path(__file__).resolve().parent/'runtime/vision_forwarding_report.json'
    p.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('位置转发检查：'+json.dumps(report,ensure_ascii=False),flush=True)
    if not report['passed']:raise SystemExit('位置转发检查未通过，不进入电机测试。')


if __name__=='__main__':main()
