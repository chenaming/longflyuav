"""Release a disarmed controller handoff. Never arm, disarm or change flight mode."""
import time

FSM='/uav1/uav_control_fsm'
TOPIC='/uav1/mavros/setpoint_raw/local'


def release_if_safe(data, graph, release, now=None):
    now=time.monotonic() if now is None else now
    state,received=data.get('state',(None,0))
    extended,ground_received=data.get('extended',(None,0))
    if not (state and 0<=now-received<1.5 and state.connected and not state.armed
            and state.system_status==3 and state.mode in ('MANUAL','POSCTL','ALTCTL','STABILIZED','ACRO')
            and extended and 0<=now-ground_received<1.5 and extended.landed_state==1):
        return False
    publishers,subscribers,_=graph
    if any(node!=FSM for node in dict(publishers).get(TOPIC,[])):
        return False
    if dict(publishers).get('/uav1/move_command'):
        return False
    # A previous independent watchdog must have relinquished its lease too.
    if any('bench60_watchdog' in node for _,nodes in publishers+subscribers for node in nodes):
        return False
    return bool(release(value=False).success)


def main():
    import fcntl
    from pathlib import Path
    import rospy,rosgraph
    from mavros_msgs.msg import State,ExtendedState
    from mavros_msgs.srv import CommandBool
    locks=[]
    for path in ('/home/nvidia/agile/fly/runtime/closed_loop_flight.lock',
                 '/home/nvidia/agile/runtime/vla/test.lock'):
        handle=Path(path).open('a');locks.append(handle)
        try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有控制/测试任务运行，不回收控制权。')
    rospy.init_node('fly_release_disarmed_handoff',anonymous=True)
    if not rospy.get_param(FSM+'/external_setpoint_control_active',False):
        print('控制权无残留。');return
    if (rospy.get_param(FSM+'/bench_disarmed',None) is not True
            or rospy.get_param(FSM+'/bench_allow_armed',None) is not False
            or rospy.get_param('/vla_direct_bridge/enable_publish',None) is not False):
        raise SystemExit('保护配置异常，不回收控制权。')
    data={}
    def receive(key,message):data[key]=(message,time.monotonic())
    rospy.Subscriber('/uav1/mavros/state',State,lambda m:receive('state',m),queue_size=1)
    rospy.Subscriber('/uav1/mavros/extended_state',ExtendedState,lambda m:receive('extended',m),queue_size=1)
    master=rosgraph.Master(rospy.get_name())
    rospy.wait_for_service(FSM+'/external_setpoint_control',timeout=3)
    release=rospy.ServiceProxy(FSM+'/external_setpoint_control',CommandBool)
    deadline=time.monotonic()+6
    while time.monotonic()<deadline and not rospy.is_shutdown():
        if release_if_safe(dict(data),master.getSystemState(),release):
            if rospy.get_param(FSM+'/external_setpoint_control_active',True):
                raise SystemExit('服务响应成功但控制权标记尚未释放，请检查原控制器。')
            print('已回收上轮残留控制权：确认未解锁、在地面；未调用解锁/上锁或切模式服务。')
            return
        time.sleep(.1)
    raise SystemExit('控制权未释放：需要新鲜的未解锁地面状态、手控模式，且没有其他控制器或看门狗运行。')


if __name__=='__main__':main()
