"""Read-only stop permission: fresh FCU state, disarmed, landed, Standby."""
import json,time


def ground_safe(state,state_at,extended,extended_at,now):
    return bool(state and extended and state.connected and not state.armed and state.system_status==3
                and 0<=now-state_at<1.5 and 0<=now-extended_at<1.5 and extended.landed_state==1)


def main():
    import rospy
    from mavros_msgs.msg import State,ExtendedState
    rospy.init_node('fly_cycle_ground_guard',anonymous=True)
    data={}
    def receive(k,m):data[k]=(m,time.monotonic())
    rospy.Subscriber('/uav1/mavros/state',State,lambda m:receive('state',m),queue_size=1)
    rospy.Subscriber('/uav1/mavros/extended_state',ExtendedState,lambda m:receive('extended',m),queue_size=1)
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        s,st=data.get('state',(None,0));e,et=data.get('extended',(None,0))
        if ground_safe(s,st,e,et,time.monotonic()):
            print(json.dumps({'ground_safe':True,'armed':False,'mode':s.mode,'landed_state':e.landed_state}));return
        time.sleep(.05)
    raise SystemExit('未确认新鲜的已上锁、已落地、Standby 状态；保留定位和 MAVROS，不关闭节点。')


if __name__=='__main__':main()
