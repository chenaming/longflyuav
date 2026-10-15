#!/usr/bin/env python3
"""Continuous bounded model transport probe, with no arming service."""
import fcntl,json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/home/nvidia/agile/vla_commissioning')
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy,rosgraph
from mavros_msgs.msg import State
from model_bench_validate import ModelBench
from model_motor_validate import MotorBench
from motor_task_guard import same_task
from setpoint_match import match_setpoints

def main():
    lock=open('/home/nvidia/agile/runtime/vla/test.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rospy.init_node('fly_unarmed_match_probe',anonymous=True)
    state=rospy.wait_for_message('/uav1/mavros/state',State,timeout=4)
    assert state.connected and not state.armed and state.mode=='MANUAL' and state.system_status==3
    assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed') is True
    assert rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed') is False
    assert rospy.get_param('/vla_direct_bridge/enable_publish') is False
    pubs=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    assert not pubs.get('/uav1/move_command')
    assert pubs.get('/uav1/mavros/setpoint_raw/local')==['/uav1/uav_control_fsm']
    smooth=rospy.get_param('/uav1/uav_control_fsm/pos_smooth_enable',True)
    test=ModelBench()
    deadline=time.monotonic()+3
    while True:
        try:test.check();break
        except RuntimeError:
            if time.monotonic()>deadline:raise
            time.sleep(.05)
    commands=[];result={'kind':'continuous_unarmed_match_probe','armed_ever':False,'status':'starting','smooth_enabled':smooth}
    try:
        test.mode(base_mode=0,custom_mode='OFFBOARD')
        deadline=time.monotonic()+5
        while test.check()[0].mode!='OFFBOARD':
            if time.monotonic()>deadline:raise RuntimeError('OFFBOARD not observed')
            time.sleep(.04)
        p=test.next_prediction()
        start=time.monotonic();end=start+5
        while time.monotonic()<end:
            assert test.check()[0].mode=='OFFBOARD'
            with test.lock:p=dict(test.prediction) if test.prediction else None
            if not p or not 0<=rospy.Time.now().to_sec()-p['observation_stamp']<=2:raise RuntimeError('model stale')
            if not same_task(p,test.report,rospy.get_param('/vla_direct_bridge/active_instruction'),rospy.get_param('/vla_direct_bridge/run_id')):raise RuntimeError('task changed')
            c,entry=MotorBench.bounded_command(test,p)
            c.header.stamp=rospy.Time.now();test.pub.publish(c)
            commands.append((time.monotonic(),entry))
            time.sleep(.04)
        with test.lock:points=[{'t':t,'xyz_m':[m.position.x,m.position.y,m.position.z],'yaw_rad':m.yaw,'source':m._connection_header.get('callerid'),'coordinate_frame':m.coordinate_frame,'type_mask':m.type_mask} for t,m in test.setpoints if start+.15<t<end]
        result.update(status='completed',duration_s=time.monotonic()-start,commands=commands,setpoints=points,
                      raw_match=match_setpoints(commands,points,smooth_enabled=False),
                      controller_match=match_setpoints(commands,points,smooth_enabled=smooth))
    except Exception as exc:result.update(status='failed',error=str(exc))
    finally:
        # No arming service exists in this probe. Preserve a mode-restoration error.
        try:
            test.mode(base_mode=0,custom_mode='MANUAL')
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                with test.lock:
                    if test.state and test.state[0].mode=='MANUAL' and not test.state[0].armed:break
                time.sleep(.05)
        except Exception as exc:result['restore_error']=str(exc)
        result['armed_ever']=test.armed_ever
        with test.lock:
            result['final_armed']=test.state[0].armed if test.state else None
            result['final_mode']=test.state[0].mode if test.state else None
        path=Path(__file__).resolve().parent/'runtime/unarmed_match_probe.json'
        path.write_text(json.dumps(result,ensure_ascii=False,indent=2))
        summary={k:v for k,v in result.items() if k not in ['commands','setpoints']}
        for key in ['raw_match','controller_match']:
            if key in summary:summary[key]={k:v for k,v in summary[key].items() if k!='matches'}
        print(json.dumps(summary,ensure_ascii=False,indent=2))
    if result['final_armed'] is not False or result['final_mode']!='MANUAL':raise SystemExit('终态未恢复，请检查飞控。')

if __name__=='__main__':main()
