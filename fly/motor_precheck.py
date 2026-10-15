#!/usr/bin/env python3
"""Read-only motor bench readiness. No flight services or control publishers."""
import argparse,json
from pathlib import Path
import time
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import State, RCIn
from std_msgs.msg import String
from motor_task_guard import rc_is_ready, same_task


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-offboard-throttle',action='store_true',help='只取消脚本低油门门槛；不绕过PX4解锁预检')
    args=parser.parse_args()
    rospy.init_node('fly_motor_precheck',anonymous=True)
    base=Path(__file__).resolve().parent
    state=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
    if not state.connected or state.armed or state.mode not in ('MANUAL','OFFBOARD') or state.system_status!=3:
        raise SystemExit('飞控状态未满足：connected=%s，armed=%s，mode=%s，system_status=%s（要求 connected=True、armed=False、MANUAL/OFFBOARD、system_status=3）。' % (state.connected,state.armed,state.mode,state.system_status))
    for parameter,expected in [('/uav1/uav_control_fsm/bench_disarmed',True),
                               ('/uav1/uav_control_fsm/bench_allow_armed',False),
                               ('/vla_direct_bridge/enable_publish',False),
                               ('/vla_direct_bridge/preview_pair_alignment',True)]:
        if rospy.get_param(parameter,None) is not expected:
            raise SystemExit('保护配置异常：'+parameter)
    publishers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    if publishers.get('/uav1/move_command'):
        raise SystemExit('检测到其他控制目标发布者：'+str(publishers['/uav1/move_command']))
    if publishers.get('/uav1/mavros/setpoint_raw/local')!=['/uav1/uav_control_fsm']:
        raise SystemExit('setpoint 来源不符合单份现有控制器要求。')
    rc=rospy.wait_for_message('/uav1/mavros/rc/in',RCIn,timeout=5)
    if not rc_is_ready(rc.channels,require_low_throttle=not args.allow_offboard_throttle):
        channels=list(rc.channels)
        raise SystemExit('遥控器条件未满足：通道数=%s，油门/CH3=%s（%s），CH9=%s（要求<1300）。' % (len(channels),channels[2] if len(channels)>2 else '无','交给PX4预检' if args.allow_offboard_throttle else '要求<1100',channels[8] if len(channels)>8 else '无'))
    diag=rospy.wait_for_message('/diagnostics',DiagnosticArray,timeout=5)
    prearm=None
    for status in diag.status:
        if status.name=='uav1/mavros: System':
            for value in status.values:
                if value.key.startswith('pre-arm check status'):prearm=value.value
    if prearm!='Ok':raise SystemExit('未观察到飞控预检 Ok，当前：'+str(prearm))
    expected={'run_id':rospy.get_param('/vla_direct_bridge/run_id'),
              'instruction':rospy.get_param('/vla_direct_bridge/active_instruction')}
    report=json.loads((base/'runtime/model_chain_report.json').read_text(encoding='utf-8'))
    if not (report.get('status')=='passed' and report.get('armed_ever') is False
            and report.get('run_id')==expected['run_id'] and report.get('instruction')==expected['instruction']
            and len(report.get('cases',[]))==8 and all(c.get('passed') for c in report['cases'])):
        raise SystemExit('需要当前轮次、当前指令的 8 个未解锁模型传递样本通过。')
    deadline=time.monotonic()+15
    p=None
    while time.monotonic()<deadline:
        try:
            candidate=json.loads(rospy.wait_for_message('/vla/validated_prediction',String,timeout=3).data)
        except (rospy.ROSException,ValueError):continue
        now=rospy.Time.now().to_sec()
        if (same_task(candidate,expected,rospy.get_param('/vla_direct_bridge/active_instruction'),
                      rospy.get_param('/vla_direct_bridge/run_id'))
                and 0<=now-candidate.get('observation_stamp',-1)<=2
                and 0<=now-candidate.get('validated_stamp',-1)<=1):
            p=candidate;break
    if p is None:raise SystemExit('没有新鲜且通过桥接检查的模型目标；不会进行电机测试。')
    snapshot=dict(expected,mode=state.mode,armed=state.armed,prearm=prearm,
                  rc_channels=list(rc.channels),request_id=p['request_id'],updated_unix=time.time(),
                  scope='read-only checks; physical prop removal/restraint must be confirmed by operator')
    (base/'runtime/motor_precheck.json').write_text(json.dumps(snapshot,ensure_ascii=False,indent=2),encoding='utf-8')
    print('检查通过：未解锁、预检 Ok、%s、单份控制器、8 个模型传递样本通过。'%('油门交给PX4预检、CH9低' if args.allow_offboard_throttle else '油门/CH9低'))
    print('指令：%s；新鲜模型 request=%s；当前模式=%s' % (expected['instruction'],p['request_id'],state.mode))
    print('本次仅检查，没有调用解锁或切模式服务。')


if __name__=='__main__':main()
