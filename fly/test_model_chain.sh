#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
mkdir -p "$BASE/runtime"
exec 9> /home/nvidia/agile/runtime/vla/test.lock
flock -n 9 || { echo '已有台架测试正在运行，请等待它结束。'; exit 1; }
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != '--check-only' ) ]]; then
  echo '用法：test_model_chain.sh [--check-only]'
  exit 1
fi
python3 - "$BASE" <<'PY'
import json,time,sys
from pathlib import Path
import rosgraph,rospy
from mavros_msgs.msg import State
from std_msgs.msg import String
rospy.init_node('fly_model_chain_precheck',anonymous=True)
state=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
if not state.connected or state.armed or state.mode!='MANUAL' or state.system_status!=3:
    raise SystemExit('要求飞控已连接、未解锁、MANUAL、Standby；不会自动修正状态。')
for parameter,expected in [('/uav1/uav_control_fsm/bench_disarmed',True),
                           ('/uav1/uav_control_fsm/bench_allow_armed',False),
                           ('/vla_direct_bridge/enable_publish',False),
                           ('/vla_direct_bridge/preview_pair_alignment',True)]:
    if rospy.get_param(parameter,None) is not expected:
        raise SystemExit('保护参数不符合测试要求：'+parameter)
publishers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
if publishers.get('/uav1/move_command'):
    raise SystemExit('已有控制目标发布者，请先核对来源：'+str(publishers['/uav1/move_command']))
if publishers.get('/uav1/mavros/setpoint_raw/local')!=['/uav1/uav_control_fsm']:
    raise SystemExit('要求只由现有 uav_control_fsm 发布 setpoint。')
run=rospy.get_param('/vla_direct_bridge/run_id')
instruction=rospy.get_param('/vla_direct_bridge/active_instruction')
deadline=time.monotonic()+12
while time.monotonic()<deadline:
    try:
        p=json.loads(rospy.wait_for_message('/vla/validated_prediction',String,timeout=3).data)
    except (rospy.ROSException,ValueError):
        continue
    now=rospy.Time.now().to_sec()
    if (p.get('run_id')==run and p.get('instruction')==instruction and p.get('target_frame')=='map'
            and 0<=now-p.get('observation_stamp',-1)<=2
            and 0<=now-p.get('validated_stamp',-1)<=1):
        print('检查通过：未解锁保护有效，收到当前指令的新鲜模型预览。')
        print('指令：'+instruction+'；request='+str(p['request_id']))
        session={'run_id':run,'instruction':instruction,'started_unix':time.time()}
        (Path(sys.argv[1])/'runtime/model_chain_session.json').write_text(json.dumps(session,ensure_ascii=False),encoding='utf-8')
        break
else:
    raise SystemExit('未取得当前指令的新鲜、通过桥接检查的模型目标；没有执行控制测试。')
PY
if [[ "${1:-}" == '--check-only' ]]; then
  echo '仅检查完成，没有切换模式或发布控制目标。'
  exit 0
fi
echo '开始未解锁模型传递测试：短暂 OFFBOARD，XY≤5cm、Z≤1cm、yaw≤3°。'
echo '请保持现有输入终端空闲，不修改指令。测试不会调用解锁服务。'
TEST_EXIT=0
python3 /home/nvidia/agile/vla_acceptance.py --stage model > "$BASE/runtime/model_chain.log" 2>&1 || TEST_EXIT=$?
python3 - "$BASE" "$TEST_EXIT" <<'PY'
import json,sys
from pathlib import Path
import rospy
from mavros_msgs.msg import State
base=Path(sys.argv[1]);code=int(sys.argv[2])
source=Path('/home/nvidia/agile/runtime/vla/model_report.json')
if not source.exists():
    raise SystemExit('测试未生成报告，请查看 '+str(base/'runtime/model_chain.log'))
r=json.loads(source.read_text())
session=json.loads((base/'runtime/model_chain_session.json').read_text(encoding='utf-8'))
if (r.get('run_id')!=session['run_id'] or r.get('instruction')!=session['instruction']
        or r.get('updated_unix',0)<session['started_unix']):
    raise SystemExit('没有取得本次会话的新报告，请查看 '+str(base/'runtime/model_chain.log'))
(base/'runtime/model_chain_report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
rospy.init_node('fly_model_chain_result',anonymous=True)
s=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
cases=r.get('cases',[])
print('测试结果：'+str(r.get('status'))+'；模型样本数：'+str(len(cases)))
for c in cases:
    print('%s：通过=%s，command=%s，setpoint=%s，最大位置误差=%.6fm' % (
        c.get('name'),c.get('passed'),c.get('command_count'),c.get('setpoint_count'),c.get('position_max_error_m',-1)))
print('最终状态：armed=%s，mode=%s' % (s.armed,s.mode))
print('报告：'+str(base/'runtime/model_chain_report.json'))
if r.get('error'):print('失败原因：'+str(r['error']))
if r.get('restore_error'):print('模式恢复失败：'+str(r['restore_error']))
ok=(code==0 and r.get('status')=='passed' and len(cases)==8
    and all(c.get('passed') for c in cases) and r.get('armed_ever') is False
    and r.get('restored_mode')=='MANUAL' and s.connected and not s.armed and s.mode=='MANUAL')
if not ok:raise SystemExit('本次尚未完整通过；先查看报告，不能作为电机或飞行成功的证据。')
print('已验证：真实模型的受限目标进入控制器，并产生对应 setpoint；全程未解锁。')
PY
