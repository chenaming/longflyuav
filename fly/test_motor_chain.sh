#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
mkdir -p "$BASE/runtime"
if [[ $# -eq 1 && "$1" == '--check-only' ]]; then
  exec python3 "$BASE/motor_precheck.py"
fi
if [[ $# -ne 2 || "$1" != '--props-removed' || "$2" != '--fixed' ]]; then
  echo '仅检查：test_motor_chain.sh --check-only'
  echo '现场确认全部拆桨、可靠固定并可停止后：test_motor_chain.sh --props-removed --fixed'
  exit 1
fi
python3 "$BASE/motor_precheck.py"
echo '即将进行正常解锁的 5 秒电机台架测试；请在现场观察，并准备随时停止。'
echo '目标限制：水平≤2cm，竖直≤3mm，yaw≤1°；不会起飞。'
echo '测试由现有 supervisor 临时交接控制器；Odin、MAVROS 和桥接继续运行。'
TEST_EXIT=0
python3 "$BASE/vla_acceptance.py" --motor --props-removed --fixed > "$BASE/runtime/motor_chain.log" 2>&1 || TEST_EXIT=$?
python3 - "$BASE" "$TEST_EXIT" <<'PY'
import json,sys
from pathlib import Path
import rospy
from mavros_msgs.msg import State
base=Path(sys.argv[1]);code=int(sys.argv[2])
precheck=json.loads((base/'runtime/motor_precheck.json').read_text(encoding='utf-8'))
source=Path('/home/nvidia/agile/runtime/vla/motor_report.json')
if not source.exists():raise SystemExit('没有本次电机报告，请查看 '+str(base/'runtime/motor_chain.log'))
r=json.loads(source.read_text())
if (r.get('updated_unix',0)<precheck['updated_unix'] or r.get('run_id')!=precheck['run_id']
        or r.get('instruction')!=precheck['instruction']):
    raise SystemExit('没有取得本次任务的新电机报告，请查看 '+str(base/'runtime/motor_chain.log'))
(base/'runtime/motor_chain_report.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
rospy.init_node('fly_motor_chain_result',anonymous=True)
s=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
print('测试状态：'+str(r.get('status')))
print('解锁响应：'+str(r.get('arming_response')))
print('不同模型请求：'+str(r.get('distinct_model_predictions')))
print('模型对应 setpoint：%s / %s' % (r.get('matched_model_setpoints'),r.get('armed_setpoints')))
print('最终状态：armed=%s，mode=%s' % (s.armed,s.mode))
print('保护恢复：bench_allow_armed='+str(rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed',None)))
if r.get('error'):print('失败原因：'+str(r['error']))
if r.get('restore_error'):print('恢复问题：'+str(r['restore_error']))
print('报告：'+str(base/'runtime/motor_chain_report.json'))
cleanup=Path('/home/nvidia/agile/runtime/vla/motor_wrapper_cleanup.json')
stopped=json.loads(cleanup.read_text()) if cleanup.exists() else {}
watchdog_path=Path('/home/nvidia/agile/runtime/vla/watchdog_report.json')
watchdog=json.loads(watchdog_path.read_text()) if watchdog_path.exists() else {}
watchdog_ok=(bool(r.get('session_token')) and watchdog.get('session_token')==r.get('session_token')
    and watchdog.get('armed_seen') is True and watchdog.get('final_armed') is False
    and not watchdog.get('error') and not watchdog.get('stop_result',{}).get('force_disarm_used',False)
    and not watchdog.get('stop_result',{}).get('normal_error'))
print('正常停机检查：'+str(r.get('normal_cleanup_passed')))
print('本轮看门狗检查：'+str(watchdog_ok))
ok=(code==0 and r.get('model_target_transport_verified') is True
    and r.get('final_armed') is False and r.get('restored_mode')=='MANUAL'
    and r.get('stop_result',{}).get('force_disarm_used') is False
    and r.get('normal_cleanup_passed') is True
    and stopped.get('disarmed_confirmed') is True and stopped.get('force_disarm_used') is False
    and not stopped.get('normal_error') and not stopped.get('mode_restore_error')
    and watchdog_ok
    and s.connected and not s.armed and s.mode=='MANUAL'
    and rospy.get_param('/uav1/uav_control_fsm/bench_allow_armed',None) is False)
if not ok:raise SystemExit('本次电机台架测试尚未完整通过；先查看报告和日志。')
print('模型到已解锁控制器的传递已验证，正常上锁及默认保护已恢复。')
print('电机是否实际转动仍需你现场观察；本测试不证明自主飞到椅子。')
PY
