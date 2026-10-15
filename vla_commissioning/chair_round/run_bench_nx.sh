#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
rospy.init_node('bench_restart_precheck',anonymous=True)
s=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
assert s.connected and not s.armed, 'Requires a connected, disarmed FCU'
assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed',False) is True, 'Requires the protected bench FSM'
PY
rosnode kill /vla_bench_validation 2>/dev/null || true
# Diagnostic finite test; returns the FCU to its original mode.
# Run only with propellers removed and the airframe secured.
nohup python3 /home/nvidia/agile/vla_commissioning/bench_validate.py > /home/nvidia/.local/state/vla-commissioning/bench.log 2>&1 < /dev/null &
echo 'Bench test started; evidence: ~/.local/state/vla-commissioning/bench_report.json'
