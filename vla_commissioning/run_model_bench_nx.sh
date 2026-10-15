#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 - <<'PY'
import rospy
from mavros_msgs.msg import State
rospy.init_node('model_bench_restart_precheck',anonymous=True)
s=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
assert s.connected and not s.armed and s.mode=='MANUAL', 'Requires connected/disarmed/MANUAL'
assert rospy.get_param('/uav1/uav_control_fsm/bench_disarmed',False) is True
assert rospy.get_param('/vla_direct_bridge/enable_publish',False) is False
print('Will switch disarmed OFFBOARD for bounded model commands, then restore MANUAL',flush=True)
PY
rosnode kill /vla_model_bench_validation 2>/dev/null || true
nohup python3 /home/nvidia/agile/vla_commissioning/model_bench_validate.py > /home/nvidia/.local/state/vla-commissioning/chair_round/model_bench.log 2>&1 < /dev/null &
echo 'Model bench started; check page step 16 for results'
