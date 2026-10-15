source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
python3 - <<'PY'
from pathlib import Path
import json,rospy,rosgraph
from mavros_msgs.msg import State,ExtendedState
from nav_msgs.msg import Odometry
rospy.init_node('read_new_forwarding_failure',anonymous=True)
for topic,cls in [('/uav1/mavros/state',State),('/uav1/mavros/extended_state',ExtendedState),('/uav1/odin1/odometry_highfreq',Odometry)]:
 try:
  m=rospy.wait_for_message(topic,cls,timeout=3)
  print(topic,str(m) if cls!=Odometry else (m.header,m.pose.pose))
 except Exception as e:print(e)
pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState()
print('SUBS',[v for v in subs if 'odometry' in v[0] or 'vision' in v[0]])
print('PUBS',[v for v in pubs if 'vision' in v[0]])
for key in rospy.get_param_names():
 if 'vision_pose_pub_rate' in key:print('PARAM',key,rospy.get_param(key))
for name in ['fly/runtime/bench_cycle_report.json','runtime/vla/status.json','runtime/vla/fsm.log']:
 p=Path('/home/nvidia/agile')/name
 with p.open('rb') as f:
  f.seek(max(0,p.stat().st_size-3500));print(name,f.read().decode(errors='replace'))
PY
