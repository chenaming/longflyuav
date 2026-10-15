#!/usr/bin/env bash
# Publish one EGO goal in the planner's local map coordinates (metres).
set -o pipefail
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
setup_env() {
    # Do not forward this script's CLI flags to catkin's setup parser.
    source /opt/ros/noetic/setup.bash || return
    source "${AGILE_WS:-$BASE/agile_ws}/devel/setup.bash"
}
setup_env || exit 1
exec python3 - "$@" <<'PY'
import argparse, math, re, sys, threading, time, xmlrpc.client
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import GoalSet
from rosgraph_msgs.msg import Log

p = argparse.ArgumentParser(prog='send_goal.sh', description='Send one EGO goal. Default XYZ: (2, 0, 1) metres in the UAV local map; +Z is up. This does not request arming or Offboard.')
p.add_argument('x', type=float, nargs='?')
p.add_argument('y', type=float, nargs='?')
p.add_argument('z', type=float, nargs='?')
p.add_argument('--relative', action='store_true', help='XYZ offsets from current position along local map axes, NOT body axes')
p.add_argument('--uav-id', type=int, default=1)
p.add_argument('--dry-run', action='store_true', help='Resolve and validate the goal without publishing')
p.add_argument('--timeout', type=float, default=10)
a = p.parse_args()
if all(v is None for v in (a.x, a.y, a.z)):
    a.x, a.y, a.z = 2.0, 0.0, 1.0
elif any(v is None for v in (a.x, a.y, a.z)):
    p.error('Provide all three coordinates X Y Z, or omit all for default (2, 0, 1)')
if a.uav_id < 0 or not all(math.isfinite(v) for v in (a.x, a.y, a.z, a.timeout)) or a.timeout <= 0:
    p.error('Require finite XYZ, positive timeout and nonnegative UAV ID')
rospy.init_node('send_ego_goal', anonymous=True, disable_signals=True)
prefix = '/uav%d' % a.uav_id
planner = prefix + '_planner_node'
try:
    state = rospy.wait_for_message(prefix+'/mavros/state', State, timeout=a.timeout)
    odom = rospy.wait_for_message(prefix+'/mavros/local_position/odom', Odometry, timeout=a.timeout)
    age = (rospy.Time.now()-odom.header.stamp).to_sec()
    if not state.connected or not -.1 <= age <= 1:
        raise RuntimeError('FCU disconnected or local odometry is stale (age %.3fs)' % age)
    goal = [a.x, a.y, a.z]
    if a.relative:
        pos = odom.pose.pose.position
        goal = [v+offset for v,offset in zip(goal, (pos.x,pos.y,pos.z))]
    # Serialize to the actual float32 wire representation before validation.
    import struct
    goal = list(struct.unpack('<3f', struct.pack('<3f', *goal)))
    ground = rospy.get_param(planner+'/grid_map/virtual_ground')
    ceiling = rospy.get_param(planner+'/grid_map/virtual_ceil')
    if not ground < goal[2] < ceiling:
        raise RuntimeError('Goal Z must be between %.3f and %.3f m' % (ground,ceiling))
    frame = rospy.get_param(planner+'/grid_map/frame_id')
    print('Goal: (%.3f, %.3f, %.3f) m; frame=%s; FCU mode=%s armed=%s' % (*goal,frame,state.mode,state.armed), flush=True)
    if a.dry_run:
        print('Dry run: no goal was published.')
        sys.exit(0)
    received = threading.Event()
    sent_at = [None]
    def on_log(msg):
        if sent_at[0] is None or msg.name != planner or msg.header.stamp < sent_at[0]: return
        match = re.search(r'Received goal:\s*([-+\deE.]+),\s*([-+\deE.]+),\s*([-+\deE.]+)', msg.msg)
        if match and all(abs(float(s)-g)<1e-4 for s,g in zip(match.groups(),goal)):
            received.set()
    sub = rospy.Subscriber('/rosout_agg', Log, on_log, queue_size=100)
    pub = rospy.Publisher(prefix+'/goal_point', GoalSet, queue_size=1, latch=False)
    # A rostopic/test monitor can connect before the actual planner. Check the
    # standard ROS node bus API for this specific outbound TCPROS connection.
    node_api = xmlrpc.client.ServerProxy(rospy.get_node_uri())
    def planner_connected():
        code, _, bus = node_api.getBusInfo(rospy.get_name())
        return code == 1 and any(row[1] == planner and row[2] == 'o' and
                                row[4] == prefix+'/goal_point' and row[5]
                                for row in bus)
    deadline = time.monotonic()+a.timeout
    while not planner_connected() and time.monotonic()<deadline and not rospy.is_shutdown():
        time.sleep(.05)
    if not planner_connected():
        raise RuntimeError('No planner subscriber; goal was not sent')
    sent_at[0] = rospy.Time.now()
    pub.publish(GoalSet(goal=goal))
    print('Published once to '+prefix+'/goal_point', flush=True)
    if received.wait(a.timeout):
        print('Planner receive callback confirmed. This is not a guarantee of a feasible path or arrival.')
    else:
        print('Published, but no matching planner receive log was observed. Inspect EGO/RViz before retrying.', file=sys.stderr)
        sys.exit(3)
except (rospy.ROSException, rospy.ServiceException, RuntimeError, KeyError, OverflowError) as exc:
    print('ERROR: '+str(exc), file=sys.stderr)
    sys.exit(1)
except KeyboardInterrupt:
    sys.exit(130)
PY
