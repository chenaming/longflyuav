#!/usr/bin/env bash
# ROS Noetic: one window, five tabs. First roslaunch starts the ROS master.
set -o pipefail
SELF=$(readlink -f "${BASH_SOURCE[0]}")
BASE=$(dirname "$SELF")
STATE="${XDG_STATE_HOME:-$HOME/.local/state}/uav-bringup"
ODIN_WS=${ODIN_WS:-$BASE/odin_ws}
AGILE_WS=${AGILE_WS:-$BASE/agile_ws}
GAP=${UAV_START_GAP:-5}
COUNT=5
NAMES=(odin mavros control_fsm ego_planner rviz)

usage() {
    cat <<'EOF'
Usage: ./start_uav.sh [--delay SECONDS | --sensors-only | --check | --status | --stop]
  default       Open one window with five tabs; start each stage 5 seconds apart.
  --delay N     Set the interval between stage launches (0-300 seconds).
  --sensors-only  Open only Odin and MAVROS for connection checks.
  --check       Check workspaces, launch files and flight-controller device.
  --status      Show the latest run's process state and log directory.
  --stop        Stop only processes created by the latest run, in reverse order.
Environment: ODIN_WS, AGILE_WS, UAV_START_GAP, FCU_URL (default /dev/ttyTHS0:921600).
Odin uses ns:=uav1 to match the control node's odometry subscription.
EGO uses exp_single_planner.launch. The fifth tab opens Odin and EGO RViz.
No separate roscore is launched; the first roslaunch starts the master if needed.
Startup does not mean sensors are ready.
The existing FSM can enter Offboard/arm in response to RC or control commands;
this launcher adds no flight commands. Use only in a suitable ground-test setup.
EOF
}

setup_ros() {
    # ROS uses local HTTP/XML-RPC; do not pass it through shell HTTP proxies.
    unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
    source /opt/ros/noetic/setup.bash || return
    if [[ $1 == 0 ]]; then
        cd "$ODIN_WS" && source devel/setup.bash
    else
        cd "$AGILE_WS" && source devel/setup.bash
    fi
}

command_for() {
    case $1 in
        0) CMD=(roslaunch odin_ros_driver odin1_ros1.launch ns:=uav1);;
        1) CMD=(roslaunch uav_control_fsm px4.launch "fcu_url:=${FCU_URL:-/dev/ttyTHS0:921600}");;
        2) CMD=(roslaunch uav_control_fsm exp_single_uav_control_fsm.launch);;
        3) CMD=(roslaunch ego_planner exp_single_planner.launch);;
        4) CMD=(roslaunch "$STATE/rviz.launch");;
        *) return 1;;
    esac
}

master_ready() {
    python3 - <<'PY'
import os, socket, sys, xmlrpc.client
socket.setdefaulttimeout(1)
try:
    code, _, _ = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')).getPid('/uav_bringup_check')
    sys.exit(0 if code == 1 else 1)
except Exception:
    sys.exit(1)
PY
}

wait_for_fcu() {
    echo 'Waiting up to 90s for a connected/disarmed FCU, readable parameters and fresh Odin odometry...'
    python3 - "$RUN/stop" <<'PY'
import os, rospy, sys, time
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
rospy.init_node('uav_bringup_wait_fcu', anonymous=True, disable_signals=True)
deadline = time.monotonic() + 90
while time.monotonic() < deadline and not rospy.is_shutdown():
    if os.path.exists(sys.argv[1]):
        print('Startup cancelled.')
        sys.exit(1)
    try:
        msg = rospy.wait_for_message('/uav1/mavros/state', State, timeout=2)
        if not msg.connected:
            time.sleep(0.2)
            continue
        if msg.armed:
            print('FCU is armed; cancelled controller startup. No flight state was changed.')
            sys.exit(1)
        rospy.wait_for_service('/uav1/mavros/param/get', timeout=2)
        param = rospy.ServiceProxy('/uav1/mavros/param/get', ParamGet)('MAV_SYS_ID')
        if not param.success:
            time.sleep(0.5)
            continue
        odom = rospy.wait_for_message('/uav1/odin1/odometry', Odometry, timeout=2)
        age = (rospy.Time.now() - odom.header.stamp).to_sec()
        if not -0.1 <= age <= 1.0:
            print('Odin timestamp is not fresh: age=%.3fs; waiting.' % age)
            time.sleep(0.5)
            continue
        print('FCU connected/disarmed, MAV_SYS_ID readable, Odin odometry fresh.')
        sys.exit(0)
    except (rospy.ROSException, rospy.ServiceException):
        pass
print('MAVROS is not ready; controller was not started.')
sys.exit(1)
PY
}

prepare_rviz() {
    mkdir -p "$STATE" || return
    python3 - "$STATE" "$ODIN_WS" "$AGILE_WS" <<'PYRVIZ'
from pathlib import Path
import sys, yaml
from xml.sax.saxutils import quoteattr
state, odin, agile = map(Path, sys.argv[1:])
# Keep separate copies so simulation and vendor configs are unchanged.
target = state/'odin_uav1.rviz'
if not target.exists():
    text = (odin/'src/odin_ros_driver/config/odin_ros.rviz').read_text()
    target.write_text(text.replace('/odin1/', '/uav1/odin1/'))
target = state/'ego_uav1.rviz'
if not target.exists():
    cfg = yaml.safe_load((agile/'src/planner/ego_planner/plan_manage/launch/ego_change.rviz').read_text())
    def adjust(displays):
        for d in displays:
            if d.get('Name') == 'ori_points':
                d.update({'Topic': '/uav1/livox/lidar_map', 'Name': 'Odin world cloud', 'Enabled': True, 'Value': True, 'Style': 'Points', 'Size (Pixels)': 2})
            if d.get('Name') == 'octomap_points':
                d['Topic'] = '/uav1_planner_node/grid_map/occupancy'
            adjust(d.get('Displays', []))
    adjust(cfg['Visualization Manager']['Displays'])
    cfg['Visualization Manager']['Tools'] = [t for t in cfg['Visualization Manager'].get('Tools', []) if t.get('Class') != 'rviz_plugins/Goal3DTool']
    target.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False))
lines = ['<launch>', '  <env name="DISABLE_ROS1_EOL_WARNINGS" value="1"/>']
for name in ('odin', 'ego'):
    args = quoteattr('-d ' + str(state/(name+'_uav1.rviz')))
    lines.append(f'  <node name="{name}_rviz" pkg="rviz" type="rviz" args={args} output="screen"/>')
lines.append('</launch>')
(state/'rviz.launch').write_text('\n'.join(lines)+'\n')
PYRVIZ
}

check() {
    prepare_rviz || return 1
    local i failed=0
    for p in /opt/ros/noetic/setup.bash "$ODIN_WS/devel/setup.bash" "$AGILE_WS/devel/setup.bash"; do
        [[ -r $p ]] || { echo "MISSING: $p"; failed=1; }
    done
    (( failed == 0 )) || return 1
    for i in 0 1 2 3 4; do
        echo "Checking ${NAMES[i]}..."
        (setup_ros "$i" && command_for "$i" && roslaunch --nodes "${CMD[@]:1}") || failed=1
    done
    local fcu=${FCU_URL:-/dev/ttyTHS0:921600}
    if [[ $fcu == /dev/* ]]; then
        [[ -r ${fcu%%:*} && -w ${fcu%%:*} ]] || echo "WARNING: FCU device ${fcu%%:*} is missing or inaccessible; MAVROS cannot connect."
    fi
    return "$failed"
}

# Match both the saved PID and unique run ID before sending any signal.
owned_pid() {
    [[ -r $1 ]] || return 1
    read -r PID < "$1"
    [[ $PID =~ ^[0-9]+$ ]] || return 1
    python3 - "$PID" "$RUN" <<'PY'
from pathlib import Path
import sys
try:
    p = Path('/proc') / sys.argv[1]
    env = p.joinpath('environ').read_bytes().split(b'\0')
    cmd = p.joinpath('cmdline').read_bytes().split(b'\0')
    run = sys.argv[2].encode()
    ok = b'UAV_BRINGUP_RUN=' + run in env or (run in cmd and b'--tab' in cmd)
    sys.exit(0 if ok else 1)
except (OSError, ValueError):
    sys.exit(1)
PY
}

stop_run() {
    [[ -d $RUN ]] || { echo 'No recorded run.'; return 0; }
    touch "$RUN/stop"
    local i j
    for i in 4 3 2 1 0; do
        if owned_pid "$RUN/$i.child"; then
            echo "Stopping ${NAMES[i]} ($PID)..."
            kill -INT -- "-$PID" 2>/dev/null || true
            for j in {1..150}; do owned_pid "$RUN/$i.child" || break; sleep 0.2; done
            if owned_pid "$RUN/$i.child"; then kill -TERM -- "-$PID" 2>/dev/null || true; fi
        fi
    done
    # Close only this run's terminal wrappers, including tabs waiting at read.
    python3 - "$RUN" <<'PY'
import os, signal, sys
from pathlib import Path
run = sys.argv[1].encode()
for p in Path('/proc').glob('[0-9]*'):
    try:
        if p.stat().st_uid != os.getuid(): continue
        args = p.joinpath('cmdline').read_bytes().split(b'\0')
        if b'--tab' in args and run in args and any(a.endswith(b'/start_uav.sh') for a in args[:2]):
            os.kill(int(p.name), signal.SIGTERM)
    except (OSError, ValueError): pass
PY
    echo "Stopped owned launches and tabs. Logs: $RUN"
}

run_tab() {
    # Each tab is a separate invocation; keep these alive for EXIT cleanup.
    i=$1; prev=$(( $1 - 1 )); child=''; rc=0
    RUN=$2; GAP=$3
    printf '%s\n' "$$" > "$RUN/$i.tab"
    # Log command output and keep the tab open after exit for diagnosis.
    exec > >(tee -a "$RUN/${NAMES[i]}.log") 2>&1
    finish_tab() {
        trap - INT TERM HUP
        if [[ -n $child ]] && owned_pid "$RUN/$i.child"; then
            kill -INT -- "-$child" 2>/dev/null || true
            for j in {1..150}; do kill -0 "$child" 2>/dev/null || break; sleep 0.2; done
            kill -TERM -- "-$child" 2>/dev/null || true
        fi
        rm -f "$RUN/$i.child" "$RUN/$i.tab"
    }
    trap 'finish_tab; exit 130' INT TERM HUP
    trap finish_tab EXIT
    if (( i > 0 )); then
        echo "Waiting for ${NAMES[prev]} to start, then ${GAP}s..."
        for j in {1..600}; do
            [[ -e $RUN/stop ]] && return
            [[ -e $RUN/$prev.started ]] && break
            sleep 0.2
        done
        [[ -e $RUN/$prev.started ]] || { echo 'Previous tab did not start; cancelled.'; return 1; }
        sleep "$GAP"
    fi
    [[ -e $RUN/stop ]] && return
    setup_ros "$i" || { echo 'Workspace setup failed.'; return 1; }
    # roslaunch owns the master it starts, or reuses an existing master.
    {
        if (( i > 0 )); then
            for j in {1..30}; do
                [[ -e $RUN/stop ]] && return
                master_ready && break
                sleep 1
            done
            master_ready || { echo 'ROS master unavailable; cancelled.'; return 1; }
        fi
        if (( i == 2 )); then
            wait_for_fcu || { echo 'Resolve the FCU connection before retrying the full launch.'; return 1; }
        fi
        [[ -e $RUN/stop ]] && return
        command_for "$i" || return
        echo "[$(date -Is)] Starting ${NAMES[i]}: ${CMD[*]}"
        export UAV_BRINGUP_RUN=$RUN
        # Separate process group permits targeted shutdown, never blanket pkill.
        setsid "$SELF" --exec "$i" > >(tee -a "$RUN/${NAMES[i]}.node.log") 2>&1 &
        child=$!
        printf '%s\n' "$child" > "$RUN/$i.child"
        date +%s > "$RUN/$i.started"
        wait "$child"; rc=$?
        rm -f "$RUN/$i.child"
        child=''
        echo "[$(date -Is)] ${NAMES[i]} exited with code $rc."
    }
    printf '%s\n' "$rc" > "$RUN/$i.exit"
    rm -f "$RUN/$i.tab"
    trap - EXIT INT TERM HUP
    read -r -p 'Press Enter to close this tab. ' || true
}

case ${1:-} in
    --exec) setup_ros "$2" && command_for "$2" && exec "${CMD[@]}"; exit $?;;
    --tab) run_tab "$2" "$3" "$4"; exit $?;;
    -h|--help) usage; exit 0;;
    --check) check; exit $?;;
    --sensors-only) COUNT=2;;
    --stop|--status)
        RUN=$(readlink -f "$STATE/latest" 2>/dev/null)
        [[ -d $RUN ]] || { echo 'No recorded run.'; exit 0; }
        if [[ $1 == --stop ]]; then stop_run; else
            echo "Logs: $RUN"
            for i in 0 1 2 3 4; do
                if owned_pid "$RUN/$i.child"; then echo "${NAMES[i]}: running (PID $PID; readiness not implied)"
                elif owned_pid "$RUN/$i.tab"; then echo "${NAMES[i]}: waiting"
                elif [[ -r $RUN/$i.exit ]]; then echo "${NAMES[i]}: exited ($(cat "$RUN/$i.exit"))"
                else echo "${NAMES[i]}: stopped/not started"; fi
            done
        fi; exit 0;;
    --delay) GAP=${2:-}; [[ $# == 2 ]] || { usage; exit 2; };;
    '') ;;
    *) usage; exit 2;;
esac
[[ $GAP =~ ^[0-9]+$ ]] && (( GAP <= 300 )) || { echo 'Delay must be 0-300 seconds.'; exit 2; }
check || exit 1
for tool in gnome-terminal setsid flock python3; do command -v "$tool" >/dev/null || { echo "Missing tool: $tool"; exit 1; }; done
mkdir -p "$STATE"
exec 9>"$STATE/start.lock"
flock -n 9 || { echo 'Another launcher is starting.'; exit 1; }
RUN=$(readlink -f "$STATE/latest" 2>/dev/null)
if [[ -d $RUN ]]; then
    for p in "$RUN"/*.child "$RUN"/*.tab; do
        if owned_pid "$p"; then echo 'A previous launch is active. Use --status or --stop first.'; exit 1; fi
    done
fi

# Reuse this user's existing graphical session when invoked over SSH.
if [[ -z ${DISPLAY:-} ]]; then
    eval "$(python3 - <<'PY'
import os, shlex
from pathlib import Path
for p in Path('/proc').glob('[0-9]*'):
    try:
        if p.stat().st_uid != os.getuid(): continue
        comm = (p/'comm').read_text().strip()
        if not comm.startswith(('gnome-terminal', 'gnome-session')): continue
        env = dict(x.split(b'=',1) for x in (p/'environ').read_bytes().split(b'\0') if b'=' in x)
        if b'DISPLAY' not in env: continue
        for k in ('DISPLAY','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS'):
            if k.encode() in env: print('export '+k+'='+shlex.quote(env[k.encode()].decode()))
        break
    except (OSError, ValueError): pass
PY
)"
fi
[[ -n ${DISPLAY:-} ]] || { echo 'No desktop session found. Run this script in a desktop terminal.'; exit 1; }
RUN=$(mktemp -d "$STATE/run-$(date +%Y%m%d-%H%M%S)-XXXX")
ln -sfn "$RUN" "$STATE/latest"
export ODIN_WS AGILE_WS FCU_URL
args=()
for ((i=0; i<COUNT; i++)); do
    if (( i == 0 )); then args+=(--window); else args+=(--tab); fi
    # --command is required for distinct per-tab commands in one invocation.
    printf -v cmd '%q ' /bin/bash "$SELF" --tab "$i" "$RUN" "$GAP"
    args+=(--title="$((i+1)). ${NAMES[i]}" --command="$cmd")
done
gnome-terminal "${args[@]}" 9>&- || exit 1
# Give all tab processes time to register before allowing another launch.
for attempt in {1..50}; do
    count=0
    for ((i=0; i<COUNT; i++)); do [[ -e $RUN/$i.tab || -e $RUN/$i.started ]] && ((count+=1)); done
    (( count == COUNT )) && break
    sleep 0.2
done
echo "Opened $COUNT tabs, interval ${GAP}s. Logs: $RUN"
echo "Status: $SELF --status"
echo "Stop:   $SELF --stop"
