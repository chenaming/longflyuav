#!/usr/bin/env bash
# 一键启动单机栈，每个 launch 单独开一个终端窗口
#
# 用法:
#   ./launch_all.sh                 # 默认 uav_id=1，FSM 会 reboot 飞控
#   ./launch_all.sh --no-rc         # 跳过 FSM 自动 reboot
#   ./launch_all.sh --uav-id 1
#   ./launch_all.sh --bg            # 无图形界面时：后台启动并写日志
#   ./launch_all.sh --help

set -o pipefail

WS="${WS:-$HOME/ljzn_ws}"
UAV_ID="${UAV_ID:-1}"
NO_RC="false"
USE_TERMINAL="auto"   # auto | yes | no
MAVROS_WAIT="${MAVROS_WAIT:-60}"
REBOOT_WAIT="${REBOOT_WAIT:-90}"
LOG_DIR="${LOG_DIR:-$HOME/.ros/uav_stack_logs}"

LAUNCH_PIDS=()

usage() {
    cat <<EOF
用法: $(basename "$0") [选项]

选项:
  --uav-id ID     无人机编号，默认 ${UAV_ID}
  --no-rc         FSM 使用 no_RC:=true，不自动 reboot 飞控
  --mavros-wait S 首次等待 MAVROS 连接超时（秒），默认 ${MAVROS_WAIT}
  --reboot-wait S FSM reboot 后等待 MAVROS 重连超时（秒），默认 ${REBOOT_WAIT}
  --bg            强制后台模式（写日志，不弹终端）
  --help          显示帮助

每个 launch 会单独开一个终端窗口（需图形桌面）:
  1. roscore（若未运行）
  2. px4.launch
  3. exp_single_uav_control_fsm.launch
  4. rs_camera.launch
  5. vins.launch

后台模式日志: ${LOG_DIR}
EOF
}

log() { echo "[$(date '+%H:%M:%S')] $*"; }

cleanup_bg() {
    log "停止后台 roslaunch..."
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${LAUNCH_PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --uav-id)      UAV_ID="$2"; shift 2 ;;
        --no-rc)       NO_RC="true"; shift ;;
        --mavros-wait) MAVROS_WAIT="$2"; shift 2 ;;
        --reboot-wait) REBOOT_WAIT="$2"; shift 2 ;;
        --bg)          USE_TERMINAL="no"; shift ;;
        -h|--help)     usage; exit 0 ;;
        *)             echo "未知参数: $1"; usage; exit 1 ;;
    esac
done

UAV_NS="uav${UAV_ID}"
MAVROS_STATE="/${UAV_NS}/mavros/state"

# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash
# shellcheck source=/dev/null
source "$WS/devel/setup.bash"

mkdir -p "$LOG_DIR"

pick_terminal_mode() {
    if [[ "$USE_TERMINAL" == "no" ]]; then
        return 1
    fi
    if [[ "$USE_TERMINAL" == "yes" ]]; then
        return 0
    fi
    [[ -n "${DISPLAY:-}" ]] || return 1
    command -v gnome-terminal >/dev/null 2>&1 && return 0
    command -v xfce4-terminal >/dev/null 2>&1 && return 0
    command -v mate-terminal >/dev/null 2>&1 && return 0
    command -v konsole >/dev/null 2>&1 && return 0
    command -v lxterminal >/dev/null 2>&1 && return 0
    command -v xterm >/dev/null 2>&1 && return 0
    return 1
}

ros_env_cmd() {
    printf 'source /opt/ros/noetic/setup.bash && source %q/devel/setup.bash' "$WS"
}

# 在新终端中执行命令；终端关闭前保留窗口便于查看报错
open_terminal() {
    local title="$1"
    local run_cmd="$2"
    local inner
    inner="$(ros_env_cmd); echo '=== ${title} ==='; ${run_cmd}; echo; echo '[${title}] 进程已退出，按 Enter 关闭窗口...'; read -r"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$title" -- bash -lc "$inner" &
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --title="$title" --command="bash -lc $(printf %q "$inner")" &
    elif command -v mate-terminal >/dev/null 2>&1; then
        mate-terminal --title="$title" -- bash -lc "$inner" &
    elif command -v konsole >/dev/null 2>&1; then
        konsole --new-tab -p tabtitle="$title" -e bash -lc "$inner" &
    elif command -v lxterminal >/dev/null 2>&1; then
        lxterminal --title="$title" -e bash -lc "$inner" &
    elif command -v xterm >/dev/null 2>&1; then
        xterm -title "$title" -e bash -lc "$inner" &
    else
        log "错误: 找不到可用的终端程序"
        return 1
    fi
}

launch_bg() {
    local name="$1"; shift
    local logfile="$LOG_DIR/${name}.log"
    log "后台启动 ${name} ..."
    roslaunch "$@" >"$logfile" 2>&1 &
    LAUNCH_PIDS+=("$!")
    log "  PID=$!  日志: ${logfile}"
}

launch_in_terminal() {
    local title="$1"; shift
    local pkg="$1" file="$2"
    shift 2
    local args=("$@")
    local arg_str=""
    local a
    for a in "${args[@]}"; do
        arg_str+=" $(printf %q "$a")"
    done
    log "打开终端: ${title}"
    open_terminal "$title" "roslaunch $(printf %q "$pkg") $(printf %q "$file")${arg_str}"
}

mavros_connected() {
    rostopic echo -n 1 "$MAVROS_STATE" 2>/dev/null | grep -q "connected: True"
}

wait_mavros() {
    local msg="$1" timeout="$2" t=0
    log "等待 ${msg}（最多 ${timeout}s）..."
    while (( t < timeout )); do
        if mavros_connected; then
            log "${msg} 成功"
            return 0
        fi
        sleep 1
        t=$((t + 1))
        (( t % 5 == 0 )) && log "  ... 已等待 ${t}s"
    done
    log "错误: ${msg} 超时"
    return 1
}

TERMINAL_MODE=false
if pick_terminal_mode; then
    TERMINAL_MODE=true
    log "使用独立终端模式"
else
    log "无图形终端，使用后台+log 模式（可用 --bg 显式指定）"
    trap cleanup_bg SIGINT SIGTERM EXIT
fi

# ---------- roscore ----------
if ! rostopic list >/dev/null 2>&1; then
    if $TERMINAL_MODE; then
        log "打开终端: roscore"
        open_terminal "roscore" "roscore"
    else
        log "启动 roscore ..."
        roscore >"$LOG_DIR/roscore.log" 2>&1 &
        LAUNCH_PIDS+=("$!")
    fi
    sleep 3
fi

launch_one() {
    local title="$1" name="$2" pkg="$3" file="$4"
    shift 4
    if $TERMINAL_MODE; then
        launch_in_terminal "$title" "$pkg" "$file" "$@"
    else
        launch_bg "$name" "$pkg" "$file" "$@"
    fi
}

# ---------- 1. MAVROS / PX4 ----------
launch_one "PX4/MAVROS" "px4" uav_control_fsm px4.launch "uav_id:=${UAV_ID}"
sleep 2
wait_mavros "MAVROS 首次连接" "$MAVROS_WAIT" || exit 1

# ---------- 2. FSM ----------
if [[ "$NO_RC" == "true" ]]; then
    launch_one "UAV Control FSM" "fsm" uav_control_fsm exp_single_uav_control_fsm.launch \
        "uav_id:=${UAV_ID}" "no_RC:=true"
    log "no_RC:=true，跳过 reboot 等待"
    sleep 3
else
    launch_one "UAV Control FSM" "fsm" uav_control_fsm exp_single_uav_control_fsm.launch \
        "uav_id:=${UAV_ID}"
    log "FSM 将应用定位源参数并 reboot 飞控，等待 MAVROS 重连..."
    sleep 8
    t=0
    while (( t < 25 )); do
        if ! mavros_connected; then
            log "检测到飞控已断开（reboot 中）"
            break
        fi
        sleep 1
        t=$((t + 1))
    done
    wait_mavros "飞控 reboot 后 MAVROS 重连" "$REBOOT_WAIT" || {
        log "警告: MAVROS 重连超时，RealSense/VINS 仍会继续启动"
    }
fi

# ---------- 3. RealSense ----------
launch_one "RealSense Camera" "realsense" realsense2_camera rs_camera.launch "uav_id:=${UAV_ID}"
sleep 3

# ---------- 4. VINS ----------
launch_one "VINS-Fusion" "vins" vins vins.launch "uav_id:=${UAV_ID}"

log "=========================================="
log "全部 launch 已启动 (/${UAV_NS}/)"
if $TERMINAL_MODE; then
    log "每个节点在独立终端中运行，请分别查看各窗口输出"
    log "关闭某个终端即可停止对应节点"
else
    log "后台模式日志: ${LOG_DIR}"
    log "按 Ctrl+C 停止所有后台节点"
fi
log "  MAVROS:  ${MAVROS_STATE}"
log "  相机:    /${UAV_NS}/camera/infra1/image_rect_raw"
log "  VINS:    /${UAV_NS}/vins_fusion/imu_propagate"
log "=========================================="

if $TERMINAL_MODE; then
    exit 0
fi

wait
