#!/usr/bin/env bash

# 只清理无人机测试程序，保留 Linux、桌面和远程连接。
# 请在飞机已经上锁、电机停转后执行。

echo "========== 清理无人机测试程序 =========="

# 第一步：停止自动启动来源。
starter_pattern='vla_stack\.py|start_.*(uav|vla|odin|openvla).*\.sh'

echo "[1/3] 停止自动启动程序……"
pkill -TERM -f "$starter_pattern" 2>/dev/null || true
sleep 2
pkill -KILL -f "$starter_pattern" 2>/dev/null || true

# 第二步：关闭 ROS、无人机节点及 Python 桥接。
# 包括所有 roslaunch 及 ROS Noetic 节点进程。
target_pattern='roscore|rosmaster|roslaunch|/opt/ros/noetic/lib/|/home/nvidia/agile/[^ ]*/devel/lib/|vla_[^ ]*\.py|send_camera\.py|mavros_node|host_sdk_sample|uav_control_fsm|ego_planner|traj_server|path_recorder|image_overlay|body_to_world'

echo "[2/3] 停止 ROS、Odin、MAVROS、控制器、桥接和监控……"
pkill -TERM -f "$target_pattern" 2>/dev/null || true
sleep 3

# 清理没有正常退出的进程；重复检查，捕获刚启动的子进程。
for attempt in 1 2 3; do
    pkill -KILL -f "$starter_pattern" 2>/dev/null || true
    pkill -KILL -f "$target_pattern" 2>/dev/null || true
    sleep 1
done

# 第三步：核对进程与端口。
echo "[3/3] 检查残留进程和端口……"

remaining="$(pgrep -af "$target_pattern" 2>/dev/null || true)"
listeners="$(ss -lntp 2>/dev/null | grep -E ':11311\b|:8088\b' || true)"

if [ -z "$remaining" ] && [ -z "$listeners" ]; then
    echo
    echo "清理完成：相关进程已退出，11311 和 8088 端口已释放。"
    echo "现在可以从 roscore 开始重新启动。"
    exit 0
fi

echo
echo "仍有残留，或后台服务又启动了程序："

if [ -n "$remaining" ]; then
    printf '%s\n' "$remaining"
fi

if [ -n "$listeners" ]; then
    printf '%s\n' "$listeners"
fi

echo
echo "请把上面的输出发来；先不要启动新程序。"
exit 1
