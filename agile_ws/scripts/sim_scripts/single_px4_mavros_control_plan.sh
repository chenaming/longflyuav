#!/usr/bin/env bash
# PX4 + MAVROS + uav_control_fsm（各开一个终端，不含 AirSim 节点）
set -eo pipefail

[[ -n "${DISPLAY:-}" ]] || { echo "需要 DISPLAY，请在图形桌面下运行" >&2; exit 1; }

AGILE_WS="${AGILE_WS:-$HOME/agile/agile_ws}"
PX4_WS="${PX4_WS:-$HOME/PX4-Autopilot}"

ROS_ENV="source /opt/ros/noetic/setup.bash && \
source ${AGILE_WS}/devel/setup.bash && \
source ${PX4_WS}/Tools/setup_gazebo.bash ${PX4_WS} ${PX4_WS}/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=${PX4_WS}:${PX4_WS}/Tools/sitl_gazebo:\${ROS_PACKAGE_PATH:-}"

exec gnome-terminal \
  --tab --title="PX4+MAVROS" -e "bash -c \"sleep 3; ${ROS_ENV}; roslaunch uav_control_fsm sim_single_px4_mavros_airsim.launch; exec bash\"" \
  --tab --title="FSM"        -e "bash -c \"sleep 10; ${ROS_ENV}; roslaunch uav_control_fsm sim_single_uav_control_fsm.launch; exec bash\""
