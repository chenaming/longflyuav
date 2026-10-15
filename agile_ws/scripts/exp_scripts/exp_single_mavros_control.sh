#!/usr/bin/env bash
# PX4 + MAVROS + uav_control_fsm（各开一个终端）
set -eo pipefail

[[ -n "${DISPLAY:-}" ]] || { echo "需要 DISPLAY，请在图形桌面下运行" >&2; exit 1; }

AGILE_WS="${AGILE_WS:-$HOME/agile/agile_ws}"
ODIN_WS="${ODIN_WS:-$HOME/agile/odin_ws}"

ROS_ENV="source /opt/ros/noetic/setup.bash && \
source ${AGILE_WS}/devel/setup.bash && \
source ${ODIN_WS}/devel/setup.bash"

exec gnome-terminal \
  --tab --title="MAVROS" -e "bash -c \"sleep 3; ${ROS_ENV}; roslaunch uav_control_fsm px4.launch; exec bash\"" \
  --tab --title="FSM" -e "bash -c \"sleep 10; ${ROS_ENV}; roslaunch uav_control_fsm exp_single_uav_control_fsm.launch; exec bash\""
