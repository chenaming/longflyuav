#!/usr/bin/env bash
# Odin1 ROS driver
set -eo pipefail

[[ -n "${DISPLAY:-}" ]] || { echo "需要 DISPLAY，请在图形桌面下运行" >&2; exit 1; }

AGILE_WS="${AGILE_WS:-$HOME/agile/agile_ws}"
ODIN_WS="${ODIN_WS:-$HOME/agile/odin_ws}"

ROS_ENV="source /opt/ros/noetic/setup.bash && \
source ${AGILE_WS}/devel/setup.bash && \
source ${ODIN_WS}/devel/setup.bash"

exec gnome-terminal \
  --tab --title="Odin" -e "bash -c \"sleep 3; ${ROS_ENV}; roslaunch odin_ros_driver odin1_ros1.launch; exec bash\""
