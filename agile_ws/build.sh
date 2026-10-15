#!/usr/bin/env bash
set -e
# Limit default parallelism for the 8 GB Orin; explicit catkin_make args override it.
if [[ $# == 0 ]]; then set -- -j2 -l2; fi
WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
cd "$WS"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
source /opt/ros/noetic/setup.bash
[[ -f "$WS/devel/setup.bash" ]] && source "$WS/devel/setup.bash"

catkin_make "$@" --source "$WS/src/uav_control_fsm/" --build "$WS/build/uav_control_fsm/" && source "$WS/devel/setup.bash"
catkin_make "$@" --source "$WS/src/planner/ego_planner/" --build "$WS/build/planner/ego_planner/" && source "$WS/devel/setup.bash"
catkin_make "$@" --source "$WS/src/perception/uav_rtsp_stream/" --build "$WS/build/perception/uav_rtsp_stream/" && source "$WS/devel/setup.bash"
catkin_make "$@" --source "$WS/src/missions/traj_tracking/" --build "$WS/build/missions/traj_tracking/" && source "$WS/devel/setup.bash"

