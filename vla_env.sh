#!/bin/bash
export VLA_BASE=/home/nvidia/agile
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
export ROS_MASTER_URI=http://localhost:11311
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/noetic/setup.bash
source "$VLA_BASE/agile_ws/devel/setup.bash"
mkdir -p "$VLA_BASE/runtime/vla"
