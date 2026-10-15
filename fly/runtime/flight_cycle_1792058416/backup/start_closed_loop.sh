#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
cd /home/nvidia/agile/fly
exec python3 ./closed_loop_flight.py "$@"
