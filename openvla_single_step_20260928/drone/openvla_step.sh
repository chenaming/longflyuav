#!/usr/bin/env bash
set -eo pipefail
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "$BASE/agile_ws/devel/setup.bash"
}
setup_ros
exec rosrun agile_openvla_bridge openvla_step_client.py "$@"
