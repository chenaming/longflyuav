#!/usr/bin/env bash
set -eo pipefail
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
unset http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY
setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "$BASE/agile_ws/devel/setup.bash"
}
setup_ros
mode=true
if [[ ${1:-} == --live ]]; then mode=false; shift; fi
if [[ ${1:-} == --preview ]]; then shift; fi
exec roslaunch agile_openvla_bridge openvla_ego_bridge.launch \
  "server_url:=${OPENVLA_URL:-http://127.0.0.1:15008/predict}" \
  "instruction:=fly forward" "dry_run:=$mode" "$@"
