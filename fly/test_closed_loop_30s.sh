#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
set -u
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$BASE/bench_cycle.py" "$@"
