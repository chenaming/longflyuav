#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
set -u
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$*" != '--props-removed --fixed' ]]; then
  echo '用法：bash test_closed_loop_60s.sh --props-removed --fixed'
  echo '拆桨固定台架：60秒电机窗口，起飞10秒/悬停2秒均为计时模拟。'
  exit 2
fi
python3 "$BASE/motor_precheck.py" --allow-offboard-throttle
exec python3 "$BASE/bench60.py" --props-removed --fixed
