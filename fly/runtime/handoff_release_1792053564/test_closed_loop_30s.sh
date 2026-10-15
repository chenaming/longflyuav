#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
set -u
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$*" != '--props-removed --fixed' ]]; then
  echo '用法：bash test_closed_loop_30s.sh --props-removed --fixed'
  echo '30秒拆桨台架，起飞10秒/悬停2秒为计时模拟；末尾AUTO.LAND确认地面后普通上锁。'
  exit 2
fi
python3 "$BASE/ownership_release.py"
python3 "$BASE/motor_precheck.py" --allow-offboard-throttle
exec python3 "$BASE/bench60.py" --props-removed --fixed
