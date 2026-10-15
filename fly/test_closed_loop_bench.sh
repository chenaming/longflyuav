#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
set -euo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$*" != '--props-removed --fixed' ]]; then
  echo '用法：bash test_closed_loop_bench.sh --props-removed --fixed'
  echo '10 秒未解锁阶段模拟 → 8 个未解锁模型样本 → 5 秒受限电机测试。'
  exit 2
fi
exec 9>"$BASE/runtime/closed_loop_flight.lock"
flock -n 9 || { echo '闭环任务已在运行'; exit 1; }
python3 "$BASE/bench_timed_preview.py"
bash "$BASE/test_model_chain.sh"
echo '下一阶段会正常解锁并执行 5 秒受限电机台架测试；保持拆桨固定，现场可停止。'
bash "$BASE/test_motor_chain.sh" --props-removed --fixed
echo '台架序列结束。10 秒阶段为模拟，未验证真实起飞到 1 米、实际导航或落地。'
