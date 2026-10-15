#!/bin/bash
set -e
SRC=$(cd "$(dirname "$0")" && pwd)
BASE=/home/nvidia/agile
if [[ ! -f "$SRC/fsm_control.hpp" && -f "$SRC/vla_autostart_bundle/fsm_control.hpp" ]]; then SRC="$SRC/vla_autostart_bundle"; fi
mkdir -p "$BASE/runtime/vla/backups" "$BASE/vla_commissioning"
STAMP=$(date +%Y%m%d-%H%M%S)
for name in vla_config.json 使用说明.md; do
    [[ ! -f "$BASE/$name" ]] || cp "$BASE/$name" "$BASE/runtime/vla/backups/$name.$STAMP"
done
for name in vla_env.sh vla_start.sh vla_stop.sh vla_status.sh vla_test.sh vla_motor_test.sh vla_stack.py vla_acceptance.py vla_report_publisher.py vla_bench.launch vla_config.json vla_direct_bridge.py vla_monitor.py 使用说明.md; do
    [[ "$SRC/$name" == "$BASE/$name" ]] || cp "$SRC/$name" "$BASE/$name"
done
for name in bench_validate.py model_bench_validate.py model_motor_validate.py motor_stop.py motor_watchdog.py test_coordinates.py test_motor_stop.py vla_direct_bridge.py; do
    cp "$SRC/$name" "$BASE/vla_commissioning/$name"
done
HEADER="$BASE/agile_ws/src/uav_control_fsm/include/uav_control_fsm/fsm_control.hpp"
if ! cmp -s "$SRC/fsm_control.hpp" "$HEADER"; then
    cp "$HEADER" "$BASE/runtime/vla/backups/fsm_control.hpp.$STAMP"
    cp "$SRC/fsm_control.hpp" "$HEADER"
    source /opt/ros/noetic/setup.bash
    cd "$BASE/agile_ws"
    catkin_make --pkg uav_control_fsm -j2 -l2
fi
chmod +x "$BASE"/vla_*.sh "$BASE"/vla_*.py
python3 - <<'PY'
import subprocess
r=subprocess.run(['crontab','-l'],capture_output=True,text=True)
lines=[x for x in r.stdout.splitlines() if '# agile-vla-autostart' not in x]
lines.append('@reboot /bin/sleep 20 && /bin/bash /home/nvidia/agile/vla_start.sh >> /home/nvidia/agile/runtime/vla/boot.log 2>&1 # agile-vla-autostart')
subprocess.run(['crontab','-'],input='\n'.join(lines)+'\n',text=True,check=True)
PY
echo 'Installed NX boot entry. Start: /home/nvidia/agile/vla_start.sh'
