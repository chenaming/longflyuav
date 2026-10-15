#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
mkdir -p "$BASE/runtime"
# Lock only the relay startup, release before starting the interactive client.
(
  flock -x 9
  if python3 - <<'PY'
import json, urllib.request
try:
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open('http://127.0.0.1:15017/healthz',timeout=2) as r:
        d=json.load(r)
    assert d['service']=='fly-inference-relay-v1'
    assert d['upstream']=='http://10.62.25.215:5007/predict'
except Exception:
    raise SystemExit(1)
PY
  then
    echo '复用已运行的 fly 推理转发。'
  else
    python3 - <<'PY'
import socket
s=socket.socket()
try:s.bind(('127.0.0.1',15017))
except OSError:raise SystemExit('15017 已被其他程序占用，请检查；不会覆盖或停止它。')
finally:s.close()
PY
    nohup python3 -u "$BASE/relay.py" >> "$BASE/runtime/relay.log" 2>&1 < /dev/null 9>&- &
    echo "$!" > "$BASE/runtime/relay.pid"
    sleep 1
    kill -0 "$(cat "$BASE/runtime/relay.pid")"
    echo '已恢复本地推理转发；现有 ROS 和桥接继续运行。'
  fi
) 9> "$BASE/runtime/relay_start.lock"
exec python3 "$BASE/fly_cli.py" "$@"
