#!/usr/bin/env bash
# Restricted SSH command: only ensures this user's existing inference service.
set -euo pipefail
case ${SSH_ORIGINAL_COMMAND:-ensure} in ensure|status|'') ;; *) echo 'Only ensure/status are permitted'; exit 2;; esac
ROOT=/home/howard/UAV-Flow
STATE="$ROOT/.vla-service"
mkdir -p "$STATE"
exec 9>"$STATE/start.lock"
flock -n 9 || { echo 'Another start check is active'; exit 0; }
if /usr/bin/python3 - <<'PY'
import socket,sys
s=socket.socket();s.settimeout(2)
try:s.connect(('127.0.0.1',5007));sys.exit(0)
except OSError:sys.exit(1)
finally:s.close()
PY
then echo 'Inference port 5007 is listening'; exit 0; fi
if /usr/bin/python3 - <<'PY'
from pathlib import Path
import os,sys
for p in Path('/proc').glob('[0-9]*'):
 try:
  if p.stat().st_uid!=os.getuid():continue
  args=(p/'cmdline').read_bytes().split(b'\0')
  if any(a in (b'openvla_act.py',b'/home/howard/UAV-Flow/OpenVLA-UAV/vla-scripts/openvla_act.py') for a in args):sys.exit(0)
 except OSError:pass
sys.exit(1)
PY
then echo 'Inference process is loading; not starting a duplicate'; exit 0; fi
source /home/howard/miniconda3/etc/profile.d/conda.sh
conda activate openvla
cd "$ROOT/OpenVLA-UAV/vla-scripts"
export CUDA_VISIBLE_DEVICES=0
nohup python openvla_act.py >> "$STATE/inference.log" 2>&1 < /dev/null 9>&- &
echo $! > "$STATE/inference.pid"
echo 'Started inference; readiness is verified by real HTTP predictions'
