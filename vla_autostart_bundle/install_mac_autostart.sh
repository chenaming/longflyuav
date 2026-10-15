#!/bin/bash
set -e
echo "Installing Mac VLA login tunnel..."
BASE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$BASE/../.." && pwd)
STATE="$ROOT/work/autostart/mac_tunnel_state"
APP="$HOME/Library/Application Support/AgileVLA"
AGENT="$HOME/Library/LaunchAgents/com.agile.vla-tunnel.plist"
if [[ -r "$STATE/pid" ]]; then
    PID=$(cat "$STATE/pid")
    if [[ "$PID" =~ ^[0-9]+$ ]] && ps -p "$PID" -o args= | grep -F 'vla_mac_tunnel.py' >/dev/null; then
        kill -TERM "$PID"
        for ((i=0;i<50;i++)); do [[ -e "$STATE/pid" ]] || break; sleep .2; done
    fi
fi
mkdir -p "$(dirname "$AGENT")"
cp "$BASE/com.agile.vla-tunnel.plist" "$AGENT"
launchctl bootout "gui/$(id -u)/com.agile.vla-tunnel" 2>/dev/null || true
launchctl bootstrap "gui/$(id -u)" "$AGENT"
sleep 3
launchctl print "gui/$(id -u)/com.agile.vla-tunnel" | head -35
cat "$APP/state/status.json"
