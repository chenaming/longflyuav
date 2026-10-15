#!/usr/bin/env bash
# Run in a Mac terminal and keep it open. No saved passwords.
set -euo pipefail
relay_dir=$(mktemp -d /tmp/openvla-relay.XXXXXX)
cleanup() {
  ssh -S "$relay_dir/drone" -O exit "${DRONE_SSH:-nvidia@192.168.1.108}" 2>/dev/null || true
  ssh -S "$relay_dir/server" -O exit -p 7027 howard@39.105.30.146 2>/dev/null || true
  rmdir "$relay_dir" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
ssh -fNM -S "$relay_dir/server" -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:15008:127.0.0.1:5008 -p 7027 howard@39.105.30.146
ssh -fNM -S "$relay_dir/drone" -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
  -R 127.0.0.1:15008:127.0.0.1:15008 "${DRONE_SSH:-nvidia@192.168.1.108}"
echo 'Relay active: drone http://127.0.0.1:15008 -> server :5008. Ctrl-C stops it.'
while ssh -S "$relay_dir/server" -O check -p 7027 howard@39.105.30.146 2>/dev/null && \
      ssh -S "$relay_dir/drone" -O check "${DRONE_SSH:-nvidia@192.168.1.108}" 2>/dev/null; do
  sleep 5
done
