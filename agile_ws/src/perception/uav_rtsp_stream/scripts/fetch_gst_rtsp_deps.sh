#!/usr/bin/env bash
# Fetch gstreamer-rtsp-server headers into uav_rtsp_stream/deps (when -dev is not installed).
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${PKG_DIR}/deps/gstrtspserver"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

if [[ -f /usr/include/gstreamer-1.0/gst/rtsp-server/rtsp-server.h ]] && \
   pkg-config --exists gstreamer-rtsp-server-1.0; then
  echo "[fetch_deps] system libgstrtspserver-1.0-dev already installed"
  exit 0
fi

if [[ -f "${DEST}/include/gstreamer-1.0/gst/rtsp-server/rtsp-server.h" ]]; then
  echo "[fetch_deps] vendored headers already present: ${DEST}"
  exit 0
fi

cd "${TMP}"
apt-get download libgstrtspserver-1.0-dev
dpkg-deb -x libgstrtspserver-1.0-dev*.deb extract
rm -rf "${DEST}"
mkdir -p "${DEST}/lib"
cp -a extract/usr/include "${DEST}/"
cp -a extract/usr/lib/x86_64-linux-gnu/. "${DEST}/lib/" 2>/dev/null || true
echo "[fetch_deps] OK: ${DEST}"
