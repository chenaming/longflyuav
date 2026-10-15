#!/usr/bin/env bash
# Low-latency RTSP playback for uav_rtsp_stream
# Usage: play_rtsp.sh [rtsp_url]

URL="${1:-rtsp://127.0.0.1:8554/uav1/live}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# GStreamer rtspsrc ignores no_proxy; local RTSP must not go through Clash.
if [[ "$URL" =~ ^rtsp://(127\.0\.0\.1|localhost)(:|/) ]]; then
  unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY
fi
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/setup_gstreamer_nv_env.sh"

if [[ -x /usr/bin/gst-launch-1.0 ]]; then
  GST_LAUNCH=/usr/bin/gst-launch-1.0
  GST_INSPECT=/usr/bin/gst-inspect-1.0
elif command -v gst-launch-1.0 >/dev/null 2>&1; then
  GST_LAUNCH="$(command -v gst-launch-1.0)"
  GST_INSPECT="$(command -v gst-inspect-1.0)"
else
  echo "ERROR: gst-launch-1.0 not found"
  exit 1
fi

export LD_LIBRARY_PATH=""
export DISPLAY="${DISPLAY:-:0.0}"

USE_GPU_DEC=0
if [[ -x "${GST_INSPECT}" ]] && "${GST_INSPECT}" nvh264dec >/dev/null 2>&1; then
  USE_GPU_DEC=1
  DECODER_NAME="nvh264dec"
else
  DECODER_NAME="avdec_h264 (CPU)"
fi

echo "Playing: $URL"
echo "Decoder: ${DECODER_NAME}"
echo "Sink: ximagesink"

if [[ "${USE_GPU_DEC}" -eq 1 ]]; then
  exec "${GST_LAUNCH}" -e \
    rtspsrc location="$URL" latency=0 drop-on-latency=true protocols=tcp \
    ! rtph264depay ! h264parse ! nvh264dec ! videoconvert \
    ! ximagesink sync=false force-aspect-ratio=true
fi

exec "${GST_LAUNCH}" -e \
  rtspsrc location="$URL" latency=0 drop-on-latency=true protocols=tcp \
  ! rtph264depay ! h264parse ! avdec_h264 max-threads=2 output-corrupt=false \
  ! videoconvert ! ximagesink sync=false force-aspect-ratio=true
