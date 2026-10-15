#!/usr/bin/env bash
# Save one JPEG frame from RTSP to verify the stream has video.
# Usage: check_rtsp_frame.sh [rtsp_url] [output.jpg]

URL="${1:-rtsp://127.0.0.1:8554/uav1/live}"
OUT="${2:-/tmp/uav_rtsp_frame.jpg}"

GST_LAUNCH=/usr/bin/gst-launch-1.0
export GST_PLUGIN_PATH="${GST_PLUGIN_PATH:-/usr/lib/x86_64-linux-gnu/gstreamer-1.0}"
export LD_LIBRARY_PATH=""

echo "Capturing one frame from $URL -> $OUT"
if ! "$GST_LAUNCH" -e \
  rtspsrc location="$URL" latency=0 protocols=tcp num-buffers=60 \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! jpegenc \
  ! filesink location="$OUT" 2>/dev/null; then
  echo "ERROR: failed to capture frame"
  exit 1
fi

if [[ -s "$OUT" ]]; then
  echo "OK: saved $(stat -c%s "$OUT") bytes to $OUT"
else
  echo "ERROR: output file empty"
  exit 1
fi
