#!/usr/bin/env bash
# Shared GStreamer env for uav_rtsp_stream (source from play/run scripts).
export GST_PLUGIN_PATH="${HOME}/.local/lib/x86_64-linux-gnu/gstreamer-1.0:/usr/lib/x86_64-linux-gnu/gstreamer-1.0:${GST_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${HOME}/.local/lib/x86_64-linux-gnu:/usr/lib/wsl/lib:/usr/local/cuda-12.2/lib64:${LD_LIBRARY_PATH:-}"
export GST_REGISTRY="${GST_REGISTRY:-${HOME}/.cache/gstreamer-1.0/registry-uav-rtsp-nvenc.bin}"
