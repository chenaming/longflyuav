#!/usr/bin/env bash
# Ubuntu 20.04 + WSL: build gst-plugins-bad NVENC (nvh264enc) — not in apt by default.
set -euo pipefail

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run as normal user (script uses sudo where needed)." >&2
  exit 1
fi

echo "[nvenc] installing build dependencies..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  gstreamer1.0-tools \
  nvidia-cuda-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  build-essential autoconf automake libtool pkg-config \
  libdrm-dev git

PC_DIR="${HOME}/.local/lib/x86_64-linux-gnu/pkgconfig"
mkdir -p "${PC_DIR}"
if [[ ! -f /usr/lib/x86_64-linux-gnu/pkgconfig/cuda-10.1.pc ]]; then
  cat > "${PC_DIR}/cuda-10.1.pc" <<'EOF'
prefix=/usr
libdir=${prefix}/lib/x86_64-linux-gnu
includedir=${prefix}/include
Name: CUDA
Description: CUDA toolkit 10.1
Version: 10.1
Cflags: -I${includedir}
Libs: -L${libdir} -lcuda -lcudart
EOF
  cat > "${PC_DIR}/cudart-10.1.pc" <<'EOF'
prefix=/usr
libdir=${prefix}/lib/x86_64-linux-gnu
includedir=${prefix}/include
Name: CUDA Runtime
Description: CUDA Runtime 10.1
Version: 10.1
Cflags: -I${includedir}
Libs: -L${libdir} -lcudart
EOF
  export PKG_CONFIG_PATH="${PC_DIR}:${PKG_CONFIG_PATH:-}"
fi

BUILD_DIR="${BUILD_DIR:-/tmp/gst-plugins-bad-nvenc}"
if [[ ! -d "${BUILD_DIR}/.git" ]]; then
  rm -rf "${BUILD_DIR}"
  git clone --depth 1 --branch 1.16.3 \
    https://gitlab.freedesktop.org/gstreamer/gst-plugins-bad.git "${BUILD_DIR}"
fi

cd "${BUILD_DIR}"
export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
if [[ ! -f Makefile ]]; then
  ./autogen.sh --disable-gtk-doc --disable-examples --disable-tests --with-cuda-prefix=/usr
fi

make -C sys/nvenc -j"$(nproc)"

PLUGIN_SRC="${BUILD_DIR}/sys/nvenc/.libs/libgstnvenc.so"
PLUGIN_DST="/usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnvenc.so"
sudo cp "${PLUGIN_SRC}" "${PLUGIN_DST}"
rm -f "${HOME}/.cache/gstreamer-1.0/registry"*.bin

if gst-inspect-1.0 nvh264enc >/dev/null 2>&1; then
  echo "[nvenc] OK: nvh264enc available at ${PLUGIN_DST}"
else
  echo "[nvenc] build finished but gst-inspect nvh264enc failed; check LD_LIBRARY_PATH=/usr/lib/wsl/lib" >&2
  exit 1
fi
