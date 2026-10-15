# v4l2-ctl 使用说明

## 1. 概述

`v4l2-ctl` 是 **V4L2（Video4Linux2）** 的命令行工具，用于查看和操作 Linux 下的 `/dev/video*` 摄像头设备。配置 `uav_rtsp_stream` 推流前，通常先用它确认**设备节点、分辨率、像素格式**是否与 `rtsp_stream.yaml` 一致。

工具来自 `v4l-utils` 包（RTSP 依赖安装时已包含）：

```bash
sudo apt install v4l-utils
```

指定设备时，`-d` 可写完整路径或编号：

```bash
v4l2-ctl -d /dev/video0   # 与 -d 0 等价
```

---

## 2. 设备发现

### 列出所有摄像头

```bash
v4l2-ctl --list-devices
```

输出示例（同一物理摄像头可能对应多个节点）：

```text
USB 2.0 Camera: USB Camera (usb-...):
    /dev/video2
    /dev/video3
    /dev/media1
```

**注意：** 同一摄像头常出现 `/dev/videoX` 与 `/dev/videoX+1` 两个节点。通常 **偶数编号（video0、video2）为 Video Capture**，奇数编号多为 Metadata，推流应选 Capture 节点。用 `--info` 确认：

```bash
v4l2-ctl -d /dev/video2 --info
```

在 `Device Caps` 中应看到 `Video Capture`。

> 当前系统版本中，`--list-devices` **没有** `-A` 短选项；列设备请直接用 `--list-devices`。

---

## 3. 设备信息查询

| 选项 | 作用 | 示例 |
|------|------|------|
| `-D`, `--info` | 显示驱动与设备能力 | `v4l2-ctl -d /dev/video0 -D` |
| `--all` | 显示该设备全部信息 | `v4l2-ctl -d /dev/video0 --all` |
| `--log-status` | 将设备状态写入内核日志（排查驱动问题时用） | `v4l2-ctl -d /dev/video0 --log-status` |

---

## 4. 格式与分辨率

配置 `/home/why/agile_ws/src/perception/uav_rtsp_stream/config/rtsp_stream.yaml` 中的 `device`、`width`、`height`、`fps` 前，必须先查设备支持项。

| 选项 | 作用 | 示例 |
|------|------|------|
| `--list-formats` | 列出支持的像素格式名称 | `v4l2-ctl -d /dev/video0 --list-formats` |
| `--list-formats-ext` | 列出格式、分辨率及帧率（**最常用**） | `v4l2-ctl -d /dev/video0 --list-formats-ext` |
| `--get-fmt-video` | 获取当前视频格式 | `v4l2-ctl -d /dev/video0 --get-fmt-video` |
| `--set-fmt-video` | 设置宽、高、像素格式 | 见下方示例 |

### 设置格式示例

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG
```

`pixelformat` 必须是 `--list-formats-ext` 输出中存在的格式。

### 常见像素格式

| 格式 | 说明 | 对 RTSP 的影响 |
|------|------|----------------|
| `YUYV` / `UYVY` | 未压缩 raw | 带宽大，CPU 编码压力高 |
| `MJPG` | 摄像头内 MJPEG 压缩 | 多数 USB 摄像头默认格式，带宽小 |
| `H264` | 摄像头直接输出 H.264 | 少数设备支持，可省编码步骤 |

若设备只支持 `MJPG` 而不支持 raw YUV，GStreamer pipeline 需增加 `jpegdec` 等解码环节。

---

## 5. 摄像头参数（亮度、曝光等）

| 选项 | 作用 | 示例 |
|------|------|------|
| `-l`, `--list-ctrls` | 列出所有可调参数及当前值 | `v4l2-ctl -d /dev/video0 -l` |
| `-L`, `--list-ctrls-menus` | 列出带菜单选项的参数 | `v4l2-ctl -d /dev/video0 -L` |
| `-C`, `--get-ctrl` | 读取指定参数 | `v4l2-ctl -d /dev/video0 -C brightness,exposure_auto` |
| `-c`, `--set-ctrl` | 设置指定参数 | `v4l2-ctl -d /dev/video0 -c brightness=128` |

### 常用参数示例

```bash
# 手动曝光
v4l2-ctl -d /dev/video0 -c exposure_auto=3
v4l2-ctl -d /dev/video0 -c exposure_absolute=250

# 亮度 / 对比度
v4l2-ctl -d /dev/video0 -c brightness=128
v4l2-ctl -d /dev/video0 -c contrast=32
```

参数名因摄像头型号而异，以 `-l` 输出为准。

---

## 6. 视频流采集与测试

不启动 ROS 也可验证摄像头能否正常出帧。

| 选项 | 作用 |
|------|------|
| `--stream-mmap` | 使用 mmap 方式开始采集 |
| `--stream-to <file>` | 将采集数据写入文件（`-` 表示 stdout） |
| `--stream-count <n>` | 采集帧数（默认持续采集） |
| `--stream-skip <n>` | 跳过前 n 帧 |

### 仅测试采集（不落盘）

终端会打印流信息，用于确认驱动与格式是否正常：

```bash
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --stream-mmap
```

按 `Ctrl+C` 停止。

### 抓取单帧到文件

```bash
# MJPEG → 保存为 jpg
v4l2-ctl -d /dev/video0 \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --stream-mmap --stream-count=1 --stream-to=frame.jpg

# Raw 格式 → 保存为 raw（需自行解码查看）
v4l2-ctl -d /dev/video0 \
  --stream-mmap --stream-count=1 --stream-to=test.raw
```

### 连续采集多帧

```bash
v4l2-ctl -d /dev/video0 \
  --stream-mmap --stream-count=100 --stream-to=/dev/null
```

---

## 7. 帮助与其它选项

```bash
v4l2-ctl --help              # 总览
v4l2-ctl --help-all          # 全部选项
v4l2-ctl --help-streaming    # 采集 / 推流相关
v4l2-ctl --help-vidcap       # 视频采集格式
v4l2-ctl --help-selection    # 裁剪区域
```

---

## 8. RTSP 推流配置流程

配置 `uav_rtsp_stream` 时，建议按以下顺序执行：

```bash
# ① 找到摄像头及 /dev/video 节点
v4l2-ctl --list-devices

# ② 确认是 Video Capture 节点
v4l2-ctl -d /dev/video2 --info

# ③ 查看支持的分辨率与格式
v4l2-ctl -d /dev/video2 --list-formats-ext

# ④ 可选：手动设置格式并抓一帧验证
v4l2-ctl -d /dev/video2 \
  --set-fmt-video=width=640,height=480,pixelformat=MJPG \
  --stream-mmap --stream-count=1 --stream-to=test.jpg

# ⑤ 按结果修改 rtsp_stream.yaml（含 input_format），再启动推流
# 配置文件：/home/why/agile_ws/src/perception/uav_rtsp_stream/config/rtsp_stream.yaml
#   --list-formats-ext 见 YUYV/UYVY → input_format: raw
#   --list-formats-ext 见 MJPG      → input_format: mjpg
#   --list-formats-ext 见 H264      → input_format: h264
roslaunch uav_rtsp_stream rtsp_stream.launch
```

---

## 9. 速查表

| 目的 | 命令 |
|------|------|
| 有哪些摄像头 | `v4l2-ctl --list-devices` |
| 设备详情与能力 | `v4l2-ctl -d 0 -D` |
| 支持哪些分辨率 / 格式 | `v4l2-ctl -d 0 --list-formats-ext` |
| 当前格式 | `v4l2-ctl -d 0 --get-fmt-video` |
| 设置格式 | `v4l2-ctl -d 0 --set-fmt-video=width=640,height=480,pixelformat=MJPG` |
| 可调参数列表 | `v4l2-ctl -d 0 -l` |
| 读 / 写参数 | `v4l2-ctl -d 0 -C brightness` / `-c brightness=128` |
| 测试采集 | `v4l2-ctl -d 0 --stream-mmap --stream-count=1 --stream-to=test.jpg` |
| 全部信息 | `v4l2-ctl -d 0 --all` |

## 10. 低延迟测试指令

在 Windows 或本机用 ffplay 拉流时，可用下列命令降低播放端缓冲（推流端需已启动）：

```bash
ffplay -fflags nobuffer -flags low_delay -framedrop -rtsp_transport tcp -sync ext rtsp://127.0.0.1:8554/uav1/live
```

| 参数 | 作用 |
|------|------|
| `-fflags nobuffer` | 输入侧少缓冲，尽量不「先存一大坨再往下送」 |
| `-flags low_delay` | 解码/处理按低延迟模式，少做「为了平滑而等待」 |
| `-framedrop` | 解码/显示跟不上时丢旧帧，不一直排队等 |
| `-rtsp_transport tcp` | RTSP 走 TCP，更稳，和延迟关系不大 |
| `-sync ext` | 用外部时钟同步，不强行按播放时间轴拖住画面 |

虚拟机内也可用仓库脚本 `scripts/play_rtsp.sh`，效果类似（GStreamer：`latency=0`、`sync=false`）。
