# perception — 感知模块

## 1. 概述

`perception/` 是 **agile_ws 的感知模块目录**，放置与机载视觉相关的功能包。当前包含 USB 摄像头 **RTSP 局域网推流**，与 VINS / RealSense 链路**独立启动**，互不占用 ROS 图像话题。

典型数据流：

```
USB 摄像头 (/dev/videoX)
    → v4l2src → x264enc
    → GStreamer RTSP Server
    → rtsp://<机载IP>:8554/uav{N}/live
```

---

## 2. 目录结构

| 目录 | 作用 |
|------|------|
| `uav_rtsp_stream/` | USB 摄像头 RTSP 推流（GStreamer RTSP Server + v4l2 直采） |

### 多机配置

每架无人机单独启动一套推流节点，通过 **`uav_id`** 自动区分：

| `uav_id` | RTSP 端口 | 路径 |
|----------|-----------|------|
| 1 | 8554 | `/uav1/live` |
| 2 | 8555 | `/uav2/live` |
| N | 8554 + N − 1 | `/uavN/live` |

---

## 3. 快速启动

### 依赖（首次安装）

```bash
sudo apt install libgstrtspserver-1.0-0 gir1.2-gst-rtsp-server-1.0 gstreamer1.0-plugins-ugly gstreamer1.0-libav v4l-utils
```

### 典型启动（单机 USB 摄像头）

```bash
roslaunch uav_rtsp_stream rtsp_stream.launch
```

局域网观看（VLC / ffplay）：

```text
rtsp://<机载IP>:8554/uav1/live
```

多机示例：

```bash
roslaunch uav_rtsp_stream rtsp_stream.launch uav_id:=2
# → rtsp://<机载IP>:8555/uav2/live
```

---

## 4. 常用参数

配置文件：`/home/why/agile_ws/src/perception/uav_rtsp_stream/config/rtsp_stream.yaml`

| 参数 | 默认 | 说明 |
|------|------|------|
| `device` | `/dev/video0` | V4L2 设备（`v4l2-ctl --list-devices` 查看） |
| `input_format` | `mjpg` | 采集格式：`raw` / `mjpg` / `h264`，须与摄像头支持项一致 |
| `width` / `height` | 640 / 480 | 采集分辨率 |
| `fps` | 30 | 帧率 |
| `bitrate` | 2000 | H.264 码率（kbps）；`h264` 模式下无效 |
| `base_port` | 8554 | 端口基准，实际端口 = base_port + uav_id − 1 |
| `publish_ros_image` | `false` | **宏开关**：`true` 时同时发布 ROS 图像话题 |
| `image_topic` | `camera/image_raw` | ROS 图像话题（相对 `/uav{N}/`） |
| `frame_id` | `camera_link` | 图像消息 `header.frame_id` |

若启动失败，先用 `v4l2-ctl -d /dev/video0 --list-formats-ext` 确认设备支持的分辨率与格式。详见 `uav_rtsp_stream/docs/v4l2-ctl.md`。
