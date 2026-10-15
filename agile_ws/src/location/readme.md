# location — 定位模块

## 1. 概述

`location/` 是 **agile_ws 的定位模块目录**，集中放置 Livox 雷达驱动与 FAST-LIO 激光惯性里程计，负责将雷达/IMU 原始数据转换为**实时位姿与点云地图**，供 `planner`、`uav_control_fsm` 等上层模块使用。

典型数据流：

```
Livox MID360 硬件
    → livox_ros_driver2（点云 + IMU）
    → fast_lio（里程计 / 局部建图）
    → /uav{N}/Odometry、点云等
```

---

## 2. 目录结构

| 目录 | 作用 |
|------|------|
| `livox_ros_driver2/` | Livox 第二代 ROS 驱动，连接雷达硬件，发布点云与 IMU 话题 |
| `fast_lio/` | FAST-LIO2 激光惯性里程计，融合点云与 IMU，输出位姿与注册点云 |

### 多机配置

多机时每架无人机各起一套 **driver + fast_lio**，并保证：

1. **`uav_id` 一致**：`livox_ros_driver2` 与 `fast_lio` 的 launch 传入**相同的** `uav_id`（如 `uav_id:=2`），话题自动带前缀 `/uav2/`。
2. **每机独立雷达配置**：为每架机准备一份 `MID360_config.json`（或在 launch 中指定不同的 `user_config_path`），配置各自的 `lidar_ip`、`host_ip` 与端口，避免网络冲突。

单机默认话题示例（`uav_id=1`）：

- 驱动输出：`/uav1/livox/lidar`、`/uav1/livox/imu`
- 里程计输出：`/uav1/Odometry`、`/uav1/cloud_registered` 等

`fast_lio` 配置文件中话题可写为 `/livox/lidar`、`/livox/imu`，节点会按 `uav_id` 自动添加 `/uav{N}` 前缀。

---

## 3. 快速启动

### 典型启动（Livox MID360，单机）

先启动雷达驱动，再启动里程计（默认 `uav_id=1`）：

```bash
roslaunch livox_ros_driver2 360_driver.launch
roslaunch fast_lio fast_lio.launch
```
