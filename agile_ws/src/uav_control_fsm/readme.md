# uav_control_fsm — 飞行控制 FSM

## 1. 概述

`uav_control_fsm` 是 **agile_ws 的飞行控制 FSM 包**，负责单机/多机的 **PX4 + MAVROS 状态机控制**：接收上层运动指令，管理解锁、Offboard、起飞/悬停/降落，将外部里程计转发给飞控，并发布 TF 与 setpoint。

典型数据流（配合 `location` 的 Fast-LIO）：

```
/uav{N}/Odometry（fast_lio）
    → odometry_bridge（vision_pose → MAVROS）
    → uav_control_fsm_node（FSM + setpoint）
    → /uav{N}/mavros/setpoint_raw/local
    ← 上层 planner / 任务模块（/uav{N}/move_command 等）
```

---

## 2. 目录结构


| 目录                | 作用                                                                                                                   |
| ----------------- | -------------------------------------------------------------------------------------------------------------------- |
| `launch/`         | 启动入口：`uav_control_fsm_unit.launch`（核心单元）、`sim_single_*`（仿真）、`exp_single_*`（实飞）、`dual_uav_*`（多机）、`px4.launch`（MAVROS） |
| `config/`         | 参数：`uav_group.yaml`（PX4/定位源/TF）、`pos_smooth_param.yaml`（平滑参数）、`path_recorder.yaml`（飞行轨迹记录）                           |
| `src/`、`include/` | 节点与库：`uav_control_fsm_node`、`odometry_bridge`、`control_manager`、`uav_path_recorder_node`                             |
| `msg/`            | 控制消息 `uav_command.msg`（Move_mode 0–10）                                                                               |




### 多机配置

多机时每架机各起一套 FSM，并保证：

1. `uav_id` **一致**：launch 中 `uav_id` 决定命名空间 `/uavX`（**从 1 起编**，单机默认为 `uav1`），须与 `location`、`mavros` 使用同一 ID。
2. `world_map_transform`：为每机设置不同的 `world → uavX_map` 静态位姿（如 `dual_uav_control_fsm.launch` 中各机 y 向偏移）。
3. **定位源**：在 `config/uav_group.yaml` 中设置 `default_location_source`（`Fast_Lio` / `Vins` / `Gazebo` / `GPS`），并配合 launch 的 `enable_odom_remap`、`odom_remap_topic` 对接外部里程计。
4. **MAVROS 独立实例**：每机单独起一个 `px4.launch`，三参数须与该机飞控、FSM、`location` 一一对应：

  | 参数           | 含义            | 配置要点                                                                                                                              |
  | ------------ | ------------- | --------------------------------------------------------------------------------------------------------------------------------- |
  | `uav_id`     | ROS 命名空间      | 与 FSM、里程计一致，MAVROS 话题在 `/uavX/mavros/...` 下                                                                                       |
  | `fcu_url`    | 飞控连接地址        | **实飞**：各机 USB 串口不同，如 `/dev/ttyACM0`、`/dev/ttyACM1`（可用 udev 固定为 `/dev/px4fmu`）；**仿真**：各机 UDP 端口不同，如 `udp://:14540@127.0.0.1:14580` |
  | `tgt_system` | MAVLink 系统 ID | 须与 PX4 参数 `MAV_SYS_ID` 一致；**与** `uav_id` **同号**（uav1→`1`，uav2→`2`，…）                                                              |

   **QGC 飞控参数（每架机单独连接 USB 设置）**：

  | QGC 参数           | 多机要求        | 说明                                                               |
  | ---------------- | ----------- | ---------------------------------------------------------------- |
  | `MAV_SYS_ID`     | **必改，各机不同** | 与 `px4.launch` 的 `tgt_system`、`uav_id` 一致；uav1→`1`，uav2→`2`，以此类推 |
  | `EKF2_AID_MASK`  | 按定位源        | 使用 Fast-LIO / VINS 时设为 `24`（视觉 + 外部导航）；纯 GPS 为 `1`               |
  | `EKF2_HGT_MODE`  | 按定位源        | 视觉定位时设为 `3`（Vision）；GPS 为 `1`                                    |
  | `COM_ARM_WO_GPS` | 视觉定位建议      | 设为 `1`，允许无 GPS 解锁                                                |

  > `EKF2_*`、`COM_ARM_WO_GPS` 等也可写在 `config/uav_group.yaml` 的 `location_profiles` 中，由 FSM 启动时经 MAVROS 自动下发；`MAV_SYS_ID` **必须在 QGC 中为每架机预先设好**，无法通过 ROS 侧批量区分硬件。
   QGC 路径：**Vehicle Setup → Parameters**，搜索参数名修改后 **Save**，再 **Reboot** 飞控。
   **实飞双机示例**（两架飞控分别接 `ttyACM0` / `ttyACM1`，`MAV_SYS_ID` 为 1 / 2）：
   再按相同 `uav_id` 分别启动 FSM 与定位（`sim_single_*` / `exp_single_*`、`fast_lio` 等）。
   **验证连接**：
   串口权限与 udev 规则见 `docs/PX4串口权限与MAVROS重连配置.md`。

单机默认话题示例（`uav_id=1`，Fast-LIO 仿真）：

- 输入：`/uav1/fast_lio_odometry`（由 launch remap 自 `/uav1/Odometry`）
- 输出：`/uav1/mavros/setpoint_raw/local`、`/uav1/mavros/vision_pose/pose`
- 控制：`/uav1/move_command`（`uav_command`）

---



## 3. 快速启动



### 典型启动（仿真单机 + Fast-LIO）

需已启动 MAVROS 与 `location`（`fast_lio`），且 `config/uav_group.yaml` 中 `default_location_source: Fast_Lio`：

```bash
roslaunch uav_control_fsm sim_single_uav_control_fsm.launch
```

（默认 `uav_id=1`；多机时显式传入，如 `uav_id:=2`。）

---



## 4. 常用参数

MAVROS / QGC 多机相关参数见 **§2 多机配置**。下表为日常调试最常改项。

### Launch 参数


| 参数                        | 配置文件/位置                                                                                                                                                                                    | 作用                                                            |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------- |
| `uav_id`                  | `/home/why/agile_ws/src/uav_control_fsm/launch/sim_single_uav_control_fsm.launch` `/home/why/agile_ws/src/uav_control_fsm/launch/exp_single_uav_control_fsm.launch`                        | 命名空间 `/uavX`，从 1 起编                                           |
| `no_RC`                   | `/home/why/agile_ws/src/uav_control_fsm/launch/sim_single_uav_control_fsm.launch`（默认 `true`） `/home/why/agile_ws/src/uav_control_fsm/launch/exp_single_uav_control_fsm.launch`（默认 `false`） | 仿真无 RC / 实飞有 RC；影响是否自动 Offboard、是否 reboot 飞控改 EKF             |
| `takeoff_height`          | 同上两个 launch 文件                                                                                                                                                                             | 自动起飞目标高度 (m)                                                  |
| `cmd_timeout_sec`         | `uav_control_fsm_unit.launch`（及 sim/exp 透传，默认 `0.5`）                                                                                                                                       | 控制指令超时：`CMD_CTRL` → `AUTO_HOVER`（不定降）                          |
| `odom_timeout_sec`        | 同上（默认 `0.5`）                                                                                                                                                                               | 里程计超时：退出 Offboard → `MANUAL_CTRL`                               |
| `loop_hz`                 | 同上两个 launch 文件                                                                                                                                                                             | FSM 主循环频率 (Hz)                                                |
| `enable_odom_remap`       | 同上两个 launch 文件                                                                                                                                                                             | 是否将外部里程计 remap 到 bridge 订阅名                                   |
| `odom_remap_topic`        | 同上两个 launch 文件                                                                                                                                                                             | 里程计源话题后缀（仿真常用 `fast_lio_odometry`，实飞 VINS 常用 `vins_odometry`） |
| `odom_remap_to_topic`     | 同上两个 launch 文件                                                                                                                                                                             | remap 目标后缀（Fast-LIO 常用 `Odometry`）                            |
| `vision_pose_pub_rate_hz` | 同上两个 launch 文件                                                                                                                                                                             | 转发飞控 vision_pose 频率；`0`=跟里程计同频，仿真常用 `30`                      |
| `world_map_transform`     | 同上两个 launch 文件 多机：`/home/why/agile_ws/src/uav_control_fsm/launch/dual_uav_control_fsm.launch`                                                                                              | `world → uavX_map` 静态位姿 `[x,y,z,roll,pitch,yaw]`              |
| `pos_smooth_enable`       | 同上两个 launch 文件                                                                                                                                                                             | Move_mode=0 是否启用软件位置平滑                                        |
| `rc_offboard_channel`     | `/home/agilebrain/agile_ws/src/uav_control_fsm/launch/exp_single_uav_control_fsm.launch`                                                                                                          | Offboard 开关 RC 通道（1-based；真机 exp 默认 6）                               |
| `fcu_url`                 | `/home/why/agile_ws/src/uav_control_fsm/launch/px4.launch`                                                                                                                                 | 飞控串口或 UDP 地址                                                  |
| `tgt_system`              | `/home/why/agile_ws/src/uav_control_fsm/launch/px4.launch`                                                                                                                                 | MAVLink 系统 ID，须与 QGC `MAV_SYS_ID`、`uav_id` 一致                 |




### `uav_group.yaml`

文件：`/home/why/agile_ws/src/uav_control_fsm/config/uav_group.yaml`


| 参数                                         | 作用                                           |
| ------------------------------------------ | -------------------------------------------- |
| `default_location_source`                  | 默认定位源：`Fast_Lio` / `Vins` / `Gazebo` / `GPS` |
| `px4_params_common.MPC_XY_VEL_MAX`         | PX4 水平最大速度 (m/s)                             |
| `px4_params_common.GF_MAX_HOR_DIST`        | 地理围栏水平范围 (m)                                 |
| `px4_params_common.GF_MAX_VER_DIST`        | 地理围栏最大高度 (m)                                 |
| `location_profiles.*.MC_YAWRATE_MAX`       | 各定位源下最大偏航角速度 (°/s)                           |
| `sensors.lidar_link.mid360_link.transform` | MID360 相对 `base_link` 外参 `[x,y,z,r,p,y]`     |
| `sensors.camera_link.D435i_link.transform` | D435i 相对 `camera_link` 外参                    |




### `pos_smooth_param.yaml`

文件：`/home/why/agile_ws/src/uav_control_fsm/config/pos_smooth_param.yaml`


| 参数                          | 作用                        |
| --------------------------- | ------------------------- |
| `pos_smooth_max_vel`        | Move_mode=0 平滑最大线速度 (m/s) |
| `pos_smooth_max_accel`      | 最大线加速度 (m/s²)             |
| `pos_smooth_arrival_dist`   | 到达判定距离 (m)                |
| `pos_smooth_lookahead_dist` | 每步 setpoint 沿目标方向前视 (m)   |




### `path_recorder.yaml`

文件：`/home/why/agile_ws/src/uav_control_fsm/config/path_recorder.yaml`


| 参数                  | 作用                            |
| ------------------- | ----------------------------- |
| `save_to_file`      | 是否保存飞行轨迹 CSV                  |
| `save_dir`          | CSV 保存目录                      |
| `min_dist`          | 相邻记录点最小间距 (m)                 |
| `path_publish_rate` | `/uavX/flight_path` 发布频率 (Hz) |




### 运行时


| 方式                                                                          | 作用                              |
| --------------------------------------------------------------------------- | ------------------------------- |
| `rostopic pub /uav1/set_location_source std_msgs/String "data: 'Fast_Lio'"` | 运行时切换定位源（可能触发 PX4 参数更新与 reboot） |


