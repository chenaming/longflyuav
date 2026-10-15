# 云边协同视觉语言导航：项目具体实施方案

> 文档定位：后续全部代码开发、联调与真机部署的唯一执行基线。  
> 本文只固化架构、接口、坐标系、阶段验收与安全策略，**不在本阶段生成节点源码**。  
> 版本：v1.0｜日期：2026-09-19｜适用栈：ROS Noetic + Odin1 + uav_control_fsm + UAV-Flow OpenVLA-UAV

---

## 0. 执行原则与范围冻结

### 0.1 本方案必须遵守的工程原则

1. **4090 只做推理，Orin 只做感知、桥接、安全与飞控。** 工作站不加入机载 ROS Master，不远程发布 `/uav1/move_command`。
2. **前期优先打通「VLA 直连控制」闭环**：VLA 输出经 `vla_bridge` 校验、限幅、平滑后，直接发布到 `/uav1/move_command`，**暂不经过 Ego-Planner**。
3. **Ego-Planner 接口必须预留，但不能成为第一阶段阻塞项。** 后续只需改 `vla_bridge` 的后端插件，不改 VLA 服务协议。
4. **所有新建节点必须使用标准 ROS 命名空间、标准消息、可被 `rqt_graph` / `rostopic hz` / `rqt_tf_tree` / `rviz` 直接观察。**
5. **先通信、后坐标、后控制、后起飞。** 任何阶段未通过验收，不得进入下一阶段。
6. **安全优先于模型效果。** 心跳超时、RC 接管、指令限幅、禁飞解锁/起飞，全部在 Orin 本地实现。

### 0.2 当前仓库事实（以源码为准，不以旧文档口头约定为准）

后续开发必须按下面「当前仓库实测接口」写代码。旧计划书中若与本表冲突，以本表为准。

| 项目 | 当前仓库事实 | 对开发的约束 |
|---|---|---|
| 机载 ROS | ROS 1 Noetic | 新包只写 rospy/roscpp，不引入 ROS 2 |
| Odin 工作空间 | `drone_ws/odin_ws`，包名 `odin_ros_driver` | 只订阅，不改 SDK 主路径 |
| 主系统工作空间 | `drone_ws/agile_ws` | `vla_bridge` 建在此工作空间 |
| 控制入口 | `/uav1/move_command`，类型 `uav_control_fsm/uav_command` | VLA 直连只走这条 |
| FSM 主循环 | `loop_hz=30` | 控制发布建议 20–30 Hz |
| 速度/轨迹指令超时 | `cmd_timeout_sec=0.5` | 速度模式必须持续刷新 stamp |
| 默认机号 | `uav_id=1`，命名空间 `/uav1` | 所有话题按 `/uav1` 书写 |
| Odin launch 默认 ns | `odin1_ros1.launch` 的 `ns` **默认为空** | **必须显式** `ns:=/uav1`，否则话题落在 `/odin1/*`，FSM remap 对不上 |
| FSM 里程计 remap | `odom_remap_to_topic:=odin1/odometry` → 订阅 `/uav1/odin1/odometry` | Odin 与 FSM 必须同命名空间 |
| 定位源配置名 | `uav_group.yaml` 里是 `Fast_Lio`，实飞 remap 到 Odin | 不要改名，不要同时再跑 FAST-LIO |
| Odin 去畸变图开关 | `sendrgbundistort: 0` | 第一阶段**不要订阅** `/odin1/image/undistorted`，它默认不发 |
| Odin 压缩图开关 | `sendrgbcompressed: 1` | VLA 图像主源用压缩 JPEG |
| Odin 原始图 | `/odin1/image`，`sensor_msgs/Image`，`bgr8` | 仅本机调试，禁止上云 |
| Odin 相机分辨率 | 设备 NV12 约 1536×1280；标定/去畸变按 1600×1296 | 上云前必须降采样 |
| Odin 相机帧率 | `dtof_fps: 100` → 约 10 Hz；`290` 可达约 29 Hz | 推理只用 2–5 Hz |
| Odin 里程计 | `/odin1/odometry`，`nav_msgs/Odometry`，`frame_id=odom`，`child_frame_id=odin1_base_link` | 给 VLA 的 proprio 从这里取 |
| Odin TF | `odom → odin1_base_link` | 与 FSM 的 `uav1_map → uav1/base_link` 不是同一棵树，必须显式对齐 |
| FSM yaw 单位 | **度 / 度每秒** | VLA 输出 yaw 是**弧度**，发布前必须换算 |
| VLA 推理入口 | `UAV-Flow-main/OpenVLA-UAV/vla-scripts/openvla_act.py` | 复用 `/predict`，不另起协议 |
| VLA 输入尺寸 | 评估脚本固定 resize 到 **224×224** | Orin 端必须先 resize 再上传 |
| VLA 已验证服务 | `10.62.25.215:5007`，`unnorm_key="sim"` | 真机微调后必须改 `unnorm_key` |
| Ego-Planner 预留入口 | `/uav1/goal_point`，`quadrotor_msgs/GoalSet` | 第一阶段只预留，不接线 |
| 现网 | 校园网 + AX3 AP；4090=`10.62.25.215`；Orin 无线约 `10.49.x.x` | 开发期可用；正式飞行再评估 LQ3 |

### 0.3 本阶段不做

- 不在 Orin 上跑 OpenVLA。
- 不量化、不蒸馏、不快慢双系统。
- 不让 VLA 直接发 MAVROS setpoint 或电机油门。
- 不让 4090 直接进 ROS。
- 不在第一阶段同时开发 IndoorUAV / openpi WebSocket。
- 不把 LQ3 SDK 写进第一版通信代码。LQ3 只作为后续可替换的 IP 通道。

---

## 1. 整体架构设计与通信协议

### 1.1 三端职责

```text
┌──────────────────────────────────────────────────────────────────────────┐
│ 工作站 4090（无 ROS）                                                     │
│   openvla_act.py  Flask  :5007                                           │
│   输入：JPEG/PNG Base64 + proprio[4] + instr                             │
│   输出：action[1,4] 绝对位姿 + action_ori[1,4] 机体系相对动作              │
└──────────────────────────────▲───────────────────────────────────────────┘
                               │ HTTP POST /predict   JSON
                               │ 物理通道：校园网 / 网线 / 未来 LQ3 IP
┌──────────────────────────────▼───────────────────────────────────────────┐
│ 机载 Orin NX（ROS Noetic）                                                │
│   vla_bridge（新建）                                                      │
│     图像预处理与发送                                                      │
│     VLA 指令接收、坐标转换、限幅、插值平滑                                 │
│     心跳 / 看门狗 / 急停                                                  │
│   既有栈：odin_ros_driver → OdometryBridge → MAVROS → PX4                │
│           uav_control_fsm  30 Hz setpoint                                │
└──────────────────────────────────────────────────────────────────────────┘
```

三者关系一句话：

- **Odin1 驱动**：只负责传感器与里程计，不感知 VLA。
- **vla_bridge**：唯一云边桥。负责「ROS ↔ HTTP ↔ ROS」。
- **工作站 VLA**：无状态单帧推理服务，不持有飞控状态机。

### 1.2 数据流向（第一阶段：VLA 直连控制）

```text
Odin1
  ├─ /uav1/odin1/image/compressed          JPEG, ~10 Hz
  └─ /uav1/odin1/odometry                  nav_msgs/Odometry, ~10 Hz
           │
           ▼
vla_image_node          降帧 2–5 Hz + 降采样 224×224 + JPEG 再编码
           │
           ▼
/vla/image_for_infer    sensor_msgs/CompressedImage
/vla/proprio            std_msgs/Float32MultiArray   [x,y,z,yaw_deg]
/vla/instruction        std_msgs/String
           │
           ▼
vla_infer_client_node   HTTP POST http://<4090_IP>:5007/predict
           │
           ▼
/vla/action_raw         vla_bridge/VLAAction     原始 JSON 映射
/vla/action_safe        vla_bridge/VLAAction     限幅后
           │
           ▼
vla_cmd_node            坐标转换 + 插值平滑 + 20–30 Hz 刷新 stamp
           │
           ▼
/uav1/move_command      uav_control_fsm/uav_command     ★ 直连，不经 Ego-Planner
           │
           ▼
uav_control_fsm 30 Hz → /uav1/mavros/setpoint_raw/local → PX4
```

并行安全链路：

```text
vla_watchdog_node
  监听 /vla/heartbeat、推理超时、图像/odom 超时、RC 状态
      ├─ 单次超时 → 停发新目标，保持上一安全悬停点
      ├─ 连续超时 → /uav1/hover_command
      └─ 持续失联或急停 → /uav1/land_command，并切断 /vla/enable_control
```

### 1.3 通信协议分层

| 层级 | 选用 | 不选用的原因 |
|---|---|---|
| 物理/链路 | 开发期：校园网 + AX3；桌面：网线；正式飞行：LQ3 / 无线网桥（候选） | 校园网不能默认当成实时控制专网 |
| 传输 | TCP | 图像和控制都需要可靠到达 |
| 云边应用 | **HTTP POST `/predict` + JSON** | 与已跑通的 UAV-Flow 仿真完全一致，可先复用 |
| 机载控制 | **ROS Topic** | 必须留在 Orin 本机，断云不断控 |
| 人工监控（可选） | `rqt_image_view` / 本机 compressed；必要时再加 RTSP | RTSP 不参与闭环控制 |
| 后续可选升级 | WebSocket 二进制 JPEG | 等 HTTP 闭环稳定后再减 Base64 开销 |

协议选择结论：

- 第一阶段 **锁定 HTTP**，不要同时开 gRPC / WebSocket / 跨机 ROS。
- Base64 会膨胀约 33%。第一阶段用「先把图压到 224×224 JPEG」把体积压下去，Base64 可接受。
- 4090 监听 `0.0.0.0:5007`。Orin 做 Client，只需知道工作站 IP。工作站不必知道 Orin IP。
- LQ3 若提供透明以太网/IP，**通信代码零修改**，只改 `server_host`。

### 1.4 Odin1 摄像头传输链路（必须按此实现）

#### 1.4.1 设备原始输出

来自 `odin_ros_driver` 源码与 README：

| 阶段 | 格式 | 分辨率 | 话题 | 开关 |
|---|---|---|---|---|
| 设备原始 | NV12 | 约 1536×1280 | 不直接发布 | SDK 内部 |
| 设备 JPEG | JPEG | 同原始 | `/uav1/odin1/image/compressed` | `sendrgbcompressed: 1` |
| 主机解码 | `sensor_msgs/Image` `bgr8` | 同原始 | `/uav1/odin1/image` | `sendrgb: 1` |
| 去畸变 | `sensor_msgs/Image` `bgr8` | 标定 1600×1296 | `/uav1/odin1/image/undistorted` | **当前关闭** |
| 深度/叠加 | 演示节点 | 高算力 | depth / overlay | **当前关闭，VLA 不用** |

结论：VLA 主图像源固定为

```text
/uav1/odin1/image/compressed    sensor_msgs/CompressedImage    format=jpeg
```

不要把 `/uav1/odin1/image`（未压缩 bgr8）送出机载网卡。

#### 1.4.2 发往工作站的编码格式

第一阶段只使用一种上云格式，避免协议分叉：

```text
Odin JPEG（~1600×1296，10 Hz）
    → Orin 解码（cv_bridge / OpenCV）
    → 中心裁剪或等比缩放（见 1.4.3）
    → resize 到 224×224 RGB
    → JPEG quality=80 再编码
    → Base64 填入 /predict 的 image 字段
```

与官方评估脚本对齐的关键点：

- 评估脚本把图变成 **PIL RGB、224×224、PNG Base64**。
- 服务端 `Image.open()` 不限制编码，JPEG 与 PNG 均可。
- 真机为省带宽，**改用 JPEG**；颜色空间必须是 **RGB**，不能把 OpenCV 的 BGR 直接编码后上云。
- `openvla_act.py` 内部 `processor(prompt, image)` 会再次按模型预处理。Orin 端只要保证「和评估脚本一样的 224×224 RGB」。

带宽估算（必须写进联调记录）：

| 方案 | 单帧体积 | 2 Hz | 5 Hz | 适用 |
|---|---:|---:|---:|---|
| 未压缩 bgr8 1600×1296 | ≈ 6.2 MB | 99 Mbps | 248 Mbps | **禁止上云** |
| 原始 JPEG 1600×1296 | 约 150–400 KB | 2.4–6.4 Mbps | 6–16 Mbps | 仅调试，不作为默认 |
| 224×224 JPEG q80 | 约 8–20 KB | 0.13–0.32 Mbps | 0.32–0.80 Mbps | **默认方案** |
| 224×224 JPEG + Base64 | 约 11–27 KB | 0.18–0.43 Mbps | 0.44–1.1 Mbps | 第一阶段实际占用 |

校园网 + AX3 开发阶段，默认推理频率 **2 Hz**。iperf3 稳定且 RTT p95 < 80 ms 后再升到 5 Hz。

#### 1.4.3 Orin 端降帧、降采样与裁剪策略

按以下固定顺序处理，参数全部放进 `vla_bridge/config/vla_bridge.yaml`：

1. **降帧（时间）**  
   - 订阅压缩图，只用最新帧，队列深度 `queue_size=1`。  
   - 推理触发用 `rospy.Timer`，默认 `inference_rate_hz: 2.0`。  
   - 上一帧推理未完成则丢弃本轮，禁止堆积。

2. **同步窗口（图像 ↔ 里程计）**  
   - 取「图像 stamp 前后 `sync_slop_sec: 0.15`」内最近的 odom。  
   - 超窗则本轮不请求，并在 `/vla/status` 记 `IMAGE_ODOM_DESYNC`。

3. **几何处理（空间）**  
   - 默认 `resize_mode: letterbox`：保持长宽比，短边填黑边到 224×224。  
   - 备选 `center_crop`：先按短边裁正方形，再缩到 224。  
   - **第一阶段用 letterbox**，避免把近处地面/天空裁掉后模型看不见目标。  
   - 微调数据集必须使用与推理完全相同的 `resize_mode`。

4. **不在第一阶段做的事**  
   - 不上 H.264 / RTSP 给模型。  
   - 不启深度补全、点云重投影、overlay。  
   - 不把 10 Hz 原图转发到 4090。

### 1.5 控制链路：解耦可插拔

`vla_cmd_node` 内部只认一个后端接口：

```text
class ControlBackend:
    def publish_pose(self, x, y, z, yaw_deg, stamp) -> None
    def hover(self) -> None
    def land(self) -> None
```

两种实现：

| 后端 | 类名 | 发布目标 | 阶段 |
|---|---|---|---|
| **VLA 直连（默认）** | `DirectFsmBackend` | `/uav1/move_command` | 阶段 4–7 主路径 |
| Ego-Planner（预留） | `EgoPlannerBackend` | `/uav1/goal_point` | 阶段 8 以后 |

配置项：

```yaml
control:
  backend: direct_fsm          # 后续可改为 ego_planner
  enable_publish: false        # 地面不飞时必须 false
```

切换后端时，图像、推理、看门狗、坐标系转换代码全部不动。

#### 1.5.1 VLA 直连控制模式（第一阶段必须实现）

推荐控制语义：

```text
VLA action（第一帧局部系绝对位姿，yaw 为弧度）
    → 变换到 ENU
    → Move_mode = 0
    → pos_ref = (x_enu, y_enu, z_fixed 或 z_enu)
    → yaw_ref = yaw_enu_deg
    → 20–30 Hz 刷新 header.stamp
```

选择 Move_mode=0 而不是速度模式的原因：

- FSM 已有 `pos_smooth_enable`，点到点更平滑。
- 位置模式到达后自动回 `AUTO_HOVER`，比速度模式更安全。
- 速度模式必须高于 `1/cmd_timeout_sec` 持续发布，断云会立刻悬停，但跟踪更抖。

第一阶段默认锁高：

```yaml
control:
  lock_altitude: true
  fixed_altitude: 1.2          # 以起飞后 hover 高度为准，现场改
  use_vla_z: false
  use_vla_yaw: false           # 先只做水平面，确认方向后再开 yaw
```

#### 1.5.2 Ego-Planner 预留接口（第一阶段只定义，不接线）

后续切换 `backend: ego_planner` 时：

```text
/uav1/goal_point    quadrotor_msgs/GoalSet
  goal[0] = x_enu
  goal[1] = y_enu
  goal[2] = z_enu          # 必须 > virtual_ground（默认 -2.0）
```

规划器再写 `/uav1/move_command`。`vla_bridge` 不再直接碰 FSM。

约束：第一阶段 launch **不启动** `exp_single_planner.launch`，避免两个节点同时写 `move_command`。

---

## 2. 关键节点与模块划分

### 2.1 新建功能包位置

```text
drone_ws/agile_ws/src/vla_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   ├── VLAAction.msg
│   ├── VLAStatus.msg
│   └── VLAHeartbeat.msg
├── config/
│   └── vla_bridge.yaml
├── launch/
│   ├── vla_bridge.launch              # 全节点
│   ├── vla_bridge_mock.launch         # 只通信，不发控制
│   └── vla_record.launch              # 采集 rosbag
├── scripts/
│   ├── vla_image_node.py
│   ├── vla_infer_client_node.py
│   ├── vla_cmd_node.py
│   ├── vla_watchdog_node.py
│   ├── mock_inference_server.py       # 工作站或笔记本可跑
│   └── bag_to_uavflow.py              # 阶段 8 再写
└── src/                               # 若平滑/TF 用 C++，放这里；第一阶段 Python 即可
```

包依赖（`package.xml`）：

```text
rospy, std_msgs, sensor_msgs, nav_msgs, geometry_msgs,
cv_bridge, tf, tf2_ros, uav_control_fsm, quadrotor_msgs
```

工作站侧 **不新建 ROS 包**。继续使用：

```text
UAV-Flow-main/OpenVLA-UAV/vla-scripts/openvla_act.py
```

仅允许增加薄封装：`/health`、`/reset`、推理耗时字段。不改 `/predict` 字段名。

### 2.2 节点职责

#### 节点 A：`vla_image_node`

- 节点名：`/vla/vla_image_node`
- 订阅：
  - `/uav1/odin1/image/compressed`
  - `/uav1/odin1/odometry`
- 发布：
  - `/vla/image_for_infer`　`sensor_msgs/CompressedImage`
  - `/vla/proprio`　`std_msgs/Float32MultiArray`　长度为 4
  - `/vla/debug/image`　可选，`sensor_msgs/Image`，供 `rqt_image_view`
- 职责：
  1. 解码 JPEG，BGR→RGB。
  2. letterbox / crop 到 224×224。
  3. 按 `inference_rate_hz` 降帧。
  4. 把当前位姿变换到 **任务第一帧局部系**，组成 proprio。
  5. 记录图像 stamp、odom stamp、两者时间差。
- 不做：HTTP、控制发布、急停决策。

#### 节点 B：`vla_infer_client_node`

- 节点名：`/vla/vla_infer_client_node`
- 订阅：
  - `/vla/image_for_infer`
  - `/vla/proprio`
  - `/vla/instruction`
- 发布：
  - `/vla/action_raw`　`vla_bridge/VLAAction`
  - `/vla/heartbeat`　`vla_bridge/VLAHeartbeat`
  - `/vla/latency`　`std_msgs/Float32`　单位秒
- 职责：
  1. 组装与 `batch_run_act_all.py` 完全同字段的 JSON。
  2. `requests.post(..., timeout=infer_timeout_sec)`。
  3. 解析 `status/action/action_ori`。
  4. 发布心跳；失败也要发 `alive=false`。
- 硬约束：
  - HTTP 必须在独立线程，禁止卡住 ROS 回调。
  - 同时只允许 1 个 in-flight 请求。
  - `request_id` 单调递增，响应必须回传同一 id。

#### 节点 C：`vla_cmd_node`

- 节点名：`/vla/vla_cmd_node`
- 订阅：
  - `/vla/action_raw`
  - `/uav1/odin1/odometry` 或 `/uav1/mavros/local_position/odom`
  - `/vla/enable_control`　`std_msgs/Bool`
  - `/vla/control_backend`　`std_msgs/String`　可选运行时切换
- 发布：
  - `/vla/action_safe`
  - `/uav1/move_command`　仅当 `backend=direct_fsm` 且 `enable_publish=true`
  - `/uav1/goal_point`　仅当 `backend=ego_planner` 且 `enable_publish=true`
- 职责：
  1. 校验 NaN/Inf/维数/步长/高度窗/偏航窗。
  2. 坐标系转换（见第 3 章）。
  3. 线性或一阶低通插值，把 2–5 Hz 目标扩成 20–30 Hz。
  4. 每次发布刷新 `header.stamp = rospy.Time.now()`，避免 FSM `max_age_sec` 丢包。
  5. `enable_publish=false` 时只打印和发布 `/vla/action_safe`。

插值规则（第一阶段写死，禁止上复杂规划）：

```text
上一安全目标 P0，新目标 P1，间隔 Δt_infer
以 control_rate_hz 在 Δt_infer 内直线插值位置
yaw 走最短角插值
单步位移仍受 max_step_xy_m 限制
推理失败时冻结在最后安全目标（位置模式）或发 hover
```

#### 节点 D：`vla_watchdog_node`

- 节点名：`/vla/vla_watchdog_node`
- 订阅：
  - `/vla/heartbeat`
  - `/vla/status`
  - `/uav1/mavros/state`
  - `/uav1/odin1/odometry`
  - `/vla/emergency_stop`
- 发布：
  - `/uav1/hover_command`
  - `/uav1/land_command`
  - `/vla/enable_control`　在故障时强制 `false`
  - `/vla/status`
- 职责：机载安全状态机，见 6.3。  
- 原则：看门狗**只降级，不升级**。不能自己解锁，不能自己进 Offboard，不能自己起飞。

### 2.3 节点协同与启动顺序

```text
roscore
    → odin1_ros1.launch ns:=/uav1
    → px4.launch
    → exp_single_uav_control_fsm.launch
    → vla_bridge.launch          # 默认 enable_publish:=false
```

`vla_bridge.launch` 内部启动顺序：watchdog → image → infer_client → cmd。  
任一前置话题未就绪，infer_client 只告警，不请求。

### 2.4 标准 ROS 话题与消息一览

新建话题一律放在 `/vla/` 下，机载既有话题保持 `/uav1/`。

| 话题 | 类型 | 发布者 | 频率目标 | 说明 |
|---|---|---|---|---|
| `/uav1/odin1/image/compressed` | `sensor_msgs/CompressedImage` | Odin | ~10 Hz | 原始主图像 |
| `/uav1/odin1/odometry` | `nav_msgs/Odometry` | Odin | ~10 Hz | 视觉惯导里程计 |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | MAVROS | ~30–50 Hz | FSM 反馈，ENU |
| `/uav1/mavros/state` | `mavros_msgs/State` | MAVROS | ~1–10 Hz | connected/armed/mode |
| `/vla/instruction` | `std_msgs/String` | 人工/任务脚本 | 事件 | 任务级自然语言 |
| `/vla/image_for_infer` | `sensor_msgs/CompressedImage` | image_node | 2–5 Hz | 已处理 224×224 JPEG |
| `/vla/proprio` | `std_msgs/Float32MultiArray` | image_node | 与上同步 | `[x,y,z,yaw_deg]` |
| `/vla/action_raw` | `vla_bridge/VLAAction` | infer_client | 2–5 Hz | 模型原始输出 |
| `/vla/action_safe` | `vla_bridge/VLAAction` | cmd_node | 2–5 Hz | 校验后 |
| `/vla/heartbeat` | `vla_bridge/VLAHeartbeat` | infer_client | 2–5 Hz | 含 RTT |
| `/vla/latency` | `std_msgs/Float32` | infer_client | 2–5 Hz | 推理+网络 |
| `/vla/status` | `vla_bridge/VLAStatus` | watchdog | 5 Hz | 统一状态 |
| `/vla/enable_control` | `std_msgs/Bool` | 人工/watchdog | 事件 | 控制总闸 |
| `/vla/emergency_stop` | `std_msgs/Bool` | 人工 | 事件 | 急停 |
| `/uav1/move_command` | `uav_control_fsm/uav_command` | cmd_node 或人工 | 20–30 Hz | 直连控制 |
| `/uav1/hover_command` | `std_msgs/Bool` | watchdog | 事件 | 悬停 |
| `/uav1/land_command` | `std_msgs/Bool` | watchdog | 事件 | 降落 |
| `/uav1/goal_point` | `quadrotor_msgs/GoalSet` | 预留 | 事件 | 后期规划 |

自定义消息必须可 `rosmsg show`。

`VLAAction.msg`：

```text
std_msgs/Header header
uint32 request_id
string frame_id                 # first_frame 或 enu 或 body
bool is_absolute
string yaw_unit                 # rad 或 deg
float32[4] action               # 服务端处理后的位姿
float32[4] action_ori           # 机体系相对 [dx,dy,dz,dyaw_rad]
float32 infer_ms
string instruction
```

`VLAHeartbeat.msg`：

```text
std_msgs/Header header
uint32 request_id
bool alive
float32 rtt_sec
string last_error
```

`VLAStatus.msg`：

```text
std_msgs/Header header
string state                    # INIT/WAIT_SENSORS/READY/REQUESTING/FAILSAFE/HOVER
bool image_ok
bool odom_ok
bool infer_ok
bool publish_enabled
string backend                  # direct_fsm / ego_planner
float32 last_infer_age_sec
string detail
```

### 2.5 命名规范（方便 ROS 工具排查）

- 节点名：`vla_<功能>_node`，放在 `/vla` 命名空间。
- 私有参数：`~server_host`、`~inference_rate_hz`，一律 yaml + launch 注入。
- 禁止匿名节点、禁止无 stamp 的控制消息、禁止复用 `/tmp` 话题名。
- 所有 Header 的 `frame_id` 必须填真实坐标系名，便于 `rviz` 和 `tf`。

---

## 3. 数据接口与消息格式（两向严格对齐）

### 3.1 工作站 VLA 真实 I/O（来自源码，禁止改字段名）

请求 `POST /predict`：

```json
{
  "image": "<base64>",
  "proprio": [x, y, z, yaw_deg],
  "instr": "fly to the right side of the table"
}
```

服务端处理（`openvla_act.py`）：

```text
image  → PIL.Image → processor(prompt, image) → Tensor(bfloat16, GPU)
proprio → np.float32[4]
prompt  = "In: Current State: {x:.1f},{y:.1f},{z:.1f},{yaw:.1f}, What action should the uav take to {instr}?\nOut:"
pred    = model.predict_action(..., unnorm_key, do_sample=False)   # np.ndarray, 长度 4
```

响应：

```json
{
  "status": "success",
  "action": [[x_abs, y_abs, z_abs, yaw_abs_rad]],
  "action_ori": [[dx, dy, dz, dyaw_rad]],
  "message": "Action generated successfully"
}
```

字段语义（以源码为准）：

| 字段 | 坐标系 | 单位 | 含义 |
|---|---|---|---|
| `proprio[0:3]` | **与训练一致的第一帧局部系** | m | 当前点相对任务起点的位置 |
| `proprio[3]` | 同上，绕竖直轴 | **度** | 当前偏航 |
| `action_ori[0:3]` | **当前机体系（水平 yaw 旋转后的前-右-上）** | m | 下一步相对位移 |
| `action_ori[3]` | 机体系偏航增量 | **弧度** | Δyaw |
| `action[0:3]` | **与 proprio 同一世界/局部系** | m | `p = p_cur + R_z(yaw_cur) · Δp_body` |
| `action[3]` | 同上 | **弧度** | `yaw_abs = Δyaw + yaw_cur_rad` |

训练侧 `SimpleVLADataset` 的动作构造与此一致：用 `raw_logs` 的相邻帧，把下一帧变到当前帧局部系，得到 `[dx,dy,dz,dyaw_rad]`。最后一帧动作为 0。

### 3.2 机载 ROS → VLA 请求的序列化

```text
sensor_msgs/CompressedImage.data     bytes JPEG
        → cv2.imdecode
        → BGR→RGB
        → letterbox 224×224
        → cv2.imencode('.jpg', q=80)
        → base64.b64encode
        → JSON["image"]          str

nav_msgs/Odometry
        pose.pose.position.{x,y,z}           ENU 米
        pose.pose.orientation 四元数
        → yaw_enu_rad = atan2(2(wz+xy), 1-2(y²+z²))
        → 变换到第一帧局部系（见 3.4）
        → yaw_deg = degrees(yaw_local)
        → JSON["proprio"] = [x_l, y_l, z_l, yaw_deg]

/vla/instruction.data
        → JSON["instr"]          str，原样携带，不做翻译
```

同步规则：

- 同一 `request_id` 的 image stamp 与 odom stamp 差 ≤ 150 ms。
- `proprio` 必须来自与图像匹配的那一帧，禁止用「最新 odom + 旧图」。

### 3.3 VLA 响应 → 机载 ROS 的反序列化

```text
JSON["action"][0]      → VLAAction.action[4]       yaw 保持 rad
JSON["action_ori"][0]  → VLAAction.action_ori[4]   yaw 保持 rad
JSON["status"]         → 非 success 则丢弃
```

`vla_cmd_node` 再映射到 `uav_command`：

```text
uav_command.header.stamp     = now
uav_command.header.frame_id  = "uav1_map"          # 与 FSM ENU 一致
uav_command.Move_mode        = 0                   # 惯性系位置 + 偏航角
uav_command.pos_ref          = {x_enu, y_enu, z_cmd}
uav_command.vel_ref          = {0,0,0}
uav_command.acc_ref          = {0,0,0}
uav_command.yaw_ref          = yaw_enu_deg         # ★ 度
uav_command.yaw_rate_ref     = 0
```

第一阶段若关闭 yaw：`yaw_ref` 填当前里程计 yaw（度），不要填 0（0 是北/ENU 的 0，不是“保持”）。

### 3.4 坐标系定义与转换

本项目同时存在 5 套坐标，必须在日志里打印 `frame_id`，禁止口头混用。

```text
odin / odom                 Odin 里程计世界，右手，近似 ENU，原点=驱动启动处
uav1_map / ENU              FSM 与 MAVROS local 使用的控制系
uav1/base_link              机体系：x 前、y 左、z 上（ROS 机体约定）
odin1_base_link             Odin 驱动机体/相机安装系
first_frame                 本任务第一帧：把当时 ENU 位姿当作原点，yaw 归零
VLA body                    仅绕 yaw 的水平机体系：x 前、y 右或左见下
```

#### 3.4.1 必须先做的方向标定（代码写死前的地面实验）

Odin `odin1_base_link` 与飞机机头、ENU 的轴向**不能靠猜测**。阶段 4 用下面实验锁死符号：

1. 飞机机头朝 ENU +X（东），记录 `/uav1/odin1/odometry` 与 `/uav1/mavros/local_position/odom` 的 yaw。
2. 人手把飞机向前平移 1 m，看两路 `position.x/y` 哪一轴增加。
3. 向上抬高 0.3 m，确认 `z` 增加。
4. 把结果写入 `vla_bridge.yaml`：

```yaml
frames:
  odom_to_enu:
    # 若两路已经同向，则为单位阵
    R: [[1,0,0],[0,1,0],[0,0,1]]
    t: [0,0,0]
  vla_body_y_sign: 1          # 若模型 +y 是右、ROS 机体 +y 是左，则填 -1
```

在符号未标定前，`enable_publish` 必须为 false。

#### 3.4.2 第一帧局部系（与训练 / 评估对齐）

任务开始（收到第一条 instruction 或手动 `/vla/reset_origin`）时锁存：

```text
p0 = (x0, y0, z0)     # ENU
ψ0                    # ENU yaw，弧度
```

之后每一帧：

```text
p_enu = (x, y, z)
ψ     = yaw_enu_rad

Δp = p_enu - p0
p_local = Rz(-ψ0) · Δp = [  cosψ0  sinψ0  0 ] Δp
                         [ -sinψ0  cosψ0  0 ]
                         [  0      0     1 ]

ψ_local_deg = degrees( wrapPi(ψ - ψ0) )
proprio = [p_local.x, p_local.y, p_local.z, ψ_local_deg]
```

这与 `SimpleVLADataset`「proprio = 相对第一帧的 x,y,z,yaw_deg」一致，也与评估脚本 `current_pose` 从 `[0,0,0,0]` 起步一致。

#### 3.4.3 VLA `action` → ENU 目标（直连 Move_mode=0）

服务端已经把 `action_ori` 加到当前 proprio 上，因此 `action` 仍在 **first_frame** 系：

```text
p_abs_local = action[0:3]
ψ_abs_local = action[3]                 # 弧度

p_enu = p0 + Rz(ψ0) · p_abs_local
ψ_enu = wrapPi(ψ_abs_local + ψ0)
yaw_ref_deg = degrees(ψ_enu)

z_cmd = fixed_altitude    若 lock_altitude
      = p_enu.z           否则
```

其中：

```text
Rz(ψ) = [[cosψ, -sinψ, 0],
         [sinψ,  cosψ, 0],
         [0,     0,    1]]
```

#### 3.4.4 VLA `action_ori` → 机体系（备用，Move_mode=5）

仅在标定确认 `vla_body` 与 FSM 机体系同向后启用：

```text
pos_ref = (action_ori[0], vla_body_y_sign * action_ori[1], action_ori[2])
yaw_ref = degrees(action_ori[3])     # FSM 要度
Move_mode = 5                       # 入队时一次性转到世界系 mode 0
```

第一阶段默认不用这条，避免与 FSM「5/6 入队即转世界系」叠加上 first_frame 变换。

#### 3.4.5 与 Odin / FSM TF 的关系

当前两棵树：

```text
Odin:   odom → odin1_base_link
FSM:    world → uav1_map → uav1/base_link → (camera/lidar/odin_link)
```

`uav_group.yaml` 里 `odin_link.enabled: false`。第一阶段 **不要假设 tf 能直接查到 odom→uav1_map**。

`vla_bridge` 自己维护：

- `p0, ψ0` 任务原点；
- `odom_to_enu` 静态矩阵（标定后写入）；
- 控制发布使用 FSM/MAVROS ENU 数，优先用 `/uav1/mavros/local_position/odom` 作为「飞机现在在哪」，用 `/uav1/odin1/odometry` 作为「给模型看的状态」。若两路差超过 `odom_disagree_m: 0.3`，watchdog 告警并禁止发布。

阶段 4 用 `rqt_tf_tree` 检查 FSM 树；Odin 树可单独开 `tf_echo odom odin1_base_link`。不要强行把两棵树焊在一起后再做第一阶段闭环。

### 3.5 单位速查表

| 量 | VLA 输入 | VLA 输出 action | VLA 输出 action_ori | FSM `uav_command` | MAVROS |
|---|---|---|---|---|---|
| 位置 | m | m | m | m | m |
| yaw | **度** | **弧度** | **弧度** | **度** | 四元数 |
| 时间戳 | 请求时的图像 stamp | 无 | 无 | 必须 now | ROS Time |

任何新代码在换算处必须写注释：`# VLA rad -> FSM deg`。

---

## 4. 真机数据采集与 VLA 微调方案

### 4.1 为何必须采真机数据

官方 OpenVLA-UAV 主要在仿真与 DJI FPV / 广角相机上训练。本机是 **Odin1 前视鱼眼/广角 + 固定安装角 + 工厂/室内新场景**，存在三类 OOD：

1. **FOV / 畸变**：Odin 1600×1296 广角 vs 训练图；letterbox 黑边也会改变视觉分布。
2. **安装角**：Odin 相对机头的俯仰/横滚若未与训练相机一致，模型会把地面当目标。
3. **场景**：工厂立柱、货架、灯光与校园/仿真不一致。

策略：先 Zero-shot 地面看动作是否「方向大致对」，再采 30–100 条短轨迹做 LoRA，而不是一上来全量重训。

### 4.2 采集模式

采集时 **不跑 VLA 控制**。飞行员用遥控器手动飞，或用已验证的 FSM 位置指令飞示范轨迹。

启动：

```text
roscore
roslaunch odin_ros_driver odin1_ros1.launch ns:=/uav1
roslaunch uav_control_fsm px4.launch
roslaunch uav_control_fsm exp_single_uav_control_fsm.launch
roslaunch vla_bridge vla_record.launch     # 只发布 /vla/instruction，不发 move_command
```

每条轨迹流程：

1. 起飞到定点悬停，记录为任务原点。
2. `rostopic pub /vla/instruction std_msgs/String "data: 'fly to the left of the red cabinet'"`
3. 开始 bag。
4. 飞手只看 FPV/Odin 图，按指令飞完（短距，< 8 m，< 20 s）。
5. 悬停 1 s，停 bag。
6. 同一指令至少从 3 个不同起点各飞 1 次。

指令先用英文短句，尽量靠近 UAV-Flow 的 Fixed Command Set（approach / pass left / hover beside / translate / rotate）。中文可另存 `instruction_zh`，但微调主字段用 `instruction`。

### 4.3 必须录制的话题

```bash
rosbag record -O ~/vla_bags/${DATE}_${INSTR_ID}.bag \
  /vla/instruction \
  /uav1/odin1/image/compressed \
  /uav1/odin1/odometry \
  /uav1/odin1/odometry_highfreq \
  /uav1/odin1/imu \
  /uav1/mavros/local_position/odom \
  /uav1/mavros/local_position/pose \
  /uav1/mavros/state \
  /uav1/mavros/rc/in \
  /uav1/move_command \
  /uav1/mavros/setpoint_raw/local \
  /tf /tf_static
```

说明：

- 图像只录 compressed。
- `odometry_highfreq` / IMU 用于后处理插值，不给在线 VLA。
- 不要录 raw cloud、render cloud，否则 bag 极大且无益于 OpenVLA-UAV 微调。
- 同时写一份 `meta.json`：场地、光照、飞手、指令、是否成功、安全事件。

### 4.4 bag → UAV-Flow 文件夹数据集

目标目录必须能被 `SimpleVLADataset` 直接读取：

```text
datasets/odin_real/<episode_id>/
    000000.jpg
    000001.jpg
    ...
    log.json
```

`log.json` 字段（与 `prepare_data.py` / `uav_dataset.py` 对齐）：

```json
{
  "id": "20260919_cabinet_left_001",
  "instruction": "fly to the left of the red cabinet",
  "instruction_unified": "fly to the left of the red cabinet",
  "raw_logs": [[x, y, z, roll_deg, yaw_deg, pitch_deg], ...],
  "preprocessed_logs": [[x_l, y_l, z_l, roll_l, yaw_l, pitch_l], ...]
}
```

`uav_dataset.py` 只取索引 `[0,1,2,4]`，即 x,y,z,yaw。roll/pitch 仍要写入，便于复盘，但训练不用。

转换脚本 `scripts/bag_to_uavflow.py` 算法（阶段 8 实现）：

1. 读 bag，按图像 stamp 对齐。
2. 图像统一抽成 **5 Hz**（论文真实数据采集率），不足则最近邻，过多则降采样。
3. 用 `/uav1/mavros/local_position/odom` 或 Odin odom（由 `pose_source` 配置）插值位姿。
4. `raw_logs`：对齐后的 ENU 位姿，yaw 为度。
5. 以第 0 帧为原点生成 `preprocessed_logs`（与 3.4.2 同一公式）。
6. 用 **与在线相同的 letterbox 224 或先存原图**。推荐 **存处理后的 224 JPEG**，保证训推一致；另存一份 `full.jpg` 可选。
7. 丢弃：odom 中断、图像空洞、位移突变 > 0.5 m/帧、未解锁、人工标记失败的轨迹。
8. 输出数据集统计：条数、时长、位移分布、yaw 分布。

不要先转 RLDS/HDF5。本仓库微调走 `SimpleVLADataset` 文件夹格式，不是 Open-X RLDS。

### 4.5 微调策略

```text
基座：官方 OpenVLA-UAV 或 openvla-7b
脚本：OpenVLA-UAV/vla-scripts/finetune_uav.py + finetune_uav.sh
方法：LoRA rank=32，lr=5e-4
数据：官方 real 子集（可选） + 本机 odin_real
norm_stats：必须用本机数据重算，dataset_key 改成 "odin_real"
推理：openvla_act.py 的 unnorm_key 从 "sim" 改为 "odin_real"
```

建议数据量门槛：

| 阶段 | 轨迹数 | 目的 |
|---|---|---|
| 格式验证 | 3–5 | 确认 `finetune_uav.py` 能遍历、loss 下降 |
| 域适应 | 30–50 | 先覆盖 4 类：前移、左绕、右绕、靠近指定物体 |
| 可用微调 | 100+ | 同一指令多起点、多光照 |

未改 `unnorm_key` 之前，禁止把微调权重直接上真机。

### 4.6 采集安全

- 采数可以手飞，桨叶安装，但必须有观察员。
- 不在采数时开 `enable_publish`。
- 每条 bag 落地后立刻 `rosbag info` 确认 compressed 与 odom 都 > 0。

---

## 5. 阶段性开发与联调计划

每阶段都列出：**目标、产物、ROS 调试命令、通过标准**。未通过不得进入下一阶段。

### 阶段 0：网络与环境基线

目标：Orin 与 4090 IP 互通，5007 端口可达。

```bash
# Orin
hostname -I
ping -c 50 10.62.25.215
nc -vz 10.62.25.215 5007

# 4090
hostname -I
iperf3 -s
# Orin
iperf3 -c 10.62.25.215 -t 30
iperf3 -c 10.62.25.215 -t 30 -R
```

通过：双向 ping 丢包 < 2%；TCP 5007 通；记下 RTT / 带宽。  
ROS 工具：本阶段不用。

### 阶段 1：Mock HTTP，不上 ROS 控制

4090 或笔记本跑 `mock_inference_server.py`，返回固定

```json
{"status":"success","action":[[0.3,0.0,1.2,0.0]],"action_ori":[[0.3,0.0,0.0,0.0]]}
```

Orin 先用纯 Python 客户端，再换成 `vla_infer_client_node` + 假图。

通过：连续 20 次 200；`request_id` 对齐；主动断网能报错。  
ROS：

```bash
rostopic echo /vla/action_raw
rostopic echo /vla/heartbeat
rostopic hz /vla/heartbeat
```

### 阶段 2：真实 `/predict`，仍不发控制

启动已验证的 `openvla_act.py`。用一张静态 JPG + proprio=`[0,0,0,0]` + 简单英文指令。

通过：返回 `action`/`action_ori` 维数为 `[1,4]`；单步推理时间写入日志（仿真约 0.11 s，真机图可能 0.1–0.3 s）。  
ROS：`rostopic echo /vla/latency`，`rosmsg show vla_bridge/VLAAction`。

### 阶段 3：接入真实 Odin 图像与里程计

```bash
roslaunch odin_ros_driver odin1_ros1.launch ns:=/uav1
roslaunch vla_bridge vla_bridge.launch enable_publish:=false
```

ROS 调试清单：

```bash
rosnode list
rosnode info /host_sdk_sample
rosnode info /vla/vla_image_node
rqt_graph

rostopic list | grep -E 'odin1|vla'
rostopic hz /uav1/odin1/image/compressed
rostopic hz /uav1/odin1/odometry
rostopic hz /vla/image_for_infer          # 应为 2–5，而不是 10
rostopic echo /vla/proprio
rqt_image_view                             # 选 /vla/debug/image，确认 RGB 方向、是否倒置

rosmsg show sensor_msgs/CompressedImage
```

通过：

- `/uav1/odin1/image/compressed` 有数据；
- `/vla/image_for_infer` 频率被限制；
- `rqt_image_view` 画面方向正确、颜色不是发蓝（BGR 未转 RGB 的典型症状）；
- 平移飞机时 `/vla/proprio` 前三维连续变化，旋转时第四维（度）变化。

### 阶段 4：坐标转换与只打印

`vla_cmd_node` 输出 ENU 目标到 `/vla/action_safe`，不发布 `move_command`。

地面实验（桨叶拆掉，人抱着飞机或放桌上）：

| 操作 | 期望 |
|---|---|
| 指令 “move forward”，机头朝东 | `action_safe` 的 ENU +X 增加 |
| 机头朝北再发同一指令 | ENU +Y 增加 |
| 指令要求升高且未锁高 | +Z 增加 |
| 正 yaw | 与机头顺时针/逆时针约定一致，记录符号 |

ROS：

```bash
rostopic echo /vla/action_raw
rostopic echo /vla/action_safe
tf_echo odom odin1_base_link
rqt_tf_tree
rviz   # 显示 /uav1/odin1/odometry 与 Marker 预测点
```

通过：三次方向实验全部正确，并写入 `vla_bridge.yaml` 的 `vla_body_y_sign` / `odom_to_enu`。

### 阶段 5：地面不飞，打到 FSM

`enable_publish:=true`，但 **不上桨、不解锁**。人工确认 FSM 仍为 `MANUAL_CTRL`。

```bash
roslaunch uav_control_fsm px4.launch
roslaunch uav_control_fsm exp_single_uav_control_fsm.launch
rostopic echo /uav1/move_command
rostopic hz /uav1/move_command            # 应 ~20–30
rosmsg show uav_control_fsm/uav_command
rostopic echo /uav1/mavros/state          # armed: False
rqt_graph                                 # 确认只有 vla_cmd_node 在写 move_command
```

通过：

- `Move_mode=0`，`yaw_ref` 是度且量级合理（不是 1.57 这种弧度误当度）；
- stamp 持续刷新；
- watchdog 杀推理进程后，`/vla/enable_control` 变 false，且不再写新目标。

### 阶段 6：无桨 Offboard 接口测试

装上飞控、拆桨。RC 高档进 Offboard，确认 setpoint 方向。重点看：

```bash
rostopic echo /uav1/mavros/setpoint_raw/local
rostopic hz /uav1/mavros/setpoint_raw/local     # 应接近 30
```

通过：断网后自动 hover 话题有输出；RC 中档能退 Offboard。

### 阶段 7：系留 / 定高短距真机闭环

条件：阶段 5–6 通过、观察员、RC 在手、围栏内、`lock_altitude=true`、`max_step_xy_m=0.25`、`use_vla_yaw=false`、`inference_rate_hz=2`。

步骤：

1. 手飞或 FSM 起飞到 1.2 m 悬停。
2. 开 `enable_publish`。
3. 发单条简单指令，例如 “move forward”。
4. 只允许一次短动，立即 hover。
5. 全程 rosbag。

ROS 联调现场命令（另开笔记本，配置 `ROS_MASTER_URI` 指向 Orin）：

```bash
rqt_graph
rostopic hz /vla/image_for_infer
rostopic hz /vla/action_raw
rostopic hz /uav1/move_command
rostopic echo /vla/status
rviz
```

通过：飞机按预期方向移动 < 1 m；异常能 RC 接管；bag 完整。

### 阶段 8：采数与微调（可与阶段 7 并行准备脚本）

见第 4 章。微调后再从阶段 2 重跑，`unnorm_key=odin_real`。

### 阶段 9：接入 Ego-Planner（可选）

改 `backend: ego_planner`，启动 `exp_single_planner.launch`，确认 `vla_cmd_node` **不再**发布 `move_command`。

```bash
rqt_graph          # move_command 的 publisher 应是 ego 的 traj_server
rostopic echo /uav1/goal_point
```

### 5.1 各阶段 ROS 工具对照表

| 阶段 | rqt_graph | rostopic hz | rosmsg show | rqt_tf_tree | rviz |
|---|---|---|---|---|---|
| 1 Mock | 看 /vla 四节点 | heartbeat、action_raw | VLAAction | 否 | 否 |
| 2 真推理 | 同上 | latency | 同上 | 否 | 否 |
| 3 真图像 | Odin + vla_image | compressed vs image_for_infer | CompressedImage | 可选 | rqt_image_view |
| 4 坐标 | 全图 | odom 两路 | Odometry | **必须** | 预测点 Marker |
| 5 不飞控制 | 确认唯一控制源 | move_command 20–30 | uav_command | 是 | setpoint |
| 7 真飞 | 现场盯断链 | 高/低频对比 | 否 | 是 | 轨迹 |

频率验收标准：

| 话题 | 期望 |
|---|---|
| `/uav1/odin1/image/compressed` | 8–12 Hz（当前 dtof_fps=100） |
| `/vla/image_for_infer` | 设定值 ±0.3 Hz |
| `/vla/action_raw` | ≤ 推理频率，允许低于设定（推理慢） |
| `/uav1/move_command` | 18–30 Hz |
| `/uav1/mavros/setpoint_raw/local` | ≈ 30 Hz |
| `/uav1/mavros/vision_pose/pose` | 与 Odin 同量级，连续 |

---

## 6. 架构隐患与工程优化

### 6.1 网络延时与图像带宽

**隐患**

1. 校园网跨网段（`10.49` ↔ `10.62`）可能被 ACL/认证打断，SSH 与 `/predict` 会一起死。
2. 若误订 `/uav1/odin1/image` 未压缩 bgr8 并上云，单路即可打满无线。
3. Base64 + Flask 同步处理时，大图会把 RTT 从 100 ms 打到数秒。
4. AX3 是消费级 AP，飞机一移动就可能抖动。
5. 现网与正式飞行网不是同一张网，用校园网测得的 2 Hz「能用」不能外推到 LQ3。

**落地措施**

- 默认只传 224×224 JPEG；yaml 里加 `assert max_jpeg_bytes: 40000`，超限丢弃。
- `vla_image_node` 统计并发布 `/vla/bandwidth_kbps`。
- HTTP timeout 默认 `1.5 s`（模型 ~0.1–0.3 s + 网络余量），超时走看门狗。
- 正式飞行前用飞行路径重测 iperf3 / ping；不达标再上 LQ3，不改应用协议。
- 工作站防火墙放行 `5007/tcp`；服务绑定 `0.0.0.0`。
- 后续优化：WebSocket 发 JPEG 二进制、去掉 Base64；仍不把 ROS 话题直接跨网。

### 6.2 VLA 低频推理 vs 机载高频控制

**隐患**

1. OpenVLA 约 2–10 Hz，PX4 Offboard 与 FSM 需要 20–50 Hz 连续 setpoint。只在推理返回时发一次 `move_command`，会出现：stamp 过期被丢、速度模式回悬停、轨迹台阶化。
2. FSM `cmd_timeout_sec=0.5`：速度模式 0.5 s 无新指令就 HOVER；位置模式虽锁存，但 `max_age_sec` 过期的**新**指令会被丢。
3. 若把模型输出当速度用，2 Hz 更新会明显顿挫。
4. 论文中的 look-ahead / action chunk 在当前 `openvla_act.py` **没有实现**，每次只回 1 步 4D。不要幻想服务端已经在做多步前瞻。

**落地措施**

- 双时钟：推理 2–5 Hz，控制发布 20–30 Hz，缓存「最新安全目标」。
- 第一阶段只用 Move_mode=0 + 已有 `pos_smooth`，把台阶变成短直线。
- `vla_cmd_node` 对位置做线性插值，对单步 `max_step_xy_m` 限幅（默认 0.25 m）。
- 推理超过 `stale_action_sec: 1.0` 未更新：冻结目标并让 watchdog 计数。
- 不要把 `inference_rate_hz` 提高到相机满帧，那只会堵塞 4090 与无线。
- 后续若接 Pi-0 多步 chunk，再在 `vla_cmd_node` 加 look-ahead 队列；OpenVLA 单步阶段不做。

### 6.3 安全保护

**隐患**

1. 现有 FSM 已有 RC 中档退 Offboard、odom 超时退 MANUAL、`land_command`。但 **VLA 断云不会自动被 FSM 感知**，因为 HTTP 不在 ROS 里。
2. 旧文档假设有 `uav_fsm_state.msg`，当前包里**只有** `uav_command.msg`。不要订阅不存在的状态消息。飞控状态用 `/uav1/mavros/state` 与日志。
3. `odin_link` TF 未启用，模型相机外参若错，会稳定地飞错方向。
4. 模型可能输出接近 0 的重复动作（仿真复现中已出现），飞机表现为「假死」而非安全悬停。
5. `enable_publish` 默认必须 false，避免节点一启动就写飞控。
6. 自动曝光在暗光下会拖影，VLA 会把模糊当障碍。

**看门狗状态机（必须实现）**

```text
INIT → WAIT_SENSORS → READY → REQUESTING → TRACKING
                         ↘ FAILSAFE → HOVER → (人工) LAND
```

触发表：

| 条件 | 动作 |
|---|---|
| 图像或 odom 中断 > 0.5 s | 禁止新推理 |
| 单次 HTTP 超时 / HTTP 5xx | 保持上一目标，`infer_fail+=1` |
| 连续失败 ≥ 3 或心跳中断 > 2 s | `/uav1/hover_command`，`enable_control=false` |
| 失败持续 > 8 s | 建议 `/uav1/land_command`（可配置），蜂鸣/日志 |
| `/vla/emergency_stop=true` | 立即 hover，切断发布 |
| `mavros/state.mode` 离开 OFFBOARD | 切断发布，不抢飞控 |
| 动作超限 / NaN | 丢弃，hover |
| RC 中档（既有 FSM） | 最高优先级，bridge 不得拉回 Offboard |
| 两路 odom 偏差 > 0.3 m | 禁止发布 |

硬禁令：

- VLA 不得发布 `/uav1/offboard_command`。
- VLA 不得解锁、不得起飞。
- 真机默认 `use_vla_yaw=false`、`lock_altitude=true`，直到阶段 7 复盘通过。

### 6.4 当前仓库里已存在、必须在开发前修掉的接口坑

1. **Odin 命名空间默认空。** 所有启动文档和 `vla_record.launch` 必须写 `ns:=/uav1`。阶段 3 用 `rostopic list | grep odin1` 验收，只允许 `/uav1/odin1/...`。
2. **去畸变图默认关闭。** 代码里不要写死 `image/undistorted`。
3. **定位源名叫 Fast_Lio。** 不要改 yaml 名去“对齐文档”，以免 PX4 参数档案对不上。
4. **`/predict` 无 `/health`、无 `/reset`。** 评估脚本会打 `/reset` 得到 404。建议在 `openvla_act.py` 加两个空路由，不影响权重。
5. **评估客户端与服务端对 `action` 还有一层「再变换到 Unreal 世界」。** 真机 **不要复制** `batch_run_act_all.py` 里对 `action` 再做一次 `transform_to_global(initial_yaw)`，否则会双重旋转。真机只使用第 3 章公式。
6. **官方 `unnorm_key="sim"` 不适合 Odin 真机。** Zero-shot 只用于看通链路；真飞效果差应先怀疑归一化键，而不是先改飞控。
7. **Ego-Planner 与直连不能同时写 `move_command`。** launch 互斥。
8. **Orin 用户在不同文档中分别为 `nvidia` / `agile`。** 部署脚本用环境变量 `ORIN_USER`，不要写死。

### 6.5 建议的默认安全参数（写入 yaml，现场只许改小不许改大）

```yaml
ground:
  protocol: http
  host: 10.62.25.215
  port: 5007
  endpoint: /predict
  infer_timeout_sec: 1.5

image:
  source: /uav1/odin1/image/compressed
  inference_rate_hz: 2.0
  resize: 224
  resize_mode: letterbox
  jpeg_quality: 80
  max_jpeg_bytes: 40000
  sync_slop_sec: 0.15

frames:
  pose_for_model: /uav1/odin1/odometry
  pose_for_control: /uav1/mavros/local_position/odom
  odom_disagree_m: 0.3
  vla_body_y_sign: 1

control:
  backend: direct_fsm
  enable_publish: false
  control_rate_hz: 25
  move_mode: 0
  lock_altitude: true
  fixed_altitude: 1.2
  use_vla_yaw: false
  max_step_xy_m: 0.25
  max_step_z_m: 0.05
  max_yaw_step_deg: 8.0
  stale_action_sec: 1.0

watchdog:
  heartbeat_timeout_sec: 2.0
  fail_to_hover_count: 3
  fail_to_land_sec: 8.0
  image_timeout_sec: 0.5
  odom_timeout_sec: 0.5
```

---

## 7. 后续代码开发顺序（本文档的施工清单）

严格按此顺序提交代码，每步对应第 5 章验收：

1. 在 `agile_ws` 创建空包 `vla_bridge`：msg、yaml、launch、四个节点空壳，能 `roslaunch` 并在 `rqt_graph` 看到四个节点。
2. 实现 `mock_inference_server.py` 与 `vla_infer_client_node` 的 HTTP。
3. 实现 `vla_image_node` 的压缩图订阅、224 JPEG、proprio。
4. 实现 `VLAAction` 解析与第 3 章坐标转换，只发布 `/vla/action_safe`。
5. 实现 `DirectFsmBackend` 与 25 Hz stamp 刷新；默认 `enable_publish=false`。
6. 实现 `vla_watchdog_node` 与 hover/land。
7. 实现 `vla_record.launch` + 录包清单。
8. 写 `bag_to_uavflow.py`，用 3 条 bag 跑通 `SimpleVLADataset`。
9. 需要时再加 `EgoPlannerBackend`。

未完成第 1–6 步，不开展真机带桨试验。

---

## 8. 联调最小命令备忘

```bash
# 机载环境
source /opt/ros/noetic/setup.bash
source $HOME/agile_ws/devel/setup.bash
source $HOME/odin_ws/devel/setup.bash

# 拓扑
rqt_graph
rosnode list
rosnode info /vla/vla_cmd_node

# 频率
rostopic hz /uav1/odin1/image/compressed
rostopic hz /vla/image_for_infer
rostopic hz /vla/action_raw
rostopic hz /uav1/move_command

# 结构
rosmsg show uav_control_fsm/uav_command
rosmsg show vla_bridge/VLAAction

# 坐标
rqt_tf_tree
tf_echo odom odin1_base_link
rostopic echo /vla/proprio

# 可视化
rqt_image_view
rviz
```

工作站：

```bash
conda activate openvla
cd /path/to/UAV-Flow-main/OpenVLA-UAV
python vla-scripts/openvla_act.py
# 另开
curl -s http://127.0.0.1:5007/health
```

---

## 9. 一句话执行基线

> 在 Orin 的 `agile_ws` 新建 `vla_bridge`，把 Odin 的 JPEG 在机载降到 224×224、2 Hz，经 HTTP `/predict` 交给 4090 上已有的 OpenVLA-UAV；把返回的 4D 位姿从第一帧局部系变到 ENU，经限幅与 25 Hz 插值后写入 `/uav1/move_command`（Move_mode=0）。云断则本地 hover。Ego-Planner 只留 `/uav1/goal_point` 插件口。真机效果不够时，用遥控飞 + rosbag 做成 `SimpleVLADataset` 文件夹再 LoRA，并把 `unnorm_key` 改为 `odin_real`。
