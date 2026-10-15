# VLA 无人机云边协同视觉语言导航项目具体实施方案

> 文档状态：实施基线（Implementation Baseline）  
> 适用版本：ROS 1 Noetic；机载端 NVIDIA Jetson Orin NX + Odin1；工作站端 RTX 4090/OpenVLA-UAV。  
> 编制日期：2026-09-18  
> 目标：以可回滚、可观测、可逐级验收的方式打通“真实图像/状态 → 工作站 VLA → 机载安全控制 → PX4”的闭环。

## 0. 先给结论：本方案冻结的工程决策

1. **一期采用 VLA 直连控制模式，但“直连”只绕过 Ego-Planner，不绕过 FSM/MAVROS/PX4。**  
   Odin1 的 ROS 驱动是传感器驱动，源码没有飞行运动控制 topic/service；运动控制唯一走 agile_ws 中的 uav_control_fsm/uav_command → MAVROS → PX4。因此一期链路固定为：

   ~~~text
   OpenVLA 工作站
       ⇅ HTTP keep-alive / JSON（一期）
   Orin NX：vla_bridge
       ⇢ /uav1/move_command（20～30 Hz，uav_control_fsm/uav_command）
       ⇢ /uav1/mavros/setpoint_raw/local
       ⇢ PX4
   Odin1
       ⇢ /odin1/image/compressed、/odin1/odometry、/odin1/imu
   ~~~

   这不是把指令直接写入 Odin 传感器 SDK，也不是绕过 FSM 直接向 PX4 发原始 setpoint。

2. **一期外部推理协议固定为 UAV-Flow 当前 Flask 服务的 /predict 兼容协议。**  
   请求必含 image（Base64 图像）、instr（不是 instruction）、proprio=[x,y,z,yaw_deg]；响应读取 action 与 action_ori。UAV-Flow 另有 deploy.py 的 /act 协议（端口 8000、instruction、JSON-Numpy 图像），两者不能混用。vla_bridge 用 profile 隔离协议，默认 profile 为 uav_flow_openvla_http_predict。

3. **一期优先使用 action_ori，把模型的局部相对位姿增量转换为机体系速度/偏航速度，发布 Move_mode=8。**  
   这样可以避免把模型的 episode-local 绝对点误当成 Odin/ENU 世界绝对点。位置模式和 action 绝对点保留为第二条经过标定的路径，必须在离线单轴试验通过后才能启用。

4. **VLA 推理 2～5 Hz，机载控制输出 20～30 Hz。**  
   Orin 只缓存最新有效动作，不排队执行过期动作；用限速、加速度约束、插值和 TTL 将低频动作变成高频安全指令。FSM 实飞入口默认 loop_hz=30，命令超时约 0.5 s；vla_bridge 不能通过无限重复旧消息来掩盖网络/推理故障。

5. **图像上行主链路使用带时间戳的 ROS JPEG，不把原始 BGR8 跨无线发送。**  
   主输入：/odin1/image/compressed（sensor_msgs/CompressedImage，format=jpeg）；备用：/odin1/image（sensor_msgs/Image，实际 bgr8）；vla_bridge 解码后按模型 profile 明确转 RGB 并重新编码，不能把 JPEG 字节误标成 rgb8；RTSP 只用于监控或未来视频流，不作为需要严格和状态对齐的 VLA 控制输入，除非补充时间戳旁路。

6. **模型状态默认按 UAV-Flow 的 episode-local 语义组织。**  
   每个任务建立 episode 原点 p0、psi0，把 Odin odom 状态转换为起点局部坐标后再形成 proprio=[x,y,z,yaw_deg]。任务 reset、人工接管、重新起飞或相机/定位源切换时必须重置 origin_id。

7. **所有坐标、单位、动作维度和归一化统计在启动时动态校验。**  
   不能因为论文写了“6-DoF”或配置默认 unnorm_key=sim 就假设真机模型是 6 维、单位是米。当前仓库 SimpleVLADataset 和 Flask 路径实际是 4 维 [x,y,z,yaw]；权威动作维度来自模型 norm_stats[unnorm_key].action.q01 的长度。

8. **安全由 Orin 本地 watchdog 决策，不能依赖工作站或 PX4 快照中的默认动作。**  
   现有 FSM 的 cmd timeout 是回悬停，odom timeout 是退出 Offboard/POSCTL；不等于自动降落或返航。PX4 参数快照中的 COM_OF_LOSS_T=1、NAV_RCL_ACT=0 等必须在目标固件上重新核对枚举和行为。

## 1. 审阅范围、事实等级与当前工程基线

### 1.1 已审阅资源

本方案建立在以下资源的交叉核验之上：

| 资源 | 审阅重点 |
|---|---|
| docs/云边协同无人机视觉语言导航项目计划书.md | 总体架构、Edge Agent、HTTP/WebSocket、阶段计划、rosbag、安全策略 |
| docs/UAV-Flow 跨平台仿真复现完整指南.md | /predict 真实字段、跨机部署、ROS 迁移建议、频率差异 |
| docs/Orin_NX_ROS部署与编译说明.md | 两个工作空间、Noetic 环境、启动/编译、真实图像与里程计检查 |
| docs/无人机 + VLA 云边协同部署网络方案技术总结.md | AX3/LQ 网络边界、IP 链路、带宽/延迟测试、Edge Agent、安全 |
| docs/项目开发手册.md | Odin 实飞启动顺序、FSM 状态、Move_mode、RC 接管、调试命令 |
| docs/Jetson Orin NX 无线连接与远程调试指南.md | Orin 网络、SSH、ROS_MASTER/ROS_IP、RViz 远程调试 |
| docs/MinerU_markdown_UAV-Flow_2100208414561165312.md | 论文中的 Flow 定义、5 Hz 数据对齐、模型差异、4090 延迟 |
| docs/1号机.params | PX4 1.13.3 参数快照；只作为待核对基线，不作为已验证安全配置 |
| drone_ws/odin_ws/src/odin_ros_driver | Odin1 ROS1/ROS2 驱动、图像/IMU/odom/点云发布、时间戳、TF、AE/AWB 服务 |
| drone_ws/agile_ws/src/uav_control_fsm | uav_command.msg、FSM 状态机、Move_mode 到 MAVROS setpoint 的转换 |
| drone_ws/agile_ws/src/perception/uav_rtsp_stream | 可选 H.264/RTSP、latest-frame 丢旧帧、码率/分辨率配置 |
| drone_ws/agile_ws/src/location、planner/ego_planner | Fast-LIO/Livox/Ego 后续接口和坐标树 |
| UAV-Flow-main/OpenVLA-UAV | Flask /predict、HF processor、ActionTokenizer、SimpleVLADataset、LoRA 微调 |
| UAV-Flow-main/UAV-Flow-Eval、dataset_tools | 仿真端状态语义、单位、图像请求、Parquet→folder 数据转换 |

docs/ 目录当前共有 9 个顶层资源：8 个 Markdown 文档和 1 个 ASCII/PX4 参数快照（1号机.params）；没有遗漏的 PDF、Office 或子目录资源。仓库中的预编译静态库和第三方大体量依赖未反汇编，按其头文件、CMake 和运行时接口作为二进制依赖处理。

### 1.2 必须区分的“已确认事实”和“设计建议”

- **已确认事实**：来自源码、launch、消息定义或文档中的确定字段；例如 Odin 发布 compressed JPEG、uav_command 的 yaw 单位为度、OpenVLA /predict 需要 instr。
- **待现场冻结**：依赖设备运行时或未随仓库提交的内容；例如 calib.yaml 的实际内参、相机物理安装角、checkpoint 的 norm_stats、现场 IP、PX4 failsafe 枚举。
- **本方案建议**：为了可执行性给出的初始值；例如 JPEG quality 80、推理 5 Hz、速度上限 1 m/s。这些数值必须在阶段验收中实测并写回配置。

### 1.3 当前工程关键事实

#### ROS 工作空间

- odin_ws 中主要功能包 odin_ros_driver 负责设备数据接入和图像/点云/里程计发布。
- agile_ws 包含 uav_control_fsm、uav_rtsp_stream、Livox/Fast-LIO、Ego-Planner、轨迹任务等。
- ROS1 使用 Noetic；Odin CMake 同时支持 ROS1/ROS2，但本项目一期使用 ROS1 分支。
- Odin 构建脚本中存在未定义 WS_DIR，agile 的 build.sh 硬编码 $HOME/agile_ws；实施时应从实际路径显式构建，不能盲目调用旧脚本。

#### Odin1 核心 ROS 接口

| 话题 | 类型 | 当前源码事实 | 一期用途 |
|---|---|---|---|
| /odin1/image | sensor_msgs/Image | JPEG 解码后发布，encoding=bgr8；图像 header.frame_id 未设置 | 备用输入/地面诊断 |
| /odin1/image/compressed | sensor_msgs/CompressedImage | format=jpeg，与图像同一采集时间；队列通常为 5 | **VLA 主输入** |
| /odin1/image/undistorted | sensor_msgs/Image | 运行时有标定才发布；frame_id 仍需检查 | 可选去畸变输入 |
| /odin1/imu | sensor_msgs/Imu | frame=imu_link；设备轴已在驱动内重排 | 健康/采集/后续时序 |
| /odin1/odometry | nav_msgs/Odometry | frame=odom，child=odin1_base_link | VLA 状态与录包 |
| /odin1/odometry_highfreq | nav_msgs/Odometry | 高频输出，队列可很大 | 时序诊断，不直接跨网 |
| /odin1/cloud_raw、cloud_slam、cloud_render | sensor_msgs/PointCloud2 | 点云单位/坐标各有设备约定 | 一期不上传 VLA |
| /odin1/wiwc | nav_msgs/Odometry | covariance 数组被复用存外参矩阵 | 不作为普通 covariance 使用 |
| /odin1/get_ae 等 | 自定义 srv | AE/AWB 调试，可能阻塞约 10 s | 禁止放入实时控制线程 |

Odin 驱动没有 cmd_vel 或飞行控制发布器。AE/AWB 服务和 /tmp/odin_command.txt 只控制传感器/算法参数，不能承载无人机运动。

#### agile/FSM 核心接口

uav_control_fsm/msg/uav_command.msg 定义：

- Move_mode=0：惯性系位置 + yaw；
- Move_mode=1：惯性系位置 + yaw rate；
- Move_mode=2/3：惯性系速度 + yaw/yaw rate；
- Move_mode=4：惯性系定高 + 水平速度；
- Move_mode=5..9：机体系位置/速度变体；
- Move_mode=10：轨迹；
- pos_ref 单位 m，vel_ref m/s，acc_ref m/s²；
- yaw_ref 单位度，yaw_rate_ref 单位度/s。

实飞入口 exp_single_uav_control_fsm.launch 默认 uav_id=1、loop_hz=30、cmd_timeout_sec=0.5、odom_timeout_sec=1.0、RC Offboard 三档开关默认通道 6。FSM 订阅 /uav1/move_command，发布 /uav1/mavros/setpoint_raw/local；cmd timeout 进入 AUTO_HOVER，odom/RC timeout 退出 Offboard/POSCTL；/uav1/land_command 只有在 FSM/PX4 当前允许的 Offboard 状态下才会请求 PX4 AUTO.LAND。

注意：control_manager.cpp 对部分模式使用 FRAME_LOCAL_NED，对 body velocity 使用 FRAME_BODY_NED，而 FSM 注释又按 ENU 的 z 向上解释。项目不能把“消息注释中的 ENU”和“MAVROS frame 常量”当成已自动解决的同一坐标系，必须在 vla_bridge 显式配置并通过单轴试验冻结符号。

#### 自定义 .msg/.srv 清单与边界

已经扫描 odin_ws/agile_ws 下的全部自定义消息和服务。按用途分组如下；vla_bridge 首版只直接依赖飞控 FSM 类接口：

| 类别 | 消息/服务 | 关键字段 | 与本项目的关系 |
|---|---|---|---|
| 飞控 FSM | uav_control_fsm/uav_command.msg | Move_mode、pos_ref、vel_ref、acc_ref、yaw_ref、yaw_rate_ref | 一期唯一控制出口；Move_mode=7/8 为 body velocity，角度用度 |
| 飞控 FSM | /uav1/hover_command、/uav1/land_command | std_msgs/Bool.data | 安全动作入口；data=true 才执行，类型不是 Empty |
| Odin 相机 | odin_ros_driver/GetAe.srv、SetAe.srv | exposure_time、gain、mode、rc、success | 仅地面图像维护；可能阻塞，不进实时桥接线程 |
| Odin 相机 | odin_ros_driver/GetAwb.srv、SetAwb.srv | rgain、bgain、mode、cct、rc、success | 同上，不作为飞行控制协议 |
| Fast-LIO | fast_lio/Pose6D.msg | offset_time、acc、gyr、vel、pos、rot | 算法内部预积分；不直接作为 VLA proprio |
| Livox | livox_ros_driver2/CustomMsg.msg、CustomPoint.msg | timebase、point_num、points[x,y,z,reflectivity] | 点云/激光链路；首版不跨网传给 VLA |
| Ego 目标 | quadrotor_msgs/GoalSet.msg | float32[3] goal | 后续 planner adapter 候选；要先确认坐标和单位 |
| Ego 轨迹 | quadrotor_msgs/PositionCommand.msg | position、velocity、acceleration、jerk、yaw、yaw_dot、trajectory_flag | planner 输出/监控，不替换首版 uav_command |
| Ego 轨迹 | quadrotor_msgs/PolynomialTrajectory.msg、LQRTrajectory.msg | trajectory_id、action、系数、时间、start_yaw/final_yaw | 后续轨迹适配；必须由 Ego 自己消费 |
| Ego 轨迹 | traj_utils/MINCOTraj.msg、PolyTraj.msg | uav_id、traj_id、start_time、系数/waypoints、duration | 多段轨迹内部接口 |
| Ego 状态 | quadrotor_msgs/Odometry.msg | curodom、kfodom、kfid、status | 定位/回环状态，不能未经 TF 对齐直接当 Odin odom |
| 飞控/硬件扩展 | quadrotor_msgs/AuxCommand、Corrections、Gains、SO3Command、TRPYCommand、OutputData、PPROutputData、StatusData、Serial | 姿态/推力、校正、串口协议、状态与电压 | 低层控制或硬件接口；一期禁止 vla_bridge 直写 |
| 调试显示 | traj_utils/DataDisp.msg | Header、a–e | 可视化调试，无控制语义 |

ROS 标准消息仍是主契约：sensor_msgs/Image/CompressedImage、sensor_msgs/Imu、nav_msgs/Odometry、mavros_msgs/State/PositionTarget、std_msgs/String/Bool/UInt8、diagnostic_msgs/DiagnosticArray。新增 vla_bridge 自定义消息只承载 schema、seq、时间戳、状态和错误码，不能复制 uav_command 或绕过 FSM。

#### UAV-Flow 当前实现

- openvla_act.py 是 Flask 开发服务，POST /predict，仓库默认配置端口 5007，默认 unnorm_key=sim、do_sample=False；代码构造器本身的 fallback port 为 5000，部署必须显式核对最终 config。
- 请求字段：image Base64、instr 字符串、proprio 数值数组。
- 服务端将图像解码为 PIL，构造包含状态和语言的 prompt，再调用 model.predict_action。
- 当前 SimpleVLADataset 读取 [x,y,z,yaw] 四维动作；动作由相邻位姿差在当前 yaw 下逆旋转得到，yaw 增量为弧度。
- 服务返回的 action_ori 是 model.predict_action 经当前 unnorm_key 反归一化后的局部相对动作（不是 Torch token，但仍需要 profile 才能确定单位/坐标）；action 是服务端依据当前 proprio 旋转/平移后得到的 episode-local 绝对位姿。
- 模型最终交付是已反归一化的 NumPy 连续动作，经 JSON tolist() 序列化，不是 ROS Tensor；HTTP 层会出现双层列表 [[...]]。
- processor 默认输入 224×224、RGB、letterbox/resize 策略由 checkpoint config 决定；融合视觉骨干可能需要双路图像 tensor，桥接端不应自行构造 tensor，只发送原始 RGB 图像。
- 论文中“6-DoF”是研究描述；当前仓库代码的动作维度必须以 checkpoint norm_stats 动态检查为准。

## 2. 六大模块一：整体架构设计与通信协议

### 2.1 三层组件和责任边界

~~~text
┌──────────────────────────────────────────────────────────────────────┐
│ 工作站（RTX 4090，VLA Server）                                      │
│  - OpenVLA-UAV /predict                                               │
│  - 读取 JPEG + episode-local proprio + instruction                   │
│  - 单飞推理，返回 action/action_ori、inference_ms、model profile     │
└───────────────────────▲───────────────────────┬──────────────────────┘
                        │ HTTP/TCP              │（未来 WebSocket/msgpack）
                        │ keep-alive            │
                WiFi/有线/LQ3 IP 链路           │
┌───────────────────────┴───────────────────────▼──────────────────────┐
│ Orin NX（vla_bridge，安全边）                                         │
│  Odin image/compressed ─┐                                             │
│  Odin odometry/imu ─────┼→ 时间对齐/预处理/状态转换                    │
│  instruction/reset ────┘→ HTTP client → response 校验                │
│                              ↓ action_ori → 坐标/单位/限幅/插值          │
│                              ↓ watchdog → move/hover/land              │
│  发布 /uav1/move_command（20～30 Hz）                                 │
└───────────────────────▲──────────────────────────────────────────────┘
                        │ ROS1 本机
┌───────────────────────┴──────────────────────────────────────────────┐
│ 机载执行层                                                             │
│ Odin1 driver → ROS image/odom/imu                                      │
│ uav_control_fsm → MAVROS → PX4                                        │
│ Ego-Planner（一期旁路；后续通过 adapter 接入）                         │
└──────────────────────────────────────────────────────────────────────┘
~~~

- 工作站负责视觉语言推理和模型版本管理，不直接知道 PX4 串口。
- Orin 负责所有实时性、安全、坐标转换、动作保持和人工接管判定。
- ROS 内部只使用标准 ROS1 topic/service；跨机只使用应用层 HTTP（一期），不把 ROS master 暴露到无线链路上。vla_bridge 运行在 Orin 的 ROS master 同一机内，工作站只看 HTTP/监控接口。
- LQ3 的定位是 IP 链路/图数传承载，不是 vla_bridge 的业务协议；更换 AX3、无线网桥或 LQ3 时，HTTP schema 不变。

### 2.2 一期协议：HTTP /predict

#### Orin → 工作站

机载端实际发送给当前 Flask 服务的最小兼容 payload：

~~~json
{
  "image": "<base64 JPEG bytes>",
  "proprio": [0.42, -0.18, 0.95, 12.0],
  "instr": "飞到目标物左侧并保持当前高度",
  "schema_version": "vla-bridge/1",
  "request_id": "uav1-episode07-000184",
  "capture_stamp_ns": 1725600000123456789,
  "state_frame": "episode_local",
  "position_unit": "m",
  "yaw_unit": "deg",
  "image_meta": {
    "source_topic": "/odin1/image/compressed",
    "source_encoding": "jpeg",
    "decoded_encoding": "bgr8",
    "wire_encoding": "rgb8_before_jpeg",
    "width": 640,
    "height": 533,
    "jpeg_quality": 80
  },
  "model_profile": "uav_flow_openvla_http_predict"
}
~~~

当前服务只依赖前三个业务字段（image、proprio、instr），其余字段供新服务、日志和诊断使用；服务端若严格 schema 校验，应由 client profile 删除未知字段，而不是改动控制逻辑。

proprio 一期严格为 [x_local_m, y_local_m, z_local_m, yaw_local_deg]，长度不是 4、包含 NaN/Inf、单位未声明时拒绝请求。若使用未经重新训练的 sim checkpoint，必须单独配置 position_unit=cm 和 unit_scale=0.01，不能把 sim 默认值默认为真实米制。

#### 工作站 → Orin

现有服务响应示例：

~~~json
{
  "status": "success",
  "action": [[1.10, -0.32, 0.98, 0.31]],
  "action_ori": [[0.24, -0.06, 0.01, 0.08]],
  "message": "Action generated successfully"
}
~~~

vla_bridge 收到后补充内部 canonical envelope：

~~~json
{
  "schema_version": "vla-bridge/1",
  "request_id": "uav1-episode07-000184",
  "status": "success",
  "action_dim": 4,
  "action_ori": [0.24, -0.06, 0.01, 0.08],
  "action": [1.10, -0.32, 0.98, 0.31],
  "action_ori_frame": "profile-defined; verify before flight",
  "action_frame": "episode_local",
  "action_mode": "delta_and_absolute",
  "position_unit": "profile-defined",
  "yaw_unit": "profile-defined",
  "capture_stamp_ns": 1725600000123456789,
  "receive_stamp_ns": 1725600000456789012,
  "inference_ms": 178.4,
  "model_key": "odin_real",
  "ttl_ms": 400
}
~~~

现有 /predict 不返回 frame、unit、absolute 标志；这些是 bridge 根据固定 model profile 和运行配置补齐的，不应伪装成模型真实输出。响应必须经过以下检查：

1. HTTP 状态为 200，JSON 可解析，status=success；
2. action/action_ori 可展开为一维数组，长度与启动时读取的 action_dim 一致；
3. 每个元素 finite，yaw 做 wrap；
4. request_id、seq、capture timestamp 与当前 in-flight 请求匹配；
5. 动作年龄未超过 TTL，旧响应不能覆盖新 seq；
6. 服务端错误只记录简短错误码，禁止把完整 traceback 直接显示给飞行控制线程。

### 2.3 HTTP、WebSocket、RTSP、gRPC 的职责决策

| 协议 | 一期决策 | 责任 | 不承担的责任 |
|---|---|---|---|
| ROS1 topic/service | 必须 | Orin 本机传感器、控制、安全、诊断 | 不跨公网/无线传原始 ROS master |
| HTTP keep-alive JSON | **默认** | 单帧请求/单步动作响应，最容易复现当前 UAV-Flow | 不提供高频飞控闭环；必须配合请求超时和 Orin 本地 watchdog |
| TCP 自定义二进制 | 暂不实现 | 只有在 HTTP/WS 的吞吐或时延测量不达标时，才作为受控内网传输候选 | 不跨 LQ3 公网暴露，不包含安全决策 |
| WebSocket + msgpack | 二期接口 | 长连接、动作 chunk、减少 JSON/base64 开销 | 不能替代本地 watchdog |
| RTSP/H.264 | 可选监控 | 人工查看/录制视频，默认 640×480、约 2.5 Mbps 配置 | 不作为带严格 ROS 时间戳的控制观测 |
| gRPC | 暂不实现 | 将来需要强 schema、流式 RPC、认证时再评估 | 当前仓库无 gRPC 服务，不把它写成现有能力 |

Orin 作为 HTTP client 主动连接工作站，避免 LQ3/校园网的入站防火墙问题。工作站服务监听 0.0.0.0:5007 只允许受控局域网访问；正式运行应替换 Flask 开发 server，至少增加请求大小限制、单飞推理锁、健康检查 /health、连接超时和可选 HMAC/网段白名单。

### 2.4 Odin1 图像链路、压缩、降帧和裁剪

#### 已确认链路

1. Odin SDK 收到相机 JPEG；
2. 驱动将 JPEG 解码为 OpenCV BGR，发布 /odin1/image，encoding=bgr8；
3. 驱动同步发布原始 JPEG 到 /odin1/image/compressed，format=jpeg；
4. 图像 header.frame_id 没有由驱动填充，bridge 必须补充 uav1/odin_camera_optical 或由参数指定；
5. 仓库未发现稳定的 sensor_msgs/CameraInfo 发布，内参/畸变来自运行时标定文件或外部配置。

协议注释、深度节点默认值中出现 1536×1280 与 1600×1296 两套尺寸，故不能在 bridge 中写死任何一个；启动时从消息 width/height、运行时 calibration 和模型 config 记录真实值。

#### 一期推荐策略

| 环节 | 初始值 | 规则 |
|---|---:|---|
| ROS 图像订阅 queue | 1 | 只保留 latest frame，丢弃旧帧，不让网络拥塞反向堵塞 Odin |
| 发送频率 | 2～5 Hz | 先以 5 Hz 测试；p95 请求延迟或 CPU 超标时降到 2～3 Hz |
| 边端最长边 | 640 px | 等比例缩放；低带宽 profile 可改 512/480；原始尺寸写入 metadata |
| JPEG quality | 75～85 | 目标单帧通常几十到一百多 KB；按 rostopic bw 和实测画质调整 |
| 色彩 | BGR8 → RGB8_before_JPEG | OpenCV 明确 cvtColor，避免红蓝通道反转；wire 上只传 JPEG bytes |
| 模型输入 resize | 工作站 processor 负责 | 默认 224，但必须读取 checkpoint config；边端不要重复拉伸到 224 |
| 裁剪 | 默认不裁剪 | 仅在训练/验证同样 FOV 的 ROI 配置下启用，并记录 crop 参数 |
| 去畸变 | 可选 | 只有 calibration 版本匹配时启用；否则保留原图 |
| 队列策略 | latest-only | 网络请求只允许一个 in-flight，过期图像直接丢弃 |

原始 BGR8 1536×1280 约 5.9 MB/帧，30 Hz 约 177 MB/s（还未计 ROS/TCP 开销），不能跨 WiFi/LQ3 传输。JPEG 5 Hz、每帧几十 KB 到一百多 KB，通常可压到数 Mbps 量级，但必须用 rostopic bw、iperf3 和实际 LQ3 测量。

#### RTSP 旁路

uav_rtsp_stream 已有 640×480、30 FPS、约 2500 kbps、8554 端口、/uavN/live 配置，并使用 latest-frame/leaky queue。它不保留原 ROS header，且代码默认 URL 可能显示为 127.0.0.1；部署时用 Orin 实际 IP 替换。若未来用 RTSP 给 VLA：

- 用 RTP/应用层序号和 capture timestamp side-channel；
- 同时保留 /odin1/image/compressed 作为带 ROS 时间戳的同步基线；
- 控制决策只接受与 odom 可对齐的帧，不能把合成 PTS 当 Odin 传感器时间；
- 断流时由 watchdog 进入 hover/manual，不等待 RTSP 自动恢复。

### 2.5 时间同步与网络测量

Odin use_host_ros_time 有设备时间、接收时间、PTP 对齐等模式；默认模式可能是设备 boot time。推荐：

1. Orin 与工作站先做 NTP/PTP 同步；
2. Odin 选择传感器时间减 PTP 平滑偏移的模式（具体参数名和实现以现场版本确认）；
3. bridge 同时保存 source_stamp、receive_stamp、send_stamp、response_stamp；
4. 不使用 odometry_bridge.cpp 覆盖为 ros::Time::now() 后的时间作为训练唯一真值；
5. 请求中以 capture_stamp 标记图像，响应中带 request_id/seq，控制端以单调时钟判断 TTL。

网络验收至少包括：

~~~bash
ping -c 100 <workstation_ip>
nc -vz <workstation_ip> 5007
curl --max-time 2 http://<workstation_ip>:5007/health
iperf3 -s
iperf3 -c <peer_ip> -t 30
iperf3 -c <peer_ip> -u -b 5M -t 30
~~~

记录平均、p95、最大 RTT、UDP jitter/loss、带宽余量和飞行区域遮挡情况；“能 ping 通”不等于能支持闭环。

## 3. 六大模块二：关键节点与模块划分

### 3.1 vla_bridge 包定位和依赖

建议新建 ROS1 catkin 包：

~~~text
drone_ws/agile_ws/src/vla_bridge/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── vla_bridge_direct.launch
│   └── vla_bridge_planner_adapter.launch
├── config/
│   ├── vla_bridge.yaml
│   ├── model_profiles.yaml
│   └── camera_calibration.yaml
├── msg/
│   ├── BridgeStatus.msg
│   └── InferenceStatus.msg
├── srv/
│   ├── ResetEpisode.srv
│   └── SetBridgeMode.srv
├── src/
│   ├── vla_observation_node.py
│   ├── vla_inference_client_node.py
│   ├── vla_command_adapter_node.py
│   └── vla_watchdog_node.py
├── scripts/
│   ├── mock_vla_server.py
│   ├── rosbag_to_uavflow.py
│   └── validate_model_contract.py
└── test/
~~~

依赖至少包括 roscpp 或 rospy、std_msgs、std_srvs、diagnostic_msgs、sensor_msgs、nav_msgs、geometry_msgs、cv_bridge、image_transport、tf/tf2_ros、uav_control_fsm；未来启用 Ego adapter 再加入 quadrotor_msgs、traj_utils。

建议状态消息只增加可观测字段，不承载控制量：

~~~text
BridgeStatus.msg
std_msgs/Header header
string state
bool enabled
string backend
uint64 heartbeat_seq
uint64 last_image_seq
uint64 last_action_seq
float32 image_age_sec
float32 odom_age_sec
float32 response_age_sec
float32 http_rtt_ms
bool fcu_connected
string px4_mode
bool rc_safe
string reason
~~~

InferenceStatus.msg 可复用 request_id、image_seq、capture_stamp_ns、receive_stamp_ns、action_dim、inference_ms、valid、error_code；动作数组仍留在 command adapter 的内部 canonical envelope 中，避免把未经限幅的模型输出暴露成控制 topic。

### 3.2 vla_observation_node

职责：

1. 订阅 /odin1/image/compressed，备用订阅 /odin1/image；
2. 使用 queue=1、tcpNoDelay、latest-only；
3. 解码、可选去畸变、等比例 resize；将 RGB 模型输入重新 JPEG 编码；
4. 订阅 /odin1/odometry，提取位置、四元数、yaw、frame_id、source stamp；
5. 维护 episode origin，生成 model-local proprio；
6. 输出内存队列给 HTTP client；可选发布不含图像内容的 observation metadata；
7. 统计 image age、decode time、dropped frame count、source/receive timestamp。

不应调用 AE/AWB 服务，不应上传点云，不应因等待工作站而阻塞 ROS 图像回调。

### 3.3 vla_inference_client_node

职责：

1. 接收 observation 最新快照和 instruction；
2. 组装 /predict 兼容请求；
3. 使用 HTTP connection pool/keep-alive，设置 connect/read 总超时；
4. 只允许一个 in-flight request；新帧可替换未发送帧，不能堆积；
5. 保存 raw JSON + canonical envelope；
6. 检查 seq、request_id、action_dim、finite、TTL、model key；
7. 发布 /vla_bridge/inference_status 和 /diagnostics；
8. 提供 reset_episode、set_enabled 服务，reset 时清空 origin、动作缓存和 seq。

### 3.4 vla_command_adapter_node

职责：

1. 接收最新 canonical action；
2. 依据 action_semantics 选择 action_ori 或 action；
3. 做单位换算、FLU/FRD/NED 转换、yaw wrap；
4. 做速度、加速度、jerk、工作空间、高度和动作 horizon 限幅；
5. 以 20～30 Hz 插值发布 /uav1/move_command；
6. 动作 TTL 到期后停止发布旧速度，改发 hover 或由 watchdog 接管；
7. 填充 header.stamp，内部另存 command_receive_monotonic；
8. 预留 direct 与 planner_adapter 两种 output backend。

一期 backend：

~~~text
action_ori [Δx,Δy,Δz,Δyaw]
    ↓ v_flu = Δp / action_horizon_sec
    ↓ yaw_rate_degps = rad2deg(Δyaw / action_horizon_sec)
    ↓ R_FLU→FRD/NED + 限幅/平滑
Move_mode=8
    ↓
/uav1/move_command
~~~

二期 backend：

~~~text
action_abs episode-local
    ↓ T_episode→odom/ENU，得到目标点
    ↓ planner adapter
quadrotor_msgs/GoalSet 或现有 EGO 入口
    ↓
Ego-Planner → /uav1/move_command
~~~

控制发布必须只有一个实际 owner：vla_command_adapter 是 /uav1/move_command 的唯一 publisher。vla_watchdog 不直接与 adapter 竞争发布速度指令，而是发布 /vla_bridge/safety_state 或调用 adapter 的 inhibit 接口；adapter 在 HOVER、MANUAL_OR_LAND 状态下停止旧动作，并只通过 /uav1/hover_command 或 /uav1/land_command 触发 FSM 的安全行为。启动时用 rostopic info 确认 /uav1/move_command 没有第二个 vla 节点 publisher。

### 3.5 vla_watchdog_node

独立于 HTTP client 和动作适配器，职责：

- 监测图像最近时间、odom 最近时间、IMU/高频 odom 沉默、HTTP RTT、最后有效 response seq、动作 TTL、FSM/PX4 state、RC；
- 发布标准 /diagnostics（diagnostic_msgs/DiagnosticArray）；
- 按三级状态机执行：OK → DEGRADED → HOVER → MANUAL_OR_LAND；
- HOVER 状态向 /uav1/hover_command 发布 std_msgs/Bool(data=true)，并停止旧速度；但 FSM 在 MANUAL_CTRL 下不会切入 Offboard hover，只记录/保持人工状态，不能把该 topic 当作无条件悬停保证；
- MANUAL_OR_LAND 按参数向 /uav1/land_command 发布 std_msgs/Bool(data=true) 或请求人工切回；FSM 只在允许的 Offboard/PX4 状态接受 land，拒绝条件必须记录并升级为人工接管。
- 不调用阻塞 AE/AWB 服务，不依赖工作站返回“安全动作”。

建议 /vla_bridge/status 使用 JSON 字符串或首版自定义状态消息，至少包含：

~~~json
{
  "state": "ACTIVE",
  "enabled": true,
  "backend": "direct",
  "heartbeat_seq": 1234,
  "last_image_age_ms": 42,
  "last_odom_age_ms": 8,
  "last_response_age_ms": 96,
  "last_action_seq": 12345,
  "http_rtt_ms": 178,
  "fcu_connected": true,
  "px4_mode": "OFFBOARD",
  "rc_safe": true,
  "reason": ""
}
~~~

watchdog 内部以 10–20 Hz 检查，status 以 2–5 Hz 发布；diagnostics 必须把每个超时项拆成独立 KeyValue，便于 rqt_runtime_monitor 或日志系统定位。heartbeat_seq 只表示 bridge 进程活着，不等于 VLA 推理健康；last_response_age、image_age、odom_age 和 FCU/RC 状态必须分别判断。

建议初始参数（无桨/系留阶段起点）：

~~~yaml
image_max_age_sec: 0.30
odom_max_age_sec: 0.20
response_ttl_sec: 0.40
degraded_timeout_sec: 0.40
hover_timeout_sec: 0.60
manual_or_land_timeout_sec: 2.0
inference_rate_hz: 5.0
command_publish_rate_hz: 30.0
max_inflight_requests: 1
~~~

### 3.6 ROS 命名规范和协同

- 节点名：vla_observation_node、vla_inference_client、vla_command_adapter、vla_watchdog。
- 绝对或显式参数化 topic：~image_topic=/odin1/image/compressed、~odom_topic=/odin1/odometry、~move_command_topic=/uav1/move_command。
- 多机使用 uav_id、odin_ns；禁止共用无前缀 /image、/odom。
- 诊断使用 /diagnostics，业务状态使用 /vla_bridge/status。
- 自定义消息首版只承载元数据，不替换 uav_control_fsm/uav_command。

## 4. 六大模块三：严格数据接口、序列化映射与坐标变换

本节给出第一版必须冻结的 observation、VLA HTTP、ROS 控制三层契约。所有实现应以此契约为准，任何字段变更都要同时更新 bridge、工作站服务和数据集转换脚本的 schema 版本。

### 4.1 机载 observation 的规范化中间结构

vla_observation_node 不直接把 ROS 消息拼接成模型输入，而是先形成统一的中间 observation：

| 字段 | 来源 | 规范化规则 |
|---|---|---|
| image | sensor_msgs/CompressedImage 或 sensor_msgs/Image | JPEG 解码为 BGR；仅在重新编码模型输入时转 RGB；保留原始 stamp、seq、宽高、来源 topic |
| capture_stamp | image.header.stamp | 优先使用 Odin 源时间；同时记录 receive_stamp 和 transport_delay |
| position_odom | nav_msgs/Odometry.pose.pose.position | 单位统一为 m；不得把厘米数据静默当作米 |
| attitude | nav_msgs/Odometry.pose.pose.orientation | 四元数转 yaw；记录原始四元数和 frame_id |
| imu | sensor_msgs/Imu | 作为诊断和可选 proprio 扩展，首版不悄悄改变模型输入维度 |
| episode_origin | reset 时锁存 | p0、psi0、origin_stamp、origin_frame |
| proprio | 由 odom 和 episode_origin 计算 | [x_local_m, y_local_m, z_local_m, yaw_local_deg]，默认长度为 4 |

局部状态定义为：

~~~text
p_local = Rz(-psi0) * (p_odom - p0)
psi_local = wrap_to_pi(psi_odom - psi0)
proprio = [p_local.x, p_local.y, p_local.z, rad2deg(psi_local)]
~~~

reset_episode 必须在静止、定位有效、TF 树唯一时执行。每个 observation 都带有：

~~~json
{
  "schema": "vla_observation_v1",
  "seq": 12345,
  "capture_stamp_ns": 0,
  "receive_stamp_ns": 0,
  "image_width": 640,
  "image_height": 512,
  "image_encoding": "rgb8_before_jpeg",
  "odom_frame": "odom",
  "base_frame": "odin1_base_link",
  "camera_frame": "odin1_camera_optical_frame",
  "position_unit": "m",
  "yaw_unit": "deg",
  "proprio": [0.12, -0.03, 1.04, 8.5]
}
~~~

示例只表示元数据，实际图像单独放在 HTTP 的 image 字段中。seq 应由 bridge 生成并贯穿请求、响应和控制日志，不能仅依赖 HTTP 到达顺序。

### 4.2 VLA 输入序列化

首版工作站接口固定使用 OpenVLA-UAV 的 POST /predict 兼容协议：

~~~json
{
  "schema": "vla_predict_v1",
  "request_id": "uav1-00000123",
  "image": "<base64 JPEG bytes>",
  "instr": "fly forward slowly and keep the target centered",
  "proprio": [0.12, -0.03, 1.04, 8.5],
  "capture_stamp_ns": 0,
  "image_seq": 12345,
  "model_key": "odin_real"
}
~~~

- 字段名必须是 instr，不要把它改成 instruction；当前 openvla_act.py 读取 data["instr"]。
- image 是 JPEG 字节的 Base64，不是 JSON 数组，也不是 Torch Tensor。Orin 端只负责 BGR/JPEG/RGB 的明确转换，不能把 CUDA Tensor 跨网络传输。
- proprio 首版为四维、米和度；若模型检查点要求六维或弧度，必须通过 profile 显式转换并在启动时拒绝不匹配配置。
- 请求只发送最新帧；旧请求完成后若 response 已过期，不得排队执行。
- instruction 的来源可为静态任务参数、/vla_bridge/instruction 或 episode manifest；自然语言内容和编码统一 UTF-8。

工作站服务返回：

~~~json
{
  "status": "success",
  "action": [[0.20, -0.01, 0.00, 0.05]],
  "action_ori": [[0.20, -0.01, 0.00, 0.05]],
  "request_id": "uav1-00000123",
  "image_seq": 12345,
  "model_key": "odin_real",
  "inference_ms": 172.4,
  "message": ""
}
~~~

现有服务实际返回 action 和 action_ori，但未必回传 request_id、seq、inference_ms；vla_bridge 应兼容缺省字段，同时在本地日志补齐关联信息。当前 openvla_act.py 的 self.act 已调用 model.predict_action(unnorm_key=...)，所以 action_ori 通常已经是反归一化后的数值；bridge 不能再次套 q01/q99。服务端 action 是应用当前 proprio 和 yaw 旋转/平移后的绝对/参考结果，action_ori 是未做该次世界/episode 变换的局部动作。两者都要保留，首版优先使用 action_ori 做显式坐标转换，避免重复旋转。

### 4.2.1 ROS、NumPy、Torch 和 JSON 的逐层映射

| 阶段 | 数据类型/形状 | 约定 |
|---|---|---|
| Odin compressed | sensor_msgs/CompressedImage.data：uint8 JPEG bytes | 保留 header.stamp、seq、format；首版默认转码为模型输入 JPEG，不假设直接透传原 JPEG 的颜色语义 |
| 解码 | NumPy/OpenCV uint8 H×W×3，BGR | 仅在需要模型输入时做一次 BGR→RGB |
| 上行图像 | RGB 图像重新编码为 JPEG bytes → Base64 UTF-8 string | JSON 的 image 字段；JPEG 字节本身不携带 ROS encoding，wire_encoding 只描述编码前的 RGB 语义；不传 NumPy/Tensor |
| ROS 状态 | geometry/quaternion → NumPy float32[4] | [x_m,y_m,z_m,yaw_deg]，先转 episode-local |
| VLA processor | PIL RGB → Torch pixel_values | 典型 [1,3,224,224]，dtype/device 由工作站 processor 决定；多路/fused processor 以 checkpoint config 为准 |
| 模型输出 | NumPy float32[action_dim] | 当前 /predict 已由 model.predict_action 按 q01/q99 反归一化；bridge 先依据 response_is_physical 判断，再做单位/坐标解释 |
| HTTP 响应 | NumPy.tolist() → JSON nested list | action/action_ori 可能是 [[...]]，bridge 必须展开并校验 |
| ROS 控制 | Python float → uav_command geometry_msgs/Point | vel_ref 为 m/s，yaw_rate_ref 为 degree/s，Move_mode 明确 |

因此不能把工作站的 Torch bfloat16、GPU tensor 或模型 token 直接塞进 ROS；ROS 端只接收经过 schema 校验的物理动作。

### 4.3 action 的语义、单位和动作维度

训练代码 uav_dataset.py 使用相邻轨迹计算局部增量：

~~~text
delta_p_body = Rz(-psi_t) * (p_(t+1) - p_t)
delta_yaw = wrap_to_pi(psi_(t+1) - psi_t)
action_ori = [delta_p_body.x, delta_p_body.y, delta_p_body.z, delta_yaw]
~~~

当前 real deployment 仍需确认模型输出的第四维是否为弧度；数据集代码内部以弧度计算，HTTP proprio 却以度拼入 prompt。这个潜在单位冲突必须在 profile 中冻结：

~~~yaml
action_dim: 4
action_position_unit: m
action_yaw_unit: rad
action_frame: body_flu
action_horizon_sec: 0.20  # 初始 profile 值；需由模型/实测 horizon 冻结
~~~

接收端执行以下顺序：

1. 解析 JSON，扁平化单元素二维数组，检查 action_dim。
2. 检查所有元素为有限数，拒绝 NaN、Inf、空数组和异常大值。
3. 读取 model profile 的 response_is_physical；当前 /predict 的 model.predict_action 已按 unnorm_key 反归一化，首版必须设为 true，禁止 bridge 再次套 q01/q99。只有未来服务明确返回 [-1,1] 归一化值时，才在 bridge 侧反归一化。
4. 根据 action_frame 转换到 ROS command frame。
5. 做速度、加速度、yaw 速率和工作空间限幅。
6. 插值生成 20–30 Hz 的 uav_command；response TTL 到期则进入降级状态。

模型归一化参数必须在服务启动时打印并校验。OpenVLA 侧通常从 norm_stats[model_key].action.q01、q99 和 mask 读取分位数；当前模型 key 默认为 sim，不能直接假定 odin_real 已存在。缺 key、action_dim 不一致或统计文件缺失时，服务必须 fail fast，而不是使用默认零统计量。

### 4.4 从 VLA 坐标到 Odin/FSM 坐标

本项目同时出现 camera optical、机体、odom/ENU 语义和 MAVROS LOCAL_NED/BODY_NED 语义，不能只凭变量名转换。第一版采用可配置变换矩阵，并用单轴地面实验冻结符号。

建议统一定义：

- 相机 optical：x 向右、y 向下、z 向前，遵循 ROS optical convention。
- 模型 action frame：body_flu，x 向前、y 向左、z 向上，单位 m。
- uav_command 的 body velocity 模式：FSM 最终发送 MAVROS FRAME_BODY_NED；源码直接把 vel_ref.x/y/z 写入 PositionTarget，具体机体轴正方向仍必须按 PX4/MAVROS 现场单轴试验冻结。
- inertial 模式：以当前工程实际 MAVROS/PX4 配置为准，不直接把 ENU/NED 名称当成数值证明。

若经单轴实验确认模型为 body_flu、FSM body 模式为 body_ned，初始映射可配置为：

~~~text
[vx_ned, vy_ned, vz_ned] = [vx_flu, -vy_flu, -vz_flu]
yaw_rate_ned = sign_yaw * yaw_rate_model
~~~

sign_yaw 不能盲猜；应分别发 x、y、z、yaw 四个小幅命令，记录 odom 变化，形成 signed_axis_test.yaml。若模型输出相机坐标，先应用标定外参：

~~~text
T_body_action = T_body_camera * T_camera_action
T_world = T_world_body * T_body_action
~~~

不要在服务端已经把 action_ori 旋到当前 yaw 后，又在 bridge 再做一次 Rz(psi)；这会造成重复旋转。若使用 /predict 返回的绝对 action，则在 episode 局部坐标中执行：

~~~text
p_odom_cmd = p0 + Rz(psi0) * p_local_cmd
psi_odom_cmd = wrap_to_pi(psi0 + psi_local_cmd)
~~~

首版推荐 action_ori + Move_mode=8（body velocity），因为它天然适合短 horizon 和速度限幅；只有在绝对位置语义、yaw 单位和坐标轴经过离线和无桨验证后，才启用位置模式 Move_mode=0。

### 4.5 ROS 两向接口

| 方向 | 接口 | 类型 | 频率/策略 |
|---|---|---|---|
| 机载 → bridge | /odin1/image/compressed | sensor_msgs/CompressedImage | 最新帧，目标 5 Hz 左右 |
| 机载 → bridge | /odin1/image | sensor_msgs/Image | 调试或低频校验，不作为跨网默认链路 |
| 机载 → bridge | /odin1/odometry | nav_msgs/Odometry | 订阅最新值，目标 ≥30 Hz |
| 机载 → bridge | /odin1/imu | sensor_msgs/Imu | 诊断/可选融合 |
| 机载 → bridge | /uav1/mavros/state | mavros_msgs/State | 状态门控 |
| 机载 → bridge | /uav1/mavros/local_position/odom | nav_msgs/Odometry | 与 FSM 实际反馈对齐 |
| bridge → 机载 | /uav1/move_command | uav_control_fsm/uav_command | 20–30 Hz keepalive |
| bridge → 机载 | /uav1/hover_command | std_msgs/Bool | 降级/人工接管，data=true |
| bridge → 机载 | /uav1/land_command | std_msgs/Bool | 仅明确配置时启用，data=true |
| bridge → 监控 | /vla_bridge/status | std_msgs/String 或自定义状态 | 每次 response/状态改变 |
| bridge → 监控 | /vla_bridge/inference_status | vla_bridge/InferenceStatus | 每次请求完成或失败 |
| watchdog → adapter | /vla_bridge/safety_state | std_msgs/UInt8 | 0=DISABLED, 1=SHADOW, 2=ACTIVE, 3=DEGRADED, 4=HOVER, 5=MANUAL_OR_LAND |
| bridge → 监控 | /diagnostics | diagnostic_msgs/DiagnosticArray | 1–5 Hz |
| 操作 → bridge | /vla_bridge/instruction | std_msgs/String | 当前 episode 的语言任务 |
| 操作 → bridge | ~reset_episode | std_srvs/Trigger | 锁存局部原点 |
| 操作 → bridge | ~set_enabled | std_srvs/SetBool | 启停直接控制 |

首版不新造一个与 uav_command 重复的控制消息。若后续接入 Ego-Planner，增加 vla_goal_adapter 或 vla_ego_adapter，将 VLA 的局部目标转换为 planner 输入，再由 planner 输出现有 move_command；direct 和 planner 模式必须互斥。

建议把服务契约固定为：

~~~text
ResetEpisode.srv
string episode_id_hint
---
bool success
string episode_id
string message

SetBridgeMode.srv
bool enabled
string backend
---
bool accepted
string active_backend
string message
~~~

若暂时使用标准 std_srvs/Trigger 或 SetBool，也要在 status 中返回 episode_id、active_backend 和拒绝原因。

## 5. 六大模块四：真机数据采集、rosbag 解包与 VLA 微调

### 5.1 采集前冻结项

每个 episode 开始前记录硬件序列号、Odin 固件/驱动 commit、相机分辨率和 FOV、相机安装外参、PX4 参数快照、ROS topic 列表、NTP/PTP 状态、模型/桥接 git commit、操作者、天气/光照和自然语言任务。docs/1号机.params 只能作为快照，不能未经现场核对推断所有 failsafe 语义。

第一阶段只允许遥控或手动飞行采集，VLA 不发布控制。选择开阔区域、低速轨迹和可复现的语言任务，覆盖直行、侧移、升降、定点、转向、目标跟随和中止动作。每次动作前后留出 1–2 秒稳定段，方便同步和切片。

### 5.2 rosbag 录制清单

建议使用分卷、无压缩或适度压缩的 rosbag；图像主链路优先记录 compressed，同时低频录 raw 用于质量抽检：

~~~bash
rosbag record --split --size=2048 \
  /odin1/image/compressed /odin1/image /odin1/image/undistorted \
  /odin1/odometry /odin1/odometry_highfreq /odin1/imu \
  /uav1/mavros/local_position/odom /uav1/mavros/state \
  /uav1/mavros/extended_state /uav1/mavros/rc/in \
  /uav1/move_command /uav1/mavros/setpoint_raw/local \
  /uav1/flight_path /tf /tf_static \
  /vla_bridge/instruction /vla_bridge/status /diagnostics
~~~

自然语言使用 /vla_bridge/instruction（std_msgs/String）发布，并在 episode manifest 再保存一次。若使用已有 uav_path_recorder_node 的 CSV，CSV 作为校验和便捷索引，rosbag 才是时间同步的主证据。不要用 odometry_bridge 重写过的当前时间戳替代 Odin 原始时间。

建议每个 episode 的目录为：

~~~text
dataset/
  episode_000123/
    000000.jpg
    000001.jpg
    log.json
    metadata.yaml
    bag_index.yaml
    calibration/
      camera.yaml
      tf_static.yaml
~~~

log.json 至少包含 episode_id、instruction、instruction_unified、image_files、image_timestamps_ns、odom_timestamps_ns、raw_logs、preprocessed_logs、action_source、position_unit、yaw_unit、camera_profile、hardware_commit、valid_mask。

### 5.3 rosbag 到训练样本

转换脚本建议拆成四步，避免一个脚本同时隐式改变坐标和图像：

1. extract_rosbag.py：解码 JPEG、提取原始/接收时间、导出 odom/IMU/command、保存 TF 和元数据。
2. align_episode.py：以图像为主时间轴，使用最近邻或短窗口线性插值对齐 odom；记录最大时间差，超阈值样本置 invalid。
3. make_local_actions.py：锁存 episode 原点，计算局部 proprio 和真实运动 action。
4. export_vla_dataset.py：输出现有 SimpleVLADataset 所需 folder/log.json，或输出 HDF5/RLDS。

当前 uav_dataset.py 读取 trajectory 的列 [0, 1, 2, 4] 作为 x、y、z、yaw，并把 yaw 从度转弧度；相邻样本 action 使用当前机体 yaw 的逆旋转。因此转换脚本必须明确生成与此兼容的列，而不能把 [x,y,z,yaw] 误写成 [x,y,z,roll]。每个 episode 最后一帧 action 置零并通过 valid_mask 排除边界损失。注意：当前 SimpleVLADataset 实现扫描 episode 根目录下的图片，不会递归扫描 images/ 子目录；因此本方案示例将 JPEG 直接放在 episode 根目录，若保留 images/ 层级，必须同步修改 loader 并增加测试。

推荐重采样到 5 Hz 左右，与真实模型推理预算接近；保存原始高频轨迹以便以后改 horizon。实际 action 优先用 odom 观测差分，/uav1/move_command 仅作为意图标签和安全审计，不能把命令当成已执行运动。

### 5.4 HDF5/RLDS 统一 schema

为了兼容当前训练代码和后续 RLDS，统一逻辑字段如下：

~~~text
observation/image_primary: uint8 [H,W,3], RGB
observation/proprio: float32 [4], [x_m,y_m,z_m,yaw_deg]
task/language_instruction: UTF-8 string
action: float32 [4], [dx_m,dy_m,dz_m,dyaw_rad]
episode_id: int64
timestamp_ns: int64
image_seq: int64
valid_mask: bool
camera_profile: string
~~~

当前 RLDS 路径默认 load_proprio=False、window_size=1、future=0，不能假设它会自动读取 proprio。需要自定义 transform，或在训练配置中明确关闭/开启该字段；否则会出现模型 prompt 有状态、训练样本却没有状态的隐性不一致。

### 5.5 FOV、安装角度与 OOD 处理

- 几何层：为每种 Odin 安装建立 camera_profile，保存内参、畸变、裁剪 ROI、外参和最终输入宽高；训练和部署必须使用同一套 resize/letterbox 规则。
- 外观层：对亮度、色温、轻微模糊、JPEG 质量和天气做受控增强，但不改变运动几何。
- 语义/动力学层：按场景、目标、速度、飞行高度和光照分层采样，确保验证集包含未见过的组合。
- 不要用任意旋转增强来“修复”真实安装角度；安装改变应通过 TF/extrinsic 和相机 profile 修正。
- 每次部署前跑彩色棋盘/刻度板和四方向单轴动作检查，验证图像左右与 body y 方向没有镜像。

### 5.6 微调和回滚

先做 50–200 条样本的过拟合格式测试，再进行 LoRA 微调。训练配置固定记录 base checkpoint、model_key、norm_stats、action_dim、processor 配置、相机 profile、数据集 hash 和 git commit。首版沿用 LoRA rank 32 等工程默认值，但按 4090 显存通过 batch size/gradient accumulation 调整，不修改数据契约来迁就显存。

离线验收包括：训练 loss 能过拟合小集、动作反归一化后单位正确、零动作输入输出有限、不同 FOV 样本不会触发维度错误、旧 checkpoint 仍能回滚。部署时先 shadow inference 只记录 action，不发布控制；连续通过 replay 和无桨测试后才启用 direct。

## 6. 六大模块五：阶段性开发、联调与 ROS 调试工具

### 6.1 阶段总览

| 阶段 | 范围 | 允许控制 | 必过条件 |
|---|---|---|---|
| P0 | 基线冻结 | 无 | topic、版本、时间同步、坐标约定和安全开关有记录 |
| P1 | Mock bridge | 模拟器/假节点 | 可注入坏包、延迟、断网、NaN，状态机正确 |
| P2 | 离线 VLA replay | 无 | rosbag→HTTP→action 可复现，schema 和 latency 合格 |
| P3 | 真机感知 | 无 | Odin 图像/odom/TF 连续，图像质量和时间差合格 |
| P4 | 地面无桨 | 仅低幅 setpoint | 单轴方向、限幅、TTL、hover/manual 逻辑通过 |
| P5 | 系留/低风险 | direct 小动作 | 5–10 分钟闭环无失控，人工接管有效 |
| P6 | 短航线 | direct | 低高度、低速、固定场景通过；有完整 bag |
| P7 | 场景微调 | shadow→direct | 新 FOV/场景 OOD 指标改善且旧任务不回退 |
| P8 | Ego 适配 | planner | direct/planner 互斥，planner timeout 和安全链路通过 |

### 6.2 P0/P1：基线与故障注入

在 Orin 和工作站分别保存：

~~~bash
rosversion -d
uname -a
rospack find odin_ros_driver
rospack find uav_control_fsm
rosnode list
rostopic list
ip addr
hostname
~~~

编译基线应显式使用真实工作区路径，并分开记录 Odin ROS1 与 agile_ws 的环境叠加顺序：

~~~bash
source /opt/ros/noetic/setup.bash
cd /home/cym/vla_uav/drone_ws/odin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
cd /home/cym/vla_uav/drone_ws/agile_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rospack find odin_ros_driver
rospack find uav_control_fsm
rospack find vla_bridge
~~~

若实际部署路径不是当前仓库路径，使用现场绝对路径替换；不要依赖 build_ros.sh 中未定义的 WS_DIR 或 agile_ws/build.sh 的 HOME 硬编码。注意：若 Odin workspace 的 catkin package 仍把 ROS2 分支作为默认安装路径，先显式指定 BUILD_SYSTEM=ROS1 或修正 CMake，再执行上述命令。两个 workspace 的最终 overlay、git commit 和编译日志必须写入 P0 记录。

P1 的 mock server 必须测试 HTTP 200、400、500、空 JSON、缺字段、错误 image、慢响应、乱序 seq、重复 response、NaN/Inf、连接拒绝和工作站断电。验收依据是 bridge 始终不发布非法 uav_command，并按状态机转入 DEGRADED/HOVER/MANUAL_OR_LAND。

### 6.3 P2：离线模型契约测试

用固定 RGB 色卡、灰图、损坏 JPEG、不同宽高比和真实 bag 帧调用 /predict。记录请求 JSON hash、image_seq、response action、推理耗时和反归一化结果。确认：

- RGB/BGR 不会交换；
- 224 输入、letterbox/resize 规则与 processor 一致；
- instr 字段和 Unicode 正常；
- proprio 的长度、米/度单位正确；
- action_ori 与 action 的旋转关系只应用一次；
- 服务无 norm_stats/model_key 时启动即报错；
- /reset 不作为当前协议依赖，因为现有 openvla_act.py 没有该路由。

### 6.4 P3：ROS 感知与拓扑检查

~~~bash
rosnode list
rosnode info /host_sdk_sample
rosnode info /vla_observation_node
rostopic list
rostopic info /odin1/image/compressed
rostopic hz /odin1/image/compressed
rostopic hz /odin1/odometry
rostopic echo -n1 /odin1/image/compressed/header
rostopic echo -n1 /odin1/odometry/header
rosmsg show sensor_msgs/CompressedImage
rosmsg show nav_msgs/Odometry
rosmsg show uav_control_fsm/uav_command
rostopic info /uav1/move_command
rqt_graph
rqt_image_view
rqt_tf_tree
rviz
~~~

验收重点：实际 driver 的全局 topic 是否被 /uav1 相对 remap 错配；image header.frame_id 是否为空；odom→base_link→camera 的 TF 是否唯一；compressed 图像的 format 是否为 jpeg；高频 odom 没有被重复发布成虚假的当前时间。

### 6.5 P4–P6：控制闭环和安全测试

先确认 FSM、PX4 状态、RC 通道 6 和人工接管：

~~~bash
rostopic hz /uav1/move_command
rostopic echo -n1 /uav1/move_command
rostopic hz /uav1/mavros/setpoint_raw/local
rostopic echo -n1 /uav1/mavros/state
rostopic echo -n1 /uav1/mavros/extended_state
rostopic echo -n1 /vla_bridge/status
rostopic echo -n1 /diagnostics
~~~

无桨时逐项发送极小 body x、body y、body z、yaw 命令，比较 odom 增量并生成 signed_axis_test.yaml。随后分别拔掉图像、odom、网络、VLA 服务，观察 response TTL、FSM 的 0.5 秒 command timeout、odom timeout 和本地 watchdog 是否按预期动作。任何方向符号不确定都不得进入有桨阶段。

系留/短航线流程固定为：手动悬停 → 开启 bridge 但 disabled → shadow response → 极小 direct 指令 → 立即 set_enabled false → 验证 hover/land/人工切换。每次测试必须留存 rosbag、diagnostics、网络 RTT 和最终状态。

### 6.6 工具与验收指标

| 工具 | 检查目标 | 通过标准 |
|---|---|---|
| rosnode list/info | 节点是否存在、订阅/发布是否正确 | 没有重复 bridge、没有意外私有 topic |
| rostopic list/info/echo | 类型、连接数、header 和字段 | 类型与 schema 完全一致 |
| rostopic hz | 图像、odom、command、setpoint 频率 | 图像按配置，command 20–30 Hz，setpoint 满足 FSM |
| rosmsg show | 消息字段和常量 | Move_mode、yaw 单位和数组长度无歧义 |
| rqt_graph | 端到端拓扑 | 图像→bridge→VLA→command 单向清晰 |
| rqt_tf_tree | 坐标树 | 无环、无双重 parent、camera 外参存在 |
| rviz | 轨迹、姿态和目标 | 单轴测试方向与视觉一致 |
| rqt_image_view | 图像色彩/裁剪 | RGB/BGR、畸变和 FOV 符合 profile |

每阶段都应记录 p50/p95 图像延迟、HTTP RTT、推理耗时、action age、command 发布率、丢帧率、CPU/GPU/内存和网络吞吐。仅看平均值不足以证明安全。

## 7. 六大模块六：架构隐患排查与工程优化

| 隐患 | 当前证据/原因 | 工程措施 |
|---|---|---|
| 原始图像带宽过大 | 1536×1280 BGR8 约 5.9 MB/帧，30 Hz 约 177 MB/s | 机载 resize、ROI、JPEG 质量 75–85、5 Hz 最新帧；RTSP 仅作监控 |
| RTSP 与同步脱节 | RTSP streamer 丢旧帧且无 ROS header | 主控制仍用 CompressedImage；若 RTSP 做模型输入，增加 seq/timestamp side channel |
| VLA 低频、FSM 高频 | VLA 约 2–5 Hz，FSM loop 默认 30 Hz，cmd timeout 0.5 s | 单飞行请求、latest-only、短 horizon 插值、速度/加速度限幅、response TTL |
| 工作站断网 | HTTP 阻塞或旧动作继续执行 | connect/read timeout、max inflight=1、断网立即降级；本地 watchdog 不信任远端 |
| FSM timeout 不是完整 failsafe | 速度/轨迹超时会 hover，但 odom/RC 超时转 manual；PX4 行为还依赖参数 | bridge 自己维护状态机，并现场验证 hover、manual、land、RTL 参数 |
| 时间戳不可靠 | source/receive 时间可能不同；某些 bridge 会用 now 覆盖 stamp | NTP/PTP、保存双时间戳、以原始 Odin 时间作训练同步、记录 transport delay |
| 坐标系命名混乱 | 文档有 ENU，FSM body 使用 FRAME_BODY_NED，注释又采用 z-up 语义 | profile 化变换、单轴实验、禁止隐式 Rz/轴翻转 |
| topic/remap 不一致 | driver 发布全局 /odin1/...，旧 launch 可能解析为 /uav1/odin1/... | 统一绝对 topic；启动后用 rosnode info、rqt_graph 实测 |
| image frame_id 为空 | Odin driver 未稳定填充 camera frame | bridge 补 frame_id 只用于明确来源；TF 仍需静态发布 |
| 压缩开关不可靠 | g_sendrgb_compressed 读取不等于所有路径都不发布 | 以实际 rostopic hz/info 和字节统计为准，不依据参数名推断 |
| OpenVLA 服务不适合生产 | Flask 开发 server、字段脆弱、/reset 不存在 | 加 schema 校验、进程守护、固定 worker/反向代理或生产 WSGI；保留健康检查 |
| norm_stats/model key 不匹配 | 默认 key 为 sim，真实动作维度/统计可能不同 | odin_real 单独统计和 checkpoint；启动 fail fast，日志打印 hash |
| FOV/安装 OOD | 仿真/训练视场和 Odin 实际镜头不同 | camera_profile、外参标定、部署同构预处理、分层验证和 shadow |
| ROS1 launch 安装路径不一致 | Odin ROS1 源文件在 launch_ROS1/，CMake 安装段主要检查/安装 launch/ | 在 P0 用 rospack/roslaunch 实测；必要时把 ROS1 launch 显式安装到 share/odin_ros_driver/launch，或固定从源码路径启动 |
| build 脚本脆弱 | build_ros.sh 使用未定义 WS_DIR，agile build.sh 硬编码 HOME/agile_ws | 第一阶段记录手工可复现命令；后续修为显式参数并 CI 编译 |
| 点云/AE/AWB误入主链 | 点云带宽大，AE/AWB service 可能阻塞约 10 秒 | 首版不上行点云；曝光服务只在地面维护阶段调用 |
| ROS master 跨网依赖 | LQ3 可能只提供 IP 链路，不等于可稳定承载 ROS master | ROS 只留在机载，跨网使用 HTTP/RTSP；必要时用 VPN/反向连接 |

### 7.1 安全状态机

建议状态为 DISABLED、SHADOW、ACTIVE、DEGRADED、HOVER、MANUAL_OR_LAND。进入 ACTIVE 必须同时满足图像、odom、FCU connected、模式允许、RC 安全和 VLA health。任何一个关键条件过期都不能继续复用最后动作。建议启动顺序仍遵循 roscore → Odin driver → MAVROS/PX4 → uav_control_fsm → vla_bridge；Ego-Planner 仅在 planner backend 启用时启动。

推荐超时层级：

~~~text
0.30 s：图像或 odom 老化，停止发送新 VLA 请求并标记 DEGRADED
0.40 s：动作 TTL 到期，adapter 停止复用旧动作并请求安全过渡
0.60 s：无法恢复，若 FSM 当前允许则触发 hover；MANUAL_CTRL 下仅保持人工接管
2.0 s：仍无图像/odom/工作站，按配置请求 manual 或 land；land 是否执行由 FSM/PX4 当前状态决定
~~~

最终 land/RTL 不能只依赖 Python bridge；应由 FSM、PX4 failsafe、RC 开关和现场人工共同验证。任何自动降落策略上线前，必须完成无桨、系留和受控高度测试。

## 8. vla_bridge 配置基线

以下是第一版无桨/系留阶段的配置骨架，数值不是飞行认证参数：

~~~yaml
uav_id: 1
image_topic: /odin1/image/compressed
raw_image_topic: /odin1/image
odom_topic: /odin1/odometry
imu_topic: /odin1/imu
move_command_topic: /uav1/move_command
hover_topic: /uav1/hover_command
land_topic: /uav1/land_command
command_backend: direct
command_publish_rate_hz: 30.0
inference_rate_hz: 5.0
http_url: http://<workstation_ip>:5007/predict
http_connect_timeout_sec: 0.10
http_read_timeout_sec: 0.25
max_inflight_requests: 1
image_queue_size: 1
image_max_age_sec: 0.30
response_ttl_sec: 0.40
resize_long_edge: 640
jpeg_quality: 80
source_encoding: bgr8_ros
wire_encoding: rgb8_before_jpeg
model_key: odin_real
expected_action_dim: 4
response_is_physical: true
position_unit: m
action_horizon_sec: 0.20  # 初始 profile 值；需由模型/实测 horizon 冻结
model_action_frame: body_flu
ros_command_frame: body_ned
flip_y: true
flip_z: true
flip_yaw: true
max_xy_speed_mps: 1.0
max_z_speed_mps: 0.5
max_yaw_rate_degps: 30.0
on_degraded: slow_down
on_hover_timeout: hover
on_long_timeout: manual
~~~

配置加载时应输出一份脱敏后的 effective_config.json，并将 schema、git commit、model hash、camera profile 和参数 hash 写入每个 rosbag 的 metadata。

## 9. 交付、版本和回滚

每个阶段以可回滚的 commit 交付，至少保存：

- 代码和 launch/config 的 commit；
- rosnode/rostopic/rqt_graph/rqt_tf_tree 验收截图或文本；
- 网络 RTT、带宽、CPU/GPU/内存报告；
- VLA request/response contract 样例和 JSON schema；
- rosbag、CSV、dataset manifest、norm_stats 和 checkpoint hash；
- 故障注入结果及最终安全状态。

运行时提供三个开关：

~~~text
command_backend = disabled | direct | planner
vla_enabled = false | true
watchdog_action = hover | manual | land
~~~

回滚顺序为先关闭 vla_enabled，再切 command_backend=disabled，确认 FSM/PX4 可手动控制，最后恢复旧 bridge/model。不得在飞行中热切换坐标 profile 或 action_dim。

## 10. 现有命名与新接口对照

| 语义 | 工程实际/推荐规范 | 处理方式 |
|---|---|---|
| Odin 原始图像 | /odin1/image，sensor_msgs/Image，bgr8 | 机载本地调试和低频录制 |
| Odin 压缩图像 | /odin1/image/compressed，sensor_msgs/CompressedImage | vla_bridge 默认输入 |
| 去畸变图像 | /odin1/image/undistorted | 仅在 profile 验证后启用 |
| Odin 里程计 | /odin1/odometry，nav_msgs/Odometry | bridge 默认状态源，核对 frame |
| 高频里程计 | /odin1/odometry_highfreq | 训练/诊断可录，不直接替代状态源 |
| FSM 控制 | /uav1/move_command，uav_control_fsm/uav_command | direct 与 planner 最终共同出口 |
| 旧文档目标点 | /uav1/goal_point 等 | 不作为首版 direct 接口，逐项核对后再适配 |
| bridge 包 | vla_bridge | 新建 ROS1 package，节点名见 3.6 |
| VLA 服务 | POST /predict | 固定 instr/proprio/image；/act 作为另一 profile |

## 11. 代码与文档证据索引

实施时优先回看以下文件，任何接口改动都要同步更新本计划：

- docs/Orin_NX_ROS部署与编译说明.md：ROS Noetic、odin_ws/agile_ws 构建和启动关系。
- docs/项目开发手册.md：启动顺序、FSM 状态、topic、超时、Move_mode 和调试命令。
- docs/云边协同无人机视觉语言导航项目计划书.md：HTTP/WebSocket/RTSP 分工、Mock→实机阶段和数据采集方向。
- docs/UAV-Flow 跨平台仿真复现完整指南.md：/predict 协议、ROS 接入和安全阶段。
- docs/无人机 + VLA 云边协同部署网络方案技术总结.md：网络、AP/LQ3、带宽与边缘安全建议。
- docs/MinerU_markdown_UAV-Flow_2100208414561165312.md：模型、动作 token、数据和推理延迟背景。
- drone_ws/odin_ws/src/odin_ros_driver/include/host_sdk_sample.h：Odin image、compressed、IMU、odom 的实际 publisher 和 frame。
- drone_ws/odin_ws/src/odin_ros_driver/launch_ROS1/odin1_ros1.launch：源码仓库中的 ROS1 launch；CMake 安装段需现场核对 launch_ROS1 是否被正确安装/映射到 catkin share 目录，不能只根据源目录存在推断 roslaunch 可见。
- drone_ws/agile_ws/src/uav_control_fsm/msg/uav_command.msg：Move_mode、字段和 yaw 单位。
- drone_ws/agile_ws/src/uav_control_fsm/include/uav_control_fsm/fsm_control.hpp：FSM 订阅、超时、hover/manual 行为。
- drone_ws/agile_ws/src/uav_control_fsm/src/control_manager.cpp：LOCAL_NED/BODY_NED 和 body 旋转实现。
- drone_ws/agile_ws/src/uav_control_fsm/src/uav_path_recorder_node.cpp：轨迹、状态和 move_command 记录。
- UAV-Flow-main/OpenVLA-UAV/vla-scripts/openvla_act.py：/predict 请求、proprio、action/action_ori 及 yaw 变换。
- UAV-Flow-main/OpenVLA-UAV/prismatic/models/vlas/openvla.py：action 解码、norm_stats 和 action_dim。
- UAV-Flow-main/OpenVLA-UAV/prismatic/vla/datasets/uav_dataset.py：局部 action、yaw、图像和 instruction 数据约定。
- UAV-Flow-main/OpenVLA-UAV/prismatic/extern/hf/modeling_prismatic.py：模型 action 反归一化和 token 输出。
- UAV-Flow-main/OpenVLA-UAV/prismatic/extern/hf/processing_prismatic.py：RGB、resize、letterbox 和 tensor 输入。
- drone_ws/agile_ws/src/perception/uav_rtsp_stream/config/rtsp_stream.yaml：RTSP 默认分辨率、码率、端口。

## 12. 不可跳过的上线检查清单

- [ ] 实际 topic 与本文件一致，所有相对 remap 已用 rosnode info 验证。
- [ ] Odin 图像编码、压缩格式、宽高、header.frame_id 和 source/receive 时间已记录。
- [ ] odom、camera、base、world TF 唯一，外参和相机 profile 已版本化。
- [ ] action_dim、model_key、norm_stats、yaw/position 单位和 action frame 已启动校验。
- [ ] body x/y/z/yaw 单轴方向已用无桨实测并保存结果。
- [ ] VLA 2–5 Hz 到 command 20–30 Hz 的插值、TTL、加速度限幅已注入故障验证。
- [ ] 图像断流、odom 断流、HTTP 超时、NaN、乱序 response、工作站断电均能安全降级。
- [ ] RC 人工接管、FSM hover、PX4 failsafe、manual、land/RTL 已在现场验证。
- [ ] rosbag 包含图像、odom、IMU、状态、命令、TF、instruction、diagnostics。
- [ ] replay、shadow、无桨、系留和短航线每阶段都有可回滚产物。
- [ ] direct/planner 互斥开关和旧版本恢复流程已演练。

本文件是后续代码开发和现场部署的基线；如果源码、实际飞控参数或现场测量与文档冲突，应先更新事实证据和接口版本，再继续开发，不得靠临时 remap 或隐式单位转换绕过冲突。
