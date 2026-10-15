# vla_bridge 代码清单

> 日期：2026-09-19  
> 分支：`feat/vla-bridge`（未改 `main`）  
> 范围：云边 VLA 第一阶段（**直连 FSM，不经 Ego-Planner**）  
> 基线：[`docs/VLA_Drone_Deployment_Plan.md`](docs/VLA_Drone_Deployment_Plan.md)  
> 进度：[`session_handoff.md`](session_handoff.md)

本轮在笔记本上完成实施方案第 7 章 **1–8 步**。默认 `control/enable_publish=false`，**不会解锁、起飞、发 Offboard**。  
4090 上的 `UAV-Flow-main`、机载 `uav_control_fsm`、Odin 驱动、Ego-Planner **都没有改业务代码**。

---

## 1. 做了什么

在 `drone_ws/agile_ws` 新建 ROS 包 `vla_bridge`：Orin 把 Odin JPEG 降到 224×224，经 HTTP `POST /predict` 交给 4090；把 4D 动作从第一帧局部系变到 ENU，限幅后 25 Hz 预览；云断则本地 hover。采集与转数据集脚本一并写好，便于以后用真机 bag 微调。

数据流（默认不写飞控）：

```text
Odin JPEG/odom（或假数据）
  → vla_image_node          2 Hz、224 letterbox、first-frame proprio
  → vla_infer_client_node   HTTP /predict → /vla/action_raw
  → vla_cmd_node            ENU 限幅 + 25 Hz 插值 → /vla/debug/move_command
  → vla_watchdog_node       超时 HOVER/LAND 预览，切断 enable_control
```

闸门：只有 `backend=direct_fsm` **且** `enable_publish=true` **且** `enable_control=true` 才写 `/uav1/move_command`。当前三项不齐。

---

## 2. 新建文件

路径均相对仓库根目录。包根：`drone_ws/agile_ws/src/vla_bridge/`。

### 2.1 包骨架与接口

| 文件 | 用途 |
|---|---|
| `package.xml` | catkin 包描述与依赖（rospy、消息、cv_bridge、rosbag 等）。 |
| `CMakeLists.txt` | 生成 3 个消息，安装 Python 脚本、launch、config。 |
| `msg/VLAAction.msg` | `/vla/action_raw`、`/vla/action_safe`：4D 位姿、`action_ori`、推理耗时、指令。 |
| `msg/VLAHeartbeat.msg` | `/vla/heartbeat`：是否存活、RTT、错误串。 |
| `msg/VLAStatus.msg` | `/vla/status`：状态机、图像/odom/推理是否正常。 |
| `config/vla_bridge.yaml` | 4090 地址、图像源、坐标系、控制限幅、看门狗超时。`enable_publish` 默认 false。 |
| `launch/vla_bridge.launch` | 启动四节点；`enable_publish` 默认 false；可覆盖 `server_host`。 |
| `launch/vla_bridge_mock.launch` | 本机联调：指向 `127.0.0.1:5007`，并灌假 Odin 图/里程计。 |
| `README.md` | 本包分步用法（Mock HTTP、限幅、25 Hz、watchdog、录包、转数据集）。 |

### 2.2 运行时节点

| 文件 | 用途 |
|---|---|
| `scripts/vla_image_node.py` | 订 Odin 压缩图与 odom；letterbox 224 RGB JPEG；按 instruction 锁第一帧原点；发 `/vla/image_for_infer`、`/vla/proprio`。 |
| `scripts/vla_infer_client_node.py` | 真实 HTTP `POST /predict`，字段只有 `image` / `proprio` / `instr`；单请求在途；发 `action_raw` 与 heartbeat。受 `/vla/allow_infer` 与图像过期约束。 |
| `scripts/vla_cmd_node.py` | NaN 丢弃；first_frame→ENU（只做一次 3.4.3）；锁高 1.2 m；`use_vla_yaw=false` 时 yaw 跟里程计；水平步长 ≤ 0.25 m；25 Hz 插值。默认只发 `/vla/debug/move_command`。 |
| `scripts/vla_watchdog_node.py` | 传感器/心跳/急停状态机；只降级：切断 `enable_control`，预览 hover/land。不解锁、不起飞、不进 Offboard。 |
| `scripts/vla_geometry.py` | 方案 3.4.2 / 3.4.3 公式：yaw、第一帧局部系、ENU、步长限幅、插值。无 ROS 依赖。 |
| `scripts/vla_image_proc.py` | JPEG 解码、letterbox/center-crop、编码。image 节点与转数据集共用，保证训推一致。 |
| `scripts/vla_backends.py` | `DirectFsmBackend` 组 `uav_command`（Move_mode=0，`yaw_ref` 为**度**）；`EgoPlannerBackend` 空壳。笔记本无 FSM 消息时回退 vendor。 |

### 2.3 本机 Mock / 采集 / 数据集

| 文件 | 用途 |
|---|---|
| `scripts/mock_inference_server.py` | 标准库 HTTP Mock `/predict`，固定短动作。不依赖 Flask。 |
| `scripts/publish_fake_observation.py` | 向 `/uav1/odin1/image/compressed` 与 odom 灌约 10 Hz 假数据。 |
| `launch/vla_record.launch` | 采集模式：强制不发飞控，latch 指令，按 4.3 列表录 bag。 |
| `scripts/vla_record_session.py` | 调 `rosbag record`，写同名 `.meta.json`；拒绝点云话题。 |
| `config/vla_record_topics.yaml` | 方案 4.3 录包话题清单。 |
| `config/vla_record_meta.template.json` | 场地、飞手、指令、success 等 meta 字段模板。 |
| `scripts/bag_to_uavflow.py` | bag → `datasets/odin_real/<id>/`（224 JPEG + `log.json`），对齐 `SimpleVLADataset`。不转 RLDS/HDF5。 |

### 2.4 笔记本兼容（Orin 完整编译后可不依赖）

| 文件 | 用途 |
|---|---|
| `scripts/vendor/uav_control_fsm/msg/_uav_command.py` | 官方 `uav_command` 的 genpy 副本（md5 与源码 .msg 一致）。本机只编了 `vla_bridge`、ROS 找不到 FSM 包时使用。Orin 上编过 `uav_control_fsm` 后走正式消息。 |

### 2.5 文档（本轮）

| 文件 | 用途 |
|---|---|
| `session_handoff.md` | 活的进度文档：施工勾选、验收数字、下一步。后续开发先读它。 |
| `code.md` | 本文件：新建/修改清单。 |

实施方案 [`docs/VLA_Drone_Deployment_Plan.md`](docs/VLA_Drone_Deployment_Plan.md) 已在仓库中，本轮**按它实现、未改接口约定**。

---

## 3. 修改文件

相对仓库原有文件（`main` / 既有包），只动了下面这些。

| 文件 | 改了什么 | 为什么 |
|---|---|---|
| `drone_ws/agile_ws/build.sh` | 末尾增加单独 `catkin_make` 编译 `vla_bridge`。 | 机载一键编译能带上新包，不塞进 FSM/规划器那几次 make。 |
| `drone_ws/agile_ws/src/vla_bridge/CMakeLists.txt` | 安装 `vla_record_session.py`、`bag_to_uavflow.py`。 | 录包与转数据集要从 devel 空间跑。 |
| `drone_ws/agile_ws/src/vla_bridge/package.xml` | 增加 `exec_depend`：`rosbag`。 | 采集 launch 依赖 `rosbag record`。 |
| `drone_ws/agile_ws/src/vla_bridge/config/vla_bridge.yaml` | 增加 `enable_auto_land`、hover/land 话题名。 | 看门狗超时策略可配置，与方案 6.3 对齐。 |
| `drone_ws/agile_ws/src/vla_bridge/launch/vla_record.launch` | 从占位改为强制 `enable_publish=false`、可选假 Odin、启动录包节点。 | 采集不能跑 VLA 控飞；笔记本可用假数据验收。 |
| `drone_ws/agile_ws/src/vla_bridge/scripts/vla_image_node.py` | 图像处理抽到 `vla_image_proc.py`；锁原点改为等 instruction / `reset_origin`。 | 与转数据集共用 letterbox；避免无任务时提前锁第一帧。 |
| `drone_ws/agile_ws/src/vla_bridge/scripts/vla_infer_client_node.py` | 订 `/vla/allow_infer`；按推理周期判断图像是否过期。 | 看门狗「禁止新推理」；2 Hz 图不能用 0.5 s 当过期（会误杀正常帧）。 |
| `drone_ws/agile_ws/src/vla_bridge/scripts/vla_cmd_node.py` | 从空壳改为 ENU 限幅 + 25 Hz 插值 + debug 预览。 | 实施方案第 4、5 步；默认仍不写 `/uav1/move_command`。 |
| `drone_ws/agile_ws/src/vla_bridge/scripts/vla_watchdog_node.py` | 从 stub INIT 改为完整 failsafe。 | 实施方案第 6 步；云断 FSM 感知不到 HTTP，必须在 Orin 本地降级。 |
| `drone_ws/agile_ws/src/vla_bridge/README.md` | 随 2–8 步补用法。 | 上机时按包内说明跑，不靠聊天记录。 |
| `session_handoff.md` | 每步更新勾选与验收。 | 约定的唯一进度文件。 |

**没有改：** `UAV-Flow-main`（含 `openvla_act.py`）、`uav_control_fsm` 源码、Ego-Planner、`odin_ros_driver`（子模块若显示 dirty，不是本轮改的）。

---

## 4. 四节点职责对照

| 节点 | 订 | 发（默认） | 不做什么 |
|---|---|---|---|
| `vla_image_node` | Odin compressed + odom、`/vla/instruction` | `/vla/image_for_infer`、`/vla/proprio`、原点 latch | 不上 HTTP、不发控制 |
| `vla_infer_client_node` | 处理后的图/proprio、指令、`allow_infer` | `/vla/action_raw`、`/vla/heartbeat`、`/vla/latency` | 不改坐标系、不发 FSM |
| `vla_cmd_node` | `action_raw`、原点、odom、`enable_control` | `/vla/action_safe`、`/vla/debug/move_command` | 默认不发 `/uav1/move_command` |
| `vla_watchdog_node` | heartbeat、图/odom、急停、NaN action | `/vla/status`、`enable_control=false`、`/vla/debug/hover\|land_command` | 不把 enable 置 true；不解锁起飞 |

---

## 5. 配置要点（`config/vla_bridge.yaml`）

| 项 | 当前值 | 含义 |
|---|---|---|
| `ground/host:port` | `10.62.25.215:5007` | 校园网 4090，`POST /predict` |
| `image/source` | `/uav1/odin1/image/compressed` | 主图；不要订默认关闭的 undistorted |
| `image/inference_rate_hz` | 2 | 上云帧率 |
| `control/enable_publish` | **false** | 总闸，上 Orin 后阶段 3–4 仍保持 false |
| `control/backend` | `direct_fsm` | 第一阶段不经规划器 |
| `control/lock_altitude` / `fixed_altitude` | true / 1.2 | 先只做水平面；真机按悬停高度改 |
| `control/use_vla_yaw` | false | `yaw_ref` 跟当前里程计（度），不要填 0 |
| `control/max_step_xy_m` | 0.25 | 单步水平限幅 |
| `frames/odom_to_enu` | 单位阵 | 阶段 4 标定后再填 |
| `watchdog/heartbeat_timeout_sec` | 2.0 | 心跳中断 → hover |
| `watchdog/fail_to_land_sec` | 8.0 | 失败持续 → land 预览 |

单位：proprio yaw **度**；模型 `action` yaw **弧度**；FSM `yaw_ref` **度**。转换处注释为 `# VLA rad -> FSM deg`。

---

## 6. 本机验收（不是真机）

| 步 | 结果 |
|---|---|
| 1 空包 | 四节点能 `roslaunch`，消息字段正确 |
| 2 Mock HTTP | 连续 200；断服务 heartbeat `alive=False` |
| 2b 校园网 4090 | `10.62.25.215:5007` 稳态 RTT 约 0.14 s；`action` 形状 `[1,4]`（`unnorm_key=sim` 在彩条图上会出数米级动作，只作链路测试） |
| 3 图像 | 假 Odin 10 Hz → `image_for_infer` 2 Hz、224、RGB 不发蓝 |
| 4 坐标 | `action_safe`：`enu`、z=1.2、水平步长 0.25 m |
| 5 控制预览 | `/vla/debug/move_command` 25 Hz，Move_mode=0，yaw 为度；无 `/uav1/move_command` |
| 6 看门狗 | TRACKING；estop 立即 HOVER；杀 Mock 约 1.3 s HOVER、8.3 s LAND |
| 7 录包 | 假 Odin bag：compressed/odom 各 337 条；无点云；无 move_command 消息 |
| 8 转数据集 | `datasets/odin_real/` 3 条 episode × 55 帧 224×224；第 0 帧 proprio `[0,0,0,0]` |

`datasets/odin_real/` 是 **Mock 产物**，不是训练用真数据，不要提交、不要改 `unnorm_key`。

---

## 7. 上 Orin 后预期改动（不是重写）

| 类型 | 内容 |
|---|---|
| 编译 | 在 Orin 完整编 `agile_ws`（含 `uav_control_fsm`），正式消息替换 vendor。 |
| 配置 | 核对 Odin 为 `/uav1/odin1/...`（`ns:=/uav1`）；阶段 4 再改 `odom_to_enu`、`vla_body_y_sign`、`fixed_altitude`。 |
| 网络 | Orin → `10.62.25.215:5007`；IP 变了只改 `ground/host`。 |
| 仍禁止 | 阶段 3–4 不要 `enable_publish:=true`；不要改 4090 的 `/predict` 字段名。 |

拷包目录：`drone_ws/agile_ws/src/vla_bridge/`（可连同改过的 `build.sh`）。不必拷 `datasets/`、不必拷 `UAV-Flow-main` 到 Orin。
