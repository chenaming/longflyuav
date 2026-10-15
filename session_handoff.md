# 项目交接与进度跟踪（session_handoff）

> **给后续 Cursor / 开发者：每次打开本仓库先读这份文件，再读实施方案，然后从「当前下一步」继续。**  
> 每次写完代码必须回头更新本文：进度勾选、本轮改动文件、验收结果、下一步。  
> 不要另起一份进度文档。

| 项 | 值 |
|---|---|
| 最后更新 | 2026-09-19 |
| 当前阶段 | 实施方案第 7 章 **第 8 步已完成**；下一步是 **第 9 步 EgoPlannerBackend（可选）** |
| 主方案（接口与架构以它为准） | [`docs/VLA_Drone_Deployment_Plan.md`](docs/VLA_Drone_Deployment_Plan.md) |
| 本机工作空间 | `/home/cym/vla_uav` |
| 机载 ROS 工作空间（本仓库副本） | `/home/cym/vla_uav/drone_ws/agile_ws` |
| Odin 驱动（本仓库副本） | `/home/cym/vla_uav/drone_ws/odin_ws` |
| VLA 代码 | `/home/cym/vla_uav/UAV-Flow-main` |

---

## 0. 新会话开场必做

1. 读本文，确认「当前下一步」和未完成勾选。
2. 读 [`docs/VLA_Drone_Deployment_Plan.md`](docs/VLA_Drone_Deployment_Plan.md) 对应章节，**不要凭记忆改话题名、字段名、坐标系**。
3. 只做当前下一步，做完验收，再改本文。
4. 未完成方案第 7 章 1–6 步，禁止真机带桨、禁止默认打开 `enable_publish`。

---

## 1. 项目目标（一句话）

4090 跑 UAV-Flow 的 OpenVLA-UAV（HTTP `/predict`）；Orin 新建 `vla_bridge`，把 Odin JPEG 降到 224×224 后上云，把 4D 动作转到 ENU，经限幅与 25 Hz 插值写入 `/uav1/move_command`（第一阶段**直连 FSM，不经 Ego-Planner**）。云断则本地 hover。

---

## 2. 施工清单（对照方案第 7 章）

| 步 | 内容 | 状态 | 完成日期 |
|---|---|---|---|
| 0 | 写实施方案 `docs/VLA_Drone_Deployment_Plan.md` | 已完成 | 2026-09-19 |
| **1** | 创建 `vla_bridge` 空包：msg / yaml / launch / 四节点壳，能 `roslaunch`，`rqt_graph` 能看见四个节点 | **已完成** | 2026-09-19 |
| **2** | `mock_inference_server.py` + `vla_infer_client_node` 真实 HTTP `/predict` | **已完成** | 2026-09-19 |
| **3** | `vla_image_node`：压缩图、224 JPEG、proprio | **已完成** | 2026-09-19 |
| **4** | 解析 `VLAAction` + 第一帧局部系→ENU，只发 `/vla/action_safe` | **已完成** | 2026-09-19 |
| **5** | `DirectFsmBackend`，25 Hz 刷新 stamp；默认 `enable_publish=false` | **已完成** | 2026-09-19 |
| 6 | `vla_watchdog_node`：超时 hover / land | **已完成** | 2026-09-19 |
| 7 | `vla_record.launch` + 录包清单（现仅有占位 launch） | **已完成** | 2026-09-19 |
| 8 | `bag_to_uavflow.py` → `SimpleVLADataset` | **已完成** | 2026-09-19 |
| 9 | `EgoPlannerBackend`（可选） | 未开始 | |

---

## 3. 当前下一步（从这里接着做）

**第 9 步：`EgoPlannerBackend`（可选，第一阶段可不做）。**

按方案 1.5.2 / 2.2。只留 `/uav1/goal_point` 插件口，**不要**启动 `exp_single_planner.launch`，也不要把 `enable_publish` 打开。本步仍可在笔记本写空壳，**不必先上 Orin**。

若先做真机传感器联调：那是方案第 5 章阶段 3（真实 Odin 图 + 里程计），**需要先把代码拷到 Orin NX**，开始前会再说一遍。

第 8 步复验（笔记本 Mock bag，不必上 Orin）：

```bash
python3 .../bag_to_uavflow.py --bags /tmp/vla_bags_step7/*.bag --out /home/cym/vla_uav/datasets/odin_real --chunk-sec 11
python3 .../bag_to_uavflow.py --check --out /home/cym/vla_uav/datasets/odin_real
# 3 条 episode，每条 55 帧 224×224，frame0 proprio = [0,0,0,0]
```

---

## 4. 第 1 步已落地的内容

### 4.1 新建包

路径：`drone_ws/agile_ws/src/vla_bridge/`

```text
vla_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── vla_bridge.yaml
│   ├── vla_record_topics.yaml
│   └── vla_record_meta.template.json
├── launch/
│   ├── vla_bridge.launch
│   ├── vla_bridge_mock.launch
│   └── vla_record.launch
├── msg/
│   ├── VLAAction.msg
│   ├── VLAHeartbeat.msg
│   └── VLAStatus.msg
└── scripts/
    ├── vla_image_node.py              # 已实现：224 letterbox + first-frame proprio
    ├── vla_infer_client_node.py       # 已实现 HTTP
    ├── vla_cmd_node.py                # 已实现：ENU 限幅 + 25 Hz 插值预览
    ├── vla_watchdog_node.py           # 已实现：超时 HOVER/LAND，只降级
    ├── vla_geometry.py                # 第一帧/ENU 公式（3.4.2 / 3.4.3）
    ├── vla_image_proc.py              # letterbox 224 JPEG
    ├── vla_backends.py                # DirectFsmBackend / EgoPlanner 空壳
    ├── vla_record_session.py          # 录 4.3 话题 + 写 meta.json
    ├── bag_to_uavflow.py              # bag → SimpleVLADataset
    ├── mock_inference_server.py       # 标准库 HTTP Mock
    └── publish_fake_observation.py    # 默认灌假 Odin 话题（10 Hz 彩条 + 运动 odom）
```

四个节点已挂进 `/vla` 命名空间，话题名与方案第 2.4 节一致。Watchdog 启动时 latch 发布 `/vla/enable_control = false`。

### 4.2 其它修改

| 文件 | 改动 |
|---|---|
| `drone_ws/agile_ws/build.sh` | 末尾增加单独编译 `vla_bridge` |
| `docs/VLA_Drone_Deployment_Plan.md` | 本轮之前已生成，本轮未改 |
| `session_handoff.md` | 本文件（新建） |

### 4.3 本机验收记录（2026-09-19）

在 `/home/cym/vla_uav/drone_ws/agile_ws` 执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make --source src/vla_bridge --build build/vla_bridge -j2
source devel/setup.bash
```

结果：

- `rospack find vla_bridge` → 源码目录
- `rosmsg show vla_bridge/VLAAction` / `VLAHeartbeat` / `VLAStatus` 字段正确
- `roslaunch vla_bridge vla_bridge.launch` 后 `rosnode list` 为：
  - `/vla/vla_watchdog_node`
  - `/vla/vla_image_node`
  - `/vla/vla_infer_client_node`
  - `/vla/vla_cmd_node`
- `rostopic hz /vla/status` ≈ **5.0 Hz**
- `/vla/heartbeat` 内容：`alive: False`，`last_error: "stub: HTTP client not implemented"`
- 话题列表已含 `/vla/image_for_infer`、`/vla/proprio`、`/vla/action_raw`、`/vla/action_safe` 以及订阅用的 `/uav1/odin1/...`（尚无真实 publisher）

编译坑：`package.xml` 必须把 `rospy` 写成 `build_depend`，否则 `catkin_package()` 报错。已修好。

注意：`build.sh` 里 `WS="$HOME/agile_ws"` 是机载账号路径。本开发机请用上面的绝对路径 `catkin_make`，不要直接跑 `build.sh` 除非 `$HOME/agile_ws` 已指向本仓库。

### 4.4 第 2 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/mock_inference_server.py` | 新建。标准库 `ThreadingHTTPServer`，`POST /predict`、`GET /health`、`POST /reset`。本机无 Flask/pip，故不用 Flask。 |
| `scripts/vla_infer_client_node.py` | 独立线程 `requests.post`；单 in-flight；`request_id` 递增；发布 `action_raw` / `heartbeat` / `latency`。 |
| `scripts/publish_fake_observation.py` | 新建。2 Hz 假 JPEG + proprio + instruction。 |
| `launch/vla_bridge.launch` | 增加 `server_host` / `server_port`，覆盖 `/ground/host`。 |
| `launch/vla_bridge_mock.launch` | 默认 `127.0.0.1:5007`，并启动假观测节点。 |
| `CMakeLists.txt` | 安装上述两个新脚本。 |
| `README.md` | 更新第 2 步用法。 |

本机验收：

- Mock 单独 `curl /health`、`curl /predict` 返回固定 `action=[[0.3,0,1.2,0]]`。
- `roslaunch vla_bridge vla_bridge_mock.launch` 后 `/vla/action_raw` **2.0 Hz**，`request_id` 连续递增（窗口内 34→56，服务端累计 72 次），`instruction=move forward`。
- 停 Mock 后 heartbeat：`alive: False`，`last_error` 含 `Connection refused`。
- `rostopic info /uav1/move_command`：**话题不存在**（未发飞控）。

JSON 额外带了 `request_id`（官方 `openvla_act.py` 只读 `image/proprio/instr`，多字段可忽略）。

### 4.5 第 3 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/vla_image_node.py` | 解码 JPEG→RGB，letterbox/center_crop 224，q80；图像/odom slop 0.15；第一帧局部系 proprio；`/vla/debug/image` 为 rgb8。 |
| `scripts/publish_fake_observation.py` | 默认 `source_mode=odin`：320×240 红/绿/蓝彩条 + 10 Hz odom（+x、偏航）。 |
| `launch/vla_bridge_mock.launch` | 假数据走 Odin 话题，不再绕过 image node。 |
| `package.xml` | 增加 `python3-opencv` / `python3-numpy`。 |
| `README.md` | 第 3 步用法。 |

行为要点：

- 原点只在收到 `/vla/instruction`（含第一条）或 `/vla/reset_origin` 后锁存；换指令会重锁。
- 超 `max_jpeg_bytes` 或 IMAGE_ODOM_DESYNC 本轮不发。
- `/vla/debug/image_odom_dt` 便于看同步差。
- **不做** HTTP、控制、急停。

本机验收（假 Odin，127.0.0.1 Mock）：

- `/uav1/odin1/image/compressed` **10.0 Hz** → `/vla/image_for_infer` **2.0 Hz**。
- infer JPEG **224×224**，3514 B，format=jpeg；左侧红、右侧绿；letterbox 上下黑边；`debug/image` encoding=`rgb8`。
- 锁原点：`p0=[1.100, 2.000, 1.200] yaw0_deg=4.00 frame=odom`（假数据 2 Hz 定时器首拍）。
- `reset_origin` 后下一帧 proprio=`[0,0,0,0]`，之后局部系位移/偏航连续变化。
- `/vla/debug/image_odom_dt` = 0.0；`/vla/action_raw` 仍 2 Hz（Mock 200）；`/uav1/move_command` 不存在。

### 4.6 第 5 章阶段 0–2 校园网复验（2026-09-19 晚）

本机（开发机，非 Orin）：`10.49.11.114`。4090：`10.62.25.215:5007`，`openvla_act.py`。

| 阶段 | 结果 |
|---|---|
| 0 网络 | ping 50/50，0% 丢失，RTT 0.085/0.205/0.360 ms；TCP 5007 通。iperf3 未测（本机无客户端 / 4090 未开 `-s`）。 |
| 1 HTTP 闭环 | 20/20 次 200；错端口能失败。因 5007 已被真实模型占用，本轮直接打 `openvla_act.py`，不再在 4090 上抢端口跑 Mock。 |
| 2 真实 `/predict` | 静态 224 JPEG + `proprio=[0,0,0,0]` + `move forward`；`action`/`action_ori` 均为 `[1,4]`。稳态推理+网络 ≈ **0.14 s**（计划仿真约 0.11 s）。ROS 侧 `/vla/action_raw` 2 Hz，`infer_ms` 130–150，heartbeat 正常。 |

注意：`unnorm_key=sim` 在彩条图上会吐出约 6–12 m 的 `action`，这是链路测试预期，**不能当真飞指令**。`enable_publish` 保持 false。

### 4.7 第 4 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/vla_geometry.py` | 新建。3.4.2 / 3.4.3 公式 + 步长限幅。**只做一次** first_frame→ENU。 |
| `scripts/vla_image_node.py` | latch 发布 `/vla/first_frame_origin`（PoseStamped）与 `/vla/origin_locked`。 |
| `scripts/vla_cmd_node.py` | 校验 NaN；first_frame→ENU；`lock_altitude`；`use_vla_yaw=false` 时 yaw 跟当前里程计；`max_step_*` 限幅；只发 `action_safe`。 |
| `README.md` | 第 4 步用法。 |

本机 Mock 验收：

- `/vla/action_safe` **2.0 Hz**，`frame_id=enu`，`yaw_unit=rad`，`z=1.2`。
- 相对当前 Odin odom 的 `step_xy=0.250`（Mock raw x=0.3 被限到 0.25），`step_yaw_deg=0`。
- `/uav1/move_command` 不存在，`enable_publish=false`。

### 4.8 第 5 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/vla_backends.py` | `DirectFsmBackend` / `EgoPlannerBackend` 空壳。 |
| `scripts/vla_cmd_node.py` | 25 Hz 直线插值 + 最短角 yaw；预览 `/vla/debug/move_command`；`yaw_ref` 为度。 |
| `scripts/vla_geometry.py` | `lerp_pose`。 |
| `scripts/vendor/uav_control_fsm/` | 笔记本未编译 FSM 时的 `uav_command` Python 消息（md5 与源码 .msg 一致）。 |
| `README.md` | 第 5 步用法。 |

闸门：`backend=direct_fsm` **且** `enable_publish=true` **且** `enable_control=true` 才写 `/uav1/move_command`。默认三项不齐，FSM 话题不存在。

本机 Mock 验收：

- 预览 **25.0 Hz**，stamp 间隔 0.040 s，`Move_mode=0`，`frame_id=uav1_map`，`z=1.2`。
- `yaw_ref` 为度（随假里程计约 10°/s），不是把弧度当度。
- `fsm_pub=False`，`/uav1/move_command` 不存在。
- 笔记本 `rostopic echo /vla/debug/move_command` 会报 message class 未编译；用带 vendor 的 Python 订户或 Orin 上编过的 `uav_control_fsm`。

### 4.9 第 6 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/vla_watchdog_node.py` | 状态机 WAIT_SENSORS→TRACKING→HOVER→LAND；切断 `enable_control`；预览 hover/land。 |
| `scripts/vla_infer_client_node.py` | 订 `/vla/allow_infer`；过期 `image_for_infer` 不再 POST。 |
| `scripts/vla_backends.py` | `hover()` / `land()`；仅 `enable_failsafe_pub` 时写 FSM。 |
| `config/vla_bridge.yaml` | `enable_auto_land`、hover/land 话题。 |
| `README.md` | 第 6 步用法。 |

闸门：默认 `enable_publish=false` 时只发 `/vla/debug/hover_command` 与 `/vla/debug/land_command`。看门狗**从不**把 `enable_control` 置 true，不解锁、不起飞、不发 `/uav1/offboard_command`。

本机 Mock 验收：

- 健康：`state=TRACKING`，`image_ok/odom_ok/infer_ok=True`，`publish_enabled=False`。
- `/vla/emergency_stop=true` → 立即 `HOVER`，`enable_control=false`。
- 杀掉 Mock：约 **1.30 s** `HOVER`（`fail_count=3`），约 **8.32 s** `LAND`。
- `/uav1/hover_command`、`/uav1/land_command`、`/uav1/move_command` 均不存在。

### 4.10 第 7 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `launch/vla_record.launch` | 强制 `enable_publish=false`；默认不启 bridge；可选假 Odin。 |
| `scripts/vla_record_session.py` | 按 4.3 列表 `rosbag record`，写同名 `.meta.json`。 |
| `config/vla_record_topics.yaml` | 录包话题清单；拒绝 cloud/points/undistorted/render。 |
| `config/vla_record_meta.template.json` | meta 字段模板。 |
| `README.md` | 第 7 步用法。 |

采集不跑 VLA 控制。`start_bridge` 默认 false。`enable_publish=true` 时录包节点直接拒绝启动。

本机假 Odin 验收：

- `/control/enable_publish=false`；`/uav1/move_command` **0 个 publisher**。
- bag 时长约 34 s：compressed **337**、odom **337**、instruction **2**（`move forward`）。
- 无点云话题；无 `move_command` 消息。
- `20260919_215940_forward.meta.json`：`site=lab`，`success=null`，`enable_publish=false`。

### 4.11 第 8 步已落地的内容（2026-09-19）

| 文件 | 改动 |
|---|---|
| `scripts/bag_to_uavflow.py` | bag→`datasets/odin_real/<id>/`；5 Hz；letterbox 224；3.4.2 `preprocessed_logs`。 |
| `scripts/vla_image_proc.py` | 从 image node 抽出 decode/letterbox/encode。 |
| `scripts/vla_geometry.py` | `rpy_from_quat`、`first_frame_log6`。 |
| `scripts/vla_image_node.py` | 改用 `vla_image_proc`。 |
| `README.md` | 第 8 步用法。 |

不写 RLDS/HDF5。`unnorm_key` 仍是 `sim`，不能拿这批 Mock 数据上真机微调。

本机验收（第 7 步 34 s Mock bag，`--chunk-sec 11`）：

- 3 条 episode，各 **55** 帧、**224×224** JPEG。
- `log.json`：`instruction=move forward`；`raw_logs`/`preprocessed_logs` 为 `[T,6]`；第 0 帧 proprio `[0,0,0,0]`。
- 水平位移约 **2.7 m** / 条；对齐 `SimpleVLADataset` 的 x,y,z,yaw 切片。

---

## 5. 硬约束（后续开发不要破坏）

来自实施方案，写代码时对照：

1. 4090 不进 ROS；Orin 做 Client，HTTP `POST /predict`，字段名只能是 `image` / `proprio` / `instr`。
2. 第一阶段控制后端是 `direct_fsm` → `/uav1/move_command`，`Move_mode=0`。不启动 Ego-Planner。
3. `control/enable_publish` 默认 **false**。
4. VLA 不得解锁、不得起飞、不得发 `/uav1/offboard_command`。
5. 图像主源：`/uav1/odin1/image/compressed`。Odin launch 必须 `ns:=/uav1`。不要订默认关闭的 undistorted。
6. `proprio`：第一帧局部系 `[x,y,z,yaw_deg]`；`action` yaw 是弧度；FSM `yaw_ref` 是**度**。
7. 真机不要复制 `batch_run_act_all.py` 对 `action` 的二次 `transform_to_global`。
8. 官方 `unnorm_key="sim"` 只用于仿真/链路测试，Odin 微调后改为 `odin_real`。
9. 新建话题在 `/vla/`，机载既有话题保持 `/uav1/`，方便 `rqt_graph` / `rostopic hz`。

---

## 6. 常用命令

```bash
source /opt/ros/noetic/setup.bash
source /home/cym/vla_uav/drone_ws/agile_ws/devel/setup.bash

# 编译本包
cd /home/cym/vla_uav/drone_ws/agile_ws
catkin_make --source src/vla_bridge --build build/vla_bridge -j2
source devel/setup.bash

# 校园网打真实 4090（enable_publish 仍为 false）
roslaunch vla_bridge vla_bridge_mock.launch server_host:=10.62.25.215
rostopic echo /vla/action_raw
rostopic echo /vla/latency

# 诊断
rqt_graph
rosnode list
rostopic hz /uav1/odin1/image/compressed
rostopic hz /vla/image_for_infer
rostopic echo /vla/proprio
rostopic hz /vla/action_raw
rostopic echo /vla/action_safe
rostopic echo /vla/heartbeat
# 25 Hz 预览（笔记本需 vendor 或已编译 uav_control_fsm）
# rostopic hz /vla/debug/move_command
rostopic echo /vla/status
rostopic echo /vla/enable_control
# 急停 / 超时预览（enable_publish=false 时没有 /uav1/hover_command）
# rostopic echo /vla/debug/hover_command
# rostopic echo /vla/debug/land_command

# 第 7 步录包（假 Odin，笔记本）
roslaunch vla_bridge vla_record.launch inject_fake_obs:=true instruction:="move forward" instr_id:=forward bag_dir:=/tmp/vla_bags

# 第 8 步：bag → 文件夹数据集
python3 /home/cym/vla_uav/drone_ws/agile_ws/src/vla_bridge/scripts/bag_to_uavflow.py \
  --bags /tmp/vla_bags_step7/*.bag --out /home/cym/vla_uav/datasets/odin_real --chunk-sec 11
```

---

## 7. 会话日志

每完成一轮开发，在本节**顶部**追加一条（新的在上）。

### 2026-09-19 — 第 8 步：bag_to_uavflow.py

- Mock bag 抽 5 Hz、letterbox 224，写成 `datasets/odin_real/` 三折 episode。
- 验收：3×55 帧，frame0 proprio 为零，图像 224×224，字段对齐 `SimpleVLADataset`。
- 下一步：第 9 步 EgoPlanner 可选；真机 Odin 联调需先上 Orin，开始前会说明。

### 2026-09-19 — 第 7 步：vla_record.launch

- 采集 launch 强制不发飞控；话题清单与方案 4.3 一致，拒绝点云。
- 验收：假 Odin bag compressed/odom 各 337 条，instruction=`move forward`，无 move_command 消息。
- 下一步：第 8 步 `bag_to_uavflow.py`。可用本机 Mock bag 做格式验证，仍不必上 Orin。

### 2026-09-19 — 第 6 步：watchdog hover/land

- 看门狗只降级：超时/急停 hover，持续失败 land，切断 `enable_control`。
- 默认只预览 `/vla/debug/hover_command` 与 `/vla/debug/land_command`，不写 FSM。
- 验收：TRACKING；estop 立即 HOVER；杀 Mock 1.30 s HOVER、8.32 s LAND；无 `/uav1/hover_command`。
- 下一步：第 7 步 `vla_record.launch`。写 launch 仍不必上 Orin。

### 2026-09-19 — 第 5 步：DirectFsmBackend 25 Hz

- 安全目标在推理间隔内直线插值，25 Hz 刷新 stamp；`yaw_ref` 为度。
- 默认只发 `/vla/debug/move_command`，不写 `/uav1/move_command`。
- 验收：25.0 Hz、Move_mode=0、z=1.2、yaw 为度、无 FSM 话题。
- 下一步：第 6 步 watchdog。仍不必先上 Orin。

### 2026-09-19 — 第 4 步：action_safe / first_frame→ENU

- image node 锁存原点并 latch 发布；cmd node 只做方案 3.4.3 一次变换 + 步长限幅。
- 验收：`action_safe` 2 Hz、`enu`、z=1.2、相对当前位姿水平步长正好 0.25 m、不跟 VLA yaw、无 `move_command`。
- 下一步：第 5 步 DirectFsmBackend 25 Hz stamp，默认仍不发飞控。

### 2026-09-19 — 第 5 章阶段 0–2 校园网复验

- 本机 `10.49.11.114` → 4090 `10.62.25.215:5007`（`openvla_act.py` 已在跑，未改服务端）。
- 阶段 0：ping 50/50、丢包 0%、RTT min/avg/max = 0.085/0.205/0.360 ms；`nc -vz` 5007 通。本机无 iperf3，带宽未测。
- 阶段 1/2 HTTP：连续 20/20 次 `POST /predict` 200；`action`/`action_ori` 形状 `[1,4]`；稳态 RTT min/avg/max = 0.134/0.142/0.166 s（首包 0.407 s）。错误端口 5999 能报错。
- 阶段 2 ROS：`vla_bridge_mock.launch server_host:=10.62.25.215`，`/ground/host` 确认 4090；`/vla/action_raw` ≈ 2.0 Hz，`infer_ms` ≈ 130–150 ms，heartbeat `alive=True`；动作是模型输出（约 11 m 级，不是 Mock 的 `[0.3,0,1.2,0]`）；`enable_publish=false`，无 `/uav1/move_command`。
- 未开飞控。下一步仍是第 4 步 `action_safe`。

### 2026-09-19 — 第 3 步：vla_image_node

- 实现压缩图→224 letterbox JPEG + 第一帧局部系 proprio；假观测改为灌 Odin 话题。
- 原点按 instruction / `reset_origin` 锁存，不在无任务时提前锁。
- 验收通过：10 Hz→2 Hz、224 RGB 不发蓝、reset 后 proprio 归零、Mock HTTP 仍通、无 `move_command`。
- 下一步：第 4 步 `action_safe`（first_frame→ENU，仍不发飞控）。

### 2026-09-19 — 第 2 步：Mock HTTP

- 实现 Mock `/predict`（标准库，不用 Flask）和 `vla_infer_client_node` 真实 HTTP。
- `vla_bridge_mock.launch` 指向 127.0.0.1，并灌假观测。
- 验收通过：连续 >20 次 200、`request_id` 递增、断服务 heartbeat 失败、无 `move_command`。
- 下一步：第 3 步 `vla_image_node`（224 letterbox + first-frame proprio）。

### 2026-09-19 — 第 1 步：vla_bridge 空包

- 新增 `drone_ws/agile_ws/src/vla_bridge/` 全套骨架（见 4.1）。
- 修改 `drone_ws/agile_ws/build.sh`，加入 vla_bridge 编译。
- 本机 `catkin_make` 成功；四节点可启动；`/vla/status` 5 Hz。
- 节点均为 stub：不请求模型，不处理图像，不发飞控。
- 下一步：第 2 步 Mock HTTP + infer client。

### 2026-09-19 — 方案文档

- 新增 `docs/VLA_Drone_Deployment_Plan.md`（架构、接口、坐标系、阶段、安全）。
- 未写业务节点代码。
