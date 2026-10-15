# 实机部署 SOP：Odin + VLA 云边协同

> **文档定位**：飞手 / 地面操作员在 **Odin1 + Orin NX + PX4/MAVROS + `vla_bridge` + 4090 OpenVLA-UAV** 链路上，做环境检查、分阶段复验、微调采数、以及系留短飞的标准作业程序。  
> **接口以源码为准**：[`docs/VLA_Drone_Deployment_Plan.md`](docs/VLA_Drone_Deployment_Plan.md)、[`docs/项目开发手册.md`](docs/项目开发手册.md)、[`code.md`](code.md)、[`session_handoff.md`](session_handoff.md)。  
> **版本**：v1.0｜2026-09-20｜ROS 1 Noetic（机载）。4090 **不进 ROS**。

---

## 0. 开场必读（硬禁令）

本栈第一阶段是 **VLA 直连 FSM**（`/uav1/move_command`，`Move_mode=0`），**不启动 Ego-Planner**。VLA **不得**发布 `/uav1/offboard_command`，**不得**自己解锁、起飞。

| 禁令 | 说明 |
|---|---|
| 未过「阶段 3–4」 | 禁止 `enable_publish:=true` |
| 未过「阶段 5–6」 | 禁止带桨、禁止 Offboard 闭环 |
| 未过「阶段 6」 | 禁止系留/短飞（阶段 7） |
| 采数时 | **禁止** `enable_publish:=true`；飞手只看 FPV 手飞或已验证的 FSM 示范 |
| `unnorm_key` | 微调出 `odin_real` 之前必须保持 `"sim"`，禁止把仿真尺度权重当真飞 |

**角色**

| 角色 | 职责 |
|---|---|
| 飞手 | 遥控器在手；Offboard 拨杆、姿态/油门接管、降落 |
| 地面操作员 | SSH / 笔记本 ROS；启动节点、发 instruction、看 topic、录 bag |
| 观察员 | 围栏外盯桨叶、人、线；喊停 |

**单位（写错会飞错方向或把弧度当度）**

| 量 | VLA `proprio` | VLA `action` | FSM `yaw_ref` | MAVROS 本地位置 |
|---|---|---|---|---|
| 位置 | m（第一帧局部系） | m | m（ENU） | m（ENU） |
| 偏航 | **度** | **弧度** | **度** | 四元数 |

---

# 1. 环境准备与配置检查（Pre-check）

## 1.1 硬件、遥控与网络拓扑

```text
飞手 RC  ──PWM──► PX4
                   ▲
                   │ MAVLink /dev/ttyACM0:921600
                   │
Odin1 USB ──► Orin NX（ROS Noetic）──HTTP POST /predict──► 4090 :5007
                   │                         （校园网 / AX3 AP）
                   └── 笔记本（SSH / 可选 ROS_MASTER 远程看图）
```

当前已知地址（**每次上机先 ping，DHCP 会变**）：

| 设备 | 典型值 | 现场填空 |
|---|---|---|
| 4090 工作站 | `10.62.25.215`，推理口 **5007** | ________ |
| Orin 无线 | 曾见 `10.49.11.126`（用户 `nvidia`）或有线 `192.168.1.101`（用户 `agile`） | ________ |
| 机载家目录 | `/home/agile/agile_ws`、`/home/agile/odin_ws` 或本仓库 `drone_ws/` | ________ |
| 笔记本 | 曾见 `10.49.11.114` | ________ |

遥控（本仓库 **真机默认**，见 `exp_single_uav_control_fsm.launch`）：

| 功能 | 通道 | 归一化 | 行为 |
|---|---|---|---|
| Offboard 三档 | **CH6**（1-based） | 高档 `norm > 0.3`（约 PWM>1650） | 请求进 Offboard；地面未解锁时 FSM 会 **AUTO_TAKEOFF** |
| | | 中档 `-0.3 … 0.3` | **退出 Offboard → POSCTL**（本栈规定的最高优先级接管） |
| | | 低档 `norm < -0.3` | 保持当前预设（不要误当「关 Offboard」） |
| 杀桨 / 急停 | PX4 `RC_MAP_KILL_SW` | **1 号机备份为通道 10** | 上机用 QGC 再确认；`docs/1号机.params`：`RC_MAP_KILL_SW=10`，`RC_MAP_ARM_SW=9`，`RC_MAP_FLTMODE=7`。`RC_MAP_OFFB_SW=0`（Offboard **不走 PX4 拨杆**，走 FSM CH6） |

- [ ] 飞手能口述：CH6 高/中/低分别做什么；能摸到 **CH10 杀桨**（以现场 QGC 为准）。
- [ ] 起飞前 CH6 **必须在低档或中档**，禁止一上电就在高档。
- [ ] 桨叶：阶段 5 及以前 **拆桨**；阶段 7 才允许带桨且有观察员。
- [ ] USB：Odin 与飞控串口（默认 `/dev/ttyACM0`）插牢；Orin 上 `ls /dev/ttyACM*`。
- [ ] SSH 无桌面时 **不要**跑 `odin.sh` / `exp_single_mavros_control.sh`（二者检查 `DISPLAY` 并用 `gnome-terminal`）；改用下面的 `roslaunch`。

## 1.2 相机固定参数（避免微调数据闪烁）

Odin 不是普通 USB v4l2 工业相机。曝光/白平衡走 **SDK 配置 + ROS 服务**（包内有 `GetAe.srv` / `SetAe.srv` / `GetAwb.srv` / `SetAwb.srv`），主配置：

```text
<odin_ws>/src/odin_ros_driver/config/control_command.yaml
```

第一阶段要求（与实施方案 0.2 节一致）：

| 参数 | 建议 | 原因 |
|---|---|---|
| `sendrgbcompressed` | `1` | VLA 主源是 JPEG |
| `sendrgbundistort` | `0` | undistorted **默认不发**，不要订 |
| `dtof_fps` | `100`（约 10 Hz 图） | 推理只要 2 Hz；不要为了 VLA 把相机拉满 |

上电后固定 AE/AWB（服务名以现场 `rosservice list` 为准）：

```bash
source /opt/ros/noetic/setup.bash
source $HOME/odin_ws/devel/setup.bash   # 或本仓库 drone_ws/odin_ws

rosservice list | grep -iE 'ae|awb|Ae|Awb'
# 记下服务全名后：
rosservice type <SetAe服务名>
# 将自动曝光关闭，写入现场测得的曝光；白平衡同理关 AWB、写固定增益
# 示例（字段名以 rossrv show 为准，不要盲抄）：
# rosservice call /<ns>/set_ae "enable_auto: false
#   exposure: <现场值>"
```

- [ ] `rqt_image_view` 看 `/uav1/odin1/image/compressed`：走一圈灯区，亮度无周期性闪。
- [ ] 对焦：Odin 为模组焦段，**不要在轨迹中途改安装角/焦距**；安装俯仰角改完必须重做阶段 4。
- [ ] 不要用 `uav_rtsp_stream` 工业相机话题冒充 Odin（那是另一路 `/dev/video*`）。

## 1.3 代码与参数修改索引

拷到 Orin 的源码目录：`drone_ws/agile_ws/src/vla_bridge/`（可连同 `drone_ws/agile_ws/build.sh`）。**不必**拷 `UAV-Flow-main`、**不必**拷 `datasets/`。

| 目的 | 文件 | 关键项 |
|---|---|---|
| 4090 地址 | `drone_ws/agile_ws/src/vla_bridge/config/vla_bridge.yaml` → `ground/host` `ground/port` | 默认 `10.62.25.215` / `5007` |
| 或启动覆盖 | `roslaunch vla_bridge vla_bridge.launch server_host:=... server_port:=5007` | 不要改 `/predict` 字段名 |
| 飞控发布总闸 | 同上 `control/enable_publish` 或 launch `enable_publish:=false` | 默认 **false** |
| 图像/里程计话题 | `image/source` `image/odometry` | `/uav1/odin1/image/compressed`、`/uav1/odin1/odometry` |
| 上云帧率/尺寸 | `inference_rate_hz` `resize` `resize_mode` | `2.0` / `224` / `letterbox` |
| 坐标系 | `frames/odom_to_enu` `vla_body_y_sign` | 阶段 4 标定前保持单位阵 / `1` |
| 锁高 | `lock_altitude` `fixed_altitude` | 按悬停高度改，默认 1.2 m |
| 单步限幅 | `max_step_xy_m` `max_yaw_step_deg` | 默认 0.25 m / 8° |
| 看门狗 | `watchdog/*` | 心跳 2 s hover；8 s land |
| Odin 开关 | `odin_ros_driver/config/control_command.yaml` | compressed=1，undistort=0 |
| Odin 命名空间 | `roslaunch odin_ros_driver odin1_ros1.launch ns:=/uav1` | 若现场 launch **已经**带 `/uav1`，不要叠一层变成 `/uav1/uav1/...` |
| FSM 里程计 remap | `exp_single_uav_control_fsm.launch` | 默认 `odom_remap_to_topic:=odin1/odometry` → `/uav1/odin1/odometry` |
| 定位源名 | `drone_ws/agile_ws/src/uav_control_fsm/config/uav_group.yaml` | **当前仓库默认 `default_location_source: Fast_Lio`**，靠 `odom_remap_to_topic:=odin1/odometry` 吃 Odin。手册希望改成 `Odin`，但 yaml 里 **还没有** `location_profiles.Odin`；直接改名会 `profile not exist`。实飞可保持 Fast_Lio+remap，或补上与 Fast_Lio 相同的 EKF 项后再改名。解锁前也可：`rostopic pub -1 /uav1/set_location_source std_msgs/String "data: 'Fast_Lio'"` |
| 起飞高度 vs 锁高 | `exp_single_uav_control_fsm.launch` `takeoff_height`（默认 **1.0**）vs `vla_bridge.yaml` `fixed_altitude`（默认 **1.2**） | **飞前改成同一数字**，否则 VLA 锁高与 FSM 起飞高度不一致 |
| 飞控串口 | `uav_control_fsm/launch/px4.launch` `fcu_url` | `/dev/ttyACM0:921600` |
| 4090 推理 | `UAV-Flow-main/OpenVLA-UAV/vla-scripts/openvla_act.py` | `http_port: 5007`，`unnorm_key: "sim"`，`model_path` 填权重 |

机载编译（Orin，路径按账号改）：

```bash
sudo apt install python3-requests   # vla_infer_client_node
source /opt/ros/noetic/setup.bash
cd $HOME/agile_ws    # 或 /home/cym/vla_uav/drone_ws/agile_ws
# 建议整仓编一次，以便使用正式 uav_control_fsm/uav_command（不再靠 vendor）
./build.sh
# 或只编桥：
catkin_make --source src/vla_bridge --build build/vla_bridge -j2
source devel/setup.bash
```

每个新终端：

```bash
source /opt/ros/noetic/setup.bash
source $HOME/agile_ws/devel/setup.bash
source $HOME/odin_ws/devel/setup.bash
```

---

# 2. 分阶段功能复验指南

原则：**先通信、后传感器、后推理、后坐标、后控制、后起飞**。未通过不得进入下一阶段。  
笔记本可用 `vla_bridge_mock.launch` 做阶段 1–2；**阶段 3 起必须 Orin + 真 Odin**。

地面操作员在笔记本上看机载 ROS 时（可选）：

```bash
export ROS_MASTER_URI=http://<ORIN_IP>:11311
export ROS_IP=<笔记本IP>
export ROS_HOSTNAME=<笔记本IP>
```

**不是 DDS**（本栈 ROS 1）。4090 **不要**设 `ROS_MASTER_URI`。

---

## 2.1 网络与服务器连接

**[测试目的]** 确认 Orin ↔ 4090 的 IP/TCP 能支撑 HTTP `/predict`；确认 SSH 能上机。

**[运行指令]** 在 **Orin** 上：

```bash
hostname -I
ping -c 50 10.62.25.215
nc -vz 10.62.25.215 5007
# 可选带宽（4090 先 iperf3 -s）
# iperf3 -c 10.62.25.215 -t 30
```

笔记本 SSH（用户/IP 以现场为准）：

```bash
ssh agile@<ORIN_IP>     # 部署文档常用
# 或 ssh nvidia@<ORIN_IP>
```

**[预期结果 / 检查项]**

- [ ] ping 50 包丢包 **< 2%**，RTT 记入下表：min/avg/max = ______ / ______ / ______ ms
- [ ] `nc` 显示 5007 **succeeded** / open
- [ ] SSH 可登录；`whoami`、`ls ~/agile_ws ~/odin_ws` 正常
- [ ] 跨网段（`10.49`↔`10.62`）不通时，先查校园网认证/ACL，不要改 VLA 代码

**[故障排查]**

| 现象 | 排查 |
|---|---|
| ping 不通 | 双方 `hostname -I`；AX3 是否 AP/Bridge；校园网是否需 Web 认证 |
| ping 通但 5007 拒绝 | 4090 防火墙；`openvla_act.py` 是否 `0.0.0.0:5007`；是否占口 |
| SSH 超时 | 先网线直连（文档示例 Orin `192.168.1.108`）再配 Wi-Fi |
| 延迟 > 50 ms 且抖动大 | 开发期可继续联调；**正式带飞前**要重测，考虑独立网桥 |

---

## 2.2 Odin 相机与里程计订阅

**[测试目的]** 话题在 `/uav1/odin1/...`，压缩图约 10 Hz，里程计有效，颜色/方向可看。

**[运行指令]**

```bash
# 终端 A：Master
roscore    # 若后续 roslaunch 会自拉 master，可省略，但多机时建议显式

# 终端 B：Odin
# Orin 机载 odin1_ros1.launch 已含 /uav1 时（部署文档现状）不要再叠 ns：
roslaunch odin_ros_driver odin1_ros1.launch
# 本仓库方案文档写的是「launch 默认 ns 为空」，那种机器必须：
# roslaunch odin_ros_driver odin1_ros1.launch ns:=/uav1
# 验收只看话题，不要同时试两种写法。

rostopic list | grep odin1
rostopic hz /uav1/odin1/image/compressed
rostopic hz /uav1/odin1/odometry
rostopic echo -n 1 /uav1/odin1/odometry
rqt_image_view    # 选 compressed
rosnode info /host_sdk_sample
```

**[预期结果 / 检查项]**

- [ ] 只出现 `/uav1/odin1/...`，**没有**裸 `/odin1/...`（除非你故意未加 ns）
- [ ] compressed **约 8–12 Hz**（`dtof_fps=100`）
- [ ] odometry `frame_id` 多为 `odom`，`child_frame_id` 多为 `odin1_base_link`
- [ ] 画面不是整体发蓝（那是 BGR/RGB 搞反，发生在 **VLA 预处理** 而不是驱动 JPEG）
- [ ] **不要**等待 `/odin1/image/undistorted`（默认不发）
- [ ] 点云/深度可存在，但 VLA 在线链路 **不订点云**

**[故障排查]**

| 现象 | 修改提示 |
|---|---|
| 没有 `/uav1/odin1/image/compressed` | `control_command.yaml` 的 `sendrgbcompressed`；USB；`ns` |
| 只有 `/odin1/...` | 加 `ns:=/uav1` 或改 launch；FSM remap 对不上 |
| odom 有、图无 | 压缩开关；带宽；驱动日志 |
| `host_sdk_sample` 起不来 | `ldd` SDK 库；插拔 Odin；见 Orin 部署文档 7.x |

深度图（可选，**不是** OpenVLA-UAV 微调必选项）：

```bash
rostopic list | grep -iE 'depth|cloud'
# 若存在独立 depth image 可记录名称；禁止 rosbag 录 *cloud* / *points*
```

---

## 2.3 VLA 推理服务复验（HTTP，无 ROS）

**[测试目的]** 4090 `POST /predict` 能吃 JPEG + proprio + 英文指令，返回 `[1,4]`。

**[运行指令]** 在 **4090**：

```bash
cd <UAV-Flow-main>/OpenVLA-UAV
# 确认 openvla_act.py 中 http_port=5007、unnorm_key="sim"、model_path 非空
python vla-scripts/openvla_act.py
```

任意一台能访问 4090 的机器：

```bash
python3 - <<'PY'
import base64, json, urllib.request
from PIL import Image
from io import BytesIO
import numpy as np
img = Image.new("RGB", (224, 224), (255, 0, 0))
buf = BytesIO(); img.save(buf, format="JPEG")
payload = {
    "image": base64.b64encode(buf.getvalue()).decode("ascii"),
    "proprio": [0.0, 0.0, 0.0, 0.0],   # x,y,z,yaw_deg
    "instr": "move forward",
}
req = urllib.request.Request(
    "http://10.62.25.215:5007/predict",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json"},
)
print(urllib.request.urlopen(req, timeout=10).read().decode())
PY
```

ROS 侧（Orin，`enable_publish:=false`）：

```bash
roslaunch vla_bridge vla_bridge.launch enable_publish:=false server_host:=10.62.25.215
# 需已有 Odin 图，或本机调试用：
# roslaunch vla_bridge vla_bridge_mock.launch server_host:=10.62.25.215
rostopic echo /vla/action_raw
rostopic echo /vla/heartbeat
rostopic hz /vla/action_raw
rostopic echo /vla/latency
```

**[预期结果 / 检查项]**

- [ ] JSON `status=success`；`action` 与 `action_ori` 均为长度 4（外层常为 `[[x,y,z,yaw]]`）
- [ ] `action`：服务端已把 `action_ori` 加到当前 proprio（第一帧局部系绝对位姿）；yaw 为 **弧度**
- [ ] `action_ori`：机体系相对 Δ；**真机桥不要再做第二次 `transform_to_global`**
- [ ] `heartbeat.alive=true`；稳态 `infer_ms` 约 0.1–0.3 s（彩条/非域图像可能给出数米级动作，**只证明链路**）
- [ ] `/vla/action_raw` 约 **2 Hz**

**[故障排查]**

| 现象 | 修改提示 |
|---|---|
| HTTP 字段错误 | 只能是 `image` / `proprio` / `instr`，不要改成 `instruction` |
| 5xx / CUDA | 4090 `model_path`、显存、`flash_attention_2` |
| `alive=false` `waiting for image` | 先过 2.2；image 节点要有 instruction 才锁原点并稳定发 proprio |
| 动作动辄 10 m | `unnorm_key=sim` + OOD 图的预期；**禁止当真飞** |
| 本机 `rostopic echo` 找不到 `uav_command` | 笔记本未编 FSM；用 debug 预览脚本或到 Orin 上 echo |

---

## 2.4 坐标系转换验证

**[测试目的]** 确认「模型第一帧局部系」→ **ENU（`uav1_map` / MAVROS local）**，而不是把相机光学系或 NED 直接当指令。本栈控制指令按 **ENU**，不是 PX4 内部 NED。

关系（不要焊死两棵 TF 树）：

```text
Odin:  odom → odin1_base_link
FSM:   world → uav1_map → uav1/base_link
odin_link TF 默认关闭。阶段 4 用 yaml 的 odom_to_enu / vla_body_y_sign 对齐。
```

`proprio = [x_l, y_l, z_l, yaw_deg]`：相对**本条 instruction 锁存的第一帧**。  
`action_safe`：`frame_id=enu`，z 在 `lock_altitude=true` 时为 `fixed_altitude`。

**[运行指令]** 拆桨，人抱机或放桌，`enable_publish:=false`：

```bash
roslaunch vla_bridge vla_bridge.launch enable_publish:=false
rostopic pub -1 /vla/instruction std_msgs/String "data: 'move forward'"
rostopic echo /vla/proprio
rostopic echo /vla/first_frame_origin
rostopic echo /vla/action_raw      # first_frame，yaw 弧度
rostopic echo /vla/action_safe     # enu，z≈1.2，水平步长 ≤ 0.25
rosrun tf tf_echo odom odin1_base_link
rosrun rqt_tf_tree rqt_tf_tree
# 时间戳对齐：应接近 0，多数 < 0.15 s
rostopic echo /vla/debug/image_odom_dt
```

地面方向实验（记符号，写入 yaml）：

| 操作 | 期望 `action_safe` |
|---|---|
| 机头朝东 + “move forward” | ENU **+X** 增加（步长被限到 ≤0.25 m） |
| 机头朝北 + 同一指令 | ENU **+Y** 增加 |
| 锁高开启 | **z 不变**（默认 1.2） |
| 原地偏航飞机 | `use_vla_yaw=false` 时 `debug/move_command.yaw_ref` 跟里程计 **度**，不是 0，也不是 1.57 这种弧度误用 |

可视化：

```bash
rviz   # 加 Odometry: /uav1/odin1/odometry 与 /uav1/mavros/local_position/odom
# PlotJuggler：拖入 /vla/action_safe、两路 odom 的 pose.pose.position.{x,y,z}
# 预览指令（笔记本需已编 uav_control_fsm；否则只看 action_safe）：
rostopic echo /vla/debug/move_command
# 两路水平差：
rostopic echo /vla/status    # odom disagree 时 detail 会写；正常控飞为 TRACKING
```

**[预期结果 / 检查项]**

- [ ] 三次朝向实验方向正确，记录 `vla_body_y_sign`、`odom_to_enu`
- [ ] `action_safe` 相对当前 odom 的 `step_xy ≤ 0.25`
- [ ] 两路 odom 差持续 **> 0.3 m** 时看门狗禁止发布（`frames/odom_disagree_m`）
- [ ] **没有** `/uav1/move_command` 的 publisher（此阶段）

**[故障排查]**

| 现象 | 修改 |
|---|---|
| 前向变成侧向 | `vla_body_y_sign: -1` 或改 `odom_to_enu/R`（阶段 4 产物，不要猜） |
| yaw_ref≈0.05 却像弧度 | 检查 cmd 节点是否 `math.degrees`；日志应有 `# VLA rad -> FSM deg` |
| proprio 不归零 | 未发 instruction / 未 `reset_origin` |
| 抱机平移 proprio 不动 | 订错 odom；或图像–odom 时间 `sync_slop_sec` 对不齐 |

---

## 2.5 飞行指令与 PX4 / Offboard（切入前）

**[测试目的]** MAVROS 连通；在 **CH6 低/中档、未解锁** 时，VLA 只预览或最多把 `move_command` 打到 FSM，飞机不应转桨。

**[运行指令]**

```bash
roslaunch uav_control_fsm px4.launch
rostopic echo /uav1/mavros/state
# connected: True；mode 多为 MANUAL/POSCTL；armed: False
rostopic echo /uav1/mavros/rc/in
# CH6 = channels[5]（0-based）。高档约 >1650，中档约 1500，低档约 <1350（以现场校准为准）
rostopic echo /uav1/control_fsm/state

roslaunch uav_control_fsm exp_single_uav_control_fsm.launch
rosnode info /uav1/uav_control_fsm   # 应订 /uav1/odin1/odometry
rostopic hz /uav1/mavros/vision_pose/pose
rostopic hz /uav1/mavros/local_position/odom
```

阶段 5（仍拆桨）：允许 `enable_publish:=true`，但 **不拨 CH6 高档**：

```bash
roslaunch vla_bridge vla_bridge.launch enable_publish:=true server_host:=10.62.25.215
rostopic pub -1 /vla/enable_control std_msgs/Bool "data: true"   # 人工开闸；看门狗不会自己置 true
rostopic hz /uav1/move_command          # 应约 25 Hz
rostopic echo /uav1/move_command        # Move_mode=0，yaw_ref 为度，stamp 刷新
rostopic echo /vla/status               # TRACKING / HOVER / LAND / WAIT_SENSORS / FAILSAFE
rostopic echo /vla/heartbeat
```

确认 VLA **没有**进 Offboard：

```bash
rostopic echo /uav1/mavros/state        # 仍不应被 VLA 改成 OFFBOARD
rostopic info /uav1/offboard_command    # vla_bridge 不得出现在 Publishers
```

看门狗：

```bash
# 停 4090 或拔网
rostopic echo /vla/enable_control       # 应变 false
rostopic echo /vla/debug/hover_command  # 预览 true
# enable_publish=true 时才会出现 /uav1/hover_command
```

**[预期结果 / 检查项]**

- [ ] `mavros/state.connected=true`，`armed=false`
- [ ] `Move_mode=0`，`header.frame_id=uav1_map`，`yaw_ref` 量级像角度
- [ ] `rqt_graph`：仅 `vla_cmd_node`（在 `enable_publish` 且 `enable_control` 时）写 `move_command`
- [ ] 杀推理后不再写新目标；`enable_control=false`
- [ ] CH6 仍非高档 → PX4 **未**进 OFFBOARD

阶段 6（拆桨 + 允许 Offboard 接口）：飞手 CH6 **高档**，看 `/uav1/mavros/setpoint_raw/local` 约 30 Hz；**中档必须能退** Offboard。断网应有 hover。

**[故障排查]**

| 现象 | 排查 |
|---|---|
| `connected=false` | `fcu_url`、`ttyACM` 权限、线 |
| `odom_valid=false` | remap、`default_location_source`、Odin 是否在发 |
| 有 preview 无 `move_command` | 缺 `enable_publish` 或 `enable_control` 或 `backend≠direct_fsm` |
| 一启动就 Offboard | CH6 在高档；或有人发了 `offboard_command`（禁止 VLA 发） |
| stamp 过期被丢 | FSM `max_age_sec=0.5`；必须 20–30 Hz 刷新（cmd 节点 25 Hz） |

---

# 3. ROSBag 数据采集 SOP（VLA 微调）

采集时 **不跑 VLA 控制**。飞手只看 FPV/Odin，按英文指令短飞。`vla_record.launch` 默认 `start_bridge:=false`、`enable_publish=false`。

## 3.1 采前准备

**任务指令（主字段必须英文，贴近 UAV-Flow Fixed Command Set）**

| instr_id | instruction（写入 bag / log.json） | instruction_zh（仅 meta） |
|---|---|---|
| forward | `move forward` | 向前平移 |
| left | `translate left` / `pass left` | 左移或左绕 |
| right | `translate right` / `pass right` | 右移或右绕 |
| approach | `approach the red cabinet` | 靠近指定物 |
| hover_beside | `hover beside the red cabinet` | 在物体旁悬停 |
| rotate | `rotate clockwise` | 原地旋转（慎用，第一阶段在线 `use_vla_yaw=false`） |

- [ ] 同一英文指令至少 **3 个不同起点** 各 1 条。
- [ ] 单条 **< 8 m、< 20 s**，结束悬停 1 s 再停 bag。
- [ ] 场地 / 光照写进 meta：`site` `lighting` `pilot`。
- [ ] AE/AWB 已固定（§1.2）；不要中途开关灯造成闪曝光。
- [ ] 围栏、观察员、CH6 不在高档（除非这条是「FSM 示范」且明确记录）。

场景位姿：在地面用胶带标起点 A/B/C；meta `notes` 写「起点 A，机头朝窗」。

## 3.2 必须录制的 Topic

与 [`config/vla_record_topics.yaml`](drone_ws/agile_ws/src/vla_bridge/config/vla_record_topics.yaml) / 方案 4.3 一致。

**微调主链路（必录）**

| Topic | 类型 | 用途 |
|---|---|---|
| `/vla/instruction` | `std_msgs/String` | 任务标签 |
| `/uav1/odin1/image/compressed` | `sensor_msgs/CompressedImage` | **唯一** RGB 训练图源 |
| `/uav1/odin1/odometry` | `nav_msgs/Odometry` | 给模型看的位姿 |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | ENU 专家轨迹（`bag_to_uavflow --pose-source`） |
| `/uav1/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 位姿备份 |
| `/uav1/mavros/state` | `mavros_msgs/State` | armed / mode；未解锁轨迹应丢弃 |
| `/uav1/mavros/rc/in` | `mavros_msgs/RCIn` | 专家遥控 |
| `/uav1/move_command` | `uav_control_fsm/uav_command` | 若手飞则为空，示范飞则有 |
| `/uav1/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 实际发给 PX4 的指令 |
| `/tf` `/tf_static` | TF | 事后对齐 |

**后处理有益（默认 yaml 已含 imu/highfreq；CameraInfo / Depth 需先确认名字再加）**

| Topic | 说明 |
|---|---|
| `/uav1/odin1/odometry_highfreq` | 已在 `vla_record_topics.yaml` |
| `/uav1/odin1/imu` | 已在默认列表 |
| `.../camera_info` | **不是** SimpleVLADataset 字段；标定/复现内参加此条。先 `rostopic list \| grep camera_info`，再 `--extra` / `~extra_topics` |
| 深度图（若有独立 `sensor_msgs/Image`） | Odin 包含 pcd2depth，**话题名以现场为准**。微调当前 **不读深度**。禁止把 `*cloud*` / `*points*` 录进同一 bag |

**禁止**

- `*cloud*` `*points*` `*render*` `*undistorted*`
- 未压缩 raw `/image` 可调试，但 **不要**当微调主集（太大且与 224 letterbox 不一致）
- 深度/点云 **不是** 当前 `SimpleVLADataset` 所需；用户若另做深度研究可单独开包，勿混进 `odin_real/`

专家动作：本栈没有单独的 `manual_control` 话题；专家意图以 **`/uav1/mavros/rc/in` + 融合后 odom +（可选）`move_command`** 为准。

## 3.3 采集操作步骤

**需要启动的节点（采数，不启 VLA 控飞）**

1. `roscore`
2. `odin1_ros1.launch`（话题在 `/uav1/odin1`）
3. `px4.launch` + `exp_single_uav_control_fsm.launch`（要 RC、state、local odom）
4. **不要** `exp_single_planner.launch`（Ego-Planner）
5. **不要**把 `vla_bridge.launch` 的 `enable_publish` 打开。`vla_record.launch` 默认 `start_bridge:=false`。无桌面 SSH 用 `roslaunch`，不要 `odin.sh`（要 `DISPLAY`）。

```bash
source /opt/ros/noetic/setup.bash
source $HOME/agile_ws/devel/setup.bash
source $HOME/odin_ws/devel/setup.bash

DATE=$(date +%Y%m%d_%H%M%S)
roslaunch vla_bridge vla_record.launch \
  instruction:="move forward" \
  instruction_zh:="向前平移" \
  instr_id:=forward \
  bag_dir:=$HOME/vla_bags \
  site:=factory lighting:=indoor pilot:=cym
```

`vla_record_session.py` 内部是 `rosbag record -O $DATE_$instr_id`，**没有** `--split`。短 demo（<20 s）用 launch 即可。补录 CameraInfo/深度：改 `vla_record_topics.yaml`（禁止 cloud/points），或下面手写命令加 topic。

长飞拆包（`--split --size=1024`，前缀 `$DATE_$TASK_NAME`）：

```bash
DATE=$(date +%Y%m%d_%H%M%S)
TASK=forward
mkdir -p $HOME/vla_bags
rosbag record --split --size=1024 -O $HOME/vla_bags/${DATE}_${TASK} \
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

流程：

1. 飞手起飞到定点悬停（手飞或已验证 FSM），此高度记入 `fixed_altitude` 候选。
2. 操作员确认 instruction 已 latch（`rostopic echo /vla/instruction`）。
3. 开始 record。
4. 飞手 **只看 Odin/FPV** 按指令飞完。
5. 悬停 ≥1 s → Ctrl-C 停 bag（`vla_record_session` 会写 `.meta.json`）。
6. 立刻 `rosbag info`（下一节）。脏数据：meta 里 `"success": false`，`safety_events` 写原因。

转换（地面电脑即可）：

```bash
python3 $(rospack find vla_bridge)/scripts/bag_to_uavflow.py \
  --bags $HOME/vla_bags/${DATE}_${TASK}*.bag \
  --out $HOME/vla_uav/datasets/odin_real \
  --pose-source auto \
  --rate-hz 5
# 含 --require-armed 时会丢未解锁段
```

## 3.4 采集后质量复验

```bash
rosbag info $HOME/vla_bags/<file>.bag
```

- [ ] `/uav1/odin1/image/compressed` 消息数 **> 0**
- [ ] `/uav1/odin1/odometry` 或 mavros odom **> 0**
- [ ] `/vla/instruction` ≥ 1
- [ ] 无 `cloud`/`points` topic
- [ ] 时长与飞手记忆一致（约 5–20 s）

时间戳对齐（抽查）：

```bash
python3 - <<'PY'
import rosbag, numpy as np
bag = rosbag.Bag("REPLACE.bag")
def stamps(topic):
    return np.array([t.to_sec() for _, _, t in bag.read_messages(topics=[topic])])
img = stamps("/uav1/odin1/image/compressed")
odo = stamps("/uav1/odin1/odometry")
print("img n/hz", len(img), (len(img)-1)/(img[-1]-img[0]+1e-6))
print("odo n/hz", len(odo), (len(odo)-1)/(odo[-1]-odo[0]+1e-6))
# 最近邻 dt
if len(img) and len(odo):
    dts = [min(abs(odo-t)) for t in img[::max(1,len(img)//20)]]
    print("sample |img-odom| min/mean/max", min(dts), np.mean(dts), max(dts))
bag.close()
PY
```

- [ ] 图–Odin odom 最近邻多数 **< 0.15 s**（与 `sync_slop_sec` 同量级）
- [ ] 明显丢帧（hz 远低于 8）→ 标失败，不进 `odin_real`
- [ ] `mavros/state.armed=false` 的整段不要当专家轨迹

---

# 4. 实机部署与实飞操作流程

仅当 **阶段 3–6 通过**、围栏内、观察员在场、`lock_altitude=true`、`max_step_xy_m=0.25`、`use_vla_yaw=false`、`inference_rate_hz=2`。第一次只允许 **一次短动 < 1 m**，然后 hover。

## 4.1 地面准备（上电与启动顺序）

```text
1. 飞控、Orin、Odin、4090 上电；CH6 低/中档；桨叶按阶段决定
2. 4090：python vla-scripts/openvla_act.py          # 确认 5007
3. Orin 终端1：roscore（可选）
4. Orin 终端2：Odin launch（/uav1/odin1 有图和 odom）
5. Orin 终端3：px4.launch                             # MAVROS connected
6. Orin 终端4：exp_single_uav_control_fsm.launch      # 无 Ego-Planner
7. Orin 终端5：vla_bridge.launch enable_publish:=false
8. 走一遍 2.2–2.4 检查单；再按阶段打开 enable_publish
```

图形桌面可用（检查 `DISPLAY`）：

```bash
cd $HOME/agile_ws/scripts/exp_scripts
./odin.sh                          # gnome-terminal 里 roslaunch odin1_ros1.launch
./exp_single_mavros_control.sh     # MAVROS + exp_single
```

SSH 无桌面则三个终端分别 `roslaunch`（§2.2 / §2.5），**不要**跑上述脚本。

然后：

```bash
roslaunch vla_bridge vla_bridge.launch enable_publish:=false server_host:=10.62.25.215
```

- [ ] `/vla/status` 能到 `TRACKING`（图像、odom、infer 均 ok）
- [ ] `publish_enabled=false`（尚未开闸）
- [ ] QGC/飞手确认 PX4 未意外 OFFBOARD

## 4.2 起飞与切换

**推荐（VLA 不负责起飞）**

1. 飞手 **RC 手飞或 POSCTL** 起飞到约 **1.2 m** 悬停（或与 `takeoff_height` 一致）。
2. 机头朝向任务前方，操作员记录 yaw。
3. 操作员：`rostopic pub -1 /vla/instruction ...`（锁第一帧原点）。
4. 确认 `/vla/proprio` 接近 `[0,0,0,当前yaw或局部0]`。

**禁止**：让 VLA 或脚本发 `/uav1/offboard_command` 来「帮忙起飞」。  
若使用 FSM 高档自动起飞：必须是飞手拨 CH6，且操作员已确认 VLA `enable_publish` 仍为 false，直到悬停稳定。

## 4.3 Offboard 切入

1. 飞手确认周围无人、观察员举手。
2. 操作员：`enable_publish:=true` 的 launch **已经**在跑，或重启带该参数；再：

```bash
rostopic pub -1 /vla/enable_control std_msgs/Bool "data: true"
```

3. 飞手 **CH6 高档** → PX4 OFFBOARD（若已在空中则进 HOVER 而非再起飞）。
4. 操作员立刻：

```bash
rostopic echo /uav1/mavros/state          # mode OFFBOARD, armed true
rostopic hz /uav1/mavros/setpoint_raw/local
rostopic hz /uav1/move_command
rostopic echo /vla/status
```

- [ ] setpoint 约 30 Hz，move_command 约 25 Hz
- [ ] 飞手拇指放在 CH6 上，随时打回 **中档**

## 4.4 任务执行与监控

```bash
rostopic pub -1 /vla/instruction std_msgs/String "data: 'move forward'"
rostopic echo /vla/action_safe
rostopic echo /vla/heartbeat
rostopic echo /vla/status
```

- 只允许 **一条** 简单指令、一次短动。
- 位移目视 **< 1 m** 或操作员发急停：

```bash
rostopic pub -1 /vla/emergency_stop std_msgs/Bool "data: true"
# 或
rostopic pub -1 /uav1/hover_command std_msgs/Bool "data: true"
```

全程录 bag（§3.3 命令，另加 `/vla/action_raw` `/vla/action_safe` `/vla/status` 便于复盘）。

## 4.5 应急接管

优先级从高到低：

| 级 | 谁 | 动作 |
|---|---|---|
| 1 | 飞手 | **CH6 中档** → 退 Offboard → POSCTL 手飞 |
| 2 | 飞手 | PX4 杀桨：1 号机 **CH10**（`RC_MAP_KILL_SW=10`）。先在拆桨状态下拨一次，确认电机停转 |
| 3 | 操作员 | `/vla/emergency_stop` → 切断 `enable_control` + hover |
| 4 | 看门狗 | 心跳 >2 s 或失败 ≥3 次 → hover；>8 s → land（须 `enable_publish` 才写 FSM 话题） |
| 5 | 降落 | CH6 **离开高档** 后 `/uav1/land_command`；杆仍在高档可能拒降或被拉回 Offboard |

- [ ] 飞手在起飞前实际打过一次「高档进 / 中档出」，确认手感。
- [ ] 断网演练已在阶段 6 拆桨做过。

任务结束：先 `enable_control=false`，CH6 中档，手降或 land_command，再停节点（先 vla_bridge，再 FSM，再 MAVROS，再 Odin）。

---

# 5. 常见问题与代码/配置对照表

| 调试现象 | 排查命令 | 改哪里 |
|---|---|---|
| ping 4090 失败 | `hostname -I`; `ping -c 5 10.62.25.215` | 网络/AX3/认证；不是 VLA 代码 |
| 5007 不通 | `nc -vz 10.62.25.215 5007` | `openvla_act.py` 的 `http_port`、防火墙、`model_path` |
| 图在 `/odin1/...` 不在 `/uav1/` | `rostopic list \| grep odin` | `odin1_ros1.launch ns:=/uav1`；不要双层 ns |
| 无 compressed | `rostopic hz /uav1/odin1/image/compressed` | `control_command.yaml` → `sendrgbcompressed: 1` |
| 画面发蓝 | `rqt_image_view` `/vla/debug/image` | `vla_image_proc.py` 已做 JPEG→RGB；检查是否订错 raw `bgr8` 当 RGB |
| `/predict` 400/字段错 | 抓包看 JSON keys | 只能 `image` `proprio` `instr`（`vla_infer_client_node.py`） |
| 推理成功但动作数米 | `rostopic echo /vla/action_raw` | 域外 + `unnorm_key=sim`；限幅看 `action_safe`；禁止当真飞 |
| `action_safe` 方向反 | 地面朝向实验 §2.4 | `vla_bridge.yaml` → `vla_body_y_sign` / `frames/odom_to_enu` |
| yaw 像弧度 | `echo /vla/debug/move_command` 的 `yaw_ref` | `vla_cmd_node.py` 必须度；FSM `uav_command.yaw_ref` 定义是度 |
| 有 debug 无 FSM 指令 | `rosparam get /control/enable_publish`; `echo /vla/enable_control` | launch `enable_publish`；人工 `enable_control`；watchdog 是否切断 |
| `move_command` stamp 被丢 | `rostopic hz /uav1/move_command` | 应 ~25 Hz；`max_age_sec` |
| 无法 Offboard | `echo /uav1/mavros/state`; CH6 PWM | FCU 未连、odom 超时、不在 MANUAL/HOVER、起飞间隔；**VLA 不要发 offboard_command** |
| 中档退不出 | `echo /uav1/mavros/rc/in` 通道 6 | `rc_offboard_channel` / 阈值；遥控器映射 |
| land 后又起飞 | 看 CH6 | 拨离高档再 `land_command` |
| 两路 odom 差大 | 同时 hz/echo 两路 odom | `odom_disagree_m`；标定 `odom_to_enu`；先停发 |
| 看门狗不 hover | `echo /vla/status` | `enable_publish=false` 时只有 `/vla/debug/hover_command` |
| 笔记本 echo 不了 `uav_command` | `rosmsg show uav_control_fsm/uav_command` | Orin 完整 `./build.sh`；笔记本可看 `/vla/action_safe` |
| `odin.sh` 立刻退出 | 看是否打印「需要 DISPLAY」 | SSH 下直接 `roslaunch odin_ros_driver odin1_ros1.launch` |
| `infer_client` ImportError requests | 节点日志 | `sudo apt install python3-requests` |
| `set_location_source` 报 profile 不存在 | FSM 日志 `profile 'Odin' not exist` | 保持 `Fast_Lio` + 现有 remap；或先在 `uav_group.yaml` 复制 Fast_Lio 块改名为 `Odin` |
| 起飞高度和锁高打架 | 对比悬停 z 与 `action_safe.z` | `takeoff_height` 与 `control/fixed_altitude` 改成同一个数 |
| bag 巨大 | `rosbag info` | 勿录 cloud；用 `--split --size=1024` |
| 微调图与在线不一致 | 对比 224 图 | 必须走 `bag_to_uavflow.py` letterbox，不要另写 resize |
| 曝光闪 | `rqt_image_view` | `SetAe`/`SetAwb` 关自动；`control_command.yaml` |
| `default_location_source` 仍是 Fast_Lio | 读 `uav_group.yaml` | **当前实机路径就是 Fast_Lio + remap Odin 里程计**（Orin 部署文档）。不要先改成 `Odin` 除非 yaml 已有同名 profile |

---

## 附录 A. 本机 Mock（不上 Orin 时）

```bash
python3 $(rospack find vla_bridge)/scripts/mock_inference_server.py --host 127.0.0.1 --port 5007
roslaunch vla_bridge vla_bridge_mock.launch
# 只验证节点与 HTTP，不能替代阶段 3 真 Odin
```

## 附录 B. HTTP 契约（勿改）

**请求 JSON**

```json
{
  "image": "<base64 JPEG>",
  "proprio": [0.0, 0.0, 0.0, 0.0],
  "instr": "move forward"
}
```

**响应 JSON**（`openvla_act.py`）

```json
{
  "status": "success",
  "action": [[x, y, z, yaw_rad]],
  "action_ori": [[dx, dy, dz, dyaw_rad]],
  "message": "Action generated successfully"
}
```

`action` 已在 4090 上变为「当前 proprio 下的第一帧局部系绝对位姿」。Orin 只做 **一次** 3.4.3 变到 ENU。
