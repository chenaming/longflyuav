# vla_bridge

Orin 端云边桥接包。按 `docs/VLA_Drone_Deployment_Plan.md` 第 7 章和仓库根目录 `session_handoff.md` 分步实现。

当前进度：第 8 步（`bag_to_uavflow.py` → `SimpleVLADataset`）。默认 `enable_publish:=false`。

## 依赖

```bash
sudo apt install python3-requests   # 推理客户端。Mock 服务只用标准库，无需 Flask
```

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/cym/vla_uav/drone_ws/agile_ws
catkin_make --source src/vla_bridge --build build/vla_bridge -j2
source devel/setup.bash
```

## 第 2 步：Mock HTTP

```bash
# 终端 A
python3 $(rospack find vla_bridge)/scripts/mock_inference_server.py

# 终端 B
roslaunch vla_bridge vla_bridge_mock.launch
```

`vla_bridge_mock.launch` 把推理地址改成 `127.0.0.1:5007`，并往 `/uav1/odin1/image/compressed` 与 `/uav1/odin1/odometry` 灌假 Odin 数据（约 10 Hz）。`vla_image_node` 降到 2 Hz、224 JPEG 后再给推理客户端。

```bash
rostopic hz /uav1/odin1/image/compressed   # ~10 Hz
rostopic hz /vla/image_for_infer           # ~2 Hz
rostopic echo /vla/proprio
rqt_image_view                             # /vla/debug/image，颜色不应发蓝
```

```bash
rostopic echo /vla/action_raw
rostopic echo /vla/heartbeat
rostopic hz /vla/action_raw
```

停掉 Mock 后，`/vla/heartbeat` 应为 `alive: False` 且 `last_error` 非空。

## 第 4 步：action_safe

`vla_cmd_node` 把 `action_raw`（first_frame）变到 ENU，按 `max_step_*` 限幅，发布 `/vla/action_safe`。`frame_id=enu`，`yaw_unit=rad`。不发布 `/uav1/move_command`。

```bash
rostopic echo /vla/action_safe
rostopic echo /vla/first_frame_origin
# z 应为 fixed_altitude（默认 1.2）；相对当前里程计的水平步长 ≤ max_step_xy_m（0.25）
```

## 第 5 步：25 Hz 预览

`/vla/debug/move_command` 为 `uav_control_fsm/uav_command`，约 25 Hz，每次刷新 stamp。`Move_mode=0`，`yaw_ref` 为度。默认不发布 `/uav1/move_command`。

```bash
rostopic hz /vla/debug/move_command
rostopic echo /vla/debug/move_command
```

## 第 6 步：watchdog

`/vla/status` 走 `INIT → WAIT_SENSORS → READY/REQUESTING → TRACKING`。心跳连续失败或中断后切 `HOVER`，持续失败后切 `LAND`。看门狗只把 `/vla/enable_control` 置 false，不解锁、不起飞、不进 Offboard。

默认 `enable_publish:=false` 时只预览：

- `/vla/debug/hover_command`
- `/vla/debug/land_command`

`/uav1/hover_command`、`/uav1/land_command`、`/uav1/move_command` 都不应存在。

```bash
rostopic echo /vla/status
rostopic echo /vla/enable_control
# 停掉 mock_inference_server 后：status 先 HOVER，约 8 s 后 LAND
# rostopic pub /vla/emergency_stop std_msgs/Bool "data: true"  → 立即 HOVER
```

## 第 7 步：录包

采集模式**不跑 VLA 控制**。`vla_record.launch` 强制 `enable_publish:=false`，只 latch `/vla/instruction` 并按方案 4.3 录 bag。不要录点云。话题清单在 `config/vla_record_topics.yaml`。

笔记本可用假 Odin 验收（真机录包要上 Orin，那一步会提前说）：

```bash
roslaunch vla_bridge vla_record.launch \
  inject_fake_obs:=true \
  instruction:="move forward" \
  instr_id:=forward \
  bag_dir:=/tmp/vla_bags \
  site:=lab lighting:=indoor pilot:=mock
# Ctrl-C 后：
rosbag info /tmp/vla_bags/*.bag
# compressed 与 odom 条数应 > 0；/uav1/move_command 必须不存在
```

真机（Odin + FSM 另开，本 launch 不要 `start_bridge:=true`）：

```bash
roslaunch vla_bridge vla_record.launch \
  instruction:="fly to the left of the red cabinet" \
  instruction_zh:="飞到红色柜子左侧" \
  instr_id:=cabinet_left \
  site:=factory lighting:=indoor pilot:=cym
```

同名 `.meta.json` 含指令、场地、飞手；`success` 落地后手改。

## 第 8 步：bag → SimpleVLADataset

把方案 4.3 的 bag 转成 `datasets/odin_real/<episode_id>/`（`000000.jpg` + `log.json`）。图像与在线相同：letterbox 224 JPEG。`preprocessed_logs` 用方案 3.4.2 第一帧局部系。不要转 RLDS/HDF5。

```bash
python3 $(rospack find vla_bridge)/scripts/bag_to_uavflow.py \
  --bags /tmp/vla_bags/*.bag \
  --out /home/cym/vla_uav/datasets/odin_real \
  --pose-source auto \
  --rate-hz 5 \
  --chunk-sec 11
python3 $(rospack find vla_bridge)/scripts/bag_to_uavflow.py --check --out /home/cym/vla_uav/datasets/odin_real
```

`--require-armed` 仅在 bag 里有 `/uav1/mavros/state` 时使用。Mock bag 不要加。
