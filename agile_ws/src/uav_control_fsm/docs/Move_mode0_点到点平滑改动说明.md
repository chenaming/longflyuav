# uav_control_fsm：Move_mode=0 点到点软件平滑改动说明

本文档记录为 **惯性系位置控制（Move_mode == 0）** 增加 MAVROS 软件平滑的实现改动。  
通信方式仍为 **MAVROS `/mavros/setpoint_raw/local`**，不引入 PX4 DDS `GotoSetpoint`，**不改变 OFFBOARD 模式逻辑**。

---

## 1. 改动背景

### 改动前

- `Move_mode == 0` 调用 `ned_pos_xyz_set()`。
- 每帧只发 **目标位置 + yaw**，`type_mask` 忽略全部速度。
- 等价于对 PX4 发 **位置阶跃**，远目标易 **超调**。

### 改动后

- 默认启用 `ned_pos_xyz_smooth_set()`。
- 根据当前 **里程计位置/速度** 生成轨迹点与速度前馈（制动曲线 `v = min(v_cruise, sqrt(2·a·dist))`）。
- `type_mask` 保留 **位置 + 速度**，忽略加速度。
- 接近目标后锁终点、速度置零，减少冲过头。

---

## 2. 修改文件清单

| 文件 | 变更类型 | 说明 |
|------|----------|------|
| `include/uav_control_fsm/control_manager.hpp` | **重构** | 类声明；`PosSmoothParams`、`ned_pos_xyz_smooth_set`、平滑状态成员 |
| `src/control_manager.cpp` | **新增** | setpoint 生成 + 平滑算法 |
| `include/uav_control_fsm/param_set.hpp` | **修改** | 6 个 ROS 平滑参数 |
| `include/uav_control_fsm/fsm_control.hpp` | **修改** | `setPosSmoothParams`；悬停时 `resetPosSmooth()` |
| `CMakeLists.txt` | **修改** | 库 `uav_control_manager` |
| `launch/sim_single_uav_control_fsm.launch` | **修改** | 默认平滑参数 |

---

## 3. ROS 参数

默认值在 `config/pos_smooth_param.yaml`；launch 顶层提供开关：

```bash
# 启用平滑（默认）
roslaunch uav_control_fsm sim_single_uav_control_fsm.launch

# 关闭平滑，恢复原 Move_mode=0 阶跃位置行为
roslaunch uav_control_fsm sim_single_uav_control_fsm.launch pos_smooth_enable:=false
```

节点私有参数示例：

```xml
<arg name="pos_smooth_enable" default="true"/>
<rosparam file="$(find uav_control_fsm)/config/pos_smooth_param.yaml" />
<param name="pos_smooth_enable" value="$(arg pos_smooth_enable)" />
```

---

## 4. 编译

```bash
cd ~/catkin_ws/build/uav_control_fsm && cmake ../../src/uav_control_fsm && make -j4
```

或：

```bash
cd ~/catkin_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="uav_control_fsm"
```

---

## 5. 测试

```bash
roslaunch uav_control_fsm sim_single_uav_control_fsm.launch

rostopic pub /uav0/offboard_command std_msgs/Bool "data: true" -1

rostopic pub /uav0/move_command uav_control_fsm/uav_command "
header:
  stamp: now
  frame_id: 'map'
Move_mode: 0
pos_ref: {x: 5.0, y: 0.0, z: 1.5}
yaw_ref: 0.0" -r 20

rostopic echo /uav0/mavros/setpoint_raw/local
```

平滑启用时：飞行中 `velocity` 非零，接近目标后趋近 0。
