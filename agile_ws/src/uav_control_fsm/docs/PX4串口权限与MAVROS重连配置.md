# PX4 串口权限与 MAVROS 重连配置说明

本文档说明：在 Jetson 上通过 MAVROS 连接 PX4 时，若**切换定位源**或**软件重启飞控**后出现串口连接失败，应如何配置 udev 规则与用户权限。

---

## 1. 问题现象

### 1.1 MAVROS 崩溃

```
[ERROR] serial0: receive: End of file
terminate called after throwing an instance of 'std::system_error'
  what():  Resource deadlock avoided
REQUIRED process [uav1/mavros-1] has died!
```

**含义**：飞控 USB 断开（多为软件 reboot），MAVROS 串口 EOF，进程异常退出。

### 1.2 MAVROS 无限重启且 Permission denied

```
[FATAL] FCU: DeviceError:serial:open: Permission denied
[uav1/mavros-1] restarting process
```

**含义**：MAVROS respawn 尝试重新打开 `/dev/ttyACM0`，但当前用户无权限访问串口。

### 1.3 典型触发场景

- `uav_control_fsm` 切换定位源后，自动修改 PX4 EKF 参数并 **reboot 飞控**
- QGC 或软件中手动重启 PX4
- USB 重新枚举（内核日志可见 `USB disconnect` → `PX4 BL FMU` → `PX4 FMU`）

---

## 2. 根本原因

### 2.1 串口权限不足

Linux 下飞控串口设备通常为：

```text
/dev/ttyACM0  →  crw-rw----  root:dialout
```

只有 **root** 和 **dialout 组**成员可访问。若当前用户不在 `dialout` 组，MAVROS 打开串口会报 `Permission denied`。

检查命令：

```bash
whoami
groups                    # 应包含 dialout
ls -la /dev/ttyACM0
```

### 2.2 PX4 重启后 USB 重枚举

飞控 reboot 后 USB 会断开再连接，设备节点重新创建。若无 udev 规则：

- 权限可能恢复为默认的 `660 root:dialout`
- **ModemManager** 可能抢先占用串口（Jetson 上常见）

### 2.3 FSM 自动 reboot 飞控

`uav_control_fsm` 在应用定位源参数（如 `Vins`）后，若 `no_RC=false`，会发送飞控重启命令，导致 MAVROS 连接中断。这是预期行为，但需要 MAVROS 能自动、稳定地重连。

---

## 3. 修复命令说明

以下三条命令需**一次性**执行（需 sudo）：

```bash
sudo cp /home/ljzn/ljzn_ws/src/uav_control_fsm/config/99-px4-auterion.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout ljzn
```

### 3.1 安装 udev 规则

```bash
sudo cp .../99-px4-auterion.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**目的**：让 Linux 在 **每次 PX4 插入/重启 USB** 时，自动对串口做固定处理。

规则文件内容（`config/99-px4-auterion.rules`）：

```udev
# Auterion / PX4 FMU v6C (idVendor=3185, idProduct=0038)
SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", ATTRS{idProduct}=="0038", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="3185", ATTRS{idProduct}=="0038", MODE="0666", GROUP="dialout", SYMLINK+="px4fmu"
```

| 规则项 | 作用 |
|--------|------|
| `ID_MM_DEVICE_IGNORE="1"` | 禁止 **ModemManager** 占用 PX4 串口 |
| `MODE="0666", GROUP="dialout"` | 设备出现时自动赋予可读写权限 |
| `SYMLINK+="px4fmu"` | 创建固定符号链接 `/dev/px4fmu`（可选使用） |

### 3.2 将用户加入 dialout 组

```bash
sudo usermod -aG dialout ljzn
```

**目的**：让当前用户有权访问 `/dev/ttyACM0`，无需 `sudo roslaunch`。

**重要**：执行后必须 **注销并重新登录**（或重启），组权限才会生效。临时生效可用：

```bash
newgrp dialout
```

---

## 4. 整体流程

```text
PX4 软件重启
    ↓
USB 断开再连接 → 出现新的 /dev/ttyACM0
    ↓
udev 规则：禁止 ModemManager 抢占 + 设置正确权限
    ↓
dialout 组：当前用户有权打开串口
    ↓
MAVROS respawn（等待 respawn_delay 秒）→ 成功连接 PX4
```

---

## 5. 工程内相关配置

### 5.1 MAVROS respawn 延迟

`launch/px4.launch` 已配置：

- `respawn_mavros:=true`：飞控断连后自动重启 MAVROS 节点
- `respawn_delay:=8.0`：等待 8 秒再重连，给 USB 重枚举留时间

`launch/mavros_px4_node.launch` 在标准 `node.launch` 基础上增加了 `respawn_delay` 参数。

### 5.2 可选：跳过 FSM 自动 reboot

若 PX4 参数已在 QGC 中设好，可在 launch 中设置：

```xml
<arg name="no_RC" default="true"/>
```

避免每次启动 FSM 都修改 EKF 参数并 reboot 飞控。参数变更后需在 QGC **手动重启飞控一次**。

### 5.3 串口 URL

默认连接：

```xml
<arg name="fcu_url" default="/dev/ttyACM0:921600" />
```

安装 udev 规则后也可使用：

```text
/dev/px4fmu:921600
```

---

## 6. 验证步骤

```bash
# 1. 确认用户组
groups
# 输出应包含 dialout

# 2. 确认设备权限
ls -la /dev/ttyACM0 /dev/px4fmu

# 3. 启动 MAVROS
roslaunch uav_control_fsm px4.launch

# 4. 确认连接
rostopic echo /uav1/mavros/state -n 1
# connected: True

# 5. 测试 reboot 后重连（改定位源或 QGC reboot 后）
# 等待约 8–15 秒，再次检查 connected: True
```

---

## 7. 临时应急（未重新登录前）

```bash
sudo chmod 666 /dev/ttyACM0
# 然后重新 roslaunch mavros
```

仅对当前会话有效，拔插 USB 或 reboot 后可能失效。**仍建议完成 dialout 组 + udev 永久配置。**

---

## 8. 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| Permission denied | 未加入 dialout 或未重新登录 | `usermod -aG dialout` + 重新登录 |
| 重启后仍连不上 | ModemManager 抢占 | 确认 udev 规则已安装并 trigger |
| respawn 太频繁 | USB 尚未就绪 | 增大 `respawn_delay`（如 10–15 秒） |
| 首次能连、改定位源后断 | FSM 自动 reboot 飞控 | 正常；等待 respawn 或设 `no_RC:=true` |
| Jetson USB 不稳定 | 线缆/HUB 问题 | 换短线、直连、避免劣质 HUB |

---

## 9. 命令速查

```bash
# 一次性安装
sudo cp /home/ljzn/ljzn_ws/src/uav_control_fsm/config/99-px4-auterion.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER

# 重新登录后验证并启动
groups
roslaunch uav_control_fsm px4.launch
rostopic echo /uav1/mavros/state
```

---

## 10. 总结

| 操作 | 目的 |
|------|------|
| **udev 规则** | PX4 重连后串口权限稳定，不被 ModemManager 抢占 |
| **usermod dialout** | 当前用户有权限访问 `/dev/ttyACM0` |
| **respawn_delay** | MAVROS 重连前等待 USB 设备就绪 |
| **no_RC:=true**（可选） | 避免 FSM 每次启动都 reboot 飞控 |

两者（udev + dialout）配合使用，可解决「重启 PX4 后 MAVROS 报 Permission denied 并不断 respawn」的问题。
