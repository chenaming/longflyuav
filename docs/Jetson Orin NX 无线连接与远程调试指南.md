---
title: Jetson Orin NX 无线连接与远程调试指南
tags:
  - Jetson
  - Orin-NX
  - SSH
  - Wi-Fi
  - VS-Code
  - ROS
---


> [!abstract] 适用场景
> Jetson Orin NX 安装在无人机上、没有显示器，通过 Windows 笔记本完成 Wi‑Fi 配置，并使用 SSH、VS Code Remote‑SSH、NoMachine 或 ROS/RViz 进行远程调试。

> [!tip] 推荐调试路线
> **网线直连配置 Wi‑Fi → 无线 SSH → VS Code Remote‑SSH → 本地 RViz 可视化**

---

## 一、最终可用方案概览

本次成功采用的是：

```text
笔记本
  │
  │ 网线直连
  │
Orin NX：192.168.1.108
  │
  │ SSH 登录后配置 Wi-Fi
  │
路由器 Wi-Fi：HUAWEI-1GRG9Q
  │
  └── Orin NX 无线 IP：10.49.11.126
```

之后的日常调试方式：

- SSH：

```bash
ssh nvidia@10.49.11.126
```

- VS Code Remote - SSH：连接 `nvidia@10.49.11.126`
- 完整桌面：使用 NoMachine
- ROS 数据可视化：优先在笔记本本地运行 RViz/RViz2

---

## 二、当前设备参数记录

| 项目 | 当前值 |
|---|---|
| Orin 用户名 | `nvidia` |
| Orin 当前密码 | `nvidia`，如果后续未修改 |
| 网线直连时笔记本 IP | `192.168.1.10` |
| 网线直连时 Orin IP | `192.168.1.108` |
| Wi-Fi 名称 | `HUAWEI-1GRG9Q` |
| Wi-Fi 当前 IP | `10.49.11.126` |
| Wi-Fi 网卡名称 | 通常为 `wlan0` |
| VS Code 远程主机名 | `Orin-NX` |
| SSH 端口 | `22` |
| NoMachine 端口 | `4000` |

> 注意：Wi-Fi 使用 DHCP 时，`10.49.11.126` 可能会变化。以后如果 IP 经常变化，建议在路由器中为 Orin 的无线网卡设置 DHCP 静态租约。

---

# 三、完整操作流程

## 第 1 步：网线直连 Orin NX

使用网线连接：

```text
笔记本电脑网口 ↔ Orin NX 网口
```

此时不要依赖 Wi-Fi，因为 Orin 还没有配置无线网络。

---

## 第 2 步：设置笔记本有线网卡 IP

在 Windows 中：

- 打开“设置”
- 进入“网络和 Internet”
- 进入“高级网络设置”
- 找到“以太网”
- 打开“更多网络适配器选项”
- 右键“以太网”
- 选择“属性”
- 双击“Internet 协议版本 4（TCP/IPv4）”
- 选择“使用下面的 IP 地址”

填写：

```text
IP 地址：192.168.1.10
子网掩码：255.255.255.0
默认网关：留空
DNS：留空
```

保存设置。

### 为什么要这样设置？

网线直连时没有路由器自动分配 IP，因此笔记本和 Orin 必须手动设置在同一个网段：

```text
笔记本：192.168.1.10
Orin：192.168.1.108
```

两者都属于：

```text
192.168.1.0/24
```

---

## 第 3 步：查找 Orin 的有线 IP

当前已经确认 Orin 的有线 IP 是：

```text
192.168.1.108
```

可以先在 Windows CMD 中测试：

```dos
ping 192.168.1.108
```

如果能看到类似：

```text
来自 192.168.1.108 的回复
```

说明网线通信正常。

### 如果不知道 Orin 的 IP

可以执行：

```dos
for /L %i in (1,1,255) do @ping -n 1 -w 10 192.168.1.%i | find "Reply"
```

这个命令会依次尝试 `192.168.1.1` 到 `192.168.1.255`，可能需要几十秒到一两分钟。

扫描后查看 ARP 表：

```dos
arp -a
```

在 `192.168.1.10` 对应的接口下面，找到除以下地址以外的动态 IP：

```text
192.168.1.255
224.x.x.x
```

本次找到的 Orin 地址是：

```text
192.168.1.108
```

---

## 第 4 步：通过 SSH 登录 Orin

在 Windows CMD 中执行：

```bash
ssh nvidia@192.168.1.108
```

第一次连接时可能出现：

```text
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

输入：

```text
yes
```

然后输入密码：

```text
nvidia
```

输入密码时，屏幕不会显示字符、星号或光标变化，这是正常现象。直接输入密码后按回车即可。

登录成功后，命令提示符通常类似：

```bash
nvidia@orin:~$
```

---

# 四、在 Orin 上配置 Wi-Fi

## 第 5 步：确认无线网卡名称

在 Orin 终端执行：

```bash
nmcli device status
```

找到类型为 `wifi` 的设备。

常见名称是：

```text
wlan0
```

如果不是 `wlan0`，后续命令中的 `wlan0` 要替换成实际名称。

例如：

```text
wlP1p1s0
```

---

## 第 6 步：扫描 Wi-Fi

执行：

```bash
sudo nmcli radio wifi on
sudo nmcli device wifi rescan
sudo nmcli device wifi list
```

找到目标 Wi-Fi：

```text
HUAWEI-1GRG9Q
```

本次扫描时发现了多个相近的热点：

```text
HUAWEI-1GRG9Q
HUAWEI-1GRG9Q_Wi-Fi5
```

如果存在同名或相近名称的多个热点，建议使用 BSSID 精确连接，避免连错。

本次记录的 BSSID：

```text
HUAWEI-1GRG9Q       62:A5:38:03:18:27
HUAWEI-1GRG9Q_Wi-Fi5 62:A5:38:03:18:35
```

---

## 第 7 步：连接 Wi-Fi

### 方法 A：直接按 Wi-Fi 名称连接

```bash
sudo nmcli device wifi connect "HUAWEI-1GRG9Q" password "你的WiFi密码" ifname wlan0
```

将：

```text
你的WiFi密码
```

替换成实际密码。

---

### 方法 B：使用 BSSID 精确连接

如果有多个相近热点，使用：

```bash
sudo nmcli device wifi connect "62:A5:38:03:18:27" password "你的WiFi密码" ifname wlan0
```

如果要连接 Wi-Fi5 版本：

```bash
sudo nmcli device wifi connect "62:A5:38:03:18:35" password "你的WiFi密码" ifname wlan0
```

连接成功后，通常会出现：

```text
Device 'wlan0' successfully activated
```

---

## 第 8 步：查看 Orin 的无线 IP

执行：

```bash
ip -4 addr show wlan0
```

也可以执行：

```bash
hostname -I
```

找到类似下面的内容：

```text
inet 10.49.11.126/24
```

其中：

```text
10.49.11.126
```

就是 Orin 的无线 IP。

本次成功获得的无线 IP 是：

```text
10.49.11.126
```

还可以使用以下命令确认当前连接：

```bash
nmcli connection show --active
```

查看默认路由：

```bash
ip route
```

---

## 第 9 步：设置 Wi-Fi 自动连接

查看连接名称：

```bash
nmcli connection show
```

假设连接名称是：

```text
HUAWEI-1GRG9Q
```

执行：

```bash
sudo nmcli connection modify "HUAWEI-1GRG9Q" connection.autoconnect yes
```

建议关闭 Wi-Fi 节能，避免无人机飞行过程中无线连接休眠或断开：

```bash
sudo iw dev wlan0 set power_save off
```

如果 `wlan0` 名称不同，需要替换为实际无线网卡名称。

---

# 五、切换到无线 SSH

## 第 10 步：拔掉网线

先不要急着拔网线，确认已经记住 Orin 的无线 IP：

```text
10.49.11.126
```

然后执行：

- 拔掉笔记本与 Orin 之间的网线
- 将笔记本有线网卡 IPv4 改回“自动获取 IP 地址”
- 让笔记本连接到同一个 Wi-Fi：

```text
HUAWEI-1GRG9Q
```

---

## 第 11 步：测试无线连接

在笔记本 CMD 中执行：

```dos
ping 10.49.11.126
```

如果能 ping 通，再执行：

```bash
ssh nvidia@10.49.11.126
```

能正常登录，说明无线网络配置完成。

---

# 六、使用 VS Code 远程开发

## 第 12 步：安装插件

在笔记本 VS Code 中：

- 打开扩展市场
- 搜索：

```text
Remote - SSH
```

- 安装 Microsoft 发布的插件

---

## 第 13 步：配置 SSH 主机

按下：

```text
Ctrl + Shift + P
```

搜索并执行：

```text
Remote-SSH: Open SSH Configuration File...
```

打开当前用户的 SSH 配置文件，通常是：

```text
C:\Users\你的用户名\.ssh\config
```

写入：

```sshconfig
Host Orin-NX
    HostName 10.49.11.126
    User nvidia
```

保存文件。

---

## 第 14 步：连接 Orin

在 VS Code 中：

- 按 `F1`
- 执行 `Remote-SSH: Connect to Host...`
- 选择：

```text
Orin-NX
```

第一次连接时：

- 如果询问目标平台，选择 `Linux`
- 如果询问是否继续连接，选择 `Continue`
- 输入 Orin 用户密码：

```text
nvidia
```

连接成功后，VS Code 左下角会显示：

```text
SSH: Orin-NX
```

此时可以：

- 打开 Orin 上的代码目录
- 使用 VS Code 终端
- 编译程序
- 查看日志
- 修改代码
- 调试 ROS 节点
- 运行无人机程序

例如打开工作空间：

```text
/home/nvidia/catkin_ws
```

或：

```text
/home/nvidia/ros2_ws
```

---

# 七、这次遇到的坑与解决方法

## 坑 1：Windows 扫描 IP 时看起来像卡住

使用以下命令时：

```dos
for /L %i in (1,1,255) do @ping -n 1 -w 10 192.168.1.%i | find "Reply"
```

命令不会实时显示所有尝试，只显示 ping 成功的设备，看起来像没有反应。

### 解决方法

- 等待几十秒到一两分钟
- 或按 `Ctrl + C` 停止
- 然后执行：

```dos
arp -a
```

- 查找新增的动态 IP

本次找到：

```text
192.168.1.108
```

---

## 坑 2：第一次 SSH 连接出现安全提示

出现：

```text
Are you sure you want to continue connecting?
```

这是 SSH 第一次连接设备时的正常提示。

### 解决方法

输入：

```text
yes
```

然后输入密码。

---

## 坑 3：输入 SSH 密码时屏幕没有任何显示

在 CMD 中输入 Linux 密码时：

- 不显示字符
- 不显示星号
- 不显示光标移动

### 解决方法

直接盲输入密码，然后按回车即可。

---

## 坑 4：VS Code 一直使用 Windows 用户名登录

之前 VS Code 使用了 Windows 用户名：

```text
26804
```

但 Orin 上的实际用户是：

```text
nvidia
```

因此登录失败，或者不断要求重复输入密码。

### 解决方法

在 SSH 配置文件中明确指定：

```sshconfig
Host Orin-NX
    HostName 10.49.11.126
    User nvidia
```

不要只配置 IP，必须配置：

```text
User nvidia
```

也可以直接使用完整 SSH 命令：

```bash
ssh nvidia@10.49.11.126
```

---

## 坑 5：多个 Wi-Fi 名称相近，可能连接错误热点

扫描时发现：

```text
HUAWEI-1GRG9Q
HUAWEI-1GRG9Q_Wi-Fi5
```

如果直接按名称连接，可能连接到错误频段或错误热点。

### 解决方法

先扫描并查看 BSSID：

```bash
nmcli device wifi list
```

然后使用 BSSID 精确连接：

```bash
sudo nmcli device wifi connect "62:A5:38:03:18:27" password "你的WiFi密码" ifname wlan0
```

---

## 坑 6：拔掉网线后无法连接

常见原因：

- 笔记本没有连接到同一个 Wi-Fi
- 笔记本有线网卡仍然使用静态 IP
- 使用了错误的无线 IP
- Orin 的 IP 因 DHCP 重新分配而改变
- 路由器开启了无线客户端隔离

### 排查顺序

在笔记本执行：

```dos
ping 10.49.11.126
```

如果 ping 不通：

- 确认笔记本连接的是 `HUAWEI-1GRG9Q`
- 确认笔记本有线网卡已恢复 DHCP
- 重新通过网线登录 Orin
- 查看新的无线 IP：

```bash
ip -4 addr show wlan0
```

- 检查路由器是否开启“AP 隔离”或“无线客户端隔离”

---

## 坑 7：无线网卡名称不一定是 wlan0

不同 Jetson 镜像或驱动可能将无线网卡命名为：

```text
wlan0
wlP1p1s0
其他名称
```

### 解决方法

始终先执行：

```bash
nmcli device status
```

找到类型为 `wifi` 的设备，再替换命令中的网卡名称。

---

# 八、远程桌面方案：NoMachine

如果需要完整操作 Orin 的桌面，可以使用 NoMachine。

## Orin 端安装虚拟显示支持

无人机通常没有接显示器，可能出现远程桌面黑屏或分辨率异常。

先安装：

```bash
sudo apt update
sudo apt install -y xserver-xorg-video-dummy
```

如果仍然黑屏，可以使用 HDMI 显卡欺骗器，让系统认为接入了显示器。

---

## 安装 NoMachine

在 Orin 上下载并安装当前版本的 ARM64 DEB 安装包。

安装命令格式：

```bash
sudo dpkg -i nomachine_arm64.deb
```

如果提示缺少依赖，执行：

```bash
sudo apt install -f
```

---

## 笔记本端连接

在笔记本安装 NoMachine 客户端，创建连接：

```text
Host：10.49.11.126
Port：4000
Protocol：NX
Username：nvidia
```

然后输入 Orin 登录密码。

### 建议

如果只是查看相机图像、点云或姿态数据，不建议优先使用完整远程桌面。远程桌面会占用更多无线带宽和 Orin GPU 资源。

---

# 九、ROS 可视化方案

## 推荐方式：笔记本本地运行 RViz

无人机上的 Orin 负责：

- 运行算法
- 发布相机数据
- 发布激光雷达数据
- 发布点云
- 发布位姿和轨迹

笔记本负责：

- 运行 RViz/RViz2
- 渲染图像
- 渲染点云
- 查看 TF 和地图

这样比远程桌面传输整个屏幕更加稳定。

---

## ROS 2

Orin 和笔记本设置相同的 Domain ID：

```bash
export ROS_DOMAIN_ID=42
```

在两台设备上都设置后，笔记本直接运行：

```bash
rviz2
```

然后添加需要查看的 Topic，例如：

```text
/camera/image_raw
/point_cloud
/tf
/odom
```

---

## ROS 1

如果使用 ROS 1，需要配置 ROS Master。

Orin 端：

```bash
export ROS_MASTER_URI=http://10.49.11.126:11311
export ROS_IP=10.49.11.126
```

笔记本端：

```bash
export ROS_MASTER_URI=http://10.49.11.126:11311
export ROS_IP=笔记本无线IP
```

然后在笔记本运行：

```bash
rviz
```

ROS 1 对网络、IP 和防火墙更敏感。如果只是远程查看数据，优先确认两台设备可以互相 ping 通。

---

# 十、日常调试快速流程

以后再次使用时，按下面顺序执行即可。

## 快速流程

- 给 Orin 和笔记本接网线
- 笔记本有线网卡设置为：

```text
192.168.1.10
255.255.255.0
```

- 测试 Orin：

```dos
ping 192.168.1.108
```

- SSH 登录：

```bash
ssh nvidia@192.168.1.108
```

- 查看 Wi-Fi：

```bash
nmcli device wifi list
```

- 连接 Wi-Fi：

```bash
sudo nmcli device wifi connect "HUAWEI-1GRG9Q" password "你的WiFi密码" ifname wlan0
```

- 查看无线 IP：

```bash
ip -4 addr show wlan0
```

- 记录 `inet` 后面的 IP
- 拔掉网线
- 笔记本连接同一个 Wi-Fi
- 笔记本有线网卡恢复 DHCP
- 测试：

```dos
ping Orin无线IP
```

- SSH 登录：

```bash
ssh nvidia@Orin无线IP
```

- VS Code 连接 `Orin-NX`

---

# 十一、最终检查清单

## 网线连接阶段

- [ ] 网线已连接
- [ ] 笔记本有线 IP 是 `192.168.1.10`
- [ ] 子网掩码是 `255.255.255.0`
- [ ] 可以 ping 通 `192.168.1.108`
- [ ] 可以 SSH 登录 `nvidia@192.168.1.108`

## Wi-Fi 配置阶段

- [ ] `nmcli device status` 能看到无线网卡
- [ ] 已扫描到 `HUAWEI-1GRG9Q`
- [ ] Wi-Fi 密码正确
- [ ] 已确认连接成功
- [ ] 已记录无线 IP
- [ ] 已设置自动连接
- [ ] 已关闭 Wi-Fi 节能

## 无线调试阶段

- [ ] 网线已拔掉
- [ ] 笔记本已连接同一个 Wi-Fi
- [ ] 笔记本有线网卡已恢复 DHCP
- [ ] 可以 ping 通 Orin 无线 IP
- [ ] 可以执行：

```bash
ssh nvidia@10.49.11.126
```

## VS Code 阶段

- [ ] 已安装 Remote - SSH
- [ ] SSH 配置中有 `User nvidia`
- [ ] 目标平台选择 Linux
- [ ] 可以看到 `SSH: Orin-NX`
- [ ] 可以打开 Orin 上的代码目录

---

# 十二、以后调试记录模板

每次重新配置或出现问题时，可以复制下面模板记录。

```markdown
# Orin NX 调试记录

## 基本信息

- 日期：
- Jetson 型号：Jetson Orin NX
- Orin 用户名：nvidia
- 使用系统：
- 笔记本系统：

## 网线连接

- 笔记本有线 IP：192.168.1.10
- Orin 有线 IP：192.168.1.108
- 是否可以 ping 通：
- 是否可以 SSH 登录：

## Wi-Fi 配置

- Wi-Fi 名称：
- BSSID：
- Orin 无线网卡名称：
- Orin 无线 IP：
- 是否设置自动连接：
- 是否关闭 Wi-Fi 节能：

## 远程连接

- SSH 是否成功：
- VS Code Remote - SSH 是否成功：
- NoMachine 是否成功：
- ROS/RViz 是否成功：

## 遇到的问题

### 现象

-

### 原因

-

### 解决方法

-

### 最终使用的命令

```bash

```

## 注意事项

-
```

---

## 最终结论

以后最稳定的调试路线是：

```text
网线直连
→ 设置笔记本为 192.168.1.10
→ SSH 登录 192.168.1.108
→ nmcli 配置 HUAWEI-1GRG9Q
→ 获取 Orin 无线 IP
→ 拔掉网线
→ 笔记本连接同一 Wi-Fi
→ SSH 登录无线 IP
→ VS Code Remote - SSH 远程开发
```

本次最关键的两个问题及解决方案是：

```text
IP 不知道：
通过 ping 扫描和 arp -a 找到 192.168.1.108

VS Code 用户名错误：
在 SSH 配置中明确写入 User nvidia
```


1号机：
IP 192.168.1.104
账号 nvidia
密码 nvidia
2号机：
IP 192.168.1.108
账号 nvidia
密码 nvidia
3号机：
IP 192.168.1.113
账号 agile
密码 agile