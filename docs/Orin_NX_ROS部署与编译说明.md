# Orin NX 机载 ROS 系统部署与编译说明

## 1. 系统概述

### 1.1 两个工作空间的分工

系统源码分为 `odin_ws` 和 `agile_ws` 两个 ROS 工作空间，分别承担 Odin 设备数据接入和无人机主系统功能。

| 工作空间 | 源码路径 | 主要职责 |
| --- | --- | --- |
| Odin 驱动工作空间 `odin_ws` | `/home/agile/odin_ws/src` | 包含 `odin_ros_driver`，负责接入 Odin 设备，将设备输出的里程计等数据发布为 ROS 消息，供主系统使用。 |
| 主系统工作空间 `agile_ws` | `/home/agile/agile_ws/src` | 包含定位、路径规划、感知、任务执行和飞行控制相关功能包，负责接收定位信息、执行上层任务，并通过 MAVROS 与 PX4 飞控交互。 |

`odin_ws` 的职责是提供 Odin 数据接口，不承担无人机任务规划和飞行控制。`agile_ws` 负责使用这些数据完成主系统功能。其中的其他定位功能包用于对应的定位方案；采用 Odin 时，不代表需要同时运行所有定位模块。

两个工作空间各自保存源码和编译结果，通过 ROS 话题交换数据。运行节点前，需要加载对应工作空间的环境，使 ROS 能够找到所需功能包。

### 1.2 Odin 里程计与主系统的关系

在采用 Odin 定位的配置中，Odin 提供外部里程计，主系统负责将其接入飞控定位链路。主要数据关系如下：

```text
Odin 设备
    ↓ 设备数据
odin_ws：odin_ros_driver
    ↓ ROS 里程计消息
agile_ws：uav_control_fsm 中的 OdometryBridge
    ↓ 外部定位位姿
MAVROS
    ↓ 外部定位输入
PX4 飞控
    ↓ 通过 MAVROS 输出本地位置、里程计和飞行状态
agile_ws：飞行控制及配置为使用该定位输出的上层功能模块
```

主系统中的 `OdometryBridge` 根据所选定位源订阅外部里程计，并将位姿转发到 MAVROS 的外部视觉位姿接口。该接口名称中的“视觉”表示 MAVROS 的消息接入通道，不要求数据必须由摄像头产生；Odin 的定位结果也通过该通道接入。

当前主系统配置使用 `Fast_Lio` 定位配置项，并在实际飞行启动文件中将对应里程计订阅名重映射到 Odin 里程计话题。因此，这里的 `Fast_Lio` 是主系统已有的定位配置项名称，不能据此判断当前里程计一定由 `agile_ws` 中的 FAST-LIO 算法产生。

Odin 驱动输出的原始里程计与 PX4 经 MAVROS 输出的本地里程计属于不同的数据环节。各功能包使用哪一路数据，应以其启动文件和话题配置为准。部署时，需要保证 Odin 发布端与主系统订阅端的话题名称、命名空间、坐标系和时间戳约定匹配。

## 2. 软件环境

当前机载端安装的编译工具和主要依赖版本，并结合两个工作空间的源码配置及现有 CMake 缓存说明使用关系。表中的版本是当前环境的核对结果，不表示项目要求的最低版本，也不表示已完成两个工作空间的全量重新编译。

### 2.1 ROS 环境与主要组件

当前使用 ROS 1 Noetic，安装路径为 `/opt/ros/noetic`，基础环境入口为 `/opt/ros/noetic/setup.bash`。Odin 驱动源码同时包含 ROS 1 和 ROS 2 构建分支，本项目使用其 ROS 1 分支。

下表列出已安装组件的上游版本，省略 Debian 包版本末尾的发行版及构建时间后缀。

| 组件 | 当前版本 | 主要用途 |
| --- | --- | --- |
| roscpp | 1.17.4 | ROS C++ 节点、话题和服务通信。 |
| tf | 1.13.4 | 坐标变换相关接口。 |
| message_generation | 0.4.1 | 生成自定义消息和服务代码。 |
| cv_bridge | 1.16.2 | ROS 图像消息与 OpenCV 图像之间的转换。 |
| image_transport | 1.12.1 | ROS 图像传输。 |
| pcl_ros | 1.7.4 | ROS 与 PCL 点云处理接口。 |
| mavros / mavros_extras / mavros_msgs | 1.20.1 | 与 PX4 飞控通信，提供飞行状态、定位及控制相关接口。 |

`std_msgs`、`sensor_msgs`、`nav_msgs` 等标准消息包也在源码依赖声明中使用。工作空间内部的自定义消息包需要随对应源码一起编译，不能用系统中的标准消息包替代。

### 2.2 主要第三方库

| 依赖库 | 当前版本 | 已安装的软件包版本 | 使用关系 |
| --- | --- | --- | --- |
| Eigen | 3.3.7 | `libeigen3-dev: 3.3.7-2` | Odin 驱动、飞行控制及规划模块中的矩阵与几何运算。 |
| Boost | 1.71 | `libboost-dev: 1.71.0.0ubuntu2` | 部分规划模块、Livox 驱动等源码声明的基础库依赖。 |
| OpenCV | 4.2.0 | `libopencv-dev: 4.2.0+dfsg-5` | Odin 图像处理、规划中的图像处理模块及视频流模块。 |
| PCL | 1.10.0 | `libpcl-dev: 1.10.0+dfsg-5ubuntu1` | Odin 点云处理、建图与规划。 |
| yaml-cpp | 0.6.2 | `libyaml-cpp-dev: 0.6.2-4ubuntu1` | Odin 驱动的 YAML 配置读取。 |
| libusb | 1.0.23 | `libusb-1.0-0-dev: 2:1.0.23-2build1` | Odin 驱动链接的 USB 通信库。 |
| OpenSSL | 1.1.1f | `libssl-dev: 1.1.1f-1ubuntu2.24` | Odin 驱动构建时要求并链接的库。 |
| GStreamer | 1.16.3 | `libgstreamer1.0-dev: 1.16.3-0ubuntu1.1` | `uav_rtsp_stream` 视频处理与推流。 |
| GStreamer Base 开发库 | 1.16.3 | `libgstreamer-plugins-base1.0-dev: 1.16.3-0ubuntu1.2` | 提供视频流模块使用的 video、app 等接口。 |
| APR | 1.6.5 | `libapr1-dev: 1.6.5-1ubuntu1.1` | Livox 驱动源码通过 `pkg-config` 查询的依赖。 |

Odin 工作空间的现有 CMake 缓存指向系统提供的 OpenCV 4、PCL、Eigen 和 yaml-cpp 配置，并选用了 `/usr/lib/aarch64-linux-gnu` 下的 libusb、OpenSSL 库。这些路径可作为后续核对编译环境的依据。

**不用刻意安装，大部分环境在部署是已经默认安装好了，只有在编译报错的时候，才会去查缺少哪些部分，再对应安装即可。**

## 3. 源码目录说明

### 3.1 工作空间总体结构
```text
/home/agile/
├── agile_ws/                  主系统工作空间
│   ├── src/                   ROS 功能包源码、配置和启动文件
│   ├── scripts/               工作空间级启动脚本
│   ├── build.sh               主系统分模块构建脚本
│   ├── build/                 编译中间文件和 CMake 缓存
│   └── devel/                 编译输出及 ROS 环境文件
└── odin_ws/                   Odin 驱动工作空间
    ├── src/                   Odin 驱动源码及配套 SDK
    ├── build/                 编译中间文件和 CMake 缓存
    └── devel/                 编译输出及 ROS 环境文件
```

`src/` 是源码主体，但主系统的 `build.sh` 和 `scripts/` 位于 `src/` 外，部署时也需要保留。`build/`、`devel/` 是构建生成的目录，其中会记录原机的绝对路径，不应作为可直接迁移的源码使用。

### 3.2 主系统源码目录

主系统源码根目录为 `/home/agile/agile_ws/src`，按定位、任务、感知、规划和飞行控制组织：

```text
/home/agile/agile_ws/src/
├── CMakeLists.txt
├── location/
│   ├── fast_lio/
│   ├── livox_ros_driver2/
│   └── lidar_world_mapper/
├── missions/
│   └── traj_tracking/
├── perception/
│   └── uav_rtsp_stream/
├── planner/
│   └── ego_planner/
│       ├── plan_manage/
│       ├── plan_env/
│       ├── path_searching/
│       ├── traj_opt/
│       ├── traj_utils/
│       ├── swarm_bridge/
│       ├── drone_detect/
│       └── Utils/
│           └── quadrotor_msgs/
└── uav_control_fsm/
```

`location`、`missions`、`perception`、`planner` 是组织源码的分类目录。ROS 功能包名称以各包 `package.xml` 中的 `<name>` 为准，不能只按文件夹名称判断。

下表的相对目录均以 `/home/agile/agile_ws/src` 为起点。

| 相对目录 | ROS 功能包名 | 主要职责 |
| --- | --- | --- |
| `location/fast_lio` | `fast_lio` | 激光雷达与 IMU 融合定位、建图，供选择该定位方案时使用。 |
| `location/livox_ros_driver2` | `livox_ros_driver2` | Livox 激光雷达数据接入，包含 ROS 1 和 ROS 2 启动配置。 |
| `location/lidar_world_mapper` | `lidar_world_mapper` | 接收点云和里程计，将点云转换到世界坐标系，滤波、累积地图并保存 PCD，真机测试还有BUG，暂时不可直接使用。 |
| `missions/traj_tracking` | `traj_tracking_demo` | 参数化轨迹跟踪示例，包括圆形、方形、三角形、六边形和八字轨迹。 |
| `perception/uav_rtsp_stream` | `uav_rtsp_stream` | 读取工业相机并发布 ROS 图像；源码同时保留 RTSP 视频流功能。 |
| `uav_control_fsm` | `uav_control_fsm` | 飞行控制状态机、控制指令处理、外部里程计桥接及飞行路径记录。 |

规划目录 `/home/agile/agile_ws/src/planner/ego_planner` 包含多个 ROS 功能包，其内部划分如下：

| 子目录                    | ROS 功能包名         | 主要职责                         |
| ---------------------- | ---------------- | ---------------------------- |
| `plan_manage`          | `ego_planner`    | 规划管理、重规划状态机及轨迹服务；保存规划节点启动文件。 |
| `plan_env`             | `plan_env`       | 规划使用的栅格地图、射线处理和坐标转换。         |
| `path_searching`       | `path_searching` | A* 路径搜索。                     |
| `traj_opt`             | `traj_opt`       | 多项式轨迹优化。                     |
| `traj_utils`           | `traj_utils`     | 轨迹相关公共工具及规划可视化。              |
| `swarm_bridge`         | `swarm_bridge`   | 多机通信桥接，包含 TCP、UDP 相关节点。      |
| `drone_detect`         | `drone_detect`   | 在深度图像中检测其他无人机。               |
| `Utils/quadrotor_msgs` | `quadrotor_msgs` | 规划相关模块共用的自定义消息。              |

其中，`traj_tracking` 对应包名 `traj_tracking_demo`，`plan_manage` 对应包名 `ego_planner`。后续使用 ROS 工具查找功能包或启动节点时，应使用包名。

### 3.3 主系统的关键文件与脚本

**飞行控制模块**

`/home/agile/agile_ws/src/uav_control_fsm` 中主要文件的分工如下：

| 包内路径 | 用途 |
| --- | --- |
| `src/uav_control_fsm_node.cpp` | 飞行控制状态机节点入口。 |
| `src/control_manager.cpp` | 控制指令处理及运动控制实现。 |
| `src/odometry_bridge.cpp` | 根据定位源配置接收外部里程计，并向 MAVROS 转发外部定位位姿。 |
| `src/uav_path_recorder_node.cpp` | 飞行路径记录节点。 |
| `include/uav_control_fsm/` | 模块头文件与接口定义。 |
| `msg/uav_command.msg` | 自定义无人机控制指令消息。 |
| `config/uav_group.yaml` | 无人机及定位源等参数配置。 |
| `config/pos_smooth_param.yaml` | 位置运动平滑相关配置。 |
| `config/path_recorder.yaml` | 路径记录参数。 |
| `launch/px4.launch` | 现有实机脚本使用的 MAVROS 启动入口。 |
| `launch/exp_single_uav_control_fsm.launch` | 现有实机脚本使用的单机控制状态机启动入口。 |
| `docs/` | 控制模块补充说明，包含串口权限及 MAVROS 重连配置等内容。 |

轨迹任务参数位于 `/home/agile/agile_ws/src/missions/traj_tracking/config/traj_params.yaml`；视频流参数位于 `/home/agile/agile_ws/src/perception/uav_rtsp_stream/config/rtsp_stream.yaml`。规划的实机启动文件位于 `/home/agile/agile_ws/src/planner/ego_planner/plan_manage/launch/exp_single_planner.launch`。

**工作空间级脚本**

以下路径均相对于 `/home/agile/agile_ws`：

| 路径 | 当前脚本的作用 |
| --- | --- |
| `build.sh` | 依次编译飞行控制、EGO 规划、视频流、轨迹跟踪模块。各模块使用单独的构建子目录，并加载主工作空间的 `devel/setup.bash`。 |
| `scripts/exp_scripts/exp_single_mavros_control.sh` | 等待 MAVROS 连接和参数就绪后，启动单机控制状态机。 |
| `scripts/exp_scripts/odin.sh` | 启动 `odin_ros_driver` 的 `odin1_ros1.launch`。 |
| `scripts/sim_scripts/single_px4_mavros_control_plan.sh` | 仿真相关启动脚本；当前内容启动 PX4 / MAVROS 与控制状态机，未单独启动规划节点。 |

当前 `build.sh` 未包含 `location/` 下的三个功能包，也不负责编译 `odin_ws`。因此，该脚本完成所列模块的构建，不等于两个工作空间的全部源码都完成编译。

`/home/agile/start_mavros_odin.sh` 是同时启动两个脚本的外层包装脚本，仍检查图形桌面环境中的 `DISPLAY`；`agile_ws/scripts/exp_scripts` 下的两个脚本可以直接在 SSH 终端中分别执行。具体启动方式见第 7 章。

### 3.4 Odin 驱动源码目录

Odin 工作空间的源码根目录为 `/home/agile/odin_ws/src`，其中包含一个 ROS 功能包 `odin_ros_driver`：

```text
/home/agile/odin_ws/src/
└── odin_ros_driver/
    ├── CMakeLists.txt
    ├── package.xml
    ├── package_ros1.xml
    ├── package_ros2.xml
    ├── src/                   驱动与图像、点云处理源码
    ├── include/               驱动及 SDK 接口头文件
    ├── lib/                   Odin Host SDK 静态库
    ├── config/                设备控制参数及 RViz 配置
    ├── launch_ROS1/           ROS 1 启动文件
    ├── launch_ROS2/           ROS 2 启动文件
    ├── srv/                   自定义服务定义
    ├── script/                ROS 1 / ROS 2 构建辅助脚本
    ├── set_param.sh           参数设置辅助脚本
    ├── README.md
    ├── CHANGELOG.md
    └── RELOCALIZATION_GUIDE.md
```

下表的路径均相对于 `/home/agile/odin_ws/src/odin_ros_driver`。

| 路径 | 用途 |
| --- | --- |
| `src/host_sdk_sample.cpp` | Odin 设备数据接入及 ROS 发布相关实现。 |
| `src/pcd2depth_ros.cpp`、`src/depth_image_ros_node.cpp` | ROS 1 点云转深度图相关实现。 |
| `src/cloud_reprojection_ros.cpp`、`src/cloud_reprojector.cpp` | 点云重投影相关实现。 |
| `src/image_overlay_node.cpp` | 图像叠加节点实现。 |
| `include/lidar_api.h`、`include/lidar_api_type.h` | SDK API 和数据类型声明。 |
| `lib/liblydHostApi_arm.a` | 当前机载 CMake 配置选用的 ARM 版 SDK 静态库。 |
| `lib/liblydHostApi_amd.a` | 源码中同时保留的 AMD64 版 SDK 静态库。 |
| `config/control_command.yaml` | Odin 驱动及相关处理节点加载的参数文件。 |
| `launch_ROS1/odin1_ros1.launch` | 当前实机脚本引用的 Odin ROS 1 启动入口。 |
| `srv/GetAe.srv`、`srv/SetAe.srv` | 自动曝光参数读写服务定义。 |
| `srv/GetAwb.srv`、`srv/SetAwb.srv` | 自动白平衡参数读写服务定义。 |
| `script/build_ros.sh`、`script/build_ros2.sh` | 对应 ROS 版本的构建辅助脚本。 |

`odin1_ros1.launch` 中包含设备接入、点云转深度图、点云重投影和图像叠加节点。因此，Odin 包中除了里程计接口，还包含配套图像与点云处理代码。

`package_ros1.xml` 和 `package_ros2.xml` 是不同 ROS 版本的功能包描述文件，当前使用的是 ROS 1 配置。`script/build_ros.sh` 包含清理构建目录及覆盖 `package.xml` 的操作，具体使用方式将在 Odin 编译章节核对说明。

### 3.5 源码、配置与生成文件的区别

| 文件或目录类型 | 含义及部署时的处理 |
| --- | --- |
| `src/`、`include/` | 节点实现和头文件，需要随功能包保留。 |
| `CMakeLists.txt`、`package.xml` | 编译规则、包名和依赖声明，需要与对应源码一致。 |
| `launch/`、`launch_ROS1/`、`config/` | 启动与参数配置，是系统运行所需内容，不能只复制 C++ 文件。 |
| `msg/`、`srv/` | 自定义消息和服务的源定义；生成后的代码由构建过程产生。 |
| Odin 包内的 `lib/` | 随项目提供的预编译 SDK，属于驱动构建所需内容，应与头文件配套保留。 |
| 工作空间根目录下的 `build/`、`devel/` | 当前机器的构建产物，迁移后按目标环境重新生成。 |
| `fast_lio/Log/`、`fast_lio/PCD/` | 源码树中存在的日志和点云数据目录，整理时应单独核对其内容与用途。 |

后续章节将以这些实际路径为依据，分别说明部署前准备、Odin 编译和主工作空间编译。

## 4. 部署前准备

本章说明编译和启动前需要完成的准备工作。以下命令在机载端终端执行；本章整理过程中仅检查了现有文件和环境，未执行备份、修改配置或重新编译。

### 4.1 确认账号与部署路径

当前部署账号为 `agile`，两个工作空间分别位于：

```text
/home/agile/agile_ws
/home/agile/odin_ws
```

可通过 SSH 登录机载端进行文件检查和编译：

```bash
ssh agile@192.168.1.101
```

登录后核对账号、目录权限和可用资源：

```bash
whoami
id
ls -ld /home/agile/agile_ws /home/agile/odin_ws
df -h /home/agile
free -h
```

当前两个工作空间均属于 `agile` 用户。后续编译使用该普通用户执行，以免生成属于 `root` 的构建文件，影响正常修改和再次编译。

现有脚本默认使用 `$HOME/agile_ws`、`$HOME/odin_ws`，部分配置还包含 `/home/agile` 下的路径。沿用当前目录可减少路径调整；如果更换账号或目录，需要同步核对构建脚本、启动脚本和配置文件中的路径。

### 4.2 保留现有源码和配置

在覆盖源码或调整配置前，保留当前机载文件的备份。主系统除 `src/` 外，还需要保留工作空间根目录下的 `scripts/`、`build.sh`，以及位于工作空间外的 `/home/agile/start_mavros_odin.sh`。

这些备份保存的是当前机载文件状态，不代表出厂原始版本。上述范围不包含 `build/`、`devel/`，也不包含飞控固件或 Odin 设备内部地图；Odin 源码包内的 SDK 静态库会随 `src/` 一起备份。

版本以第 2 章记录为参照。当前 `.bashrc` 已包含 ROS Noetic 和两个工作空间的环境加载语句。迁移到尚未编译的新目录时，`devel/setup.bash` 可能不存在，应在对应工作空间编译成功后再加载，避免将“环境文件尚未生成”误判为 ROS 安装失败。

现有实机启动脚本按以下顺序加载运行环境：
```
source /opt/ros/noetic/setup.bash
source /home/agile/agile_ws/devel/setup.bash
source /home/agile/odin_ws/devel/setup.bash
```

这一组命令用于两个工作空间已有编译结果的运行阶段。后续各编译章节会分别说明构建时需要加载的环境。

### 4.4 确认设备访问条件

当前 `agile` 用户已加入 `dialout`、`video`、`plugdev` 等用户组。连接设备后，应同时核对设备节点是否存在及当前用户是否具有访问权限。

**PX4 飞控串口**

现有 `/home/agile/agile_ws/src/uav_control_fsm/launch/px4.launch` 默认配置为：

```text
fcu_url = /dev/ttyTHS0:921600
```

**Odin USB 连接**

检查 USB 设备和连接拓扑：

```bash
lsusb
lsusb -t
应输出：Bus 002 Device 002: ID 2207:0019 Fuzhou Rockchip Electronics Company hawk
```

当前系统中存在 `/etc/udev/rules.d/99-odin-usb.rules`。设备接入后，还需要通过驱动日志确认连接成功、数据正常输出，不能仅凭规则文件存在判断 Odin 已就绪。

### 4.5 核对运行环境与接口配置

**终端环境**

SSH 终端可用于源码检查、编译和分别启动节点。外层脚本 `/home/agile/start_mavros_odin.sh` 会检查 `DISPLAY`；如果没有图形桌面，应直接使用第 7 章介绍的两个工作空间脚本或手动启动命令。

可检查：

```bash
command -v gnome-terminal
echo "$DISPLAY"
ls -l /home/agile/start_mavros_odin.sh \
  /home/agile/agile_ws/scripts/exp_scripts/exp_single_mavros_control.sh \
  /home/agile/agile_ws/scripts/exp_scripts/odin.sh
```

本次检查确认相关脚本具有执行权限。普通 SSH 会话没有图形显示环境时，运行外层一键脚本会被其 `DISPLAY` 检查中止；两个工作空间脚本可以分别在 SSH 会话中运行。

**ROS 网络与定位接口**

运行前检查当前终端是否残留其他机器的 ROS 网络设置：

```bash
printenv ROS_MASTER_URI ROS_IP ROS_HOSTNAME
```

机载本机运行与跨机器查看话题需要采用相应的网络配置。SSH 连接成功只说明远程终端可达，不能据此判断跨机器 ROS 话题通信已经配置完成。

启动前还需要核对以下配置关系：

| 检查项 | 当前文件中的配置与核对要求 |
| --- | --- |
| 无人机编号 | MAVROS 和实机控制启动文件均默认 `uav_id=1`，对应 `/uav1` 命名空间；两端编号应保持一致。 |
| 定位源 | `uav_group.yaml` 使用 `Fast_Lio` 配置项，实机启动文件将对应里程计订阅名重映射到 `/uav1/odin1/odometry`，其含义见第 1 章。 |
| Odin 发布与主系统订阅 | Odin 启动文件默认使用 `/uav1` 命名空间，里程计与控制节点配置保持一致。 |
| 里程计输出开关 | Odin 的 `control_command.yaml` 当前为 `streamctrl: 1`、`sendodom: 1`，配置允许数据流和里程计输出；设备是否实际发布需启动后验证。 |
| 时间戳 | Odin 当前为 `use_host_ros_time: 0`。后续需检查消息时间戳与主系统超时判断是否匹配，不能仅凭话题有数据判断定位链路正常。 |
| 坐标系与安装关系 | 核对 `uav_group.yaml` 中的坐标系、传感器启用项及安装变换是否对应实际设备。当前配置包含多种传感器项，不能把默认配置直接当作实际安装标定。 |

完成路径、备份、基础环境和配置核对后，即可按后续章节进行编译。设备连接和话题数据的最终验证在系统启动后完成。

## 5. Odin 驱动部署与编译

### 5.1 编译方式与源码位置

确认驱动源码完整位于以下目录：

```text
/home/agile/odin_ws/src/odin_ros_driver
```

首次部署时，将完整的 `odin_ros_driver` 源码包放入该目录，保留 `include/`、`lib/`、`config/`、`launch_ROS1/`、`srv/` 等配套内容。当前机载已经具备该目录，可以在完成第 4 章备份后使用现有源码编译。迁移到另一台机器时，只部署源码和配套文件，由目标机器生成 `build/`、`devel/`。

当前项目通过 `catkin_make` 构建，关键设置如下：

| 设置 | 本章使用值 | 说明 |
| --- | --- | --- |
| 工作空间 | `/home/agile/odin_ws` | 在工作空间根目录执行构建。 |
| ROS 构建分支 | `BUILD_SYSTEM=ROS1` | 显式选择 CMake 中的 ROS 1 分支。 |
| 包描述文件 | `package_ros1.xml` 对应的 `package.xml` | catkin 实际读取 `package.xml`。 |
| C++ 标准 | C++17 | 由项目 `CMakeLists.txt` 设置。 |
| SDK 库 | `lib/liblydHostApi_arm.a` | 当前 ARM 机载配置选择的设备 SDK。 |
| 并行任务数 | `-j2 -l2` | 本章采用的初始构建设置，用于限制并发负载；不是源码硬性要求。 |

原有 `/home/agile/odin_ws/src/odin_ros_driver/script/build_ros.sh` 在构建函数中执行 `cd $WS_DIR`，但脚本内部没有定义 `WS_DIR`，随后会执行相对路径的目录删除。工作目录可能因此不符合预期。本章采用下面明确指定目录的构建命令，不直接调用该辅助脚本。

### 5.2 执行 ROS 1 编译

在机载端以 `agile` 用户执行：

```bash
cd /home/agile/odin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DBUILD_SYSTEM=ROS1 -j2 -l2
```

当前源码已使用 ROS 1 的 `package.xml`，可直接编译。编译成功后，按下一节加载环境并检查结果。

### 5.3 核对编译结果

编译成功后，在当前终端加载 Odin 工作空间环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/agile/odin_ws/devel/setup.bash
rospack find odin_ros_driver
```

预期功能包路径为：

```text
/home/agile/odin_ws/src/odin_ros_driver
```

如果返回其他副本的路径，应先核对终端加载的工作空间，确保后续使用的是本次编译的源码。
这些检查不需要启动设备节点。`rossrv list` 在这里列出的是可找到的服务类型定义，不表示设备端已经提供运行中的服务。

### 5.4 编译未完成时的处理

如果构建报错，先查看终端输出中最早出现的实际错误，再处理末尾的构建失败汇总。

| 现象 | 优先检查内容 |
| --- | --- |
| 找不到 `catkin_make` 或 catkin | 确认已加载 `/opt/ros/noetic/setup.bash`。 |
| 出现 ROS 2 / ament 相关配置错误 | 确认使用 `-DBUILD_SYSTEM=ROS1`，且 `package.xml` 对应 `package_ros1.xml`。 |
| 找不到预编译 `lydHostApi` 库 | 检查源码包内的 ARM SDK 静态库是否完整、可读，核对 CMake 输出中的选库路径。 |
| 找不到 OpenCV、PCL、yaml-cpp、OpenSSL 或 libusb | 对照第 2 章核对相应开发库及 CMake 查找路径，保留具体报错信息。 |
| 编译器进程被终止，日志包含 `Killed` | 结合内存状态或系统日志确认原因；若是内存不足，可将构建参数降为 `-j1 -l1` 后重试。 |
| CMake 报告源目录或缓存路径不匹配 | 检查是否复制了其他路径或机器的 `build/`；备份后移开旧构建产物，再在目标路径重新生成。 |

当前原有辅助脚本中的目录清理问题尚未修改，不能用反复执行该脚本的方式代替错误定位。

## 6. 主 ROS 工作空间编译

### 6.1 编译范围与顺序

主系统工作空间为 `/home/agile/agile_ws`。当前 `build.sh` 按以下顺序分别编译四个模块，各模块使用独立的 `build` 子目录，共用 `/home/agile/agile_ws/devel`，可直接使用build.sh编译，也可以按照6.2章节自定义编译。

| 顺序 | 模块 | 源码相对目录 |
| --- | --- | --- |
| 1 | 飞行控制 | `src/uav_control_fsm` |
| 2 | EGO 规划 | `src/planner/ego_planner` |
| 3 | 视频流 | `src/perception/uav_rtsp_stream` |
| 4 | 轨迹跟踪 | `src/missions/traj_tracking` |

本章沿用上述目录划分，分别给出编译命令。`location/` 下的定位与雷达功能包不在当前脚本的编译范围内；Odin 驱动按第 5 章单独编译。

### 6.2 执行编译

在机载端以 `agile` 用户打开终端，先进入工作空间并加载 ROS：

```bash
cd /home/agile/agile_ws
source /opt/ros/noetic/setup.bash
```

以下命令均在这个终端、这个目录下执行。每个模块编译成功后，通过 `&&` 加载新生成的工作空间环境。

**1. 飞行控制模块**

```bash
catkin_make --source src/uav_control_fsm --build build/uav_control_fsm -j2 -l2 && source devel/setup.bash
```

该模块生成飞行控制节点、里程计桥接相关库及 `uav_command` 消息。轨迹跟踪模块依赖它，应先完成此步骤。

**2. EGO 规划模块**

```bash
catkin_make --source src/planner/ego_planner --build build/planner/ego_planner -j2 -l2 && source devel/setup.bash
```

该步骤编译规划目录中的管理、环境、路径搜索、轨迹优化及相关消息和工具包。

**3. 视频流模块**

```bash
catkin_make --source src/perception/uav_rtsp_stream --build build/perception/uav_rtsp_stream -j2 -l2 && source devel/setup.bash
```

当前该模块的 CMake 缓存记录了 `GST_RTSP_SERVER_LIB-NOTFOUND`，且未发现生成的 `rtsp_server_node`。如果此步骤提示缺少 RTSP Server 头文件或库，需要先补齐对应依赖再编译；只安装 GStreamer 基础库不足以完成该模块构建。

如本次不使用视频流功能，可以暂不编译此模块，继续编译轨迹跟踪，但应将视频流记录为未完成。

**4. 轨迹跟踪模块**

```bash
catkin_make --source src/missions/traj_tracking --build build/missions/traj_tracking -j2 -l2 && source devel/setup.bash
```

目录名为 `traj_tracking`，实际 ROS 包名为 `traj_tracking_demo`。如果提示找不到 `uav_control_fsm`，先确认第 1 步成功且已加载 `devel/setup.bash`。

### 6.3 加载环境与检查结果

编译完成后，新打开的终端需加载：

```bash
source /opt/ros/noetic/setup.bash
source /home/agile/agile_ws/devel/setup.bash
```

检查主要功能包路径：

```bash
rospack find uav_control_fsm
rospack find ego_planner
rospack find traj_tracking_demo
```

预期依次指向：

```text
/home/agile/agile_ws/src/uav_control_fsm
/home/agile/agile_ws/src/planner/ego_planner/plan_manage
/home/agile/agile_ws/src/missions/traj_tracking
```

主要节点的输出位置如下：

| 模块 | 节点程序路径 |
| --- | --- |
| 飞行控制 | `/home/agile/agile_ws/devel/lib/uav_control_fsm/uav_control_fsm_node` |
| 路径记录 | `/home/agile/agile_ws/devel/lib/uav_control_fsm/uav_path_recorder_node` |
| EGO 规划 | `/home/agile/agile_ws/devel/lib/ego_planner/ego_planner_node` |
| 轨迹服务 | `/home/agile/agile_ws/devel/lib/ego_planner/traj_server` |
| 视频流 | `/home/agile/agile_ws/devel/lib/uav_rtsp_stream/rtsp_server_node` |
| 轨迹跟踪 | `/home/agile/agile_ws/devel/lib/traj_tracking_demo/traj_tracking_node` |

`rospack find` 只证明 ROS 能找到功能包，不能单独证明编译成功。应同时确认对应编译命令无错误退出、节点程序已生成。

本次文档核对发现机载已有飞行控制、规划和轨迹跟踪节点产物，视频流节点产物未找到；未实际执行上述编译命令。各节点与 Odin、MAVROS 的联合启动方式将在下一章说明。

## 7. 配置与启动流程

### 7.1 启动前配置

当前实机启动文件已经固定无人机编号、命名空间、串口和里程计重映射关系，启动时不需要再追加 `uav_id:=`、`ns:=` 或 `odom_remap_to_topic:=` 参数。以下命令均以单机配置为准，飞机应保持未解锁。

| 检查项 | 当前配置 |
| --- | --- |
| 飞控串口 | 由 `px4.launch` 内的 `fcu_url` 配置，当前为 `/dev/ttyTHS0:921600`。 |
| 无人机命名空间 | `/uav1`。 |
| Odin 里程计话题 | `/uav1/odin1/odometry`。 |
| 控制节点里程计输入 | 已在实机启动配置中重映射到 `/uav1/odin1/odometry`。 |
| 定位源配置项 | `uav_group.yaml` 中的 `default_location_source: Fast_Lio`。 |
| 遥控器配置 | `no_RC=false`。 |

控制节点连接飞控后，会在手动控制、未解锁条件下核对 `uav_group.yaml` 中的 PX4 参数。如果参数发生修改，当前配置可能请求重启飞控；参数全部匹配时跳过写入和重启。因此，应先核对配置，在未解锁状态下完成启动检查。

### 7.2 加载终端环境

每个新终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/agile/agile_ws/devel/setup.bash
source /home/agile/odin_ws/devel/setup.bash
```

各终端应连接同一个 ROS Master。首个 `roslaunch` 会在需要时启动本机 ROS Master，无需重复启动 `roscore`。

### 7.3 使用 `.sh` 脚本启动

脚本目录为 `/home/agile/agile_ws/scripts/exp_scripts`。建议先在一个终端启动 Odin：

```bash
cd /home/agile/agile_ws/scripts/exp_scripts
./odin.sh
```

该脚本加载两个工作空间环境，然后启动 `odin1_ros1.launch`，输出 `/uav1/odin1/odometry`。保持该终端运行。

再开一个终端启动 MAVROS 和飞行控制状态机：

```bash
cd /home/agile/agile_ws/scripts/exp_scripts
./exp_single_mavros_control.sh
```

该脚本先启动 MAVROS，等待飞控连接及 PX4 参数就绪，确认成功后才启动 `uav_control_fsm`。如果 MAVROS 未就绪，脚本会停止并不会启动控制状态机。

### 7.4 分别启动 Odin、MAVROS 和控制状态机

需要分别查看各节点日志时，可以使用三个终端手动启动。每个终端先按 7.2 节加载环境。

**终端 1：Odin 定位**

```bash
roslaunch odin_ros_driver odin1_ros1.launch
```

**终端 2：MAVROS**

```bash
roslaunch uav_control_fsm px4.launch
```

**终端 3：飞行控制状态机**

```bash
roslaunch uav_control_fsm exp_single_uav_control_fsm.launch
```

手动启动时，先确认 MAVROS 的 `connected` 为 `True`，再启动控制状态机。不要给上述 launch 命令追加已在文件中配置好的参数。

### 7.5 检查定位数据链路

```bash
rostopic hz /uav1/odin1/odometry
rostopic hz /uav1/mavros/vision_pose/pose
rostopic hz /uav1/mavros/local_position/odom
```

三项分别对应 Odin 原始里程计、控制模块转发的外部定位位姿，以及 PX4 经 MAVROS 输出的本地里程计。控制节点的订阅关系可用下面命令核对：

```bash
rosnode info /uav1/uav_control_fsm
```

订阅列表应包含 `/uav1/odin1/odometry`。

### 7.6 启动工业相机并查看 ROS 图像

工业相机由 `uav_rtsp_stream` 读取，当前启动文件已关闭 RTSP，只发布 ROS 图像。先确保摄像头设备节点与配置文件中的 `device` 一致，然后执行：

```bash
roslaunch uav_rtsp_stream rtsp_stream.launch
```

当前配置使用 `/dev/video0`、MJPEG、640 × 480、30 FPS。启动成功后，图像话题为：

```text
/uav1/camera/image_raw
```

查看图像频率：

```bash
rostopic hz /uav1/camera/image_raw
```

使用 `rqt_image_view` 查看画面：

```bash
rqt_image_view /uav1/camera/image_raw
```

如果该命令未直接选中话题，启动 `rqt_image_view` 后，在界面左上角的话题下拉框中选择 `/uav1/camera/image_raw`。本启动方式不需要也不会发布 RTSP 地址。

### 7.7 停止方式

在对应启动终端按 `Ctrl+C`，即可停止该终端启动的 ROS 节点。使用 `.sh` 脚本时，同样在脚本所在终端按 `Ctrl+C`；脚本会清理其启动的子进程。

## 8. 部署验收

按第 7 章启动系统后，在已加载 ROS 和工作空间环境的终端中进行以下检查。本章验收基础通信与定位功能，检查期间保持飞机未解锁。

### 8.1 检查节点与飞控连接

查看当前节点：

```bash
rosnode list
```

按第 7 章的单机配置，应能找到以下基础节点：

```text
/host_sdk_sample
/uav1/mavros
/uav1/uav_control_fsm
/uav1/path_recorder
```

节点出现在列表中后，还应检查对应终端没有反复退出、重启或持续报错。图像、点云处理节点是否实际输出数据，取决于 Odin 配置中的相应功能开关。

检查飞控状态：

```bash
rostopic echo /uav1/mavros/state
```

通过标准：`connected` 持续为 `True`；地面检查期间 `armed` 为 `False`。如果启动阶段发生参数更新导致的飞控重启，应等重连稳定后再判断。观察完成后按 `Ctrl+C` 结束查看。

### 8.2 检查定位话题连接

确认 Odin 话题的发布与订阅关系：

```bash
rostopic info /uav1/odin1/odometry
```

通过标准：消息类型为 `nav_msgs/Odometry`，发布者包含 `/host_sdk_sample`，订阅者包含 `/uav1/uav_control_fsm`。

再检查外部定位位姿：

```bash
rostopic info /uav1/mavros/vision_pose/pose
```

通过标准：消息类型为 `geometry_msgs/PoseStamped`，发布者包含 `/uav1/uav_control_fsm`，订阅者包含 `/uav1/mavros`。如果只有发布者没有 MAVROS 订阅者，需要核对 MAVROS 插件加载情况。

### 8.3 检查数据是否持续更新

依次查看下面三个话题。每项观察一段时间后按 `Ctrl+C`，再执行下一条：

```bash
rostopic hz /uav1/odin1/odometry
rostopic hz /uav1/mavros/vision_pose/pose
rostopic hz /uav1/mavros/local_position/odom
```

通过标准：三个话题均持续收到消息，正常运行期间没有反复断流。记录实际频率即可，不要求三者频率相同，也不将控制状态机的 30 Hz 循环频率作为 Odin 输出频率标准。

检查消息时间戳：

```bash
rostopic echo /uav1/odin1/odometry/header
```

通过标准：`stamp` 持续更新，不长期为零、不停在同一个值。当前 Odin 配置使用设备时间，应结合主系统日志核对时间基准；若出现定位超时或时间异常，需继续检查，不能仅凭话题频率正常判定通过。

### 8.4 检查位置与姿态变化

分别在两个终端查看 Odin 和 MAVROS 的位姿：

```bash
rostopic echo /uav1/odin1/odometry
```

```bash
rostopic echo /uav1/mavros/local_position/odom
```

在飞机未解锁的地面状态下，先保持静止，再小幅平移或转动整机，检查以下现象：

| 检查方式 | 通过标准 |
| --- | --- |
| 保持静止 | 位置和姿态没有明显持续漂移或突跳，不出现 `nan`、`inf` 等无效数值。 |
| 小幅平移 | 两路位置均随实际运动连续变化，方向与各自约定的坐标系一致。 |
| 小幅转动 | 两路姿态均有相应变化，没有明显冻结或无原因跳变。 |
| 停止移动 | 输出恢复稳定，控制节点没有持续报告定位丢失或超时。 |

Odin 原始里程计与 PX4 本地里程计可能使用不同的原点或坐标变换，不要求位置数值逐项相等。验收重点是运动对应关系、连续性，以及配置的坐标变换是否正确。话题有消息但位姿不随运动变化，仍不能判定定位正常。

### 8.6 验收结论

以下项目全部满足，可记录为“Odin 与主系统基础部署验收通过”：

1. 所选基础模块编译成功，节点能够正常启动。
2. MAVROS 与飞控连接稳定。
3. Odin 里程计能够进入控制节点，并转发至 MAVROS。
4. PX4 本地里程计持续输出，位置与姿态随实际运动正常变化。
5. 检查期间没有持续定位异常、通信中断或节点反复退出。

记录验收日期、通过的模块及未完成项即可。本章提供验收方法，本次文档编写未实际启动系统或执行上述验收；基础部署通过也不等于已经完成自主飞行测试。

## 9. 常见问题排查

### 9.1 编译失败或找不到功能包

| 现象 | 检查与处理 |
| --- | --- |
| `catkin_make: command not found` | 先执行 `source /opt/ros/noetic/setup.bash`，再按对应章节编译。 |
| 找不到工作空间的 `devel/setup.bash` | 检查路径及编译是否成功。首次部署尚未生成该文件时，先加载 ROS 基础环境完成编译。 |
| `Resource not found`，或 `roslaunch` 找不到包 | 加载对应工作空间环境，再用 `rospack find 包名` 确认路径。 |
| 包能找到，但节点程序找不到 | 检查 `devel/lib/包名/` 下是否生成对应程序，确认该模块编译成功。 |
|  |  |
| CMake 缓存指向其他源码路径 | 检查是否迁移了旧 `build/`。备份后移开旧构建产物，在当前路径重新编译。 |
| 编译器报 `Killed` | 查看内存及系统日志。若确认内存不足，将编译参数降为 `-j1 -l1` 后重试。 |
| 编译目录出现权限错误 | 检查目录和文件所属用户；主工作空间使用 `agile` 用户编译，避免用 `sudo catkin_make` 继续生成 root 所属文件。 |

检查主要功能包路径时可执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/agile/agile_ws/devel/setup.bash
source /home/agile/odin_ws/devel/setup.bash
rospack find odin_ros_driver
rospack find uav_control_fsm
```

主系统各模块使用独立构建目录。某个模块成功或总脚本运行到末尾，不能代替其他模块的构建检查；出现问题时，按第 6 章分别编译并查看当前模块的报错。

### 9.2 Odin 启动失败或没有里程计

**命名空间配置错误**

当前 `odin1_ros1.launch` 已在启动文件中配置 `/uav1` 命名空间，启动时不需要追加命名空间参数。若出现命名空间解析错误，先确认使用的是当前工作空间中的启动文件。

**设备没有连接或话题没有数据**

依次检查 USB 设备、连接拓扑和里程计话题：

```bash
lsusb
lsusb -t
rostopic info /uav1/odin1/odometry
rostopic hz /uav1/odin1/odometry
```

如果系统没有识别设备，先检查供电、数据线和 USB 接口；如果设备已经识别但驱动报访问失败，检查设备访问权限及 `/etc/udev/rules.d/99-odin-usb.rules` 是否匹配当前设备。

如果节点在运行但没有里程计，查看 Odin 终端中的连接与 SDK 报错，并核对 `/home/agile/odin_ws/src/odin_ros_driver/config/control_command.yaml` 中的 `streamctrl` 和 `sendodom` 是否为 `1`。话题名称出现在列表中，不代表设备已经持续提供数据。

**SDK 或动态库错误**

编译阶段找不到 `lydHostApi` 时，检查 `/home/agile/odin_ws/src/odin_ros_driver/lib/liblydHostApi_arm.a` 是否完整。启动阶段出现 `error while loading shared libraries` 时，可查看依赖解析结果：

```bash
ldd /home/agile/odin_ws/devel/lib/odin_ros_driver/host_sdk_sample
```

重点查看是否存在 `not found`，按具体库名核对安装位置和运行环境。不要将 AMD64 版 SDK 或其他架构的库用于 ARM 机载程序。

### 9.3 MAVROS 无法连接或反复断开

先检查实际串口和当前用户组：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null
id
```

| 现象 | 检查与处理 |
| --- | --- |
| 找不到 `/dev/ttyACM0` | 检查飞控是否上电、USB 数据连接是否正常，以及串口是否被枚举为其他名称。 |
| 串口存在但报 `Permission denied` | 对照设备节点的所有者、用户组和权限检查访问条件；新增用户组权限后需重新登录生效。 |
| 串口可以打开但 `connected: False` | 检查 `fcu_url` 的设备路径、波特率、飞控端通信配置，以及是否有其他程序占用串口。 |
| 控制节点启动后飞控短暂断开 | 查看是否有参数修改和飞控重启日志。若是启动配置触发重启，等待重连；持续反复断开仍需排查。 |
| 飞控重启后无法恢复连接 | 检查重新枚举后的串口路径是否改变，并确认 MAVROS 重启日志。 |

状态检查命令：

```bash
rostopic echo /uav1/mavros/state
```

当前源码中的 `99-px4-acm0.rules` 只针对 USB 标识 `3185:0038`。使用该规则前应核对实际设备；源码内存在规则文件，不代表系统已经安装或应用它。

### 9.4 Odin 有数据，主系统没有定位

按数据链路逐段检查：

| 检查结果 | 优先排查位置 |
| --- | --- |
| `/uav1/odin1/odometry` 有数据，控制节点没有订阅 | 核对当前启动文件和完整话题名。 |
| 控制节点已订阅，但 `/uav1/mavros/vision_pose/pose` 没有输出 | 检查定位源配置、控制节点初始化是否完成及其终端报错。 |
| 外部定位位姿有输出，MAVROS 没有订阅 | 检查 MAVROS 节点、命名空间及外部视觉定位插件是否加载。 |
| MAVROS 收到外部定位，本地里程计异常 | 检查飞控连接、外部定位融合参数、消息时间戳及坐标关系。 |
| 三路话题均有频率，但位置不动或突跳 | 按第 8 章检查实际位姿变化，不能仅以消息频率判断定位正常。 |

查看控制节点的实际订阅列表：

```bash
rosnode info /uav1/uav_control_fsm
```

控制节点应订阅 `/uav1/odin1/odometry`。如果订阅关系不一致，先确认使用的是当前 `exp_single_mavros_control.sh` 及其配套启动文件。

### 9.5 启动脚本与远程终端问题

**脚本提示需要 DISPLAY**

只有外层脚本 `/home/agile/start_mavros_odin.sh` 会检查 `DISPLAY`。普通 SSH 会话中如果没有图形桌面，应按第 7 章分别执行 `odin.sh` 和 `exp_single_mavros_control.sh`，或使用手动启动命令。

**手动启动正常，一键脚本失败**

对比脚本中的命令与手动成功执行的命令，重点检查当前启动文件中的命名空间、里程计话题、飞控串口及环境加载路径。脚本内容以机载实际文件为准。

**SSH 可以登录，但其他电脑看不到 ROS 话题**

检查两端 ROS 网络环境：

```bash
printenv ROS_MASTER_URI ROS_IP ROS_HOSTNAME
```

确认两端连接同一个 ROS Master，且节点公布的地址能被对端访问。SSH 端口连通不代表 ROS 节点间通信已经建立。

出现问题时，保留失败的完整命令、最早的具体报错和相关话题状态，先定位故障所在模块，再调整对应配置。
