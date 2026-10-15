# 使用 GDB 调试 single_drone_interactive.launch 中的节点

`single_drone_interactive.launch` 会启动两个 ego_planner 相关节点：
1. **ego_planner_node**（规划器主节点，在 `advanced_param.xml` 中）
2. **traj_server**（轨迹服务器，在 `run_in_sim.xml` 中）

任选下面一种方式即可用 GDB 调试。

---

## 方式一：在 launch 里给节点加 `launch-prefix`（推荐）

只调试**一个**节点时，直接让该节点用 GDB 启动。

### 1. 用 Debug 编译（保留符号、便于断点）

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Debug
# 或 catkin build ego_planner -DCMAKE_BUILD_TYPE=Debug
source devel/setup.bash
```

### 2. 修改 launch，给要调试的节点加 GDB

**调试规划器主节点 `ego_planner_node`：**

编辑 `launch/include/advanced_param.xml`，找到：

```xml
<node pkg="ego_planner" name="uav$(arg uav_id)_planner_node" type="ego_planner_node" output="screen">
```

改成（任选其一）：

```xml
<!-- 在当前终端用 GDB 运行，需先 run -->
<node pkg="ego_planner" name="uav$(arg uav_id)_planner_node" type="ego_planner_node" output="screen" launch-prefix="gdb -ex run --args">

<!-- 或：在新 xterm 窗口里开 GDB（方便看日志） -->
<node pkg="ego_planner" name="uav$(arg uav_id)_planner_node" type="ego_planner_node" output="screen" launch-prefix="xterm -e gdb -ex run --args">
```

**调试轨迹服务器 `traj_server`：**

编辑 `launch/include/run_in_sim.xml`，找到：

```xml
<node pkg="ego_planner" name="uav$(arg uav_id)_traj_server" type="traj_server" output="screen">
```

改成：

```xml
<node pkg="ego_planner" name="uav$(arg uav_id)_traj_server" type="traj_server" output="screen" launch-prefix="gdb -ex run --args">
```

### 3. 正常启动 launch

```bash
roslaunch ego_planner single_drone_interactive.launch
```

带 `launch-prefix="gdb -ex run --args"` 的节点会先被 GDB 接管，在终端里可以：
- `Ctrl+C` 停住程序，再输入 `run` 继续，或设断点后 `run`
- 设断点：`b 文件名:行号` 或 `b 函数名`
- 继续：`c`，单步：`n` / `s`，打印变量：`p 变量名`

**如果希望先设断点再跑**，把 `launch-prefix` 改成不带 `-ex run`：

```xml
launch-prefix="gdb --args"
```

这样节点启动后会停在 GDB，你设完断点再输入 `run`。

---

## 方式二：其他节点用 roslaunch 起，要调试的节点单独用 GDB 起

适合「只调试 ego_planner_node，其它照常 roslaunch」。

### 1. 先起除规划器外的部分（需要能单独起 traj_server 等）

可以写一个**不启动 ego_planner_node** 的 launch（例如复制 `run_in_sim.xml` 成 `run_in_sim_no_planner.xml`，注释掉 `advanced_param.xml` 的 include，或单独起 traj_server）。  
这里为简单起见，用「两个终端」做法：

**终端 1：** 正常起完整 launch（所有节点都起）：

```bash
roslaunch ego_planner single_drone_interactive.launch
```

**终端 2：** 先停掉本机上的 ego_planner_node，再用 GDB 起一次（这样 GDB 里的是你控制的进程）：

```bash
# 查进程名
ps aux | grep ego_planner_node

# 停掉由 roslaunch 起的规划器（用上面看到的进程号）
kill <PID>

# 用 GDB 启动 ego_planner_node（必须带和 launch 里一致的参数/remap）
source ~/catkin_ws/devel/setup.bash
rosrun ego_planner ego_planner_node __name:=uav0_planner_node \
  __ns:=/ \
  ~odom_world:=/uav0/mavros/local_position/odom \
  ~mandatory_stop:=/mandatory_stop_to_planner \
  # ... 其它 remap 和 param 很多，建议用方式一
```

因为 remap 和 param 很多，**一般不推荐**手写一长串，更推荐用**方式一**在 launch 里加 `launch-prefix`。

若你坚持用方式二，可以：
- 用 `rosparam dump /tmp/ego.yaml` 在正常 run 一次时导出参数；
- 用 `rosrun --prefix 'gdb -ex run --args' ego_planner ego_planner_node` 再按需加 `_param:=value` 或从 yaml load。

---

## 方式三：不改 launch，用「调试用」launch 文件（可选）

在 `launch/` 下加一个专门用来 GDB 调试的 launch，例如 `single_drone_interactive_gdb.launch`，内容里 include 原来的 `run_in_sim.xml`，但**单独写一行**带 `launch-prefix="gdb -ex run --args"` 的 `ego_planner_node` 节点（并注释掉 `advanced_param.xml` 里对 ego_planner_node 的启动），这样平时用原 launch，调试时用 `single_drone_interactive_gdb.launch`，不用改回改去。

（具体写法就是：复制 `advanced_param.xml` 里该 node 那一行到新 launch，加上 `launch-prefix="gdb -ex run --args"`，并保证参数/remap 一致。）

---

## 常用 GDB 命令（在 GDB 里输入）

- `run`（或 `r`）：运行程序  
- `break 文件:行号`（或 `b planner_manager.cpp:123`）：断点  
- `break 函数名`（如 `b EGOPlanner::plan`）：函数断点  
- `continue`（`c`）：继续执行  
- `next`（`n`）：单步（不进入函数）  
- `step`（`s`）：单步（进入函数）  
- `print 变量名`（`p x`）：打印变量  
- `backtrace`（`bt`）：看调用栈  
- `quit`（`q`）：退出 GDB  

---

## 小结

- **要调 ego_planner_node 或 traj_server**：用**方式一**，在 `advanced_param.xml` 或 `run_in_sim.xml` 里对应 `<node>` 上加  
  `launch-prefix="gdb -ex run --args"`，  
  需要先设断点再跑时用 `launch-prefix="gdb --args"`，再在 GDB 里 `run`。
- **编译**：用 `-DCMAKE_BUILD_TYPE=Debug` 编译，断点和符号更可靠。
- 其它节点（如仿真、octomap）照常由 `single_drone_interactive.launch` 启动即可。
