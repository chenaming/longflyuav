# 实时单帧推理闭环飞行入口

文件位于 NX `/home/nvidia/agile/fly`。该入口默认只读；只有显式 `--execute` 才会请求正常解锁、起飞和降落。旧的 `start_fly.sh` 仍只显示预览，旧电机台架入口保持原有用途。

## 实现的流程

1. 保留现有 ROS、Odin、MAVROS、主机推理服务。检查地面、未解锁、定位/遥控新鲜、单份控制器、推理新鲜和 PX4 失联策略。
2. 发布本次自然语言并重置观测原点；等待本次指令、当前图像的新鲜有效结果。这个地面结果只用于启动条件，不作为空中目标。
3. 通过 `/uav1/uav_control_fsm/external_setpoint_control` 交接设定值发布权：原 FSM 保留定位转发，停止发布 setpoint。新节点独占发布 `/uav1/mavros/setpoint_raw/local`。
4. 先以 30 Hz 发送地面保持设定值至少 2 秒，再请求 OFFBOARD。必须从飞控状态确认切换成功；重新检查有新鲜图像结果，才请求正常解锁。确认 armed=True 后开始起飞。
5. 起飞目标为起飞点的 `z + 1.0 m`，保持起飞时的 X/Y/yaw。位置指令按 0.2 m/s 推进，并限制与当前位置的领先距离；这是指令限制，不是实测最大飞行速度保证。
6. 高度/位置误差在 8 cm 内，实际速度和转速低，并稳定 2 秒后，丢弃起飞前和悬停前的结果，只接受悬停完成后拍摄的新图像结果。
7. 接收每个新模型目标后替换当前目标，持续平滑发送位置和 yaw。目标没有 FIFO 队列，不累计多个旧增量。
8. 满足下面的到达启发式时请求 `AUTO.LAND`。确认飞控进入降落模式后停止外部 setpoint，由 PX4 降落。确认在地面持续 2 秒后可请求一次普通上锁；只有真实状态显示在地面且 armed=False 才记为完成。没有强制解锁或强制上锁。

## 最新图像策略

`latest_observation.py` 只保存一张最新图像，新图像覆盖旧图像。短小的里程计历史只用于匹配该最新图像的时间戳，不保存待推理图像队列。最新图像暂时配不上位姿时等待，不退回较旧图像。

桥接默认推理启动频率上限 5 Hz，同时仅允许一个 HTTP 请求。推理过程中继续覆盖图像，完成后在下个调度时刻取最新图像。实际推理频率受服务器耗时限制，不能保证固定 5 Hz。图像进入请求时最多 0.30 秒旧；结果的原始观测年龄仍必须在 2 秒以内。已经发出的那一帧会完成计算，但超时、过期、旧任务或旧原点的结果不会用于控制。

30 Hz 的控制发送与网络推理解耦。推理变慢不会使控制按推理频率断续发送。策略线程卡住超过 0.30 秒则停止延续旧设定值，让飞控已配置的 OFFBOARD 失联策略接管。

## 自动降落判据和限制

这是用户选择的“连续小动作 + 实际稳定”启发式，**不是视觉确认已经到椅子旁**。

- 至少 8 个不同图像、不同请求的有效结果；图像时间和接收时间跨度都至少 2 秒，结果间隔不超过 1 秒。
- 模型原始位移的三维模长不超过 3 cm，原始 yaw 增量不超过 3°；目标相对拍照位姿及当前位姿也满足这个条件。
- 实际速度不超过 0.08 m/s、转速不超过 3°/s，持续稳定至少 2 秒。
- 悬停后导航至少持续 5 秒。重复一个结果、旧帧、无返回、限幅后的假零动作均不计入。

模型如果在错误的位置连续输出小动作，仍会误判到达并降落；报告始终保留 `semantic_goal_verified=false`。不会因为模型返回一个停止动作就证明识别到了椅子。

默认飞行范围：距起飞点水平半径 3 m，导航高度相对起飞点 0.5–1.5 m，导航最长 60 秒；每次目标相对当前/拍照位姿均要求 XY≤0.35 m、Z≤0.05 m、yaw≤8°。没有避障或落点平整度检测。超过模型动作限制的结果拒绝，不放大原有保护阈值。

推理过期先固定位置悬停，连续 5 秒无有效结果则请求异常降落；该结果记为中止，不记为到达。定位、时钟、任务或发布权异常也会中止。`AUTO.LAND` 未确认时保持设定值并报告需要现场接管；不空中强制停机。遥控器切出 OFFBOARD 后不会自动重新进入。Ctrl+C/SIGTERM 请求降落，不等于立即切断电机。

## 使用

现有 ROS/Odin/MAVROS/推理服务保持运行。不要同时执行模型传递测试或电机台架测试，也不要在飞行中从旧输入终端更改任务或重置原点。

先只读检查（这条不会解锁）：

```bash
cd /home/nvidia/agile
bash ./fly/start_closed_loop.sh --check-only --allow-paired-estimate
```

当前桥接使用 `paired_estimate`，尚未完成完整外参标定。`--allow-paired-estimate` 是明确接受这一限制的选项，不代表标定通过；省略时该模式下预检查会拒绝。

下面才是**真实飞行执行命令**。它与拆桨电机台架测试不同，会请求解锁并上升 1 m；本版本还没有 PX4 SITL 或实飞通过记录，不能把软件测试通过当作可以直接装桨飞行的依据。

```bash
cd /home/nvidia/agile
bash ./fly/start_closed_loop.sh --execute --allow-paired-estimate --instruction "fly to the chair"
```

检查状态和报告：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /vla/flight_status
```

`/home/nvidia/agile/fly/runtime/flight_report.json` 保存状态转换、模型请求、服务响应和飞控状态。原监控页面的 move_command 一栏针对旧台架入口；新入口直接发送 setpoint，请看 `/vla/flight_status` 和 MAVROS setpoint。

## 已做验证

- 21 项 Python 纯逻辑回归：最新单帧、旧任务/旧结果/重复帧、起飞阶段、连续小动作到达、未落地不能上锁、时钟跳变和人工接管。
- 5 项 ROS 适配器测试：默认只读、保留 yaw、屏蔽 yaw_rate、策略停顿不延续旧目标、停止后不解锁、服务不排队；模式/解锁接口全部替换为 Python 测试桩，没有调用真实飞控接口。
- 完整软件策略模拟：初始推理→OFFBOARD→正常解锁→相对起飞1m→稳定→新帧导航→AUTO.LAND→落地→普通上锁。模拟是确定性简化运动，不是 PX4 SITL。
- NX 原控制器 C++ 编译成功。
- NX 未解锁真实 ROS 交接验证：原 setpoint 停发，定位继续，释放后恢复；全程 MANUAL、未解锁，没有调用飞控模式或解锁服务。

可重复执行纯软件测试：

```bash
cd /home/nvidia/agile/fly
python3 ./test_closed_loop.py
python3 ./simulate_closed_loop.py
```

源码：`flight_core.py` 是状态机；`closed_loop_flight.py` 是 ROS/服务/30 Hz 发送适配；`latest_observation.py` 是最新图像缓冲。阈值在 `flight_core.py` 的 `Config` 中，改动后需重新验证。

当前集成备份：`/home/nvidia/agile/fly/runtime/closed_loop_backup_1792036460`。只对 FSM 和桥接做过一次局部重载，ROS、Odin、MAVROS 没有重启。

2026-10-15 验证记录：主机重新启动后，8 秒采集到 8 个不同图像的有效目标，最大观测年龄约 0.626 秒。只读飞行检查当时被 CH9=2000（高位）拦住，油门=999，飞控 MANUAL、未解锁；尚未通过完整实机起飞前检查。

接口依据：[PX4 v1.13 Offboard](https://docs.px4.io/v1.13/en/flight_modes/offboard)、[MAVROS setpoint_raw 坐标转换](https://github.com/mavlink/mavros/blob/1.20.0/mavros/src/plugins/setpoint_raw.cpp)。
