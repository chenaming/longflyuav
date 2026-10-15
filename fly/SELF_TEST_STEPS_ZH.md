# 手控接管原因与分阶段测试

## 已确认的事实（2026-10-15）

- 固件报告 1.13.3。2026-10-15 最近一轮在 45.99 秒从 OFFBOARD 切到 POSCTL；切换当时仍 armed=True，台架程序和独立看门狗均退出控制，没有执行上锁。ROS 历史日志在约 3 秒后才记录到未解锁。
- 模式切换瞬间 CH3 约 1360，随后约 1426–1430；RC3_MIN=1000、RC3_TRIM=1500、RC3_MAX=2000、RC3_DZ=10、MPC_HOLD_DZ=0.1。因此当时不能按“已在标定中位”解释。位置模式下低于中位可能成为下降输入。
- CH6 是模式开关：低位映射 MANUAL、中位映射 POSCTL、高位映射 OFFBOARD。CH9 是解锁/上锁开关，CH10 是急停，不是普通模式接管开关。
- COM_DISARM_LAND=2s，COM_DISARM_PRFLT=10s。PX4 源码中，落地自动上锁取决于内部“本轮曾起飞”的标志和 land_detector 判定；仅因切到 POSCTL 并不会无条件计时两秒停机。
- 拆桨、固定时不会产生实际升力。台架振动/推力与运动不一致，飞控落地/起飞判定不能当作真实飞行状态。当前最符合证据的解释是切模式后识别到地面，触发落地自动上锁；但尚无停机瞬间的 FCU 原因文本，不能把这一解释写成已唯一确认。
- 机载日志列表返回 num_logs=0，FTP 也没有可读日志。参数和保护未修改；空中接管悬停尚未实飞验证。

依据：

- https://github.com/PX4/PX4-Autopilot/blob/v1.13.3/src/modules/commander/Commander.cpp
- https://docs.px4.io/v1.13/en/flight_modes/position_mc
- https://docs.px4.io/v1.13/en/advanced_config/parameter_reference

## 第一次：拆桨记录确切停机原因

保持全部拆桨、可靠固定、人在旁可停止。复用现有服务。若刚开机尚未启动基础服务：

```bash
cd /home/nvidia/agile
source /opt/ros/noetic/setup.bash
source /home/nvidia/agile/agile_ws/devel/setup.bash
python3 ./vla_stack.py start
```

保留已运行的自然语言终端。如果尚无指令终端，再运行：

```bash
cd /home/nvidia/agile
bash ./fly/start_fly.sh
```

输入 `fly to the chair`，先取得一条当前图像的有效预览。刚开机换了 run_id，或者换了指令时，需要在未解锁、MANUAL 下更新八个传递样本：

```bash
cd /home/nvidia/agile
bash ./fly/test_model_chain.sh
```

保持上述输入终端空闲，不再修改任务。

终端一，开始只读记录（不会切模式、不会解锁）：

```bash
cd /home/nvidia/agile
bash ./fly/record_takeover.sh --seconds 180
```

终端二，拆桨台架测试：

```bash
cd /home/nvidia/agile
bash ./fly/test_closed_loop_30s.sh --props-removed --fixed
```

本脚本正常解锁；起飞前 10 秒和随后的 2 秒悬停均为地面计时模拟，不发送 1 米爬升。12 秒后采用新图像循环推理，但目标限制为本次起点 XY 2cm、Z 3mm、yaw 1°。

运行到约 15–20 秒后，仅把油门移到实际标定中位约 1500，再把 CH6 切至位置模式。不要同时动 CH9/CH10。查看终端一是否记录 OFFBOARD/armed=True → POSCTL/armed=True；随后是否出现 landed_state=1、Disarmed by landing、armed=False。不要为保持地面电机转动而增加油门。

必要时可在另一个已加载 ROS 的终端读取实际通道（数组第三项是 CH3）：

```bash
rostopic echo /uav1/mavros/rc/in/channels
```

人工接管会结束本轮台架测试，不算完整 30 秒通过；程序与看门狗不在接管后自动上锁。若地面电机仍转，确认机体在地面、把油门回最低，使用已验证的遥控器上锁操作停止；不要依靠关闭 ROS 停机。

记录器观察到接管后上锁会再记录 3 秒并自动结束；也可 Ctrl+C 结束记录器（它只读）。JSONL 原始记录和 JSON 诊断写到：

```text
/home/nvidia/agile/fly/runtime/takeover_records/
```

诊断只有收到明确的飞控原因文本才标为 confirmed_from_fcu_message。只有约两秒时间差或参数值，不会伪造确认。

## 第二次：完整跑满台架窗口（与接管试验分开）

复位到未解锁、MANUAL；确认原发布权已恢复。若前轮人工接管后还显示 external_setpoint_control_active=1，先确认实际未解锁，再由维护人员恢复发布权，不能在已解锁时重复启动测试。

使用同一记录命令和 30 秒台架命令。全程保持 OFFBOARD，不拨模式；结束后由代码请求 AUTO.LAND，确认地面后普通上锁，不要求油门回低。目标是完整窗口、持续 setpoint、有效新图像不断替换、过期保持而不使用旧目标、最终普通上锁。2026-10-15 这项已实测通过：38个新图像、900/900 setpoint、油门1407–1409、一次普通上锁成功且未强制上锁，详见 BENCH30.md。达到 30 秒与飞到椅子是两种不同结果。

## 上桨后的阶段

1. 先解决或明确记录上述上锁原因，并检查实际油门中位。完成实际定位尺度、机体轴/外参、飞控估计器与手控位置模式的基础验证；软件测试通过不等于这些项目已通过。
2. 上桨前断电，按正常飞行规程安装。带桨测试在适合的空旷场地，由熟练飞手控制并保持人员距离。禁止带桨运行 test_motor_chain.sh、test_closed_loop_bench.sh 或 test_closed_loop_30s.sh；这些是拆桨台架入口，仍有专用强制上锁兜底。
3. 第一架次只验证手控位置模式，不启动自主任务。手控 POSCTL 起飞到约 1 米，在真实中位保持短时悬停，确认位置/高度稳定、真实高度与里程计一致、空中 extended_state 为 IN_AIR(2) 且一直 armed=True；随后人工降落和上锁。如果空中出现 ON_GROUND(1)、自动上锁、明显漂移或异常估计器状态，停止后续自动飞行并分析日志。
4. 第二阶段先用 PX4 SITL/HITL 验证正式 OFFBOARD 起飞、悬停、模式接管、失联和落地上锁，再安排受控实机试验。首个实机 OFFBOARD 试验只应做固定点起飞/悬停与 POSCTL 接管，不让模型导航到椅子。当前正式入口会自动进入模型导航，尚没有独立固定点悬停试飞入口，也没有 PX4 SITL/实飞通过记录，因此当前不要把完整任务入口作为上桨首飞命令。
5. 固定点 OFFBOARD 和空中接管验证通过后，再进行模型目标的逐步导航、到达判据及自动降落验证。当前配对坐标估算、到达启发式和目标语义尚未完整实飞验证；椅子附近飞行不得作为首个验证场景。

当前可执行的正式入口只读检查：

```bash
cd /home/nvidia/agile
bash ./fly/start_closed_loop.sh --check-only --allow-paired-estimate
```

它不会解锁。正式入口要求从地面未解锁开始；不能先手控起飞，再运行该入口来接管空中的飞机。单独把遥控器拨到 OFFBOARD，也不等于已经启动了新任务或有持续目标流。

在真正空中接管时，油门中位后选择 POSCTL；程序确认实际模式后停发目标，不抢回 OFFBOARD、不发送上锁。该行为已做软件测试和拆桨切模式验证，但真实空中悬停接管仍需上述阶段验证。MANUAL 的中位油门不等于高度保持；CH9/CH10 的上锁/急停操作不能替代普通模式接管。
