# fly：自然语言输入与 OpenVLA 坐标返回

安装目录：`/home/nvidia/agile/fly`。适用当前 NX 上已运行的 ROS Noetic 和 `/vla_direct_bridge`。

在 NX 终端运行一次：

```bash
bash /home/nvidia/agile/fly/start_fly.sh
```

出现 `请输入指令 >` 后输入你自己的中文或英文自然语言，回车。
例如：`向前移动一点，然后悬停。`。得到结果后可在同一终端继续输入。
`q` 退出输入终端；ROS、MAVROS、桥接和本地推理转发继续运行。

每次显示目标的 x/y/z（米）、yaw（度）、坐标系、相对拍照位姿偏移、请求编号、桥接检查结论和记录路径。
原始 `action` / `action_ori`、观测输入和换算记录都保存到 `runtime/result_*.json`；最近一次结果为 `runtime/latest.json`。

## 下一步：未解锁模型目标到控制器

`test_model_chain.sh` 复用已检查的原有 `vla_acceptance.py --stage model` 有限测试，要求现有台架保护 true/false、飞控未解锁 MANUAL/Standby、当前指令有新鲜的有效模型目标，以及没有其他 move_command 发布者。

先检查、完全不切模式或发目标：

```bash
bash /home/nvidia/agile/fly/test_model_chain.sh --check-only
```

执行有限的未解锁测试：

```bash
bash /home/nvidia/agile/fly/test_model_chain.sh
```

执行时保持原输入终端空闲、不要更改指令。测试会短暂切 OFFBOARD，取 8 个真实模型目标，分别限幅为 XY≤5cm、Z≤1cm、yaw≤3° 的中间点，再发布到现有控制器并检查实际 setpoint 数值，最后恢复 MANUAL。它不会调用解锁服务，也不会修改原有台架保护或重新启动 ROS/MAVROS。

原始目标与实际中间点分别保存在 `runtime/model_chain_report.json`；日志为 `runtime/model_chain.log`。检查过期报告不会作为本次成功证据。
截至本入口安装时，只执行了 `--check-only`，通过语法和实时预条件检查，尚未执行此入口的控制传递测试。
该有限测试通过也只证明未解锁消息传递，不能证明电机转动或自主到达椅子。

## 已通过模型传递后的五秒电机台架

用户已在现场确认全部拆桨、可靠固定、可观察和停止。此入口仍要求显式传入物理状态标志。
先读取 [MOTOR_TEST.md](MOTOR_TEST.md) 了解有限测试范围。

只读检查：

```bash
bash /home/nvidia/agile/fly/test_motor_chain.sh --check-only
```

已满足现场条件时亲自执行：

```bash
bash /home/nvidia/agile/fly/test_motor_chain.sh --props-removed --fixed
```

该命令会正常请求解锁，电机测试窗口约五秒，然后上锁并恢复默认保护。整个命令可能持续30–50秒，操作者应全程观察。
测试目标为受限模型中间点（XY≤2cm、Z≤3mm、yaw≤1°），不会验证实际到达椅子。
旧电机模块的中文任务写死问题已在 `fly` 内副本修正；旧文件保持原样。
截至安装时，新入口已通过六项本地回归检查、NX编译/语法检查及只读预检查；尚未执行新入口的解锁测试。

## 当前链路

自然语言 → `/vla/instruction` → 现有桥接配对真实 Odin 图像/位姿和 MAVROS 位姿 → `127.0.0.1:15017/predict` → fly 转发 → `192.168.1.10:5008/predict` → 主机 `10.62.25.215:5007/predict` → 原有桥接坐标换算 → fly 终端显示。

现有桥接源文件保持原样。`relay.py` 只补回原先缺失的本地 HTTP 入口，POST 请求正文和服务器返回内容保持原样。
转发关闭环境代理，仅监听 NX 本机回环地址。`start_fly.sh` 会复用已经运行的 fly 转发，拒绝覆盖其他占用 15017 的服务。

`fly_cli.py` 不启动 ROS、MAVROS、Odin、控制器或第二份桥接。它要求现有桥接为 `enable_publish=false` 和 `preview_pair_alignment=true`，并检查无人机控制目标话题没有发布者。
它只发布语言指令和观测原点重置；读取桥接的 `COORD_TRACE` 和带请求编号的 `/vla/validated_prediction`。
连续输入和重复输入都等待新图像结果，使用 run_id、request_id、instruction、image_stamp 关联坐标和检查证据。

## 如何理解验证结果

- “返回坐标”证明真实观测经主机模型产生了坐标，且服务器 action 与 action_ori 的组合检查通过。
- “桥接预览检查通过”证明同一次请求的坐标通过桥接限制。控制器自己的台架范围仍更小。
- “桥接预览检查未通过”仍会显示原始模型坐标及原因，不把候选坐标伪装成可执行目标。
- 配对坐标仍为原有桥接的预览估算，完整机体外参没有标定。
- 当前工具不会向 `/uav1/move_command` 发布，也不会解锁、切模式或驱动电机。
- 自然语言被接收不代表模型正确理解方向或目标物体。测试中“向左”曾产生主要向 X 正方向的偏移；语义执行尚不可靠。

当前桥接在后台持续推理。退出 fly 只关闭交互输入，不终止后台桥接。
无需重新启动现有整套程序。若现有桥接退出，应先检查 supervisor 当前状态，避免重复启动。

## 文件

| 文件 | 用途 |
| --- | --- |
| `start_fly.sh` | 复用/启动单份本地转发，进入交互输入 |
| `relay.py` | 补回 15017 本地推理转发 |
| `fly_cli.py` | 输入、结果关联、坐标显示、JSON 保存 |
| `trace_result.py` | 校验坐标、单位、请求对应关系与检查结论 |
| `runtime/relay.log` | 转发运行日志 |
| `runtime/latest.json` | 最近一次完整结果 |

依赖来自已安装 ROS Noetic、agile_ws 消息包和 Python 3 标准库；不安装新 ROS 软件包。
旧桥接必须包含交接记录中的 `COORD_TRACE`、`validated_prediction`、`run_id` 逻辑。
