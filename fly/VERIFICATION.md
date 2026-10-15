# 安装与验证记录

2026-10-13，NX `/home/nvidia/agile/fly` 已安装新代码。

## 已完成

- 登录 NX 后读取最新版 `vla_direct_bridge.py`、`vla_stack.py` 和当前 ROS 状态。
- 找到旧桥接 `127.0.0.1:15017/predict` 连接拒绝的直接原因：本地端口没有服务。
- 确认 NX 能通过 Windows `192.168.1.10:5008` 到达主机 `/predict`，GET 返回预期的 405。
- 在 fly 中补上回环转发，复用现有桥接，未重启原有无人机程序。
- 本地 5 项测试通过：坐标不被再次换算、不同请求证据不混用、拒绝结果保持拒绝、无效数值/组合误差/控制配置拒绝、HTTP 转发请求和响应正文保留。
- NX Python 编译和 Bash 语法检查通过。
- 第一条中文实机推理返回后，在同一个交互进程中再次输入相同指令和另一条不同中文指令，均取得关联正确的坐标。

## 同一进程连续输入结果

| 指令 | request_id | x (m) | y (m) | z (m) | yaw (deg) | 桥接检查 |
| --- | --- | --- | --- | --- | --- | --- |
| 向前移动一点，然后悬停。 | 6962 | 0.245844 | 0.104796 | 0.011117 | -4.993 | 通过 |
| 向左移动一点，然后悬停。 | 6963 | 0.250902 | -0.032836 | 0.010406 | -5.269 | 通过 |

run_id：`chair-00866a5ee82c`。坐标系：`map`。
上述为真实模型输出；“向左”一条相对拍照位姿 `dx=+0.264748 m, dy=-0.024614 m`，没有表现出预期的明显左移，语言方向语义尚不能视为可靠。

## 程序与飞控状态

验证前后同一 supervisor PID 3188、同一 MAVROS PID 3320、同一桥接 PID 3427。
桥接原文件 SHA256 保持 `11f95f45e66b2798149e3dd7ecb5de405cca539f15d95562821b28ea5c083b7f`。

验证后：connected=True，armed=False，mode=MANUAL，system_status=3。
`bench_disarmed=true`、`bench_allow_armed=false`，`/uav1/move_command` 无发布者。

证明范围为：实机观测 → 主机推理 → 坐标返回 → 桥接预览检查。
尚未验证模型方向语义正确、控制器执行、电机输出或实际到达目标。

## 你亲自验证的下一步

在 NX 的终端执行：

```bash
bash /home/nvidia/agile/fly/start_fly.sh
```

出现提示后输入你自己的自然语言，回车。完整记录在 `/home/nvidia/agile/fly/runtime/latest.json`。
