# 拆桨计时台架测试

用户要求拆桨执行，并希望不能到达 1 m 时在 10 秒后推进。正式飞行逻辑没有把超时当作到达高度：实飞超时仍中止起飞。

独立台架入口：

```bash
cd /home/nvidia/agile
bash ./fly/test_closed_loop_bench.sh --props-removed --fixed
```

该入口顺序执行：

1. `bench_timed_preview.py`：保持未解锁 MANUAL，读取当前任务的新鲜图像推理，计时 10 秒。没有发送 1 m 目标，没有调用切模式或解锁服务。这一阶段明确记为模拟起飞。
2. 记录阶段截止时间，等待截止时间以后拍摄的新图像推理，历史结果全部忽略。
3. `test_model_chain.sh`：既有未解锁模型传递测试，验证本轮 8 个模型目标与控制 setpoint。
4. `test_motor_chain.sh --props-removed --fixed`：既有 5 秒实际电机台架，正常解锁；目标限制 XY≤2 cm、Z≤3 mm、yaw≤1°，独立看门狗保持有效。报告检查真实上锁和保护恢复。

该序列验证实时模型数据和既有受限电机链路。它没有执行 `closed_loop_flight.py --execute`，不能证明新实飞入口已经通过 1 m 起飞、导航、落地测试。10 秒模拟阶段不会让拆桨飞机因持续高度误差而要求越来越大的爬升输出。

前置条件仍是遥控器信号有效、油门 CH3<1100、CH9<1300、拆桨固定、现场可观察和停止。任何阶段失败都会停止后续阶段，不自动重试解锁。

报告：`fly/runtime/bench_timed_preview.json`、`fly/runtime/model_chain_report.json`、`fly/runtime/motor_chain_report.json`。
