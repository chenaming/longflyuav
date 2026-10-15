# lidar_world_mapper

支持三种模式：

| mode | 输入 | 说明 |
|------|------|------|
| **deskew**（默认） | 原始 Livox + IMU + 外部 Odom | **不依赖 FAST-LIO**，本包 IMU 去畸变 |
| body_odom | 已去畸变 body 点云 + Odom | 兼容接 FAST-LIO 输出 |
| already_world | 已在 world 的点云 | 只滤波建图 |

## deskew 数据流（不启 FAST-LIO）

```
/uav1/livox/lidar  ─┐
/uav1/livox/imu    ─┼► IMU积分去畸变 ─► 外参/Odom落到 world ─► 滤波融合 ─► PCD
外部 odom          ─┘   (如 /uav1/mavros/local_position/odom)
```

说明：

- **去畸变**只消除一帧扫描内的运动拖影，不输出全局定位。
- **绝对位姿**必须来自 PX4/VIO/其它里程计，否则世界地图会对不齐。
- 启动后约 1s 用 IMU 静止段估计陀螺零偏与重力；建议先静置再飞。

## 运行

```bash
mkdir -p ~/maps
# 雷达驱动 + mavros（示例），不要开 fast_lio
roslaunch lidar_world_mapper mapper.launch \
  uav_id:=1 \
  pcd_path:=$HOME/maps/lidar_world_map.pcd
# 默认已订阅 /uav1/mavros/local_position/odom；多机改 uav_id 即可
```

手动存图：

```bash
rosservice call /lidar_world_mapper/save_map
```

## 关键参数

- `extrinsic_T/R`：雷达在 IMU 下位姿（与 mid360 官方/原 FAST-LIO 配置一致）
- `odom_child`：`imu` 表示 odom 是 IMU 在 world 下的位姿；若是雷达则设 `lidar`
- `enable_translation_deskew`：平移补偿不稳定时可先设 `false`，只做旋转去畸变

## 干净地图建议

1. 时间同步：雷达/IMU/odom 尽量同一时钟域  
2. `min_range` / `z_min`/`z_max` 裁近距与高度飞点  
3. `map_voxel` + SOR/ROR 融合去噪  
4. `max_speed_for_insert` 限制急动时入库  
5. 外参、odom 坐标系搞错会造成系统性拖影——这比滤波更致命  

详细滤波项见 `config/mapper.yaml`。
