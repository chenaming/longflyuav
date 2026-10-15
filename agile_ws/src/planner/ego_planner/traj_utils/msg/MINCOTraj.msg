int16 uav_id
int32 traj_id
time start_time
float32 des_clearance # 期望的与其他飞机之间的最小距离

uint8 order
float32[3] start_p
float32[3] start_v
float32[3] start_a
float32[3] end_p
float32[3] end_v
float32[3] end_a
float32[] inner_x # 中间点数组（x方向）
float32[] inner_y
float32[] inner_z
float32[] duration # 每段轨迹所需时间