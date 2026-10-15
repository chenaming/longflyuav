std_msgs/Header header

# 移动模式 
uint8 Move_mode
## 移动子模式 
uint8 pos_xyz = 0               ## 惯性系 位置 + 偏航角度控制
uint8 pos_xyz_yaw_rat = 1       ## 惯性系 位置 + 偏航角速度控制 到达目标位置后，会绕自身一直旋转（可用于指定区域搜索功能）
uint8 vel_xyz = 2               ## 惯性系 速度 + 偏航角度控制
uint8 vel_xyz_yaw_rat = 3       ## 惯性系 速度 + 偏航角速度控制
uint8 pos_z_vel_xy = 4          ## 惯性系 定高速度 + 偏航角度控制

uint8 body_pos_xyz = 5          ## 机体系 位置 + 偏航角度控制
uint8 body_pos_xyz_yaw_rat = 6  ## 机体系 位置 + 偏航角速度控制
uint8 body_vel_xyz = 7          ## 机体系 速度 + 偏航角度控制
uint8 body_vel_xyz_yaw_rat = 8  ## 机体系 速度 + 偏航角速度控制
uint8 body_pos_z_vel_xy = 9     ## 机体系 定高速度 + 偏航角度控制
uint8 traj = 10                 ## 轨迹控制

# 控制量
geometry_msgs/Point pos_ref     ## 期望位置，单位：m
geometry_msgs/Point vel_ref     ## 期望速度，单位：m/s
geometry_msgs/Point acc_ref     ## 期望加速度，单位：m/s^2
float32 yaw_ref                 ## 期望角度，单位：度（°）
float32 yaw_rate_ref            ## 期望角速度，单位：度/秒(°/s)