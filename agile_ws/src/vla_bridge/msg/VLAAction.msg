std_msgs/Header header
uint32 request_id
string frame_id                 # first_frame / enu / body
bool is_absolute
string yaw_unit                 # rad or deg
float32[4] action               # processed pose from /predict
float32[4] action_ori           # body-frame relative [dx, dy, dz, dyaw_rad]
float32 infer_ms
string instruction
