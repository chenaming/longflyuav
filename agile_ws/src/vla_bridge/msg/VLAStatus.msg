std_msgs/Header header
string state                    # INIT / WAIT_SENSORS / READY / REQUESTING / FAILSAFE / HOVER
bool image_ok
bool odom_ok
bool infer_ok
bool publish_enabled
string backend                  # direct_fsm / ego_planner
float32 last_infer_age_sec
string detail
