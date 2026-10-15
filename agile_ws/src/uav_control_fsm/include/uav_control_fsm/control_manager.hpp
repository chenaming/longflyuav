/*********************************************************
control_manager.hpp: 控制管理器，将mavros的接口进行封装，根据控制指令生成 setpoint。
控制指令包括：
1. 悬停：hover_setpoint_set
2. 起飞：takeoff_setpoint_set
3. 位置控制：惯性系 机体系
4. 速度控制：惯性系 机体系
5. 轨迹控制：trajectory_set
*********************************************************/
#ifndef UAV_CONTROL_FSM_CONTROL_MANAGER_HPP
#define UAV_CONTROL_FSM_CONTROL_MANAGER_HPP

#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <uav_control_fsm/uav_command.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <cmath>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

namespace uav_control_fsm {

/** Move_mode=0 惯性系位置平滑参数（由 FSM 从 ROS 参数加载后注入） */
struct PosSmoothParams
{
    bool enable{true};
    double max_vel{1.0};          // v_cruise (m/s)
    double max_accel{0.8};        // 制动曲线加速度上限 (m/s^2)
    double arrival_dist{0.1};     // 到达判定距离 (m)，锁终点并置零速度
    double lookahead_dist{0.5};    // 轨迹点前视距离 (m)
    double vel_deadband{0.05};      // 速度死区 (m/s)，低于此视为已停
};

/** 控制模块：根据指令/悬停/起飞生成 setpoint，由状态机在 OFFBOARD 下调用。
 *  每台 UAV 的 ControlFSM 持有一个 ControlManager 实例（非 static），便于后续按机保存控制状态。 */
class ControlManager
{
public:
    void setPosSmoothParams(const PosSmoothParams& params) { pos_smooth_params_ = params; }
    void resetPosSmooth();

    /** 根据 control_command 的 Move_mode 填 setpoint（对应多种 *_set） */
    void control_command_set(mavros_msgs::PositionTarget& out,
                             const uav_control_fsm::uav_command& cmd,
                             const nav_msgs::Odometry& odom);
    /** 悬停 setpoint */
    void hover_setpoint_set(mavros_msgs::PositionTarget& out, const Eigen::Vector4d& hover_pose);
    /** 起飞 setpoint：只填 x,y,yaw 与 type_mask，z/velocity.z 由 FSM 在调用后设 */
    void takeoff_setpoint_set(mavros_msgs::PositionTarget& out, const Eigen::Vector4d& start_pose);

private:
    struct PosSmoothState
    {
        bool active{false};
        bool locked{false};
        Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
    };

    void current_pos_hover_set(mavros_msgs::PositionTarget& out, const nav_msgs::Odometry& odom);
    void ned_pos_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void ned_pos_xyz_smooth_set(mavros_msgs::PositionTarget& out,
                                const uav_control_fsm::uav_command& cmd,
                                const nav_msgs::Odometry& odom);
    void ned_pos_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void ned_vel_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void ned_vel_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void ned_pos_z_vel_xy_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void body_pos_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                          const nav_msgs::Odometry& odom);
    void body_pos_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                               const nav_msgs::Odometry& odom);
    void body_vel_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void body_vel_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);
    void body_pos_z_vel_xy_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                               const nav_msgs::Odometry& odom);
    void trajectory_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd);

    PosSmoothParams pos_smooth_params_;
    PosSmoothState pos_smooth_state_;
};

} // namespace uav_control_fsm

#endif
