/*********************************************************
control_manager.cpp: setpoint 生成与 Move_mode=0 软件平滑实现。
*********************************************************/
#include "uav_control_fsm/control_manager.hpp"

#include <algorithm>

namespace uav_control_fsm {

namespace {

constexpr uint16_t kMaskPosYaw =
    mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

constexpr uint16_t kMaskPosOnly =
    mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

} // namespace

void ControlManager::resetPosSmooth()
{
    pos_smooth_state_.active = false;
    pos_smooth_state_.locked = false;
    pos_smooth_state_.goal.setZero();
}

void ControlManager::control_command_set(mavros_msgs::PositionTarget& out,
                                         const uav_control_fsm::uav_command& cmd,
                                         const nav_msgs::Odometry& odom)
{
    if (cmd.Move_mode >= 0 && cmd.Move_mode <= 9)
    {
        switch (cmd.Move_mode)
        {
            case 0:
                if (pos_smooth_params_.enable)
                    ned_pos_xyz_smooth_set(out, cmd, odom);
                else
                    ned_pos_xyz_set(out, cmd);
                break;
            case 1: ned_pos_xyz_rate_set(out, cmd); break;
            case 2: ned_vel_xyz_set(out, cmd); break;
            case 3: ned_vel_xyz_rate_set(out, cmd); break;
            case 4: ned_pos_z_vel_xy_set(out, cmd); break;
            case 5: body_pos_xyz_set(out, cmd, odom); break;
            case 6: body_pos_xyz_rate_set(out, cmd, odom); break;
            case 7: body_vel_xyz_set(out, cmd); break;
            case 8: body_vel_xyz_rate_set(out, cmd); break;
            case 9: body_pos_z_vel_xy_set(out, cmd, odom); break;
            default: ned_pos_xyz_set(out, cmd); break;
        }
    }
    else if (cmd.Move_mode == 10)
        trajectory_set(out, cmd);
    else
        current_pos_hover_set(out, odom);
}

void ControlManager::hover_setpoint_set(mavros_msgs::PositionTarget& out, const Eigen::Vector4d& hover_pose)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly;
    out.position.x = hover_pose(0);
    out.position.y = hover_pose(1);
    out.position.z = hover_pose(2);
    out.yaw = hover_pose(3);
}

void ControlManager::takeoff_setpoint_set(mavros_msgs::PositionTarget& out, const Eigen::Vector4d& start_pose)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.position.x = start_pose(0);
    out.position.y = start_pose(1);
    out.position.z = start_pose(2);
    out.yaw = start_pose(3);
}

void ControlManager::current_pos_hover_set(mavros_msgs::PositionTarget& out, const nav_msgs::Odometry& odom)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly;
    out.position.x = odom.pose.pose.position.x;
    out.position.y = odom.pose.pose.position.y;
    out.position.z = odom.pose.pose.position.z;
    out.yaw = tf::getYaw(odom.pose.pose.orientation);
}

void ControlManager::ned_pos_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly;
    out.position.x = cmd.pos_ref.x;
    out.position.y = cmd.pos_ref.y;
    out.position.z = cmd.pos_ref.z;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;
}

void ControlManager::ned_pos_xyz_smooth_set(mavros_msgs::PositionTarget& out,
                                            const uav_control_fsm::uav_command& cmd,
                                            const nav_msgs::Odometry& odom)
{
    const Eigen::Vector3d target(cmd.pos_ref.x, cmd.pos_ref.y, cmd.pos_ref.z);
    const Eigen::Vector3d current(odom.pose.pose.position.x,
                                  odom.pose.pose.position.y,
                                  odom.pose.pose.position.z);

    if (!pos_smooth_state_.active || (target - pos_smooth_state_.goal).norm() > 1e-3)
    {
        pos_smooth_state_.goal = target;
        pos_smooth_state_.active = true;
        pos_smooth_state_.locked = false;
    }

    const Eigen::Vector3d err = pos_smooth_state_.goal - current;
    const double dist = err.norm();

    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosYaw;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;

    const double arrival = std::max(pos_smooth_params_.arrival_dist, 1e-3);
    if (dist <= arrival || pos_smooth_state_.locked)
    {
        pos_smooth_state_.locked = true;
        out.position.x = pos_smooth_state_.goal.x();
        out.position.y = pos_smooth_state_.goal.y();
        out.position.z = pos_smooth_state_.goal.z();
        out.velocity.x = 0.0;
        out.velocity.y = 0.0;
        out.velocity.z = 0.0;
        return;
    }

    const double max_vel = std::max(pos_smooth_params_.max_vel, 1e-3);
    const double max_accel = std::max(pos_smooth_params_.max_accel, 1e-3);
    const double v_brake = std::sqrt(2.0 * max_accel * dist);
    double v_cmd = std::min(max_vel, v_brake);

    const Eigen::Vector3d dir = err / dist;
    const double lookahead = std::max(pos_smooth_params_.lookahead_dist, 1e-3);
    const double step = std::min(lookahead, dist);
    const Eigen::Vector3d sp = current + dir * step;

    if (v_cmd < pos_smooth_params_.vel_deadband)
        v_cmd = 0.0;

    out.position.x = sp.x();
    out.position.y = sp.y();
    out.position.z = sp.z();
    out.velocity.x = dir.x() * v_cmd;
    out.velocity.y = dir.y() * v_cmd;
    out.velocity.z = dir.z() * v_cmd;
}

void ControlManager::ned_pos_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly | mavros_msgs::PositionTarget::IGNORE_YAW;
    out.position.x = cmd.pos_ref.x;
    out.position.y = cmd.pos_ref.y;
    out.position.z = cmd.pos_ref.z;
    out.yaw_rate = cmd.yaw_rate_ref * DEG_TO_RAD;
}

void ControlManager::ned_vel_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.velocity.z = cmd.vel_ref.z;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;
}

void ControlManager::ned_vel_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.velocity.z = cmd.vel_ref.z;
    out.yaw_rate = cmd.yaw_rate_ref * DEG_TO_RAD;
}

void ControlManager::ned_pos_z_vel_xy_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    // XY 速度 + Z 位置定高（与 body_pos_z_vel_xy_set 同掩码语义）
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.position.z = cmd.pos_ref.z;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;
}

void ControlManager::body_pos_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                                      const nav_msgs::Odometry& odom)
{
    const double current_yaw = tf::getYaw(odom.pose.pose.orientation);
    const double d_pos_body_x = cmd.pos_ref.x;
    const double d_pos_body_y = cmd.pos_ref.y;
    const double d_pos_local_x = d_pos_body_x * std::cos(current_yaw) - d_pos_body_y * std::sin(current_yaw);
    const double d_pos_local_y = d_pos_body_x * std::sin(current_yaw) + d_pos_body_y * std::cos(current_yaw);
    const double target_z = odom.pose.pose.position.z + cmd.pos_ref.z;
    const double target_x = odom.pose.pose.position.x + d_pos_local_x;
    const double target_y = odom.pose.pose.position.y + d_pos_local_y;
    const double target_yaw = current_yaw + cmd.yaw_ref * DEG_TO_RAD;

    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly;
    out.position.x = target_x;
    out.position.y = target_y;
    out.position.z = target_z;
    out.yaw = target_yaw;
}

void ControlManager::body_pos_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                                           const nav_msgs::Odometry& odom)
{
    const double current_yaw = tf::getYaw(odom.pose.pose.orientation);
    const double d_pos_body_x = cmd.pos_ref.x;
    const double d_pos_body_y = cmd.pos_ref.y;
    const double d_pos_local_x = d_pos_body_x * std::cos(current_yaw) - d_pos_body_y * std::sin(current_yaw);
    const double d_pos_local_y = d_pos_body_x * std::sin(current_yaw) + d_pos_body_y * std::cos(current_yaw);
    const double target_z = odom.pose.pose.position.z + cmd.pos_ref.z;
    const double target_x = odom.pose.pose.position.x + d_pos_local_x;
    const double target_y = odom.pose.pose.position.y + d_pos_local_y;
    const double yaw_rate = cmd.yaw_rate_ref * DEG_TO_RAD;

    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = kMaskPosOnly | mavros_msgs::PositionTarget::IGNORE_YAW;
    out.position.x = target_x;
    out.position.y = target_y;
    out.position.z = target_z;
    out.yaw_rate = yaw_rate;
}

void ControlManager::body_vel_xyz_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.velocity.z = cmd.vel_ref.z;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;
}

void ControlManager::body_vel_xyz_rate_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.velocity.z = cmd.vel_ref.z;
    out.yaw_rate = cmd.yaw_rate_ref * DEG_TO_RAD;
}

void ControlManager::body_pos_z_vel_xy_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd,
                                           const nav_msgs::Odometry& odom)
{
    const double current_yaw = tf::getYaw(odom.pose.pose.orientation);
    const double vel_body_x = cmd.vel_ref.x;
    const double vel_body_y = cmd.vel_ref.y;
    const double vel_local_x = vel_body_x * std::cos(current_yaw) - vel_body_y * std::sin(current_yaw);
    const double vel_local_y = vel_body_x * std::sin(current_yaw) + vel_body_y * std::cos(current_yaw);
    const double target_z = odom.pose.pose.position.z + cmd.pos_ref.z;
    const double target_yaw = current_yaw + cmd.yaw_ref * DEG_TO_RAD;

    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.velocity.x = vel_local_x;
    out.velocity.y = vel_local_y;
    out.velocity.z = 0.0;
    out.position.z = target_z;
    out.yaw = target_yaw;
}

void ControlManager::trajectory_set(mavros_msgs::PositionTarget& out, const uav_control_fsm::uav_command& cmd)
{
    out.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    out.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    out.position.x = cmd.pos_ref.x;
    out.position.y = cmd.pos_ref.y;
    out.position.z = cmd.pos_ref.z;
    out.velocity.x = cmd.vel_ref.x;
    out.velocity.y = cmd.vel_ref.y;
    out.velocity.z = cmd.vel_ref.z;
    out.acceleration_or_force.x = cmd.acc_ref.x;
    out.acceleration_or_force.y = cmd.acc_ref.y;
    out.acceleration_or_force.z = cmd.acc_ref.z;
    out.yaw = cmd.yaw_ref * DEG_TO_RAD;
}

} // namespace uav_control_fsm
