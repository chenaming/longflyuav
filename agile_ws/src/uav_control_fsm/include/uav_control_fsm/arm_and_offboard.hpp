#ifndef UAV_CONTROL_FSM_ARM_AND_OFFBOARD_HPP
#define UAV_CONTROL_FSM_ARM_AND_OFFBOARD_HPP

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <string>

namespace uav_control_fsm {

/**
 * @brief Arm 与 Offboard 接口（与 uav_control_module_node 行为一致）
 * 起飞、进入/退出 OFFBOARD 时由 ControlFSM 直接调用本接口。
 */
class ArmAndOffboard
{
public:
    ArmAndOffboard(ros::NodeHandle& nh, int uav_id)
        : nh_(nh), uav_id_(uav_id)
    {
        std::string prefix = "/uav" + std::to_string(uav_id_);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(prefix + "/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(prefix + "/mavros/set_mode");
        arm_cmd_.request.value = false;
    }

    /** 解锁/上锁（对应原节点 px4_arming） */
    bool arm(bool arm_or_disarm)
    {
        arm_cmd_.request.value = arm_or_disarm;
        bool ok = arming_client_.call(arm_cmd_) && arm_cmd_.response.success;
        return ok;
    }

    /** 请求进入 OFFBOARD 模式（对应原节点 request_offboard） */
    void request_offboard()
    {
        offb_set_mode_.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
            ROS_INFO("[uav_control_fsm] UAV%d: OFFBOARD mode enabled", uav_id_);
    }

    /** 设置 PX4 模式
     * 常用: "OFFBOARD", "POSITION", "AUTO.LAND" 
     * MANUAL（纯手动）
     * ACRO（特技）
     * ALTCTL（高度控制）
     * POSCTL（位置控制）
     * OFFBOARD（Offboard 模式）
     * STABILIZED（自稳模式）
     * RATTITUDE（混合模式）
     * 自动模式：
     * AUTO.MISSION（任务模式）
     * AUTO.LOITER (悬停模式)
     * AUTO.RTL（返航模式）
     * AUTO.LAND（降落模式）
     * AUTO.RTGS （返回地面站当前位置，RTL返固定点，RTGS返移动点）
     * AUTO.READY（准备模式，自动任务的准备状态）
     * AUTO.TAKEOFF（起飞模式）
     * AUTO.FOLLOW_TARGET（跟随目标模式）
     * AUTO.PRECLAND（预降落模式）
     */
    void set_px4_mode(const std::string& mode)
    {
        offb_set_mode_.request.custom_mode = mode;
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent)
            ROS_INFO("[uav_control_fsm] UAV%d: set_mode %s sent OK", uav_id_, mode.c_str());
        else
            ROS_WARN("[uav_control_fsm] UAV%d: set_mode %s FAILED (mode_sent=%d)",
                     uav_id_, mode.c_str(),
                     static_cast<int>(offb_set_mode_.response.mode_sent));
    }

private:
    ros::NodeHandle nh_;
    int uav_id_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::SetMode offb_set_mode_;
};

} // namespace uav_control_fsm

#endif // UAV_CONTROL_FSM_ARM_AND_OFFBOARD_HPP
