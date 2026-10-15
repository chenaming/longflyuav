#include <ros/ros.h>
#include "uav_control_fsm/param_set.hpp"
#include "uav_control_fsm/control_manager.hpp"
#include "uav_control_fsm/arm_and_offboard.hpp"
#include "uav_control_fsm/tf_tree_manager.hpp"
#include "uav_control_fsm/fsm_control.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_control_fsm_node");
    ros::NodeHandle nh;         // 默认句柄，用于话题/服务
    ros::NodeHandle pnh("~");   // 节点私有句柄，用于加载节点私有参数

    uav_control_fsm::ControlFSM fsm(nh, pnh);
    fsm.waitForFCU();

    ros::Rate rate(fsm.getLoopHz());

    while (ros::ok())
    {
        ros::spinOnce();
        fsm.process();
        if (fsm.shouldPublishSetpoint())
            fsm.publishSetpoint();
        rate.sleep();
    }
    return 0;
}
