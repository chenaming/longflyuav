#include "uav_control_fsm/control_manager.hpp"
#include <iostream>
#include <iomanip>

int main() {
    uav_control_fsm::ControlManager manager;
    uav_control_fsm::PosSmoothParams config;
    config.enable = true;
    manager.setPosSmoothParams(config);
    nav_msgs::Odometry odom;
    odom.pose.pose.orientation.w = 1.;
    uav_control_fsm::uav_command command;
    command.Move_mode = 0;
    command.pos_ref.x = .020;
    command.pos_ref.z = .003;
    command.yaw_ref = 1.;
    mavros_msgs::PositionTarget output;
    manager.control_command_set(output, command, odom);
    for(int i=0;i<8;i++) {
        command.pos_ref.x = .020 + i*.00008;
        manager.control_command_set(output, command, odom);
        std::cout << std::setprecision(15) << command.pos_ref.x << " " << output.position.x << " " << output.yaw << "\n";
    }
    return 0;
}
