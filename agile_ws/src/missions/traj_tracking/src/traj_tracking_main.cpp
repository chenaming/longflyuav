#include <ros/ros.h>

#include "traj_tracking_demo/traj_tracking_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_tracking_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  traj_tracking_demo::TrajTrackingNode node(nh, pnh);
  node.spin();
  return 0;
}
