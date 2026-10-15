#pragma once

#include <memory>
#include <string>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <uav_control_fsm/uav_command.h>
#include <visualization_msgs/Marker.h>

#include "traj_tracking_demo/traj_generator_base.hpp"
#include "traj_tracking_demo/traj_types.hpp"

namespace traj_tracking_demo {

class TrajTrackingNode
{
public:
  explicit TrajTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void spin();

private:
  void loadParams();
  void setupInterfaces();
  void onTimer(const ros::TimerEvent& event);
  void onOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void onStart(const std_msgs::Bool::ConstPtr& msg);

  void transitionTo(DemoState next);
  void beginGoToStart();
  void publishReferencePath();
  void publishMove(const TrajSample& sample);
  void publishTargetMarker(const TrajSample& sample);
  void publishHover();
  void publishLand();

  bool hasOdom() const;
  bool arrivedAt(double tx, double ty, double tz) const;
  void updateArrivalHold(double tx, double ty, double tz, const ros::Time& now);

  double linearDuration(double x0, double y0, double z0, double x1, double y1, double z1, double speed) const;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int uav_id_{0};
  int move_mode_{10};
  TrajConfig traj_config_;
  FinishAction finish_action_{FinishAction::Hover};
  double approach_speed_{1.0};
  double publish_rate_{30.0};
  double wait_hover_sec_{2.0};
  double arrival_tol_{0.3};
  double arrival_hold_sec_{1.0};
  double ref_path_sample_step_{0.05};
  bool auto_start_{true};

  std::string prefix_;
  std::string map_frame_;
  ros::Publisher move_pub_;
  ros::Publisher hover_pub_;
  ros::Publisher land_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher target_marker_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber start_sub_;
  ros::Timer timer_;

  DemoState state_{DemoState::Idle};
  std::unique_ptr<TrajGeneratorBase> generator_;

  nav_msgs::Odometry odom_;
  bool odom_received_{false};

  ros::Time phase_start_;
  ros::Time arrival_hold_start_;
  bool arrival_hold_active_{false};

  double go_start_from_x_{0.0};
  double go_start_from_y_{0.0};
  double go_start_from_z_{0.0};
  double go_start_to_x_{0.0};
  double go_start_to_y_{0.0};
  double go_start_to_z_{0.0};
  double go_start_duration_{0.0};

  double return_from_x_{0.0};
  double return_from_y_{0.0};
  double return_from_z_{0.0};
  double return_duration_{0.0};

  bool finish_cmd_sent_{false};
  bool ref_path_published_{false};
};

}  // namespace traj_tracking_demo
