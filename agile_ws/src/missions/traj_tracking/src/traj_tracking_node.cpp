#include "traj_tracking_demo/traj_tracking_node.hpp"

#include <cmath>

namespace traj_tracking_demo {

TrajTrackingNode::TrajTrackingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  loadParams();
  setupInterfaces();
  generator_ = createTrajGenerator(traj_config_);

  const double period = 1.0 / std::max(1.0, publish_rate_);
  timer_ = nh_.createTimer(ros::Duration(period), &TrajTrackingNode::onTimer, this);

  ROS_INFO("[traj_tracking_demo] uav_id=%d move_mode=%d traj_type=%s finish_action=%s",
           uav_id_, move_mode_, traj_config_.type.c_str(), finishActionToString(finish_action_));
  ROS_INFO("[traj_tracking_demo] map_frame=%s center=(%.2f,%.2f) alt=%.2f speed=%.2f laps=%d",
           map_frame_.c_str(), traj_config_.center_x, traj_config_.center_y, traj_config_.altitude,
           traj_config_.speed, traj_config_.num_laps);
  ROS_INFO("[traj_tracking_demo] side_length=%.2f radius=%.2f publish_rate=%.1f Hz",
           traj_config_.side_length, traj_config_.radius, publish_rate_);

  if (auto_start_)
    transitionTo(DemoState::Ready);
}

void TrajTrackingNode::loadParams()
{
  pnh_.param("uav_id", uav_id_, 0);
  pnh_.param("move_mode", move_mode_, 10);
  pnh_.param("traj_type", traj_config_.type, traj_config_.type);
  pnh_.param("center_x", traj_config_.center_x, traj_config_.center_x);
  pnh_.param("center_y", traj_config_.center_y, traj_config_.center_y);
  pnh_.param("altitude", traj_config_.altitude, traj_config_.altitude);
  pnh_.param("speed", traj_config_.speed, traj_config_.speed);
  pnh_.param("radius", traj_config_.radius, traj_config_.radius);
  pnh_.param("side_length", traj_config_.side_length, traj_config_.side_length);
  pnh_.param("num_laps", traj_config_.num_laps, traj_config_.num_laps);
  pnh_.param("start_angle_deg", traj_config_.start_angle_deg, traj_config_.start_angle_deg);
  pnh_.param("clockwise", traj_config_.clockwise, traj_config_.clockwise);

  std::string finish_str = "hover";
  pnh_.param("finish_action", finish_str, finish_str);
  finish_action_ = finishActionFromString(finish_str);

  pnh_.param("approach_speed", approach_speed_, approach_speed_);
  pnh_.param("publish_rate", publish_rate_, publish_rate_);
  pnh_.param("wait_hover_sec", wait_hover_sec_, wait_hover_sec_);
  pnh_.param("arrival_tol", arrival_tol_, arrival_tol_);
  pnh_.param("arrival_hold_sec", arrival_hold_sec_, arrival_hold_sec_);
  pnh_.param("ref_path_sample_step", ref_path_sample_step_, ref_path_sample_step_);
  pnh_.param("auto_start", auto_start_, auto_start_);

  if (move_mode_ != 10 && move_mode_ != 0)
  {
    ROS_WARN("[traj_tracking_demo] unsupported move_mode=%d, fallback to 10", move_mode_);
    move_mode_ = 10;
  }

  prefix_ = "/uav" + std::to_string(uav_id_);
  map_frame_ = "uav" + std::to_string(uav_id_) + "_map";
}

void TrajTrackingNode::setupInterfaces()
{
  move_pub_ = nh_.advertise<uav_control_fsm::uav_command>(prefix_ + "/move_command", 10);
  hover_pub_ = nh_.advertise<std_msgs::Bool>(prefix_ + "/hover_command", 1);
  land_pub_ = nh_.advertise<std_msgs::Bool>(prefix_ + "/land_command", 1);
  ref_path_pub_ = nh_.advertise<nav_msgs::Path>(prefix_ + "/traj_reference_path", 1, true);
  target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(prefix_ + "/traj_target_marker", 1);

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      prefix_ + "/mavros/local_position/odom", 10, &TrajTrackingNode::onOdom, this);
  start_sub_ = nh_.subscribe<std_msgs::Bool>(
      prefix_ + "/start", 1, &TrajTrackingNode::onStart, this);
}

void TrajTrackingNode::spin()
{
  ros::spin();
}

void TrajTrackingNode::onOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_ = *msg;
  odom_received_ = true;
}

void TrajTrackingNode::onStart(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) return;
  if (state_ != DemoState::Idle && state_ != DemoState::Done) return;
  ref_path_published_ = false;
  transitionTo(DemoState::Ready);
}

void TrajTrackingNode::transitionTo(DemoState next)
{
  state_ = next;
  phase_start_ = ros::Time::now();
  arrival_hold_active_ = false;
  finish_cmd_sent_ = false;
  ROS_INFO("[traj_tracking_demo] -> %s", demoStateToString(state_));

  if (next == DemoState::Tracking && !ref_path_published_)
  {
    publishReferencePath();
    ref_path_published_ = true;
  }
}

void TrajTrackingNode::beginGoToStart()
{
  const TrajSample start_pt = generator_->startPoint();
  go_start_from_x_ = odom_.pose.pose.position.x;
  go_start_from_y_ = odom_.pose.pose.position.y;
  go_start_from_z_ = odom_.pose.pose.position.z;
  go_start_to_x_ = start_pt.x;
  go_start_to_y_ = start_pt.y;
  go_start_to_z_ = start_pt.z;
  go_start_duration_ = linearDuration(
      go_start_from_x_, go_start_from_y_, go_start_from_z_,
      go_start_to_x_, go_start_to_y_, go_start_to_z_, approach_speed_);
}

void TrajTrackingNode::publishReferencePath()
{
  nav_msgs::Path path = buildReferencePath(*generator_, map_frame_, ref_path_sample_step_);
  path.header.stamp = ros::Time::now();
  ref_path_pub_.publish(path);
  ROS_INFO("[traj_tracking_demo] published reference path (%zu points) on %s/traj_reference_path",
           path.poses.size(), prefix_.c_str());
}

void TrajTrackingNode::publishMove(const TrajSample& sample)
{
  uav_control_fsm::uav_command cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = map_frame_;
  cmd.Move_mode = static_cast<uint8_t>(move_mode_);
  cmd.pos_ref.x = sample.x;
  cmd.pos_ref.y = sample.y;
  cmd.pos_ref.z = sample.z;
  cmd.vel_ref.x = sample.vx;
  cmd.vel_ref.y = sample.vy;
  cmd.vel_ref.z = sample.vz;
  cmd.acc_ref.x = sample.ax;
  cmd.acc_ref.y = sample.ay;
  cmd.acc_ref.z = sample.az;
  cmd.yaw_ref = static_cast<float>(sample.yaw_deg);
  move_pub_.publish(cmd);
  publishTargetMarker(sample);
}

void TrajTrackingNode::publishTargetMarker(const TrajSample& sample)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = map_frame_;
  marker.ns = "traj_target";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = sample.x;
  marker.pose.position.y = sample.y;
  marker.pose.position.z = sample.z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0f;
  marker.color.g = 0.2f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(0.5);
  target_marker_pub_.publish(marker);
}

void TrajTrackingNode::publishHover()
{
  std_msgs::Bool msg;
  msg.data = true;
  hover_pub_.publish(msg);
}

void TrajTrackingNode::publishLand()
{
  std_msgs::Bool msg;
  msg.data = true;
  land_pub_.publish(msg);
}

bool TrajTrackingNode::hasOdom() const
{
  return odom_received_;
}

bool TrajTrackingNode::arrivedAt(double tx, double ty, double tz) const
{
  const double dx = odom_.pose.pose.position.x - tx;
  const double dy = odom_.pose.pose.position.y - ty;
  const double dz = odom_.pose.pose.position.z - tz;
  return std::hypot(dx, dy) < arrival_tol_ && std::abs(dz) < arrival_tol_;
}

void TrajTrackingNode::updateArrivalHold(double tx, double ty, double tz, const ros::Time& now)
{
  if (arrivedAt(tx, ty, tz))
  {
    if (!arrival_hold_active_)
    {
      arrival_hold_active_ = true;
      arrival_hold_start_ = now;
    }
  }
  else
  {
    arrival_hold_active_ = false;
  }
}

double TrajTrackingNode::linearDuration(double x0, double y0, double z0,
                                        double x1, double y1, double z1, double speed) const
{
  const double dist = std::hypot(std::hypot(x1 - x0, y1 - y0), z1 - z0);
  speed = std::max(0.05, speed);
  return dist / speed;
}

void TrajTrackingNode::onTimer(const ros::TimerEvent& event)
{
  const ros::Time now = event.current_real;
  const double phase_t = (now - phase_start_).toSec();

  switch (state_)
  {
    case DemoState::Idle:
    case DemoState::Done:
      return;

    case DemoState::Ready:
      if (!hasOdom()) return;
      if (phase_t < wait_hover_sec_) return;
      beginGoToStart();
      transitionTo(DemoState::GoToStart);
      return;

    case DemoState::GoToStart:
    {
      const TrajSample s = linearSample(
          phase_t, go_start_duration_,
          go_start_from_x_, go_start_from_y_, go_start_from_z_,
          go_start_to_x_, go_start_to_y_, go_start_to_z_);
      publishMove(s);

      updateArrivalHold(go_start_to_x_, go_start_to_y_, go_start_to_z_, now);
      const bool time_done = phase_t >= go_start_duration_;
      const bool hold_done = arrival_hold_active_ &&
                             (now - arrival_hold_start_).toSec() >= arrival_hold_sec_;
      if (time_done && hold_done)
        transitionTo(DemoState::Tracking);
      return;
    }

    case DemoState::Tracking:
    {
      TrajSample s = generator_->sample(phase_t);
      if (move_mode_ == 10)
        fillAccFromVelocityDiff(s, *generator_, phase_t);
      publishMove(s);
      if (generator_->finished(phase_t))
      {
        if (finish_action_ == FinishAction::ReturnOriginLand)
        {
          return_from_x_ = odom_.pose.pose.position.x;
          return_from_y_ = odom_.pose.pose.position.y;
          return_from_z_ = odom_.pose.pose.position.z;
          return_duration_ = linearDuration(
              return_from_x_, return_from_y_, return_from_z_,
              traj_config_.center_x, traj_config_.center_y, traj_config_.altitude,
              approach_speed_);
          transitionTo(DemoState::ReturnToOrigin);
        }
        else if (finish_action_ == FinishAction::Land)
        {
          transitionTo(DemoState::FinishLand);
        }
        else
        {
          transitionTo(DemoState::FinishHover);
        }
      }
      return;
    }

    case DemoState::ReturnToOrigin:
    {
      const TrajSample s = linearSample(
          phase_t, return_duration_,
          return_from_x_, return_from_y_, return_from_z_,
          traj_config_.center_x, traj_config_.center_y, traj_config_.altitude);
      publishMove(s);

      updateArrivalHold(traj_config_.center_x, traj_config_.center_y, traj_config_.altitude, now);
      const bool time_done = phase_t >= return_duration_;
      const bool hold_done = arrival_hold_active_ &&
                             (now - arrival_hold_start_).toSec() >= arrival_hold_sec_;
      if (time_done && hold_done)
        transitionTo(DemoState::FinishLand);
      return;
    }

    case DemoState::FinishHover:
      if (!finish_cmd_sent_)
      {
        publishHover();
        finish_cmd_sent_ = true;
        transitionTo(DemoState::Done);
      }
      return;

    case DemoState::FinishLand:
      if (!finish_cmd_sent_)
      {
        publishLand();
        finish_cmd_sent_ = true;
        transitionTo(DemoState::Done);
      }
      return;
  }
}

}  // namespace traj_tracking_demo
