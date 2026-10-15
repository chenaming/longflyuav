/*********************************************************
odometry_bridge.cpp: 按当前定位源订阅对应里程计话题，转换为 mavros 的 vision_pose。
- 实现见 odometry_bridge.hpp 注释；本文件为动态库实现。
*********************************************************/
#include "uav_control_fsm/odometry_bridge.hpp"

OdometryBridge::OdometryBridge(ros::NodeHandle& nh, int uav_id)
    : nh_(nh),
      cb_nh_(nh),
      uav_id_(uav_id),
      vision_pose_pub_created_(false),
      vision_pose_pub_rate_hz_(0.0),
      timer_publish_enabled_(false),
      vision_pose_timer_armed_(false),
      have_odom_for_vision_(false)
{
    prefix_ = "/uav" + std::to_string(uav_id_);
    set_location_source_ = "GPS";  // 初始状态由 FSM 在构造后调用 setLocationSource(default) 设置

    // 将 bridge 内部的订阅/定时器绑定到独立回调队列，避免受外部 spinOnce()/loop_hz 限速。
    cb_nh_.setCallbackQueue(&cb_queue_);
    cb_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &cb_queue_);
    cb_spinner_->start();

    ros::NodeHandle bridge_nh(cb_nh_, "odometry_bridge");
    bridge_nh.param("vision_pose_pub_rate_hz", vision_pose_pub_rate_hz_, 0.0);
    if (vision_pose_pub_rate_hz_ > 1e-6)
        ROS_INFO("[OdometryBridge] uav%d vision_pose_pub_rate_hz=%.3f (timer publish)", uav_id_, vision_pose_pub_rate_hz_);
    else
        ROS_INFO("[OdometryBridge] uav%d vision_pose_pub_rate_hz<=0 (publish on every odom)", uav_id_);

    updateSubscription();
}

OdometryBridge::~OdometryBridge()
{
    stopVisionPoseTimer();
    if (cb_spinner_)
    {
        cb_spinner_->stop();
        cb_spinner_.reset();
    }
}

void OdometryBridge::stopVisionPoseTimer()
{
    if (vision_pose_timer_armed_)
    {
        vision_pose_timer_.stop();
        vision_pose_timer_armed_ = false;
    }
}

void OdometryBridge::startVisionPoseTimer()
{
    stopVisionPoseTimer();
    if (vision_pose_pub_rate_hz_ <= 1e-6)
        return;
    vision_pose_timer_ = cb_nh_.createTimer(ros::Duration(1.0 / vision_pose_pub_rate_hz_),
                                           &OdometryBridge::timerVisionPosePub, this);
    vision_pose_timer_armed_ = true;
}

std::string OdometryBridge::getOdomTopicFromLocationSource(const std::string& set_location_source)
{
    if (set_location_source == "Gazebo")          return "gazebo_odometry";
    if (set_location_source == "Fast_Lio")    return "fast_lio_odometry";
    if (set_location_source == "Vins")        return "vins_odometry";
    return "";
}

void OdometryBridge::updateSubscription()
{
    std::string new_topic = getOdomTopicFromLocationSource(set_location_source_);
    if (new_topic == odom_sub_topic_) return;

    odom_sub_.shutdown();
    odom_sub_topic_.clear();
    stopVisionPoseTimer();
    timer_publish_enabled_ = false;
    have_odom_for_vision_ = false;

    if (!new_topic.empty())
    {
        std::string full_topic = prefix_ + "/" + new_topic;
        odom_sub_ = cb_nh_.subscribe(full_topic, 10, &OdometryBridge::odomCallback, this);
        odom_sub_topic_ = new_topic;
        if (!vision_pose_pub_created_)
        {
            vision_pose_pub_ = cb_nh_.advertise<geometry_msgs::PoseStamped>(prefix_ + "/mavros/vision_pose/pose", 10);
            vision_pose_pub_created_ = true;
        }
        timer_publish_enabled_ = (vision_pose_pub_rate_hz_ > 1e-6);
        if (timer_publish_enabled_)
            startVisionPoseTimer();
        ROS_INFO("[OdometryBridge] uav%d set_location_source=%s -> subscribe %s -> mavros/vision_pose/pose",
                 uav_id_, set_location_source_.c_str(), full_topic.c_str());
    }
    else
        ROS_INFO("[OdometryBridge] uav%d set_location_source=%s (GPS) -> no subscribe, no publish", uav_id_, set_location_source_.c_str());
}

void OdometryBridge::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vision_pose_.header = msg->header;
    vision_pose_.header.stamp = ros::Time::now();
    vision_pose_.pose = msg->pose.pose;
    have_odom_for_vision_ = true;
    if (!timer_publish_enabled_)
        vision_pose_pub_.publish(vision_pose_);
}

void OdometryBridge::timerVisionPosePub(const ros::TimerEvent&)
{
    if (!have_odom_for_vision_)
        return;
    vision_pose_.header.stamp = ros::Time::now();
    vision_pose_pub_.publish(vision_pose_);
}

void OdometryBridge::setLocationSource(const std::string& source)
{
    if (source.empty() || source == set_location_source_) return;
    set_location_source_ = source;
    updateSubscription();
}
