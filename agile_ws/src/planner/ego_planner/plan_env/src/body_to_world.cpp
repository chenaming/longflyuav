/**
 * @file body_to_world.cpp
 * @brief 将 Livox/AirSim 雷达点云从雷达坐标系变换到 uav*_map 地图坐标系
 *
 * 订阅：
 *   - 雷达点云（雷达坐标系）
 *   - /uav{id}/mavros/local_position/pose  机体在 map 中的位姿
 * 发布：
 *   - /uav{id}/livox/lidar_map          变换后的点云（uav{id}_map 坐标系）
 *
 * 变换关系：p_map = T_map_base * T_base_lidar * p_lida
 *
 * 关键点：按点云 header.stamp 在 pose 缓冲中插值（位置线性 + 姿态 SLERP），
 * 避免转向时用「最新姿态」拧旧点云造成重影。
 */

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>

class BodyToWorld
{
public:
  BodyToWorld(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pnh.param("uav_id", uav_id_, 0);
    // 点云 stamp 与匹配/插值得到的 pose 允许的最大时间差 (s)
    pnh.param("max_pose_age", max_pose_age_, 0.1);
    // pose 缓冲时长 (s)，需大于典型雷达延迟
    pnh.param("pose_buffer_duration", pose_buffer_duration_, 1.0);
    if (pose_buffer_duration_ < max_pose_age_ * 2.0)
      pose_buffer_duration_ = max_pose_age_ * 2.0;
    pnh.param("max_pending_clouds", max_pending_clouds_, 30);
    if (max_pending_clouds_ < 1)
      max_pending_clouds_ = 1;

    pnh.param("lidar_extrinsic/x", lidar_ext_x_, 0.0);
    pnh.param("lidar_extrinsic/y", lidar_ext_y_, 0.0);
    pnh.param("lidar_extrinsic/z", lidar_ext_z_, 0.1);
    pnh.param("lidar_extrinsic/roll", lidar_ext_roll_, 0.0);
    pnh.param("lidar_extrinsic/pitch", lidar_ext_pitch_, 0.0);
    pnh.param("lidar_extrinsic/yaw", lidar_ext_yaw_, 0.0);

    pnh.param("self_filter/enable", self_filter_enable_, true);
    pnh.param("self_filter/radius", self_filter_radius_, 0.5);
    if (self_filter_radius_ < 0.0)
      self_filter_radius_ = 0.0;
    self_filter_radius_sq_ = self_filter_radius_ * self_filter_radius_;

    const std::string uav_prefix = "uav" + std::to_string(uav_id_);
    std::string default_map_frame = uav_prefix + "_map";
    std::string default_lidar_topic = "/" + uav_prefix + "/livox/lidar";
    std::string default_pose_topic = "/" + uav_prefix + "/mavros/local_position/pose";
    std::string default_output_topic = "/" + uav_prefix + "/livox/lidar_map";

    pnh.param("map_frame", map_frame_, default_map_frame);
    pnh.param("lidar_topic", lidar_topic_, default_lidar_topic);
    pnh.param("pose_topic", pose_topic_, default_pose_topic);
    pnh.param("output_topic", output_topic_, default_output_topic);

    T_base_lidar_ = makeTransform(lidar_ext_x_, lidar_ext_y_, lidar_ext_z_,
                                  lidar_ext_roll_, lidar_ext_pitch_, lidar_ext_yaw_);

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    pose_sub_ = nh.subscribe(pose_topic_, 200, &BodyToWorld::poseCallback, this);
    cloud_sub_ = nh.subscribe(lidar_topic_, 10, &BodyToWorld::cloudCallback, this);

    ROS_INFO("[body_to_world] uav_id=%d", uav_id_);
    ROS_INFO("[body_to_world] lidar topic : %s", lidar_topic_.c_str());
    ROS_INFO("[body_to_world] pose topic  : %s", pose_topic_.c_str());
    ROS_INFO("[body_to_world] output topic: %s", output_topic_.c_str());
    ROS_INFO("[body_to_world] map frame   : %s", map_frame_.c_str());
    ROS_INFO("[body_to_world] sync: max_pose_age=%.3fs, pose_buffer=%.3fs (interpolate at cloud stamp)",
             max_pose_age_, pose_buffer_duration_);
    ROS_INFO("[body_to_world] pending cloud queue=%d (strictly bracketed poses)",
             max_pending_clouds_);
    if (self_filter_enable_)
      ROS_INFO("[body_to_world] self filter: radius=%.2f m (base_link)", self_filter_radius_);
    else
      ROS_INFO("[body_to_world] self filter: disabled");
  }

private:
  static Eigen::Affine3d makeTransform(double x, double y, double z,
                                       double roll, double pitch, double yaw)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                     .toRotationMatrix();
    T.translation() = Eigen::Vector3d(x, y, z);
    return T;
  }

  static Eigen::Affine3d poseToAffine(const geometry_msgs::Pose &pose)
  {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    T.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z)
                     .normalized()
                     .toRotationMatrix();
    return T;
  }

  static geometry_msgs::Pose interpolatePose(const geometry_msgs::Pose &a,
                                             const geometry_msgs::Pose &b,
                                             double alpha)
  {
    alpha = std::max(0.0, std::min(1.0, alpha));
    geometry_msgs::Pose out;
    out.position.x = (1.0 - alpha) * a.position.x + alpha * b.position.x;
    out.position.y = (1.0 - alpha) * a.position.y + alpha * b.position.y;
    out.position.z = (1.0 - alpha) * a.position.z + alpha * b.position.z;

    Eigen::Quaterniond qa(a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z);
    Eigen::Quaterniond qb(b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
    qa.normalize();
    qb.normalize();
    if (qa.dot(qb) < 0.0)
      qb.coeffs() = -qb.coeffs();
    const Eigen::Quaterniond q = qa.slerp(alpha, qb);
    out.orientation.w = q.w();
    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    return out;
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (msg->header.stamp.isZero())
    {
      ROS_WARN_THROTTLE(2.0, "[body_to_world] pose stamp is zero, sync quality degraded");
    }

    {
    std::lock_guard<std::mutex> lock(mutex_);
    // 保持时间升序；重复 stamp 则覆盖末尾
    if (!pose_buf_.empty() && msg->header.stamp < pose_buf_.back().header.stamp)
    {
      // 乱序：丢掉过旧的乱序包
      if ((pose_buf_.back().header.stamp - msg->header.stamp).toSec() > 0.05)
        return;
    }
    if (!pose_buf_.empty() && msg->header.stamp == pose_buf_.back().header.stamp)
      pose_buf_.back() = *msg;
    else
      pose_buf_.push_back(*msg);

    const ros::Time newest = pose_buf_.back().header.stamp;
    while (pose_buf_.size() > 2 &&
           (newest - pose_buf_.front().header.stamp).toSec() > pose_buffer_duration_)
    {
      pose_buf_.pop_front();
    }
    }
    processPendingClouds();
  }

  /**
   * @brief 取 cloud_stamp 对应位姿：两侧 pose 线性/SLERP；单侧则取最近且 |dt|<=max_pose_age_
   * @return false 表示缓冲不足或时间差过大
   */
  bool lookupPoseAt(const ros::Time &stamp, geometry_msgs::PoseStamped &out, double &dt_out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pose_buf_.empty())
      return false;
    if (stamp.isZero() || stamp < pose_buf_.front().header.stamp ||
        stamp > pose_buf_.back().header.stamp)
      return false;


    // 点云无有效 stamp：退化为最新 pose（并告警）
    if (stamp.isZero())
    {
      out = pose_buf_.back();
      dt_out = 0.0;
      ROS_WARN_THROTTLE(2.0, "[body_to_world] cloud stamp is zero, fallback to latest pose");
      return true;
    }

    // stamp 早于缓冲：取最早一帧（若仍在允许范围）
    if (stamp <= pose_buf_.front().header.stamp)
    {
      dt_out = (stamp - pose_buf_.front().header.stamp).toSec();
      if (std::fabs(dt_out) > max_pose_age_)
        return false;
      out = pose_buf_.front();
      out.header.stamp = stamp;
      return true;
    }

    // stamp 晚于缓冲：取最新一帧（点云比最新 pose 还新，常见于排队延迟）
    if (stamp >= pose_buf_.back().header.stamp)
    {
      dt_out = (stamp - pose_buf_.back().header.stamp).toSec();
      if (std::fabs(dt_out) > max_pose_age_)
        return false;
      out = pose_buf_.back();
      out.header.stamp = stamp;
      return true;
    }

    // 区间内：找到右端点，插值
    size_t hi = 1;
    while (hi < pose_buf_.size() && pose_buf_[hi].header.stamp < stamp)
      ++hi;
    const size_t lo = hi - 1;
    const double dt_before = (stamp - pose_buf_[lo].header.stamp).toSec();
    const double dt_after = (pose_buf_[hi].header.stamp - stamp).toSec();
    dt_out = std::max(dt_before, dt_after);
    if (dt_out > max_pose_age_)
      return false;
    const double t0 = pose_buf_[lo].header.stamp.toSec();
    const double t1 = pose_buf_[hi].header.stamp.toSec();
    const double ts = stamp.toSec();
    const double denom = t1 - t0;
    const double alpha = (denom > 1e-9) ? ((ts - t0) / denom) : 0.0;

    out.header.stamp = stamp;
    out.header.frame_id = pose_buf_[hi].header.frame_id;
    out.pose = interpolatePose(pose_buf_[lo].pose, pose_buf_[hi].pose, alpha);
    dt_out = 0.0;  // 已对齐到 stamp
    return true;
  }

  void processCloud(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    geometry_msgs::PoseStamped pose;
    double sync_dt = 0.0;
    if (!lookupPoseAt(msg->header.stamp, pose, sync_dt))
    {
      ROS_WARN_THROTTLE(1.0,
                        "[body_to_world] no pose near cloud stamp (age limit=%.3fs, buf empty or too far)",
                        max_pose_age_);
      return;
    }

    ROS_DEBUG_THROTTLE(1.0, "[body_to_world] sync_dt=%.4fs (0=interpolated)", sync_dt);

    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);
    if (cloud_in.empty())
      return;

    const Eigen::Affine3d T_map_lidar = poseToAffine(pose.pose) * T_base_lidar_;

    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    cloud_out.points.reserve(cloud_in.points.size());

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
      if (!std::isfinite(cloud_in.points[i].x) || !std::isfinite(cloud_in.points[i].y) ||
          !std::isfinite(cloud_in.points[i].z))
        continue;

      const Eigen::Vector3d p_in(cloud_in.points[i].x, cloud_in.points[i].y, cloud_in.points[i].z);

      if (self_filter_enable_)
      {
        const Eigen::Vector3d p_base = T_base_lidar_ * p_in;
        if (p_base.squaredNorm() < self_filter_radius_sq_)
          continue;
      }

      const Eigen::Vector3d p_out = T_map_lidar * p_in;
      cloud_out.points.emplace_back(static_cast<float>(p_out.x()), static_cast<float>(p_out.y()),
                                    static_cast<float>(p_out.z()));
    }

    if (cloud_out.points.empty())
      return;

    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    cloud_out.is_dense = true;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_out, cloud_msg);
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = map_frame_;
    cloud_pub_.publish(cloud_msg);
  }
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!cloud_buf_.empty() &&
          msg->header.stamp < cloud_buf_.back()->header.stamp)
      {
        ROS_WARN_THROTTLE(1.0, "[body_to_world] drop out-of-order cloud");
        return;
      }

      if (cloud_buf_.size() >= static_cast<size_t>(max_pending_clouds_))
      {
        ROS_WARN_THROTTLE(
            1.0,
            "[body_to_world] pending cloud queue full (%d), drop oldest cloud",
            max_pending_clouds_);
        cloud_buf_.pop_front();
      }

      cloud_buf_.push_back(msg);
    }

    processPendingClouds();
  }

  void processPendingClouds()
  {
    while (ros::ok())
    {
      sensor_msgs::PointCloud2ConstPtr msg;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cloud_buf_.empty() || pose_buf_.empty())
          return;

        const sensor_msgs::PointCloud2ConstPtr &pending = cloud_buf_.front();
        const ros::Time stamp = pending->header.stamp;

        if (stamp.isZero())
        {
          ROS_WARN_THROTTLE(
              1.0,
              "[body_to_world] drop cloud with zero stamp; acquisition time is required");
          cloud_buf_.pop_front();
          continue;
        }

        if (stamp < pose_buf_.front().header.stamp)
        {
          ROS_WARN_THROTTLE(
              1.0,
              "[body_to_world] drop cloud older than pose buffer by %.4fs",
              (pose_buf_.front().header.stamp - stamp).toSec());
          cloud_buf_.pop_front();
          continue;
        }

        // Do not use the latest pose for a newer cloud. Keep the cloud queued
        // until a pose at or after its acquisition time arrives.
        if (stamp > pose_buf_.back().header.stamp)
          return;

        msg = pending;
        cloud_buf_.pop_front();
      }

      processCloud(msg);
    }
  }

  int uav_id_;
  double max_pose_age_;
  double pose_buffer_duration_;
  int max_pending_clouds_;
  double lidar_ext_x_, lidar_ext_y_, lidar_ext_z_;
  double lidar_ext_roll_, lidar_ext_pitch_, lidar_ext_yaw_;

  bool self_filter_enable_;
  double self_filter_radius_;
  double self_filter_radius_sq_;

  std::string map_frame_;
  std::string lidar_topic_;
  std::string pose_topic_;
  std::string output_topic_;

  Eigen::Affine3d T_base_lidar_;

  std::mutex mutex_;
  std::deque<geometry_msgs::PoseStamped> pose_buf_;
  std::deque<sensor_msgs::PointCloud2ConstPtr> cloud_buf_;

  ros::Publisher cloud_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber cloud_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "body_to_world");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  BodyToWorld node(nh, pnh);
  ros::spin();
  return 0;
}
