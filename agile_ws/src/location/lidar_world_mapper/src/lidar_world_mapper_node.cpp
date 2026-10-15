#include <lidar_world_mapper/imu_deskew.hpp>
#include <lidar_world_mapper/voxel_map.hpp>

#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using PointXT = pcl::PointXYZINormal;
using CloudXT = pcl::PointCloud<PointXT>;

Eigen::Isometry3d odomToIsometry(const nav_msgs::Odometry & odom)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const auto & p = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;
  T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  return T;
}

CloudT::Ptr transformCloud(const CloudT & in, const Eigen::Isometry3d & T)
{
  CloudT::Ptr out(new CloudT);
  out->points.reserve(in.points.size());
  for (const auto & pt : in.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    const Eigen::Vector3d pw = T * Eigen::Vector3d(pt.x, pt.y, pt.z);
    PointT q;
    q.x = static_cast<float>(pw.x());
    q.y = static_cast<float>(pw.y());
    q.z = static_cast<float>(pw.z());
    q.intensity = pt.intensity;
    out->points.push_back(q);
  }
  out->width = static_cast<uint32_t>(out->points.size());
  out->height = 1;
  out->is_dense = false;
  return out;
}

CloudT toXYZI(const CloudXT & in)
{
  CloudT out;
  out.points.reserve(in.points.size());
  for (const auto & p : in.points) {
    PointT q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.intensity;
    out.points.push_back(q);
  }
  out.width = static_cast<uint32_t>(out.points.size());
  out.height = 1;
  out.is_dense = false;
  return out;
}

Eigen::Matrix3d rotFromVector9(const std::vector<double> & v)
{
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  if (v.size() == 9) {
    R << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
  }
  return R;
}

}  // namespace

class LidarWorldMapper
{
public:
  LidarWorldMapper(ros::NodeHandle & nh, ros::NodeHandle & pnh)
  : nh_(nh), pnh_(pnh)
  {
    loadParams();
    map_.setResolution(static_cast<float>(map_voxel_));
    setupExtrinsic();

    pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>(scan_world_topic_, 1);
    pub_deskew_ = nh_.advertise<sensor_msgs::PointCloud2>(deskew_topic_, 1);
    pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    save_srv_ = pnh_.advertiseService("save_map", &LidarWorldMapper::saveMapCb, this);
    clear_srv_ = pnh_.advertiseService("clear_map", &LidarWorldMapper::clearMapCb, this);

    if (mode_ == "deskew") {
      sub_imu_ = nh_.subscribe(imu_topic_, 2000, &LidarWorldMapper::imuCb, this);
      sub_odom_buf_ = nh_.subscribe(odom_topic_, 200, &LidarWorldMapper::odomBufCb, this);
      if (lidar_type_ == "livox") {
        sub_livox_ = nh_.subscribe(cloud_topic_, 20, &LidarWorldMapper::livoxCb, this);
      } else {
        sub_cloud_only_ = nh_.subscribe(cloud_topic_, 20, &LidarWorldMapper::stdCloudDeskewCb, this);
      }
      ROS_INFO(
        "[lidar_world_mapper] mode=deskew lidar=%s(%s) imu=%s odom=%s", cloud_topic_.c_str(),
        lidar_type_.c_str(), imu_topic_.c_str(), odom_topic_.c_str());
      ROS_WARN(
        "[lidar_world_mapper] deskew does not need FAST-LIO, but still needs external odom "
        "(e.g. /uavN/mavros/local_position/odom) for a consistent world map");
    } else if (mode_ == "already_world") {
      sub_cloud_only_ = nh_.subscribe(cloud_topic_, 5, &LidarWorldMapper::cloudOnlyCb, this);
      ROS_INFO("[lidar_world_mapper] mode=already_world cloud=%s", cloud_topic_.c_str());
    } else {
      // body_odom: 已去畸变的 body 点云 + odom（旧逻辑，兼容 FAST-LIO 输出）
      sub_cloud_.subscribe(nh_, cloud_topic_, 20);
      sub_odom_.subscribe(nh_, odom_topic_, 50);
      sync_.reset(new Sync(SyncPolicy(30), sub_cloud_, sub_odom_));
      sync_->registerCallback(boost::bind(&LidarWorldMapper::syncCb, this, _1, _2));
      ROS_INFO(
        "[lidar_world_mapper] mode=body_odom cloud=%s odom=%s", cloud_topic_.c_str(),
        odom_topic_.c_str());
    }

    if (publish_map_hz_ > 0.0) {
      map_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_map_hz_), &LidarWorldMapper::mapTimerCb, this);
    }
  }

  ~LidarWorldMapper()
  {
    if (save_on_shutdown_) {
      saveMapLocked("shutdown");
    }
  }

private:
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  void loadParams()
  {
    int uav_id = 1;
    pnh_.param("uav_id", uav_id, 1);
    const std::string prefix = "/uav" + std::to_string(uav_id);

    pnh_.param<std::string>("mode", mode_, std::string("deskew"));  // deskew | body_odom | already_world
    // legacy bool
    bool input_already_world = false;
    pnh_.param("input_already_world", input_already_world, false);
    if (input_already_world) {
      mode_ = "already_world";
    }

    pnh_.param<std::string>("lidar_type", lidar_type_, std::string("livox"));  // livox | pointcloud2
    pnh_.param<std::string>("cloud_topic", cloud_topic_, prefix + "/livox/lidar");
    pnh_.param<std::string>("imu_topic", imu_topic_, prefix + "/livox/imu");
    pnh_.param<std::string>("odom_topic", odom_topic_, prefix + "/mavros/local_position/odom");
    pnh_.param<std::string>("odom_child", odom_child_, std::string("imu"));  // imu | lidar
    pnh_.param<std::string>("scan_world_topic", scan_world_topic_, prefix + "/map_builder/scan_world");
    pnh_.param<std::string>("deskew_topic", deskew_topic_, prefix + "/map_builder/cloud_deskewed");
    pnh_.param<std::string>("map_topic", map_topic_, prefix + "/map_builder/map");
    pnh_.param<std::string>("world_frame", world_frame_, "map");

    pnh_.param("blind", blind_, 0.5);
    pnh_.param("point_filter_num", point_filter_num_, 3);
    pnh_.param("enable_translation_deskew", enable_translation_deskew_, true);
    pnh_.param("odom_time_slop", odom_time_slop_, 0.05);

    std::vector<double> extrinT{0, 0, 0};
    std::vector<double> extrinR{1, 0, 0, 0, 1, 0, 0, 0, 1};
    pnh_.param("extrinsic_T", extrinT, extrinT);
    pnh_.param("extrinsic_R", extrinR, extrinR);
    extrinT_ = extrinT;
    extrinR_ = extrinR;

    pnh_.param("min_range", min_range_, 0.5);
    pnh_.param("max_range", max_range_, 80.0);
    pnh_.param("z_min", z_min_, -2.0);
    pnh_.param("z_max", z_max_, 5.0);
    pnh_.param("frame_voxel", frame_voxel_, 0.1);
    pnh_.param("map_voxel", map_voxel_, 0.1);

    pnh_.param("enable_sor", enable_sor_, true);
    pnh_.param("sor_mean_k", sor_mean_k_, 20);
    pnh_.param("sor_stddev", sor_stddev_, 1.0);

    pnh_.param("enable_ror", enable_ror_, true);
    pnh_.param("ror_radius", ror_radius_, 0.35);
    pnh_.param("ror_min_neighbors", ror_min_neighbors_, 4);

    pnh_.param("skip_frames", skip_frames_, 0);
    pnh_.param("max_speed_for_insert", max_speed_for_insert_, -1.0);
    pnh_.param("publish_map_hz", publish_map_hz_, 1.0);

    pnh_.param("pcd_save_en", pcd_save_en_, true);
    pnh_.param("save_on_shutdown", save_on_shutdown_, true);
    pnh_.param<std::string>("pcd_path", pcd_path_, "/tmp/lidar_world_map.pcd");
    pnh_.param("pcd_binary", pcd_binary_, true);
  }

  void setupExtrinsic()
  {
    R_il_ = rotFromVector9(extrinR_);
    t_il_ = Eigen::Vector3d::Zero();
    if (extrinT_.size() == 3) {
      t_il_ << extrinT_[0], extrinT_[1], extrinT_[2];
    }
    deskew_.setExtrinsic(R_il_, t_il_);
    deskew_.setEnableTranslation(enable_translation_deskew_);
  }

  void imuCb(const sensor_msgs::ImuConstPtr & msg) { deskew_.pushImu(msg); }

  void odomBufCb(const nav_msgs::OdometryConstPtr & msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buf_.push_back(msg);
    while (odom_buf_.size() > 500) {
      odom_buf_.pop_front();
    }
  }

  bool lookupOdom(double t, nav_msgs::Odometry & out, Eigen::Vector3d & v_imu) const
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_buf_.empty()) {
      return false;
    }
    nav_msgs::OdometryConstPtr best;
    double best_dt = 1e9;
    for (const auto & o : odom_buf_) {
      const double dt = std::abs(o->header.stamp.toSec() - t);
      if (dt < best_dt) {
        best_dt = dt;
        best = o;
      }
    }
    if (!best || best_dt > odom_time_slop_) {
      return false;
    }
    out = *best;
    v_imu << best->twist.twist.linear.x, best->twist.twist.linear.y, best->twist.twist.linear.z;
    // twist is usually in child_frame; if odom is IMU, this is IMU velocity in body/world-ish.
    return true;
  }

  Eigen::Isometry3d lidarPoseFromOdom(const nav_msgs::Odometry & odom) const
  {
    const Eigen::Isometry3d T_w_child = odomToIsometry(odom);
    if (odom_child_ == "lidar") {
      return T_w_child;
    }
    // odom = IMU in world: T_wl = T_wi * T_il
    Eigen::Isometry3d T_il = Eigen::Isometry3d::Identity();
    T_il.linear() = R_il_;
    T_il.translation() = t_il_;
    return T_w_child * T_il;
  }

  void livoxCb(const livox_ros_driver2::CustomMsg::ConstPtr & msg)
  {
    CloudXT cloud;
    cloud.points.reserve(msg->point_num / std::max(1, point_filter_num_));

    const int filter_n = std::max(1, point_filter_num_);
    uint32_t valid_num = 0;
    PointXT last_added;
    bool has_last = false;

    for (uint32_t i = 0; i < msg->point_num; ++i) {
      const auto & pt = msg->points[i];
      // Same acceptance rule as FAST-LIO avia_handler (Mid360 / Avia CustomMsg)
      if (pt.line >= 4) {
        continue;
      }
      if ((pt.tag & 0x30) != 0x10 && (pt.tag & 0x30) != 0x00) {
        continue;
      }
      ++valid_num;
      if (valid_num % static_cast<uint32_t>(filter_n) != 0) {
        continue;
      }

      PointXT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.intensity = pt.reflectivity;
      p.curvature = pt.offset_time / float(1000000);  // ms

      const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                        static_cast<double>(p.z) * p.z;
      if (r2 < blind_ * blind_) {
        continue;
      }
      if (has_last) {
        if (std::abs(p.x - last_added.x) < 1e-7f && std::abs(p.y - last_added.y) < 1e-7f &&
            std::abs(p.z - last_added.z) < 1e-7f) {
          continue;
        }
      }
      cloud.points.push_back(p);
      last_added = p;
      has_last = true;
    }

    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
    if (cloud.empty()) {
      return;
    }

    std::sort(cloud.points.begin(), cloud.points.end(), [](const PointXT & a, const PointXT & b) {
      return a.curvature < b.curvature;
    });

    const double t_beg = msg->header.stamp.toSec();
    const double t_end = t_beg + cloud.points.back().curvature / 1000.0;
    handleDeskewedScan(cloud, t_beg, t_end, msg->header.stamp);
  }

  void stdCloudDeskewCb(const sensor_msgs::PointCloud2ConstPtr & msg)
  {
    // PointCloud2 without per-point time: skip motion deskew, treat as one timestamp.
    CloudT raw;
    pcl::fromROSMsg(*msg, raw);
    CloudXT cloud;
    cloud.points.reserve(raw.size());
    for (const auto & pt : raw.points) {
      PointXT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.intensity = pt.intensity;
      p.curvature = 0.f;
      cloud.points.push_back(p);
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    const double t = msg->header.stamp.toSec();
    handleDeskewedScan(cloud, t, t, msg->header.stamp);
  }

  void handleDeskewedScan(
    const CloudXT & cloud_raw, double t_beg, double t_end, const ros::Time & stamp)
  {
    if (!deskew_.ready()) {
      ROS_WARN_THROTTLE(2.0, "[lidar_world_mapper] waiting IMU init (~1s static recommended)");
      return;
    }

    nav_msgs::Odometry odom;
    Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
    if (!lookupOdom(t_end, odom, v0)) {
      ROS_WARN_THROTTLE(2.0, "[lidar_world_mapper] no odom near scan end (topic=%s)", odom_topic_.c_str());
      return;
    }

    if (max_speed_for_insert_ >= 0.0) {
      const double speed = v0.norm();
      if (speed > max_speed_for_insert_) {
        return;
      }
    }

    // If odom twist is in world, rotate into IMU for integration start velocity.
    Eigen::Vector3d v0_imu = v0;
    if (odom_child_ == "imu") {
      const Eigen::Matrix3d R_wi = odomToIsometry(odom).linear();
      // Prefer body twist if available: many stacks publish body-frame twist.
      // Keep as-is when already body; user can disable translation deskew if wrong.
      (void)R_wi;
    }

    CloudXT deskewed;
    if (!deskew_.undistort(cloud_raw, t_beg, t_end, v0_imu, deskewed)) {
      ROS_WARN_THROTTLE(2.0, "[lidar_world_mapper] deskew failed (IMU coverage?)");
      // fallback: use raw end-as-is
      deskewed = cloud_raw;
    }

    if (pub_deskew_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(deskewed, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = "lidar";
      pub_deskew_.publish(msg);
    }

    const Eigen::Isometry3d T_wl = lidarPoseFromOdom(odom);
    CloudT body = toXYZI(deskewed);
    CloudT::Ptr world = transformCloud(body, T_wl);
    ingestWorldCloud(*world, stamp, &odom);
  }

  void syncCb(
    const sensor_msgs::PointCloud2ConstPtr & cloud_msg,
    const nav_msgs::OdometryConstPtr & odom_msg)
  {
    CloudT raw;
    pcl::fromROSMsg(*cloud_msg, raw);
    CloudT::Ptr world = transformCloud(raw, odomToIsometry(*odom_msg));
    ingestWorldCloud(*world, cloud_msg->header.stamp, odom_msg.get());
  }

  void cloudOnlyCb(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
  {
    CloudT raw;
    pcl::fromROSMsg(*cloud_msg, raw);
    ingestWorldCloud(raw, cloud_msg->header.stamp, nullptr);
  }

  void ingestWorldCloud(
    const CloudT & in_world, const ros::Time & stamp, const nav_msgs::Odometry * odom_msg)
  {
    if (skip_frames_ > 0 && (++frame_count_ % (skip_frames_ + 1)) != 0) {
      return;
    }
    if (odom_msg && max_speed_for_insert_ >= 0.0) {
      const auto & v = odom_msg->twist.twist.linear;
      const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
      if (speed > max_speed_for_insert_) {
        return;
      }
    }
    if (in_world.empty()) {
      return;
    }

    CloudT::Ptr filtered = filterFrame(in_world);
    if (!filtered || filtered->empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_.insertCloud(*filtered);
    }

    if (pub_scan_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 out;
      pcl::toROSMsg(*filtered, out);
      out.header.stamp = stamp;
      out.header.frame_id = world_frame_;
      pub_scan_.publish(out);
    }
  }

  CloudT::Ptr filterFrame(const CloudT & in_world)
  {
    CloudT::Ptr cloud(new CloudT);
    cloud->points.reserve(in_world.points.size());
    for (const auto & pt : in_world.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const double r = std::sqrt(
        static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y +
        static_cast<double>(pt.z) * pt.z);
      if (r < min_range_ || r > max_range_) {
        continue;
      }
      if (pt.z < z_min_ || pt.z > z_max_) {
        continue;
      }
      cloud->points.push_back(pt);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    if (cloud->empty()) {
      return cloud;
    }

    if (frame_voxel_ > 1e-4) {
      CloudT::Ptr ds(new CloudT);
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(
        static_cast<float>(frame_voxel_), static_cast<float>(frame_voxel_),
        static_cast<float>(frame_voxel_));
      vg.filter(*ds);
      cloud.swap(ds);
    }

    if (enable_sor_ && cloud->size() > static_cast<std::size_t>(sor_mean_k_)) {
      CloudT::Ptr sor_out(new CloudT);
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_);
      sor.filter(*sor_out);
      cloud.swap(sor_out);
    }

    if (enable_ror_ && !cloud->empty()) {
      CloudT::Ptr ror_out(new CloudT);
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(cloud);
      ror.setRadiusSearch(ror_radius_);
      ror.setMinNeighborsInRadius(ror_min_neighbors_);
      ror.filter(*ror_out);
      cloud.swap(ror_out);
    }
    return cloud;
  }

  void mapTimerCb(const ros::TimerEvent &)
  {
    if (pub_map_.getNumSubscribers() == 0) {
      return;
    }
    CloudT::Ptr map_cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (map_.size() == 0) {
        return;
      }
      map_cloud = map_.toCloud();
    }
    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(*map_cloud, out);
    out.header.stamp = ros::Time::now();
    out.header.frame_id = world_frame_;
    pub_map_.publish(out);
  }

  bool saveMapCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
  {
    const bool ok = saveMapLocked("service");
    res.success = ok;
    res.message = ok ? ("saved to " + pcd_path_) : "save failed or empty map";
    return true;
  }

  bool clearMapCb(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_.clear();
    res.success = true;
    res.message = "map cleared";
    return true;
  }

  bool saveMapLocked(const std::string & reason)
  {
    if (!pcd_save_en_) {
      return false;
    }
    CloudT::Ptr map_cloud;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (map_.size() == 0) {
        ROS_WARN("[lidar_world_mapper] skip save (%s): empty map", reason.c_str());
        return false;
      }
      map_cloud = map_.toCloud();
    }

    CloudT cleaned = *map_cloud;
    if (enable_ror_ && !cleaned.empty()) {
      pcl::RadiusOutlierRemoval<PointT> ror;
      ror.setInputCloud(map_cloud);
      ror.setRadiusSearch(std::max(ror_radius_, map_voxel_ * 2.5));
      ror.setMinNeighborsInRadius(std::max(2, ror_min_neighbors_ - 1));
      ror.filter(cleaned);
    }

    const int rc = pcd_binary_ ? pcl::io::savePCDFileBinary(pcd_path_, cleaned)
                               : pcl::io::savePCDFileASCII(pcd_path_, cleaned);
    if (rc != 0) {
      ROS_ERROR("[lidar_world_mapper] failed to write %s", pcd_path_.c_str());
      return false;
    }
    ROS_INFO(
      "[lidar_world_mapper] saved %zu points to %s (%s)", cleaned.size(), pcd_path_.c_str(),
      reason.c_str());
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  boost::shared_ptr<Sync> sync_;
  ros::Subscriber sub_cloud_only_;
  ros::Subscriber sub_livox_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_odom_buf_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_deskew_;
  ros::Publisher pub_map_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer clear_srv_;
  ros::Timer map_timer_;

  std::mutex mutex_;
  mutable std::mutex odom_mutex_;
  lidar_world_mapper::VoxelMap map_;
  lidar_world_mapper::ImuDeskew deskew_;
  std::deque<nav_msgs::OdometryConstPtr> odom_buf_;
  int frame_count_{0};

  std::string mode_;
  std::string lidar_type_;
  std::string cloud_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string odom_child_;
  std::string scan_world_topic_;
  std::string deskew_topic_;
  std::string map_topic_;
  std::string world_frame_;

  std::vector<double> extrinT_;
  std::vector<double> extrinR_;
  Eigen::Matrix3d R_il_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_il_{Eigen::Vector3d::Zero()};

  double blind_{0.5};
  int point_filter_num_{3};
  bool enable_translation_deskew_{true};
  double odom_time_slop_{0.05};

  double min_range_{0.5};
  double max_range_{80.0};
  double z_min_{-2.0};
  double z_max_{5.0};
  double frame_voxel_{0.1};
  double map_voxel_{0.1};

  bool enable_sor_{true};
  int sor_mean_k_{20};
  double sor_stddev_{1.0};
  bool enable_ror_{true};
  double ror_radius_{0.35};
  int ror_min_neighbors_{4};

  int skip_frames_{0};
  double max_speed_for_insert_{-1.0};
  double publish_map_hz_{1.0};

  bool pcd_save_en_{true};
  bool save_on_shutdown_{true};
  std::string pcd_path_;
  bool pcd_binary_{true};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lidar_world_mapper");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  LidarWorldMapper node(nh, pnh);
  ros::spin();
  return 0;
}
