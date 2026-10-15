#pragma once

#include <sensor_msgs/Imu.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <vector>

namespace lidar_world_mapper
{

inline Eigen::Matrix3d expSO3(const Eigen::Vector3d & phi)
{
  const double angle = phi.norm();
  Eigen::Matrix3d Eye = Eigen::Matrix3d::Identity();
  if (angle < 1e-8) {
    return Eye;
  }
  const Eigen::Vector3d axis = phi / angle;
  Eigen::Matrix3d K;
  K << 0.0, -axis.z(), axis.y(), axis.z(), 0.0, -axis.x(), -axis.y(), axis.x(), 0.0;
  return Eye + std::sin(angle) * K + (1.0 - std::cos(angle)) * K * K;
}

struct TimedImu
{
  double t{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
};

struct RelPose
{
  double t_offset{0.0};  // seconds from scan begin
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
};

/**
 * IMU-only scan deskew (no LiDAR odometry / EKF).
 * Integrates high-rate IMU across one LiDAR sweep and compensates each point
 * into the scan-end LiDAR frame.
 *
 * Note: absolute mapping still needs an external pose (PX4/VIO/etc).
 */
class ImuDeskew
{
public:
  using PointXT = pcl::PointXYZINormal;
  using CloudXT = pcl::PointCloud<PointXT>;

  void setExtrinsic(const Eigen::Matrix3d & R_imu_lidar, const Eigen::Vector3d & t_imu_lidar)
  {
    R_il_ = R_imu_lidar;
    t_il_ = t_imu_lidar;
  }

  void setGyroBias(const Eigen::Vector3d & bg) { bg_ = bg; }
  void setAccBias(const Eigen::Vector3d & ba) { ba_ = ba; }
  void setGravityInWorld(const Eigen::Vector3d & g_w) { g_w_ = g_w; }
  void setEnableTranslation(bool en) { enable_translation_ = en; }
  void setBufferDuration(double sec) { buffer_duration_ = sec; }

  void pushImu(const sensor_msgs::ImuConstPtr & msg)
  {
    TimedImu sample;
    sample.t = msg->header.stamp.toSec();
    sample.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    sample.acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;

    std::lock_guard<std::mutex> lock(mutex_);
    imu_buf_.push_back(sample);
    while (!imu_buf_.empty() && sample.t - imu_buf_.front().t > buffer_duration_) {
      imu_buf_.pop_front();
    }

    // Static-ish init of gravity / bias (first few seconds).
    if (!imu_inited_) {
      ++init_count_;
      mean_gyro_ += sample.gyro;
      mean_acc_ += sample.acc;
      if (init_count_ >= init_need_) {
        mean_gyro_ /= static_cast<double>(init_count_);
        mean_acc_ /= static_cast<double>(init_count_);
        bg_ = mean_gyro_;
        // Stationary specific-force mean ≈ gravity projected in IMU frame.
        g_imu0_ = mean_acc_;
        imu_inited_ = true;
      }
    }
  }

  bool ready() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return imu_inited_ && imu_buf_.size() > 5;
  }

  /**
   * @param cloud_in  LiDAR points in lidar frame; curvature = offset time in milliseconds
   * @param t_beg     absolute time of first point / scan begin
   * @param t_end     absolute time of last point / scan end
   * @param v0_imu    optional IMU linear velocity at scan begin (world or body-relative).
   *                  For relative integration we use body velocity at begin in IMU frame.
   * @param cloud_out deskewed cloud in lidar frame at t_end
   */
  bool undistort(
    const CloudXT & cloud_in, double t_beg, double t_end, const Eigen::Vector3d & v0_imu,
    CloudXT & cloud_out)
  {
    if (cloud_in.empty() || t_end < t_beg) {
      return false;
    }

    std::deque<TimedImu> imu_slice;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!imu_inited_ || imu_buf_.size() < 2) {
        return false;
      }
      // Keep one sample before begin for integration continuity.
      for (const auto & s : imu_buf_) {
        if (s.t < t_beg - 0.05) {
          continue;
        }
        if (s.t > t_end + 0.05) {
          break;
        }
        imu_slice.push_back(s);
      }
    }
    if (imu_slice.size() < 2) {
      return false;
    }

    std::vector<RelPose> poses;
    poses.reserve(imu_slice.size() + 2);

    RelPose p0;
    p0.t_offset = 0.0;
    p0.R.setIdentity();
    p0.p.setZero();
    p0.v = v0_imu;
    p0.gyro = imu_slice.front().gyro - bg_;
    p0.acc = imu_slice.front().acc - ba_;
    poses.push_back(p0);

    // Cancel gravity using init-mean specific force expressed in scan-begin IMU frame.
    const Eigen::Vector3d g_beg = g_imu0_;

    for (std::size_t i = 0; i + 1 < imu_slice.size(); ++i) {
      const TimedImu & head = imu_slice[i];
      const TimedImu & tail = imu_slice[i + 1];
      if (tail.t <= t_beg) {
        continue;
      }

      double dt = 0.0;
      if (head.t < t_beg) {
        dt = tail.t - t_beg;
      } else {
        dt = tail.t - head.t;
      }
      if (dt <= 0.0) {
        continue;
      }

      const Eigen::Vector3d gyro = 0.5 * (head.gyro + tail.gyro) - bg_;
      const Eigen::Vector3d acc_meas = 0.5 * (head.acc + tail.acc) - ba_;

      RelPose next = poses.back();
      next.R = poses.back().R * expSO3(gyro * dt);
      if (enable_translation_) {
        // a_lin ≈ R * f_meas - g_beg  (approx., good for short LiDAR sweeps)
        const Eigen::Vector3d a = poses.back().R * acc_meas - g_beg;
        next.p = poses.back().p + poses.back().v * dt + 0.5 * a * dt * dt;
        next.v = poses.back().v + a * dt;
        next.acc = a;
      } else {
        next.p = poses.back().p;
        next.v = poses.back().v;
        next.acc = Eigen::Vector3d::Zero();
      }
      next.gyro = gyro;
      next.t_offset = std::max(0.0, tail.t - t_beg);
      poses.push_back(next);
    }

    // Extrapolate / truncate to scan end
    if (poses.back().t_offset < (t_end - t_beg) - 1e-4) {
      const double dt = (t_end - t_beg) - poses.back().t_offset;
      RelPose next = poses.back();
      next.R = poses.back().R * expSO3(poses.back().gyro * dt);
      if (enable_translation_) {
        const Eigen::Vector3d a = poses.back().acc;
        next.p = poses.back().p + poses.back().v * dt + 0.5 * a * dt * dt;
        next.v = poses.back().v + a * dt;
      }
      next.t_offset = t_end - t_beg;
      poses.push_back(next);
    }

    const RelPose & pose_end = poses.back();
    const Eigen::Matrix3d R_e = pose_end.R;
    const Eigen::Vector3d p_e = pose_end.p;

    cloud_out = cloud_in;
    // Sort by time
    std::sort(cloud_out.points.begin(), cloud_out.points.end(), [](const PointXT & a, const PointXT & b) {
      return a.curvature < b.curvature;
    });

    auto it_pt = cloud_out.points.end() - 1;
    for (auto it_pose = poses.end() - 1; it_pose != poses.begin(); --it_pose) {
      const RelPose & head = *(it_pose - 1);
      const RelPose & tail = *it_pose;

      for (; it_pt->curvature / 1000.0 > head.t_offset; --it_pt) {
        const double dt = it_pt->curvature / 1000.0 - head.t_offset;
        const Eigen::Matrix3d R_i = head.R * expSO3(tail.gyro * dt);
        Eigen::Vector3d p_i = head.p;
        if (enable_translation_) {
          p_i = head.p + head.v * dt + 0.5 * tail.acc * dt * dt;
        }

        const Eigen::Vector3d P_l(it_pt->x, it_pt->y, it_pt->z);
        const Eigen::Vector3d P_imu = R_il_ * P_l + t_il_;
        // Point from instant i into scan-begin frame, then to end IMU, then lidar end
        const Eigen::Vector3d P_beg = R_i * P_imu + p_i;
        const Eigen::Vector3d P_imu_end = R_e.transpose() * (P_beg - p_e);
        const Eigen::Vector3d P_l_end = R_il_.transpose() * (P_imu_end - t_il_);

        it_pt->x = static_cast<float>(P_l_end.x());
        it_pt->y = static_cast<float>(P_l_end.y());
        it_pt->z = static_cast<float>(P_l_end.z());

        if (it_pt == cloud_out.points.begin()) {
          break;
        }
      }
    }
    return true;
  }

private:
  mutable std::mutex mutex_;
  std::deque<TimedImu> imu_buf_;
  double buffer_duration_{10.0};

  Eigen::Matrix3d R_il_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_il_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d g_w_{0.0, 0.0, -9.81};
  Eigen::Vector3d g_imu0_{0.0, 0.0, 9.81};
  bool enable_translation_{true};

  bool imu_inited_{false};
  int init_count_{0};
  int init_need_{200};  // ~1s at 200Hz
  Eigen::Vector3d mean_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mean_acc_{Eigen::Vector3d::Zero()};
};

}  // namespace lidar_world_mapper
