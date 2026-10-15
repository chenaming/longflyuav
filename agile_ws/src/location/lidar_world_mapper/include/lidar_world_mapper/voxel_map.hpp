#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_world_mapper
{

struct VoxelKey
{
  int32_t x{0};
  int32_t y{0};
  int32_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    // 64-bit mix of three 32-bit integers
    const uint64_t a = static_cast<uint32_t>(key.x);
    const uint64_t b = static_cast<uint32_t>(key.y);
    const uint64_t c = static_cast<uint32_t>(key.z);
    uint64_t h = a * 73856093ull ^ b * 19349663ull ^ c * 83492791ull;
    return static_cast<std::size_t>(h);
  }
};

/**
 * Incremental voxel map: keep one representative point per voxel.
 * Averaging intensity/xyz reduces transient noise from motion blur / multipath.
 */
class VoxelMap
{
public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  explicit VoxelMap(float resolution = 0.1f)
  : resolution_(resolution > 1e-4f ? resolution : 0.1f)
  {
  }

  void setResolution(float resolution)
  {
    resolution_ = resolution > 1e-4f ? resolution : 0.1f;
  }

  void clear()
  {
    voxels_.clear();
  }

  void insertCloud(const CloudT & cloud)
  {
    for (const auto & pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }
      const VoxelKey key = toKey(pt);
      auto it = voxels_.find(key);
      if (it == voxels_.end()) {
        voxels_.emplace(key, AccPoint{pt.x, pt.y, pt.z, pt.intensity, 1});
      } else {
        auto & acc = it->second;
        acc.x += pt.x;
        acc.y += pt.y;
        acc.z += pt.z;
        acc.intensity += pt.intensity;
        ++acc.count;
      }
    }
  }

  CloudT::Ptr toCloud() const
  {
    CloudT::Ptr out(new CloudT);
    out->points.reserve(voxels_.size());
    for (const auto & kv : voxels_) {
      const AccPoint & acc = kv.second;
      PointT pt;
      const float inv = 1.0f / static_cast<float>(acc.count);
      pt.x = acc.x * inv;
      pt.y = acc.y * inv;
      pt.z = acc.z * inv;
      pt.intensity = acc.intensity * inv;
      out->points.push_back(pt);
    }
    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
    return out;
  }

  std::size_t size() const { return voxels_.size(); }

private:
  struct AccPoint
  {
    float x{0.f};
    float y{0.f};
    float z{0.f};
    float intensity{0.f};
    uint32_t count{0};
  };

  VoxelKey toKey(const PointT & pt) const
  {
    VoxelKey key;
    key.x = static_cast<int32_t>(std::floor(pt.x / resolution_));
    key.y = static_cast<int32_t>(std::floor(pt.y / resolution_));
    key.z = static_cast<int32_t>(std::floor(pt.z / resolution_));
    return key;
  }

  float resolution_;
  std::unordered_map<VoxelKey, AccPoint, VoxelKeyHash> voxels_;
};

}  // namespace lidar_world_mapper
