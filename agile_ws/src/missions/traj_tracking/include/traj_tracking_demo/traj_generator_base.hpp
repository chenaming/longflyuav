#pragma once

#include <memory>
#include <string>

#include <nav_msgs/Path.h>

#include "traj_tracking_demo/traj_types.hpp"

namespace traj_tracking_demo {

class TrajGeneratorBase
{
public:
  virtual ~TrajGeneratorBase() = default;

  virtual TrajSample startPoint() const = 0;
  virtual TrajSample sample(double t_sec) const = 0;
  virtual double duration() const = 0;

  virtual bool finished(double t_sec) const
  {
    return t_sec >= duration() - 1e-6;
  }
};

std::unique_ptr<TrajGeneratorBase> createTrajGenerator(const TrajConfig& config);

TrajSample linearSample(double t_sec, double duration_sec,
                        double x0, double y0, double z0,
                        double x1, double y1, double z1);

double yawDegFromVel(double vx, double vy);

/** mode 10：由速度差分估计加速度前馈（与 pos/vel 同帧） */
inline void fillAccFromVelocityDiff(TrajSample& s, const TrajGeneratorBase& gen, double t_sec,
                                    double dt = 0.02)
{
  const TrajSample next = gen.sample(t_sec + dt);
  s.ax = (next.vx - s.vx) / dt;
  s.ay = (next.vy - s.vy) / dt;
  s.az = (next.vz - s.vz) / dt;
}

/** 按空间步长采样整条参考轨迹，供 RViz Path 显示 */
nav_msgs::Path buildReferencePath(const TrajGeneratorBase& gen,
                                  const std::string& frame_id,
                                  double sample_step = 0.05);

}  // namespace traj_tracking_demo
