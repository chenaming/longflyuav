#include "traj_tracking_demo/traj_generator_base.hpp"
#include "traj_tracking_demo/traj_types.hpp"

#include <algorithm>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <ros/ros.h>
#include <stdexcept>

namespace traj_tracking_demo {

namespace {

constexpr double kPi = M_PI;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace

FinishAction finishActionFromString(const std::string& s)
{
  if (s == "land") return FinishAction::Land;
  if (s == "return_origin_land") return FinishAction::ReturnOriginLand;
  return FinishAction::Hover;
}

const char* finishActionToString(FinishAction action)
{
  switch (action)
  {
    case FinishAction::Land: return "land";
    case FinishAction::ReturnOriginLand: return "return_origin_land";
    default: return "hover";
  }
}

const char* demoStateToString(DemoState state)
{
  switch (state)
  {
    case DemoState::Ready: return "READY";
    case DemoState::GoToStart: return "GO_TO_START";
    case DemoState::Tracking: return "TRACKING";
    case DemoState::ReturnToOrigin: return "RETURN_TO_ORIGIN";
    case DemoState::FinishHover: return "FINISH_HOVER";
    case DemoState::FinishLand: return "FINISH_LAND";
    case DemoState::Done: return "DONE";
    default: return "IDLE";
  }
}

double yawDegFromVel(double vx, double vy)
{
  if (std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9) return 0.0;
  return std::atan2(vy, vx) * kRadToDeg;
}

TrajSample linearSample(double t_sec, double duration_sec,
                        double x0, double y0, double z0,
                        double x1, double y1, double z1)
{
  TrajSample s;
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dz = z1 - z0;

  if (duration_sec <= 1e-6 || t_sec >= duration_sec)
  {
    s.x = x1;
    s.y = y1;
    s.z = z1;
    s.vx = 0.0;
    s.vy = 0.0;
    s.vz = 0.0;
  }
  else
  {
    const double frac = std::max(0.0, t_sec) / duration_sec;
    s.x = x0 + dx * frac;
    s.y = y0 + dy * frac;
    s.z = z0 + dz * frac;
    s.vx = dx / duration_sec;
    s.vy = dy / duration_sec;
    s.vz = dz / duration_sec;
  }

  s.yaw_deg = yawDegFromVel(s.vx, s.vy);
  return s;
}

namespace {

class CircleGenerator : public TrajGeneratorBase
{
public:
  explicit CircleGenerator(const TrajConfig& c)
    : cx_(c.center_x), cy_(c.center_y), z_(c.altitude), r_(std::max(0.1, c.radius)),
      speed_(std::max(0.05, c.speed)), start_rad_(c.start_angle_deg * kDegToRad),
      num_laps_(std::max(1, c.num_laps)), sign_(c.clockwise ? -1.0 : 1.0)
  {
  }

  TrajSample startPoint() const override
  {
    return sample(0.0);
  }

  TrajSample sample(double t_sec) const override
  {
    const double omega = sign_ * speed_ / r_;
    const double theta = start_rad_ + omega * t_sec;
    TrajSample s;
    s.x = cx_ + r_ * std::cos(theta);
    s.y = cy_ + r_ * std::sin(theta);
    s.z = z_;
    s.vx = -r_ * omega * std::sin(theta);
    s.vy = r_ * omega * std::cos(theta);
    s.vz = 0.0;
    s.yaw_deg = yawDegFromVel(s.vx, s.vy);
    return s;
  }

  double duration() const override
  {
    return num_laps_ * 2.0 * kPi * r_ / speed_;
  }

private:
  double cx_, cy_, z_, r_, speed_, start_rad_, sign_;
  int num_laps_;
};

// Point on axis-aligned square perimeter by arc length (CCW), centered at (cx,cy).
void squarePointAtArc(double cx, double cy, double half, double arc, double& x, double& y,
                      double& vx, double& vy, double speed, int sign)
{
  const double p = 8.0 * half;
  arc = std::fmod(arc, p);
  if (arc < 0.0) arc += p;

  const double s = speed * static_cast<double>(sign);

  if (arc < 2.0 * half)
  {
    x = cx - half + arc;
    y = cy - half;
    vx = s;
    vy = 0.0;
  }
  else if (arc < 4.0 * half)
  {
    const double d = arc - 2.0 * half;
    x = cx + half;
    y = cy - half + d;
    vx = 0.0;
    vy = s;
  }
  else if (arc < 6.0 * half)
  {
    const double d = arc - 4.0 * half;
    x = cx + half - d;
    y = cy + half;
    vx = -s;
    vy = 0.0;
  }
  else
  {
    const double d = arc - 6.0 * half;
    x = cx - half;
    y = cy + half - d;
    vx = 0.0;
    vy = -s;
  }
}

double squareArcFromAngle(double /*cx*/, double /*cy*/, double half, double angle_rad)
{
  const double lx = std::cos(angle_rad);
  const double ly = std::sin(angle_rad);
  const double ax = std::abs(lx);
  const double ay = std::abs(ly);
  double t = half / std::max(ax, 1e-9);
  if (ay > 1e-9)
    t = std::min(t, half / ay);

  const double px = lx * t;
  const double py = ly * t;

  if (std::abs(py + half) < 1e-6 && px >= -half && px <= half)
    return px + half;
  if (std::abs(px - half) < 1e-6 && py >= -half && py <= half)
    return 2.0 * half + (py + half);
  if (std::abs(py - half) < 1e-6 && px <= half && px >= -half)
    return 4.0 * half + (half - px);
  return 6.0 * half + (half - py);
}

class SquareGenerator : public TrajGeneratorBase
{
public:
  explicit SquareGenerator(const TrajConfig& c)
    : cx_(c.center_x), cy_(c.center_y), z_(c.altitude), half_(std::max(0.1, c.side_length) * 0.5),
      speed_(std::max(0.05, c.speed)), num_laps_(std::max(1, c.num_laps)),
      sign_(c.clockwise ? -1 : 1),
      start_arc_(squareArcFromAngle(cx_, cy_, half_, c.start_angle_deg * kDegToRad)),
      perimeter_(8.0 * half_)
  {
  }

  TrajSample startPoint() const override
  {
    return sample(0.0);
  }

  TrajSample sample(double t_sec) const override
  {
    const double dist = std::min(speed_ * t_sec, perimeter_ * num_laps_);
    const double arc = start_arc_ + sign_ * dist;
    TrajSample s;
    s.z = z_;
    squarePointAtArc(cx_, cy_, half_, arc, s.x, s.y, s.vx, s.vy, speed_, sign_);
    s.vz = 0.0;
    s.yaw_deg = yawDegFromVel(s.vx, s.vy);
    return s;
  }

  double duration() const override
  {
    return perimeter_ * num_laps_ / speed_;
  }

private:
  double cx_, cy_, z_, half_, speed_;
  int num_laps_, sign_;
  double start_arc_, perimeter_;
};

void polygonPointAtArc(double cx, double cy, int n, double circum_r, double arc,
                       double& x, double& y, double& vx, double& vy, double speed, int sign)
{
  const double side = 2.0 * circum_r * std::sin(kPi / static_cast<double>(n));
  const double perimeter = side * static_cast<double>(n);
  arc = std::fmod(arc, perimeter);
  if (arc < 0.0) arc += perimeter;

  const int edge = std::min(n - 1, static_cast<int>(arc / side));
  const double along = arc - static_cast<double>(edge) * side;
  const double base_angle = -kPi * 0.5 + 2.0 * kPi * static_cast<double>(edge) / static_cast<double>(n);
  const double next_angle = base_angle + 2.0 * kPi / static_cast<double>(n);

  const double x0 = cx + circum_r * std::cos(base_angle);
  const double y0 = cy + circum_r * std::sin(base_angle);
  const double x1 = cx + circum_r * std::cos(next_angle);
  const double y1 = cy + circum_r * std::sin(next_angle);

  const double frac = along / side;
  x = x0 + (x1 - x0) * frac;
  y = y0 + (y1 - y0) * frac;

  const double inv_side = 1.0 / side;
  vx = (x1 - x0) * inv_side * speed * static_cast<double>(sign);
  vy = (y1 - y0) * inv_side * speed * static_cast<double>(sign);
}

double polygonArcFromAngle(int n, double circum_r, double angle_rad)
{
  const double side = 2.0 * circum_r * std::sin(kPi / static_cast<double>(n));
  const double px = std::cos(angle_rad);
  const double py = std::sin(angle_rad);
  int edge = 0;
  double best = 0.0;
  double min_err = 1e9;

  for (int i = 0; i < n; ++i)
  {
    const double a0 = -kPi * 0.5 + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    const double a1 = a0 + 2.0 * kPi / static_cast<double>(n);
    const double vx = std::cos(a1) - std::cos(a0);
    const double vy = std::sin(a1) - std::sin(a0);
    const double denom = vx * vx + vy * vy;
    if (denom < 1e-12) continue;
    double t = ((px - std::cos(a0)) * vx + (py - std::sin(a0)) * vy) / denom;
    t = std::max(0.0, std::min(1.0, t));
    const double ex = std::cos(a0) + vx * t - px;
    const double ey = std::sin(a0) + vy * t - py;
    const double err = ex * ex + ey * ey;
    if (err < min_err)
    {
      min_err = err;
      edge = i;
      best = static_cast<double>(edge) * side + t * side;
    }
  }
  (void)min_err;
  return best;
}

class TriangleGenerator : public TrajGeneratorBase
{
public:
  explicit TriangleGenerator(const TrajConfig& c)
    : cx_(c.center_x), cy_(c.center_y), z_(c.altitude),
      circum_r_(std::max(0.1, c.side_length) / std::sqrt(3.0)),
      speed_(std::max(0.05, c.speed)),
      start_arc_(polygonArcFromAngle(3, circum_r_, c.start_angle_deg * kDegToRad)),
      perimeter_(3.0 * 2.0 * circum_r_ * std::sin(kPi / 3.0)),
      num_laps_(std::max(1, c.num_laps)), sign_(c.clockwise ? -1 : 1)
  {
  }

  TrajSample startPoint() const override { return sample(0.0); }

  TrajSample sample(double t_sec) const override
  {
    const double dist = std::min(speed_ * t_sec, perimeter_ * num_laps_);
    const double arc = start_arc_ + sign_ * dist;
    TrajSample s;
    s.z = z_;
    polygonPointAtArc(cx_, cy_, 3, circum_r_, arc, s.x, s.y, s.vx, s.vy, speed_, sign_);
    s.vz = 0.0;
    s.yaw_deg = yawDegFromVel(s.vx, s.vy);
    return s;
  }

  double duration() const override { return perimeter_ * num_laps_ / speed_; }

private:
  double cx_, cy_, z_, circum_r_, speed_, start_arc_, perimeter_;
  int num_laps_, sign_;
};

class HexagonGenerator : public TrajGeneratorBase
{
public:
  explicit HexagonGenerator(const TrajConfig& c)
    : cx_(c.center_x), cy_(c.center_y), z_(c.altitude),
      side_(std::max(0.1, c.side_length)),
      circum_r_(side_),
      speed_(std::max(0.05, c.speed)),
      start_arc_(polygonArcFromAngle(6, circum_r_, c.start_angle_deg * kDegToRad)),
      perimeter_(6.0 * side_),
      num_laps_(std::max(1, c.num_laps)), sign_(c.clockwise ? -1 : 1)
  {
  }

  TrajSample startPoint() const override { return sample(0.0); }

  TrajSample sample(double t_sec) const override
  {
    const double dist = std::min(speed_ * t_sec, perimeter_ * num_laps_);
    const double arc = start_arc_ + sign_ * dist;
    TrajSample s;
    s.z = z_;
    polygonPointAtArc(cx_, cy_, 6, circum_r_, arc, s.x, s.y, s.vx, s.vy, speed_, sign_);
    s.vz = 0.0;
    s.yaw_deg = yawDegFromVel(s.vx, s.vy);
    return s;
  }

  double duration() const override { return perimeter_ * num_laps_ / speed_; }

private:
  double cx_, cy_, z_, side_, circum_r_, speed_, start_arc_, perimeter_;
  int num_laps_, sign_;
};

double figure8ArcLength(double r, int samples = 800)
{
  r = std::max(0.1, r);
  samples = std::max(64, samples);
  double length = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  for (int i = 0; i <= samples; ++i)
  {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
    const double x = r * std::sin(theta);
    const double y = 0.5 * r * std::sin(2.0 * theta);
    if (i > 0)
      length += std::hypot(x - prev_x, y - prev_y);
    prev_x = x;
    prev_y = y;
  }
  return length;
}

class Figure8Generator : public TrajGeneratorBase
{
public:
  explicit Figure8Generator(const TrajConfig& c)
    : cx_(c.center_x), cy_(c.center_y), z_(c.altitude), r_(std::max(0.1, c.radius)),
      speed_(std::max(0.05, c.speed)),
      start_rad_(c.start_angle_deg * kDegToRad),
      omega_(speed_ / std::max(r_, 0.1)),
      num_laps_(std::max(1, c.num_laps))
  {
  }

  TrajSample startPoint() const override { return sample(0.0); }

  TrajSample sample(double t_sec) const override
  {
    const double theta = start_rad_ + omega_ * t_sec;
    TrajSample s;
    s.x = cx_ + r_ * std::sin(theta);
    s.y = cy_ + 0.5 * r_ * std::sin(2.0 * theta);
    s.z = z_;
    s.vx = r_ * omega_ * std::cos(theta);
    s.vy = r_ * omega_ * std::cos(2.0 * theta);
    s.vz = 0.0;
    s.yaw_deg = yawDegFromVel(s.vx, s.vy);
    return s;
  }

  double duration() const override
  {
    return num_laps_ * figure8ArcLength(r_) / speed_;
  }

private:
  double cx_, cy_, z_, r_, speed_, start_rad_, omega_;
  int num_laps_;
};

}  // namespace

std::unique_ptr<TrajGeneratorBase> createTrajGenerator(const TrajConfig& config)
{
  if (config.type == "square") return std::unique_ptr<TrajGeneratorBase>(new SquareGenerator(config));
  if (config.type == "triangle") return std::unique_ptr<TrajGeneratorBase>(new TriangleGenerator(config));
  if (config.type == "hexagon") return std::unique_ptr<TrajGeneratorBase>(new HexagonGenerator(config));
  if (config.type == "figure8") return std::unique_ptr<TrajGeneratorBase>(new Figure8Generator(config));
  if (config.type == "circle") return std::unique_ptr<TrajGeneratorBase>(new CircleGenerator(config));
  throw std::invalid_argument("unknown traj_type: " + config.type);
}

nav_msgs::Path buildReferencePath(const TrajGeneratorBase& gen,
                                  const std::string& frame_id,
                                  double sample_step)
{
  nav_msgs::Path path;
  path.header.frame_id = frame_id;
  sample_step = std::max(0.01, sample_step);

  const double duration_sec = gen.duration();
  const double dt = 0.02;
  bool has_last = false;
  double last_x = 0.0;
  double last_y = 0.0;
  double last_z = 0.0;

  auto appendSample = [&](const TrajSample& s) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = s.x;
    pose.pose.position.y = s.y;
    pose.pose.position.z = s.z;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    last_x = s.x;
    last_y = s.y;
    last_z = s.z;
    has_last = true;
  };

  for (double t = 0.0; t <= duration_sec; t += dt)
  {
    const TrajSample s = gen.sample(t);
    if (!has_last)
    {
      appendSample(s);
      path.header.stamp = ros::Time::now();
      continue;
    }
    const double dx = s.x - last_x;
    const double dy = s.y - last_y;
    const double dz = s.z - last_z;
    if (std::hypot(std::hypot(dx, dy), dz) >= sample_step)
      appendSample(s);
  }

  const TrajSample end_pt = gen.sample(duration_sec);
  if (!has_last)
    appendSample(end_pt);
  else
  {
    const double dx = end_pt.x - last_x;
    const double dy = end_pt.y - last_y;
    const double dz = end_pt.z - last_z;
    if (std::hypot(std::hypot(dx, dy), dz) > 1e-6)
      appendSample(end_pt);
  }

  if (path.poses.empty())
    path.header.stamp = ros::Time::now();
  else
    path.header.stamp = ros::Time::now();

  return path;
}

}  // namespace traj_tracking_demo
