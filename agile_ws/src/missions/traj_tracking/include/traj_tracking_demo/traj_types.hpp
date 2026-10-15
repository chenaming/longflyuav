#pragma once

#include <string>

namespace traj_tracking_demo {

struct TrajSample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
  double yaw_deg{0.0};
};

struct TrajConfig
{
  std::string type{"circle"};
  double center_x{0.0};
  double center_y{0.0};
  double altitude{1.5};
  double speed{1.0};
  double radius{3.0};
  double side_length{3.0};
  int num_laps{1};
  double start_angle_deg{0.0};
  bool clockwise{true};
};

enum class FinishAction
{
  Hover,
  Land,
  ReturnOriginLand
};

enum class DemoState
{
  Idle,
  Ready,           // 等待 odom（假定已由 FSM 进入 OFFBOARD/悬停）
  GoToStart,
  Tracking,
  ReturnToOrigin,
  FinishHover,
  FinishLand,
  Done
};

FinishAction finishActionFromString(const std::string& s);
const char* finishActionToString(FinishAction action);
const char* demoStateToString(DemoState state);

}  // namespace traj_tracking_demo
