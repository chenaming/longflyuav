/**
 * @file uav_path_recorder_node.cpp
 * @brief 记录无人机实际轨迹与 move_command 期望指令（宽表 CSV），并发布 Path 供 RViz。
 *
 * 订阅：
 *   - /uav{id}/mavros/local_position/odom
 *   - /uav{id}/mavros/state
 *   - /uav{id}/mavros/extended_state
 *   - /uav{id}/move_command
 * 发布：
 *   - /uav{id}/flight_path  (nav_msgs/Path, frame: uav{id}_map)
 *
 * CSV（save_to_file=true，解锁后写、上锁停）：
 *   宽表 = 实际位姿 + Move_mode + 期望指令；
 *   move_command 按消息全量写入（约 100Hz）；
 *   odom 仍按 min_dist 写入（便于无指令时段保留实际轨迹）。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <uav_control_fsm/uav_command.h>

#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>

class UavPathRecorder
{
public:
  UavPathRecorder(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("uav_id", uav_id_, 0);
    pnh.param("min_dist", min_dist_, 0.05);
    pnh.param("max_points", max_points_, 10000);
    pnh.param("save_to_file", save_to_file_, false);
    pnh.param("save_dir", save_dir_, std::string("~/flight_logs"));
    pnh.param("stop_on_landing", stop_on_landing_, false);
    pnh.param("clear_path_on_rearm", clear_path_on_rearm_, true);
    pnh.param("path_publish_rate", path_publish_rate_, 10.0);

    if (min_dist_ < 0.0)
      min_dist_ = 0.0;
    if (max_points_ < 2)
      max_points_ = 2;
    if (path_publish_rate_ <= 0.0)
      path_publish_rate_ = 10.0;

    const std::string uav_prefix = "uav" + std::to_string(uav_id_);
    const std::string default_map_frame = uav_prefix + "_map";
    const std::string default_odom_topic = "/" + uav_prefix + "/mavros/local_position/odom";
    const std::string default_state_topic = "/" + uav_prefix + "/mavros/state";
    const std::string default_extended_state_topic = "/" + uav_prefix + "/mavros/extended_state";
    const std::string default_path_topic = "/" + uav_prefix + "/flight_path";
    const std::string default_move_command_topic = "/" + uav_prefix + "/move_command";

    pnh.param("map_frame", map_frame_, default_map_frame);
    pnh.param("odom_topic", odom_topic_, default_odom_topic);
    pnh.param("state_topic", state_topic_, default_state_topic);
    pnh.param("extended_state_topic", extended_state_topic_, default_extended_state_topic);
    pnh.param("path_topic", path_topic_, default_path_topic);
    pnh.param("move_command_topic", move_command_topic_, default_move_command_topic);

    path_.header.frame_id = map_frame_;

    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 10, true);
    odom_sub_ = nh.subscribe(odom_topic_, 50, &UavPathRecorder::odomCallback, this);
    state_sub_ = nh.subscribe(state_topic_, 10, &UavPathRecorder::stateCallback, this);
    extended_state_sub_ =
        nh.subscribe(extended_state_topic_, 10, &UavPathRecorder::extendedStateCallback, this);
    move_command_sub_ =
        nh.subscribe(move_command_topic_, 100, &UavPathRecorder::moveCommandCallback, this);

    const double timer_period = 1.0 / path_publish_rate_;
    publish_timer_ = nh.createTimer(ros::Duration(timer_period), &UavPathRecorder::publishTimerCallback, this);

    ROS_INFO("[path_recorder] uav_id=%d", uav_id_);
    ROS_INFO("[path_recorder] odom topic : %s", odom_topic_.c_str());
    ROS_INFO("[path_recorder] cmd topic  : %s", move_command_topic_.c_str());
    ROS_INFO("[path_recorder] path topic : %s", path_topic_.c_str());
    ROS_INFO("[path_recorder] map frame  : %s", map_frame_.c_str());
    ROS_INFO("[path_recorder] save_to_file=%s, stop_on_landing=%s",
             save_to_file_ ? "true" : "false",
             stop_on_landing_ ? "true" : "false");
  }

  ~UavPathRecorder()
  {
    closeFile();
  }

private:
  static std::string expandHomePath(const std::string& path)
  {
    if (path.empty() || path[0] != '~')
      return path;
    const char* home = getenv("HOME");
    if (!home)
      return path;
    if (path.size() == 1)
      return std::string(home);
    if (path[1] == '/')
      return std::string(home) + path.substr(1);
    return path;
  }

  static bool ensureDirectory(const std::string& dir)
  {
    if (dir.empty())
      return false;
    struct stat st;
    if (stat(dir.c_str(), &st) == 0)
      return S_ISDIR(st.st_mode);
    return mkdir(dir.c_str(), 0755) == 0;
  }

  static std::string makeTimestampString()
  {
    const ros::Time now = ros::Time::now();
    const time_t sec = static_cast<time_t>(now.sec);
    struct tm tm_local;
    localtime_r(&sec, &tm_local);
    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y%m%d_%H%M%S");
    oss << "_" << std::setw(3) << std::setfill('0') << (now.nsec / 1000000);
    return oss.str();
  }

  bool isOnGround() const
  {
    if (!has_extended_state_)
      return false;
    return extended_state_.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
  }

  bool shouldRecord() const
  {
    if (!armed_)
      return false;
    if (stop_on_landing_ && isOnGround())
      return false;
    return true;
  }

  void handleArmTransition(bool armed_now)
  {
    if (armed_now && !last_armed_)
    {
      if (clear_path_on_rearm_)
        clearPath();
      if (save_to_file_)
        openFile();
      ROS_INFO("[path_recorder] UAV%d: start recording", uav_id_);
    }
    else if (!armed_now && last_armed_)
    {
      closeFile();
      path_dirty_ = true;
      ROS_INFO("[path_recorder] UAV%d: stop recording (disarmed)", uav_id_);
    }
    last_armed_ = armed_now;
  }

  void clearPath()
  {
    path_.poses.clear();
    has_last_recorded_point_ = false;
    has_last_odom_ = false;
    has_last_cmd_ = false;
    path_.header.stamp = ros::Time::now();
    path_dirty_ = true;
  }

  void openFile()
  {
    closeFile();

    const std::string dir = expandHomePath(save_dir_);
    if (!ensureDirectory(dir))
    {
      ROS_ERROR("[path_recorder] UAV%d: failed to create save dir: %s", uav_id_, dir.c_str());
      return;
    }

    const std::string filename = dir + "/uav" + std::to_string(uav_id_) + "_trajectory_" +
                                 makeTimestampString() + ".csv";
    file_stream_.open(filename, std::ios::out | std::ios::trunc);
    if (!file_stream_.is_open())
    {
      ROS_ERROR("[path_recorder] UAV%d: failed to open file: %s", uav_id_, filename.c_str());
      return;
    }

    file_stream_ << "timestamp,event,"
                    "actual_x,actual_y,actual_z,actual_yaw,"
                    "Move_mode,"
                    "cmd_pos_x,cmd_pos_y,cmd_pos_z,"
                    "cmd_vel_x,cmd_vel_y,cmd_vel_z,"
                    "cmd_acc_x,cmd_acc_y,cmd_acc_z,"
                    "cmd_yaw_ref,cmd_yaw_rate_ref\n";
    file_stream_ << std::fixed << std::setprecision(6);
    current_file_path_ = filename;
    ROS_INFO("[path_recorder] UAV%d: saving to %s", uav_id_, filename.c_str());
  }

  void closeFile()
  {
    if (file_stream_.is_open())
    {
      file_stream_.flush();
      file_stream_.close();
      if (!current_file_path_.empty())
      {
        ROS_INFO("[path_recorder] UAV%d: closed file %s", uav_id_, current_file_path_.c_str());
        current_file_path_.clear();
      }
    }
  }

  void writeCsvField(double value, bool valid)
  {
    if (valid)
      file_stream_ << value;
  }

  void writeCsvRow(double stamp, const std::string& event)
  {
    if (!file_stream_.is_open())
      return;

    file_stream_ << stamp << "," << event << ",";

    writeCsvField(last_odom_x_, has_last_odom_);
    file_stream_ << ",";
    writeCsvField(last_odom_y_, has_last_odom_);
    file_stream_ << ",";
    writeCsvField(last_odom_z_, has_last_odom_);
    file_stream_ << ",";
    writeCsvField(last_odom_yaw_, has_last_odom_);
    file_stream_ << ",";

    if (has_last_cmd_)
      file_stream_ << static_cast<int>(last_cmd_.Move_mode);
    file_stream_ << ",";

    writeCsvField(last_cmd_.pos_ref.x, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.pos_ref.y, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.pos_ref.z, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.vel_ref.x, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.vel_ref.y, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.vel_ref.z, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.acc_ref.x, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.acc_ref.y, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.acc_ref.z, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.yaw_ref, has_last_cmd_);
    file_stream_ << ",";
    writeCsvField(last_cmd_.yaw_rate_ref, has_last_cmd_);
    file_stream_ << "\n";
  }

  void updateCachedOdom(const nav_msgs::Odometry& odom)
  {
    const geometry_msgs::Point& p = odom.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      return;

    last_odom_x_ = p.x;
    last_odom_y_ = p.y;
    last_odom_z_ = p.z;
    last_odom_yaw_ = tf::getYaw(odom.pose.pose.orientation);
    has_last_odom_ = true;
  }

  void appendPathPoint(const nav_msgs::Odometry& odom)
  {
    const geometry_msgs::Point& p = odom.pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      return;

    if (has_last_recorded_point_)
    {
      const double dx = p.x - last_recorded_point_.x;
      const double dy = p.y - last_recorded_point_.y;
      const double dz = p.z - last_recorded_point_.z;
      if ((dx * dx + dy * dy + dz * dz) < min_dist_ * min_dist_)
        return;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.header.frame_id = map_frame_;
    pose_stamped.pose = odom.pose.pose;
    path_.poses.push_back(pose_stamped);
    path_.header.stamp = pose_stamped.header.stamp;

    if (static_cast<int>(path_.poses.size()) > max_points_)
      path_.poses.erase(path_.poses.begin());

    last_recorded_point_ = p;
    has_last_recorded_point_ = true;
    path_dirty_ = true;

    const double stamp = odom.header.stamp.isZero() ? ros::Time::now().toSec() : odom.header.stamp.toSec();
    writeCsvRow(stamp, "odom");
  }

  void stateCallback(const mavros_msgs::StateConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    armed_ = msg->armed;
    handleArmTransition(armed_);
  }

  void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    extended_state_ = *msg;
    has_extended_state_ = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    updateCachedOdom(*msg);
    if (!shouldRecord())
      return;
    appendPathPoint(*msg);
  }

  void moveCommandCallback(const uav_control_fsm::uav_command::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_cmd_ = *msg;
    has_last_cmd_ = true;
    if (!shouldRecord())
      return;

    const double stamp = msg->header.stamp.isZero() ? ros::Time::now().toSec() : msg->header.stamp.toSec();
    writeCsvRow(stamp, "cmd");
  }

  void publishTimerCallback(const ros::TimerEvent&)
  {
    nav_msgs::Path path_to_pub;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!path_dirty_)
        return;
      path_to_pub = path_;
      path_dirty_ = false;
    }
    path_pub_.publish(path_to_pub);
  }

  int uav_id_;
  double min_dist_;
  int max_points_;
  bool save_to_file_;
  std::string save_dir_;
  bool stop_on_landing_;
  bool clear_path_on_rearm_;
  double path_publish_rate_;

  std::string map_frame_;
  std::string odom_topic_;
  std::string state_topic_;
  std::string extended_state_topic_;
  std::string path_topic_;
  std::string move_command_topic_;

  bool armed_ = false;
  bool last_armed_ = false;
  bool has_extended_state_ = false;
  mavros_msgs::ExtendedState extended_state_;

  nav_msgs::Path path_;
  bool path_dirty_ = false;
  bool has_last_recorded_point_ = false;
  geometry_msgs::Point last_recorded_point_;

  bool has_last_odom_ = false;
  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  double last_odom_z_ = 0.0;
  double last_odom_yaw_ = 0.0;

  bool has_last_cmd_ = false;
  uav_control_fsm::uav_command last_cmd_;

  std::ofstream file_stream_;
  std::string current_file_path_;

  std::mutex mutex_;

  ros::Publisher path_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber extended_state_sub_;
  ros::Subscriber move_command_sub_;
  ros::Timer publish_timer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_path_recorder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  UavPathRecorder recorder(nh, pnh);
  ros::spin();
  return 0;
}
