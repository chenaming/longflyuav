/*********************************************************
odometry_bridge.hpp: 按当前定位源订阅对应里程计话题，转换为 mavros 的 vision_pose 供 PX4 使用。
- 启动时根据 default_location_source 只订阅对应话题（Fast_Lio / Vins / Gazebo）。
- 运行时由 FSM 在 set_location_source_cb 中根据定位源调用 setLocationSource()，不再在 bridge 内订阅 set_location_source。
- 收到数据后发布到 /uavX/mavros/vision_pose/pose。
- 可选参数 ~odometry_bridge/vision_pose_pub_rate_hz：>0 时按该频率定时发布最新位姿（可高于传感器）；<=0 时与传感器同频。
- 定位源为 GPS 时不订阅、不发布 vision_pose。
- 实现位于 src/odometry_bridge.cpp，编译为动态库 uav_odometry_bridge。
*********************************************************/
#ifndef UAV_CONTROL_FSM_ODOMETRY_BRIDGE_HPP
#define UAV_CONTROL_FSM_ODOMETRY_BRIDGE_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/timer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>

class OdometryBridge
{
public:
    OdometryBridge(ros::NodeHandle& nh, int uav_id);
    ~OdometryBridge();  // 停止定时器，避免回调在析构后触发

    /** 由 FSM 在切换定位源时调用，根据 source 更新订阅的里程计话题（GPS 时不订阅） */
    void setLocationSource(const std::string& source);

private:
    /** 根据 set_location_source 返回要订阅的里程计话题后缀，空串表示不订阅（如 GPS） */
    static std::string getOdomTopicFromLocationSource(const std::string& set_location_source);
    /** 按当前 set_location_source_ 更新订阅（只保留一个活跃订阅） */
    void updateSubscription();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void timerVisionPosePub(const ros::TimerEvent& e);

    ros::NodeHandle nh_;
    // 独立回调队列：让 vision_pose 的订阅/定时器不受主循环 spinOnce() 频率影响
    ros::CallbackQueue cb_queue_;
    ros::NodeHandle cb_nh_;
    std::unique_ptr<ros::AsyncSpinner> cb_spinner_;
    int uav_id_;
    std::string prefix_;
    std::string set_location_source_;
    std::string odom_sub_topic_;
    ros::Subscriber odom_sub_;
    ros::Publisher vision_pose_pub_;
    ros::Timer vision_pose_timer_;
    bool vision_pose_pub_created_;  // 仅当定位源非 GPS 时创建一次，避免 GPS 时发布
    double vision_pose_pub_rate_hz_;  // <=0：每帧里程计发布；>0：定时器按该 Hz 重复发最新位姿
    double vision_pose_max_age_s_;
    ros::Time last_published_stamp_;
    bool timer_publish_enabled_;
    bool vision_pose_timer_armed_;
    bool have_odom_for_vision_;
    geometry_msgs::PoseStamped vision_pose_;

    void stopVisionPoseTimer();
    void startVisionPoseTimer();
    void publishFreshVisionPose();
};

#endif
