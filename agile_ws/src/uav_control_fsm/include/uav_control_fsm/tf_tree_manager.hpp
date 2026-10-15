#ifndef UAV_CONTROL_FSM_TF_MANAGER_HPP
#define UAV_CONTROL_FSM_TF_MANAGER_HPP

// ROS 与 TF
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <map>

#include "uav_control_fsm/param_set.hpp"

namespace uav_control_fsm {

/**
 * TF 树管理器（从 config/uav_group.yaml 加载）
 *
 * 配置方式：在 launch 的 <group ns="uav0"> 内使用
 *   <rosparam file="$(find uav_control_fsm)/config/uav_group.yaml" subst_value="true" />
 * 将配置载入参数服务器后，本类从 group 命名空间（/uav0、/uav1 等）读取。
 *
 * TF 树结构示例：
 *   world (可选)
 *     └── uav0_map
 *           └── uav0/base_link
 *                 ├── uav0/camera_link
 *                 │     ├── uav0/D435i_link
 *                 │     └── uav0/336_link
 *                 └── uav0/lidar_link
 *                       ├── uav0/mid360_link
 *                       └── uav0/odin_link
 */
class TFManager
{
public:
    /** 构造函数：传入节点句柄与无人机 ID，初始化后从参数服务器加载 TF 树配置 */
    TFManager(ros::NodeHandle& nh, int uav_id);
    ~TFManager() = default;

    /** 根据当前里程计更新并发布整棵 TF 树（world->map->base_link->各传感器） */
    void update(const nav_msgs::Odometry& odom);

    /** 设置传感器相对于 base_link 的变换，单位：位置米，姿态弧度 */
    void setSensorTransform(const std::string& sensor_name,
                            double x, double y, double z,
                            double roll = 0.0, double pitch = 0.0, double yaw = 0.0);

private:
    /** 由平移和四元数构造 geometry_msgs::Transform */
    geometry_msgs::Transform createTransform(double x, double y, double z,
                                             double qx, double qy, double qz, double qw);
    /** 由平移和 tf 四元数构造 geometry_msgs::Transform */
    geometry_msgs::Transform createTransform(double x, double y, double z, const tf::Quaternion& q);
    /** 欧拉角 (roll, pitch, yaw) 转四元数 */
    tf::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

    ros::NodeHandle nh_;              // 节点句柄
    ros::NodeHandle group_nh_;       // 本机命名空间，如 /uav0、/uav1，用于读参数
    tf::TransformBroadcaster tf_broadcaster_;  // TF 广播器，用于发布变换
    tf::TransformListener tf_listener_;       // TF 监听器（预留）
    int uav_id_;                      // 无人机编号
    std::string uav_prefix_;         // 前缀字符串，如 "uav0"
    std::string world_frame_;        // 世界坐标系 ID，如 "world"
    std::string map_frame_;          // 地图坐标系 ID，如 "uav0_map"
    std::string base_link_frame_;    // 机体坐标系 ID，如 "uav0/base_link"

    /** 单个 TF 坐标系配置：变换、是否启用、是否静态、父坐标系名 */
    struct SensorTFConfig
    {
        geometry_msgs::Transform transform;
        bool enabled;
        bool is_static;
        std::string parent; // 空表示挂载在 base_link
    };
    std::map<std::string, SensorTFConfig> sensor_configs_;

    geometry_msgs::Transform world_to_map_transform_;  // world 到 map 的固定变换（多机时用）
    bool enable_world_to_map_;   // 是否发布 world -> map 这一层
    ros::Time last_publish_time_;  // 上次发布 TF 的时间戳，用于避免同一时刻重复发布
};

// ============ 实现 ============

TFManager::TFManager(ros::NodeHandle& nh, int uav_id)
    : nh_(nh), uav_id_(uav_id), tf_listener_(ros::Duration(10.0)),
      enable_world_to_map_(false), last_publish_time_(0, 0)
{
    // 构建本机前缀与 group 命名空间，用于从 /uav0、/uav1 等读参数
    uav_prefix_ = "uav" + std::to_string(uav_id_);
    group_nh_ = ros::NodeHandle("/" + uav_prefix_);
    // 默认坐标系 ID，可由参数覆盖
    world_frame_ = "world";
    map_frame_ = uav_prefix_ + "_map";
    base_link_frame_ = uav_prefix_ + "/base_link";
    // 默认 world->map 为单位变换，若启用 world_to_map 则由参数提供
    world_to_map_transform_ = createTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    // TF 树配置的解析在 param_set 中，这里只消费解析结果
    TFTreeParamSet tf_param;
    tf_param.loadFromParam(group_nh_, uav_prefix_);
    world_frame_ = tf_param.world_frame;
    map_frame_ = tf_param.map_frame;
    base_link_frame_ = tf_param.base_link_frame;
    enable_world_to_map_ = tf_param.enable_world_to_map;
    world_to_map_transform_ = tf_param.world_to_map_transform;

    sensor_configs_.clear();
    for (const auto& kv : tf_param.sensors)
    {
        SensorTFConfig cfg;
        cfg.transform = kv.second.transform;
        cfg.enabled = kv.second.enabled;
        cfg.is_static = kv.second.is_static;
        cfg.parent = kv.second.parent;
        sensor_configs_[kv.first] = cfg;
    }

    ROS_INFO("[uav_control_fsm] UAV%d TF manager initialized", uav_id_);
    if (enable_world_to_map_)
        ROS_INFO("  TF: %s -> %s -> %s", world_frame_.c_str(), map_frame_.c_str(), base_link_frame_.c_str());
    else
        ROS_INFO("  TF: %s -> %s", map_frame_.c_str(), base_link_frame_.c_str());
}

void TFManager::update(const nav_msgs::Odometry& odom)
{
    ros::Time t = odom.header.stamp;
    if (t == last_publish_time_)
        return;
    last_publish_time_ = t;

    // 1. 若启用，发布 world -> map 的固定变换（多机时各机 map 在 world 中的位置）
    if (enable_world_to_map_)
    {
        geometry_msgs::TransformStamped world_to_map;
        world_to_map.header.stamp = t;
        world_to_map.header.frame_id = world_frame_;
        world_to_map.child_frame_id = map_frame_;
        world_to_map.transform = world_to_map_transform_;
        tf_broadcaster_.sendTransform(world_to_map);
    }

    // 2. 发布 map -> base_link：由里程计提供位姿，连接地图与机体
    geometry_msgs::TransformStamped map_to_base;
    map_to_base.header.stamp = t;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_link_frame_;
    map_to_base.transform.translation.x = odom.pose.pose.position.x;
    map_to_base.transform.translation.y = odom.pose.pose.position.y;
    map_to_base.transform.translation.z = odom.pose.pose.position.z;
    map_to_base.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(map_to_base);

    // 3. 发布 base_link / 大类坐标系 -> 各传感器坐标系
    for (const auto& p : sensor_configs_)
    {
        if (!p.second.enabled)
            continue;
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = t;
        tf_stamped.header.frame_id = p.second.parent.empty()
            ? base_link_frame_
            : (uav_prefix_ + "/" + p.second.parent);
        tf_stamped.child_frame_id = uav_prefix_ + "/" + p.first;
        tf_stamped.transform = p.second.transform;
        tf_broadcaster_.sendTransform(tf_stamped);
    }
}

void TFManager::setSensorTransform(const std::string& sensor_name,
                                   double x, double y, double z,
                                   double roll, double pitch, double yaw)
{
    SensorTFConfig cfg;
    cfg.transform = createTransform(x, y, z, eulerToQuaternion(roll, pitch, yaw));
    cfg.enabled = true;
    cfg.is_static = true;
    cfg.parent.clear();
    if (sensor_configs_.count(sensor_name))
    {
        cfg.enabled = sensor_configs_[sensor_name].enabled;
        cfg.is_static = sensor_configs_[sensor_name].is_static;
        cfg.parent = sensor_configs_[sensor_name].parent;
    }
    sensor_configs_[sensor_name] = cfg;
}

geometry_msgs::Transform TFManager::createTransform(double x, double y, double z,
                                                    double qx, double qy, double qz, double qw)
{
    geometry_msgs::Transform t;
    t.translation.x = x;
    t.translation.y = y;
    t.translation.z = z;
    t.rotation.x = qx;
    t.rotation.y = qy;
    t.rotation.z = qz;
    t.rotation.w = qw;
    return t;
}

geometry_msgs::Transform TFManager::createTransform(double x, double y, double z, const tf::Quaternion& q)
{
    return createTransform(x, y, z, q.x(), q.y(), q.z(), q.w());
}

tf::Quaternion TFManager::eulerToQuaternion(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

} // namespace uav_control_fsm

#endif // UAV_CONTROL_FSM_TF_MANAGER_HPP
