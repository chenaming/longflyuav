/*********************************************************
param_set.hpp: 参数设置器，从 ROS 参数服务器加载参数，并设置 PX4 参数。
包括：
1. 控制相关参数：..., rc_offboard_channel（三档开关单通道）, rc_offboard_enter/exit_threshold
2. TF 树参数：world_frame, map_frame, base_link_frame, enable_world_to_map, world_to_map_transform, sensors
3. PX4 参数：applyParamsFromLocationProfileIfNeeded, waitForService, rebootFCU

控制相关参数：
1. uav_id: 无人机ID
2. loop_hz: 循环频率
3. takeoff_height: 起飞高度
4. no_RC: 是否不使用RC
5. cmd_timeout_sec: 命令超时时间
6. odom_timeout_sec: 位置超时时间
7. rc_timeout_sec: RC超时时间
8. rc_offboard_channel: 三档开关所在 RC 通道（1-based，默认 6）
9. rc_offboard_enter_threshold: 高档阈值，归一化值 > 此值 -> 进入 Offboard
10. rc_offboard_exit_threshold: 与 enter 围成中档区间 [exit, enter] -> 退出 Offboard；低档（< exit）保持当前状态
*********************************************************/
#ifndef UAV_CONTROL_FSM_PARAM_SET_HPP
#define UAV_CONTROL_FSM_PARAM_SET_HPP

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <geometry_msgs/Transform.h>
#include <tf/tf.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/CommandLong.h>

#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <algorithm>

#include "uav_control_fsm/control_manager.hpp"

namespace uav_control_fsm {

/** PX4 参数对比后按需写入的结果统计 */
struct Px4ApplyResult
{
    int params_checked{0};
    int params_unchanged{0};
    int params_changed{0};
    int params_failed{0};
    bool profile_valid{false};

    bool all_matched() const
    {
        return profile_valid && params_checked > 0 &&
               params_changed == 0 && params_failed == 0;
    }
};

/** 控制相关参数：从 ROS 参数服务器加载，状态机只读使用 */
struct ParamSet
{
    int uav_id; // 无人机ID
    double loop_hz; // 循环频率
    double takeoff_height; // 起飞高度
    bool no_RC; // 是否不使用RC
    double cmd_timeout_sec; // 命令超时时间
    double odom_timeout_sec; // 位置超时时间
    double rc_timeout_sec; // RC超时时间
    int rc_offboard_channel;         // 三档开关 RC 通道（1-based）
    double rc_offboard_enter_threshold;  // 高档：> 此值进入 Offboard
    double rc_offboard_exit_threshold;   // 中档下界：exit ≤ norm ≤ enter 退出；低档 < exit 保持
    double rc_channel_mid;
    double rc_channel_scale;
    PosSmoothParams pos_smooth; // Move_mode=0 惯性系位置软件平滑

    ParamSet()
        : uav_id(0), loop_hz(20.0), takeoff_height(2.0), no_RC(false),
          cmd_timeout_sec(0.5), odom_timeout_sec(0.5), rc_timeout_sec(1.0),
          rc_offboard_channel(6),
          rc_offboard_enter_threshold(0.3), rc_offboard_exit_threshold(-0.3),
          rc_channel_mid(1500.0), rc_channel_scale(500.0) {}
    // 从 ROS 参数服务器加载参数（使用私有命名空间 nh，参数名为相对名）
    void loadFromROSParam(ros::NodeHandle& nh)
    {
        nh.param("uav_id", uav_id, 0);
        nh.param("loop_hz", loop_hz, 20.0);
        nh.param("takeoff_height", takeoff_height, 2.0);
        nh.param("no_RC", no_RC, false);
        nh.param("cmd_timeout_sec", cmd_timeout_sec, 0.5);
        nh.param("odom_timeout_sec", odom_timeout_sec, 0.5);
        nh.param("rc_timeout_sec", rc_timeout_sec, 1.0);
        nh.param("rc_offboard_channel", rc_offboard_channel, 6);
        nh.param("rc_offboard_enter_threshold", rc_offboard_enter_threshold, 0.3);
        nh.param("rc_offboard_exit_threshold", rc_offboard_exit_threshold, -0.3);
        nh.param("rc_channel_mid", rc_channel_mid, 1500.0);
        nh.param("rc_channel_scale", rc_channel_scale, 500.0);
        nh.param("pos_smooth_enable", pos_smooth.enable, true);
        nh.param("pos_smooth_max_vel", pos_smooth.max_vel, 1.0);
        nh.param("pos_smooth_max_accel", pos_smooth.max_accel, 0.8);
        nh.param("pos_smooth_arrival_dist", pos_smooth.arrival_dist, 0.1);
        nh.param("pos_smooth_lookahead_dist", pos_smooth.lookahead_dist, 0.5);
        nh.param("pos_smooth_vel_deadband", pos_smooth.vel_deadband, 0.05);
        rc_offboard_channel = std::max(1, std::min(rc_offboard_channel, 18));
        if (rc_channel_scale < 1e-3)
            rc_channel_scale = 500.0;
        if (rc_offboard_enter_threshold <= rc_offboard_exit_threshold)
            rc_offboard_enter_threshold = rc_offboard_exit_threshold + 0.1;
        pos_smooth.max_vel = std::max(pos_smooth.max_vel, 1e-3);
        pos_smooth.max_accel = std::max(pos_smooth.max_accel, 1e-3);
        pos_smooth.arrival_dist = std::max(pos_smooth.arrival_dist, 1e-3);
        pos_smooth.lookahead_dist = std::max(pos_smooth.lookahead_dist, 1e-3);
        pos_smooth.vel_deadband = std::max(pos_smooth.vel_deadband, 0.0);
    }
};

/**
 * @brief TF 树参数（对应 config/uav_group.yaml 中的 TF 段）
 * 说明：sensors 下按 camera_link / lidar_link 等大类分组，每类下可配置多个具体传感器；
 * 需在 launch 中置于 <group ns="uav0"> 下用 rosparam 加载，使参数落在 /uav0/... 下。
 */
struct TFTreeParamSet
{
    // TF 坐标系配置（大类或具体传感器）
    struct SensorTFConfig
    {
        geometry_msgs::Transform transform;
        bool enabled{true};
        bool is_static{true};
        std::string parent; // 空表示挂载在 base_link；否则为父坐标系名（如 camera_link）
    };

    // 坐标系名称
    std::string world_frame{"world"}; // 世界坐标系，默认: world
    std::string map_frame;        // 地图坐标系，默认: uavX_map（由 loadFromParam 填）
    std::string base_link_frame;  // 机体坐标系，默认: uavX/base_link（由 loadFromParam 填）

    // world -> map（可选）
    bool enable_world_to_map{false}; // 是否启用world -> map的坐标转换
    geometry_msgs::Transform world_to_map_transform; // world -> map的变换

    // TF 坐标系：frame 名 -> 配置（含大类 camera_link/lidar_link 与具体传感器）
    std::map<std::string, SensorTFConfig> sensors;

    TFTreeParamSet()
    {
        // world_to_map 默认单位变换
        world_to_map_transform.translation.x = 0.0;
        world_to_map_transform.translation.y = 0.0;
        world_to_map_transform.translation.z = 0.0;
        world_to_map_transform.rotation.x = 0.0;
        world_to_map_transform.rotation.y = 0.0;
        world_to_map_transform.rotation.z = 0.0;
        world_to_map_transform.rotation.w = 1.0;
    }

    /** 从 group 命名空间（/uav0、/uav1...）读取 TF 树配置（config/uav_group.yaml） */
    void loadFromParam(ros::NodeHandle& group_nh, const std::string& uav_prefix)
    {
        map_frame = uav_prefix + "_map";
        base_link_frame = uav_prefix + "/base_link";

        group_nh.param("world_frame", world_frame, std::string("world"));
        group_nh.param("map_frame", map_frame, map_frame);
        group_nh.param("base_link_frame", base_link_frame, base_link_frame);
        group_nh.param("enable_world_to_map", enable_world_to_map, false);
        if (enable_world_to_map)
        {
            geometry_msgs::Transform t;
            if (loadTransformFromParam(group_nh, "world_to_map/transform", t))
                world_to_map_transform = t;
        }

        sensors.clear();
        XmlRpc::XmlRpcValue sensors_param;
        if (!group_nh.getParam("sensors", sensors_param) || sensors_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            return;

        for (auto it = sensors_param.begin(); it != sensors_param.end(); ++it)
        {
            const std::string entry_name = it->first;
            if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                continue;

            if (isSensorCategory(it->second))
                loadSensorCategory(group_nh, entry_name, it->second);
            else
                loadFlatSensor(group_nh, entry_name, it->second);
        }
    }

private:
    static bool isReservedFrameKey(const std::string& key)
    {
        return key == "enabled" || key == "static" || key == "transform";
    }

    static bool isSensorCategory(const XmlRpc::XmlRpcValue& node)
    {
        for (auto it = node.begin(); it != node.end(); ++it)
        {
            if (!isReservedFrameKey(it->first) && it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                return true;
        }
        return false;
    }

    static void fillFrameFlags(const XmlRpc::XmlRpcValue& node, SensorTFConfig& cfg)
    {
        cfg.enabled = true;
        cfg.is_static = true;
        if (node.hasMember("enabled"))
        {
            const XmlRpc::XmlRpcValue v = node["enabled"];
            if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                cfg.enabled = static_cast<bool>(v);
        }
        if (node.hasMember("static"))
        {
            const XmlRpc::XmlRpcValue v = node["static"];
            if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                cfg.is_static = static_cast<bool>(v);
        }
    }

    void loadFlatSensor(ros::NodeHandle& group_nh, const std::string& frame_name, const XmlRpc::XmlRpcValue& node)
    {
        SensorTFConfig cfg;
        fillFrameFlags(node, cfg);
        cfg.parent.clear();
        geometry_msgs::Transform t;
        const std::string transform_param = "sensors/" + frame_name + "/transform";
        if (!loadTransformFromParam(group_nh, transform_param, t))
            return;
        cfg.transform = t;
        sensors[frame_name] = cfg;
    }

    void loadSensorCategory(ros::NodeHandle& group_nh, const std::string& category_name, const XmlRpc::XmlRpcValue& node)
    {
        SensorTFConfig category_cfg;
        fillFrameFlags(node, category_cfg);
        category_cfg.parent.clear();
        geometry_msgs::Transform category_tf;
        const std::string category_tf_param = "sensors/" + category_name + "/transform";
        if (loadTransformFromParam(group_nh, category_tf_param, category_tf))
            category_cfg.transform = category_tf;
        else
            category_cfg.transform = identityTransform();

        bool any_child_enabled = false;
        for (auto it = node.begin(); it != node.end(); ++it)
        {
            if (isReservedFrameKey(it->first))
                continue;
            if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                continue;

            const std::string sensor_name = it->first;
            SensorTFConfig sensor_cfg;
            fillFrameFlags(it->second, sensor_cfg);
            sensor_cfg.parent = category_name;
            geometry_msgs::Transform sensor_tf;
            const std::string sensor_tf_param = "sensors/" + category_name + "/" + sensor_name + "/transform";
            if (!loadTransformFromParam(group_nh, sensor_tf_param, sensor_tf))
                continue;
            sensor_cfg.transform = sensor_tf;
            sensors[sensor_name] = sensor_cfg;
            if (sensor_cfg.enabled)
                any_child_enabled = true;
        }

        if (!node.hasMember("enabled"))
            category_cfg.enabled = any_child_enabled;
        sensors[category_name] = category_cfg;
    }

    static geometry_msgs::Transform identityTransform()
    {
        geometry_msgs::Transform t;
        t.translation.x = 0.0;
        t.translation.y = 0.0;
        t.translation.z = 0.0;
        t.rotation.x = 0.0;
        t.rotation.y = 0.0;
        t.rotation.z = 0.0;
        t.rotation.w = 1.0;
        return t;
    }

    static tf::Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
    {
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return q;
    }

    static geometry_msgs::Transform createTransform(double x, double y, double z, const tf::Quaternion& q) // 创建变换
    {
        geometry_msgs::Transform t;
        t.translation.x = x;
        t.translation.y = y;
        t.translation.z = z;
        t.rotation.x = q.x();
        t.rotation.y = q.y();
        t.rotation.z = q.z();
        t.rotation.w = q.w();
        return t;
    }

    /** 读取 6 元组 [x,y,z,roll,pitch,yaw] -> Transform */
    static bool loadTransformFromParam(ros::NodeHandle& group_nh, // 从ROS参数服务器加载变换
                                       const std::string& param_name,
                                       geometry_msgs::Transform& transform) // 变换
    {
        std::vector<double> v;
        if (!group_nh.getParam(param_name, v) || v.size() < 6)
            return false;
        const double x = v[0], y = v[1], z = v[2];
        const double roll = v[3], pitch = v[4], yaw = v[5];
        transform = createTransform(x, y, z, eulerToQuaternion(roll, pitch, yaw)); // 创建变换
        return true;
    }
};

/**
 * PX4 参数动态设置：通过 MAVROS /mavros/param/set 服务设置飞控参数。
 * 支持按脚本中的默认值批量设置，也可单参数 setParam。
 * 使用前需保证 MAVROS 已连接对应 UAV（如 /uav0/mavros）。
 */
class PX4ParamSetter
{
public:
    static constexpr double kPx4ParamFloatEpsilon = 0.001;

    /**
     * @param nh 节点句柄（可带命名空间）
     * @param uav_id 无人机 ID，服务名为 /uav{uav_id}/mavros/param/set；若 uav_id<0 则使用 /mavros/param/set
     */
    explicit PX4ParamSetter(ros::NodeHandle& nh, int uav_id = 0)
        : uav_id_(uav_id)
    {
        std::string prefix = (uav_id >= 0) ? ("/uav" + std::to_string(uav_id)) : "";
        param_set_client_ = nh.serviceClient<mavros_msgs::ParamSet>(prefix + "/mavros/param/set");
        param_get_client_ = nh.serviceClient<mavros_msgs::ParamGet>(prefix + "/mavros/param/get");
        command_client_ = nh.serviceClient<mavros_msgs::CommandLong>(prefix + "/mavros/cmd/command");
    }

    bool setParam(const std::string& param_id, double value)
    {
        mavros_msgs::ParamSet srv;
        srv.request.param_id = param_id;
        srv.request.value.integer = 0;
        srv.request.value.real = value;
        if (!param_set_client_.call(srv))
            return false;
        return srv.response.success;
    }

    bool setParamInt(const std::string& param_id, int64_t value)
    {
        mavros_msgs::ParamSet srv;
        srv.request.param_id = param_id;
        srv.request.value.integer = value;
        srv.request.value.real = 0.0;
        if (!param_set_client_.call(srv))
            return false;
        return srv.response.success;
    }

    bool getParam(const std::string& param_id, double& out)
    {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = param_id;
        if (!param_get_client_.call(srv) || !srv.response.success)
            return false;
        out = srv.response.value.real;
        return true;
    }

    bool getParamInt(const std::string& param_id, int64_t& out)
    {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = param_id;
        if (!param_get_client_.call(srv) || !srv.response.success)
            return false;
        out = srv.response.value.integer;
        return true;
    }

    /** 对比 px4_params_common + profile px4_params，仅写入与飞控不一致的参数 */
    Px4ApplyResult applyParamsFromLocationProfileIfNeeded(
        ros::NodeHandle& nh,
        const std::string& profile_name,
        std::function<void(const std::string&, bool)> log_cb = nullptr)
    {
        Px4ApplyResult result;

        XmlRpc::XmlRpcValue common;
        if (nh.getParam("px4_params_common", common) &&
            common.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            if (common.hasMember("int"))
            {
                XmlRpc::XmlRpcValue& int_params = common["int"];
                if (int_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    applyIntParamsIfNeeded(int_params, result, log_cb);
            }
            if (common.hasMember("float"))
            {
                XmlRpc::XmlRpcValue& float_params = common["float"];
                if (float_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    applyFloatParamsIfNeeded(float_params, result, log_cb);
            }
        }

        XmlRpc::XmlRpcValue profiles;
        if (!nh.getParam("location_profiles", profiles) ||
            profiles.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[uav_control_fsm] PX4ParamSetter: param 'location_profiles' not found or type is not struct");
            return result;
        }

        if (!profiles.hasMember(profile_name))
        {
            ROS_WARN("[uav_control_fsm] PX4ParamSetter: profile '%s' not exist in 'location_profiles'",
                     profile_name.c_str());
            return result;
        }

        XmlRpc::XmlRpcValue& prof = profiles[profile_name];
        if (prof.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("[uav_control_fsm] PX4ParamSetter: profile '%s' type is not struct", profile_name.c_str());
            return result;
        }

        result.profile_valid = true;

        if (prof.hasMember("px4_params"))
        {
            XmlRpc::XmlRpcValue& px4_params = prof["px4_params"];
            if (px4_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                if (px4_params.hasMember("int"))
                {
                    XmlRpc::XmlRpcValue& int_params = px4_params["int"];
                    if (int_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                        applyIntParamsIfNeeded(int_params, result, log_cb);
                }
                if (px4_params.hasMember("float"))
                {
                    XmlRpc::XmlRpcValue& float_params = px4_params["float"];
                    if (float_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                        applyFloatParamsIfNeeded(float_params, result, log_cb);
                }
            }
        }

        return result;
    }

    bool waitForService(double timeout_sec = 10.0)
    {
        const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
        while (ros::ok() && ros::Time::now() < deadline)
        {
            if (param_set_client_.exists() && param_get_client_.exists())
                return true;
            ros::Duration(0.05).sleep();
        }
        return param_set_client_.exists() && param_get_client_.exists();
    }

    bool rebootFCU()
    {
        mavros_msgs::CommandLong srv;
        srv.request.command = 246;
        srv.request.param1 = 1.0;
        srv.request.param2 = 0.0;
        srv.request.param3 = 0.0;
        srv.request.param4 = 0.0;
        srv.request.param5 = 0.0;
        srv.request.param6 = 0.0;
        srv.request.param7 = 0.0;
        srv.request.confirmation = 1;
        if (!command_client_.call(srv))
            return false;
        return srv.response.success;
    }

private:
    static bool xmlRpcToInt(const XmlRpc::XmlRpcValue& v, int& out)
    {
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            out = static_cast<int>(v);
            return true;
        }
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            out = static_cast<int>(static_cast<double>(v));
            return true;
        }
        return false;
    }

    static bool xmlRpcToDouble(const XmlRpc::XmlRpcValue& v, double& out)
    {
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            out = static_cast<double>(v);
            return true;
        }
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
            out = static_cast<double>(static_cast<int>(v));
            return true;
        }
        return false;
    }

    void applyIntParamsIfNeeded(XmlRpc::XmlRpcValue& int_params,
                                Px4ApplyResult& result,
                                const std::function<void(const std::string&, bool)>& log_cb)
    {
        for (auto it = int_params.begin(); it != int_params.end(); ++it)
        {
            const std::string param_id = it->first;
            int target = 0;
            if (!xmlRpcToInt(it->second, target))
                continue;

            ++result.params_checked;
            int64_t current = 0;
            if (getParamInt(param_id, current) && current == static_cast<int64_t>(target))
            {
                ++result.params_unchanged;
                continue;
            }

            const bool success = setParamInt(param_id, static_cast<int64_t>(target));
            if (success)
                ++result.params_changed;
            else
                ++result.params_failed;
            if (log_cb)
                log_cb(param_id, success);
        }
    }

    void applyFloatParamsIfNeeded(XmlRpc::XmlRpcValue& float_params,
                                  Px4ApplyResult& result,
                                  const std::function<void(const std::string&, bool)>& log_cb)
    {
        for (auto it = float_params.begin(); it != float_params.end(); ++it)
        {
            const std::string param_id = it->first;
            double target = 0.0;
            if (!xmlRpcToDouble(it->second, target))
                continue;

            ++result.params_checked;
            double current = 0.0;
            if (getParam(param_id, current) &&
                std::fabs(current - target) <= kPx4ParamFloatEpsilon)
            {
                ++result.params_unchanged;
                continue;
            }

            const bool success = setParam(param_id, target);
            if (success)
                ++result.params_changed;
            else
                ++result.params_failed;
            if (log_cb)
                log_cb(param_id, success);
        }
    }

    int uav_id_;
    ros::ServiceClient param_set_client_;
    ros::ServiceClient param_get_client_;
    ros::ServiceClient command_client_;
};

} // namespace uav_control_fsm

#endif
