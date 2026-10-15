/*********************************************************
fsm_control.hpp: 状态机，最主要的部分。会根据当前状态和控制指令生成 setpoint，并发布到 mavros。
状态包括：
1. 手动控制：MANUAL_CTRL
2. 自动控制：OFFBOARD_CTRL
3. 自动起飞：AUTO_TAKEOFF
4. 自动悬停：AUTO_HOVER
5. 自动降落：AUTO_LAND
*********************************************************/
#ifndef UAV_CONTROL_FSM_FSM_CONTROL_HPP
#define UAV_CONTROL_FSM_FSM_CONTROL_HPP

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <cmath>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ExtendedState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>
#include <uav_control_fsm/uav_command.h>

#include "uav_control_fsm/param_set.hpp"
#include "uav_control_fsm/control_manager.hpp"
#include "uav_control_fsm/arm_and_offboard.hpp"
#include "uav_control_fsm/tf_tree_manager.hpp"
#include "uav_control_fsm/odometry_bridge.hpp"

namespace uav_control_fsm {

/** 主状态：手动 / Offboard 控制 */
enum class MainState : uint8_t
{
    MANUAL_CTRL = 0,
    OFFBOARD_CTRL = 1
};

/** Offboard 下的子状态 */
enum class OffboardSubState : uint8_t
{
    AUTO_TAKEOFF = 0,
    AUTO_HOVER = 1,
    CMD_CTRL = 2   // 具体控制由 control_command_.Move_mode 决定，由 ControlManager 按 0-10 分发（ned_pos_xyz_set / body_vel_xyz_set / trajectory_set 等）
};

/** CMD_CTRL 下是否按移动/轨迹模式进入（用于 HOVER->CMD_CTRL 判断） */
enum class ControlModeKind : uint8_t { move_mode = 0, traj_mode = 1 };

/**
 * 控制状态机：主状态 MANUAL_CTRL / OFFBOARD_CTRL，OFFBOARD 下分子状态 TAKEOFF/HOVER/LAND/CMD_CTRL；
 * CMD_CTRL 时由 control_manager_.control_command_set 根据 Move_mode 调用对应 set（ned_pos_xyz_set 等）。
 */
class ControlFSM
{
public:
    /** nh: 用于话题/服务；pnh: 用于参数（launch 中 node 下的 param 在私有命名空间） */
    ControlFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~ControlFSM() = default;

    /** 主循环中调用：状态转换 + Offboard/解锁 + 降落检测，并更新 position_target_ */
    void process();

    /** 是否处于需要持续发布 setpoint 的状态 */
    bool shouldPublishSetpoint() const;

    /** 发布当前 position_target_（由主循环在 shouldPublishSetpoint() 为 true 时调用） */
    void publishSetpoint();

    /** 等待 FCU 连接（阻塞），构造函数后由节点按需调用 */
    void waitForFCU();

    double getLoopHz() const { return loop_hz_; }

private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void current_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg);
    void move_command_cb(const uav_control_fsm::uav_command::ConstPtr& msg);
    void offboard_command_cb(const std_msgs::Bool::ConstPtr& msg);
    void land_command_cb(const std_msgs::Bool::ConstPtr& msg);
    void hover_command_cb(const std_msgs::Bool::ConstPtr& msg);
    // 动态切换定位 set_location_source（例如 "GPS"、"Fast_Lio"、"Vins"），从 location_profiles 中读取并设置 PX4 参数
    void set_location_source_cb(const std_msgs::String::ConstPtr& msg);
    /** 统一切换定位源：OdometryBridge + PX4 参数（先读后比，按需写入/reboot） */
    void applyLocationSource(const std::string& source, const char* context);

    void px4_mode_switch(MainState main_state, OffboardSubState sub_state);
    bool odom_is_received(const ros::Time& now_time);
    bool cmd_is_received(const ros::Time& now_time);
    bool rc_is_received(const ros::Time& now_time);
    bool rcChannelValid(int channel_1based) const;
    float rcChannelNormalized(int channel_1based) const;
    bool rcOffboardEnterTriggered() const;
    bool rcOffboardExitTriggered() const;
    /** 进入 Offboard：地面/未在空中 -> AUTO_TAKEOFF；已解锁且在天上 -> AUTO_HOVER */
    bool enter_offboard_mode(const ros::Time& now, const char* source);
    void set_hover_pose();
    // 日志回调函数，对应 log_cb，可通过 applyParamsFromLocationProfileIfNeeded 来调用
    void px4_param_logger(const std::string& paramName, bool success);
    
    // bool get_landed();
    // void land_detector();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;              // 私有命名空间句柄（用于读取节点私有参数，如 location_source）
    ros::NodeHandle group_nh_;       // /uavX 命名空间，读取 location_profiles 等 launch 加载的参数
    ParamSet param_set_;
    std::unique_ptr<ArmAndOffboard> arm_offboard_;
    std::unique_ptr<TFManager> tf_manager_;
    std::unique_ptr<PX4ParamSetter> px4_param_setter_;  // PX4 参数动态设置（保留 setParam/setParamInt 接口）
    std::unique_ptr<OdometryBridge> odometry_bridge_;   // 按定位源订阅里程计并转发到 mavros/vision_pose/pose（GPS 时不订阅不发布）

    std::string active_location_source_;  // 当前生效的定位 profile 名（如 GPS、Vins）

    int uav_id_;
    double loop_hz_;
    double takeoff_height_;
    bool no_RC_;
    bool bench_disarmed_ = false;
    bool bench_allow_armed_ = false;
    ros::Subscriber bench_lease_sub_;
    ros::WallTime bench_lease_start_;
    ros::WallTime bench_lease_received_;
    bool bench_lease_enabled_ = false;
    bool benchArmedLeaseValid() const {
        const auto now = ros::WallTime::now();
        return bench_allow_armed_ && bench_lease_enabled_ &&
            bench_lease_start_ > ros::WallTime(0) &&
            (now - bench_lease_start_).toSec() < 10.0 &&
            (now - bench_lease_received_).toSec() < 0.25;
    }
    bool bench_publish_ready_ = false;
    ros::Time bench_state_received_;
    double cmd_timeout_sec_;
    double odom_timeout_sec_;
    double rc_timeout_sec_;
    int rc_offboard_channel_;
    double rc_offboard_enter_threshold_;
    double rc_offboard_exit_threshold_;
    double rc_channel_mid_;
    double rc_channel_scale_;
    /** land 后拨杆仍在高位时，禁止 RC 再次自动进 Offboard，直到拨杆离开高档 */
    bool block_rc_offboard_enter_;

    mavros_msgs::State current_state_;
    mavros_msgs::ExtendedState extended_state_;
    bool has_extended_state_;
    bool offboard_enabled_;  // 锁存标记：本轮是否已经成功检测到 FCU 进入 OFFBOARD
    MainState main_state_;
    OffboardSubState offboard_sub_state_;
    ControlModeKind control_mode_kind_;  // 用于 HOVER->CMD_CTRL 判断（move/traj）
    uav_control_fsm::uav_command control_command_;
    mavros_msgs::PositionTarget position_target_;
    nav_msgs::Odometry current_odom_;
    mavros_msgs::RCIn rc_data_;
    ControlManager control_manager_;

    ros::Time last_request_;
    ros::Time last_set_hover_pose_time_;
    ros::Time takeoff_land_time_;
    ros::Time last_status_print_time_;
    Eigen::Vector4d hover_pose_;
    Eigen::Vector4d takeoff_start_pose_;
    // bool landed_;
    ros::Time cmd_timeout_;
    ros::Time odom_timeout_;
    ros::Time rc_timeout_;

    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber current_odom_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber move_command_sub_;
    ros::Subscriber offboard_command_sub_;
    ros::Subscriber land_command_sub_;
    ros::Subscriber hover_command_sub_;
    ros::Subscriber set_location_source_sub_;  // 订阅定位源切换命令 /uavX/set_location_source
    ros::Publisher setpoint_pub_;
};

ControlFSM::ControlFSM(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh),
      main_state_(MainState::MANUAL_CTRL),
      offboard_sub_state_(OffboardSubState::AUTO_HOVER),
      arm_offboard_(nullptr),
      tf_manager_(nullptr),
      px4_param_setter_(nullptr),
      odometry_bridge_(nullptr),
      has_extended_state_(false),
      offboard_enabled_(false),
      block_rc_offboard_enter_(false),
      param_set_()
{
    param_set_.loadFromROSParam(pnh);
    pnh.param("bench_disarmed", bench_disarmed_, false);
    pnh.param("bench_allow_armed", bench_allow_armed_, false);
    if (bench_disarmed_ && bench_allow_armed_) {
        ROS_WARN("MOTOR BENCH: explicit armed lease, hard 10-second limit; no automatic arm/takeoff");
        bench_lease_sub_ = nh_.subscribe<std_msgs::Bool>("/vla/motor_bench_lease", 1,
            [this](const std_msgs::Bool::ConstPtr& msg) {
                const auto now = ros::WallTime::now();
                if (msg->data && bench_lease_start_ == ros::WallTime(0)) bench_lease_start_ = now;
                bench_lease_enabled_ = msg->data;
                bench_lease_received_ = now;
            });
    }
    if (bench_disarmed_)
        ROS_WARN("BENCH DISARMED: no automatic arming, takeoff, or mode changes; bounded Move_mode=0 only");
    uav_id_ = param_set_.uav_id;
    group_nh_ = ros::NodeHandle("/uav" + std::to_string(uav_id_));
    loop_hz_ = param_set_.loop_hz;
    takeoff_height_ = param_set_.takeoff_height;
    no_RC_ = param_set_.no_RC;
    cmd_timeout_sec_ = param_set_.cmd_timeout_sec;
    odom_timeout_sec_ = param_set_.odom_timeout_sec;
    rc_timeout_sec_ = param_set_.rc_timeout_sec;
    rc_offboard_channel_ = param_set_.rc_offboard_channel;
    rc_offboard_enter_threshold_ = param_set_.rc_offboard_enter_threshold;
    rc_offboard_exit_threshold_ = param_set_.rc_offboard_exit_threshold;
    rc_channel_mid_ = param_set_.rc_channel_mid;
    rc_channel_scale_ = param_set_.rc_channel_scale;
    control_manager_.setPosSmoothParams(param_set_.pos_smooth);

    // 2. 初始化解锁 / Offboard 辅助对象与 TF 管理
    arm_offboard_ = std::make_unique<ArmAndOffboard>(nh_, uav_id_);
    tf_manager_ = std::make_unique<TFManager>(nh_, uav_id_);
    // 3. 初始化 PX4 参数设置器（通过 MAVROS /uavX/mavros/param/set）
    px4_param_setter_ = std::make_unique<PX4ParamSetter>(nh_, uav_id_);
    // 4. 初始化里程计桥接（初始默认定位源由下方 setLocationSource 根据参数设置）
    odometry_bridge_ = std::make_unique<OdometryBridge>(nh_, uav_id_);
    std::string default_source;
    group_nh_.param("default_location_source", default_source, std::string("GPS"));
    active_location_source_ = default_source;
    odometry_bridge_->setLocationSource(default_source);

    std::string prefix = "/uav" + std::to_string(uav_id_);
    state_sub_ = nh_.subscribe<mavros_msgs::State>(prefix + "/mavros/state", 10, &ControlFSM::state_cb, this);
    extended_state_sub_ = nh_.subscribe<mavros_msgs::ExtendedState>(prefix + "/mavros/extended_state", 10, &ControlFSM::extended_state_cb, this);
    current_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(prefix + "/mavros/local_position/odom", 10, &ControlFSM::current_odom_cb, this);
    move_command_sub_ = nh_.subscribe<uav_control_fsm::uav_command>(prefix + "/move_command", 10, &ControlFSM::move_command_cb, this);
    offboard_command_sub_ = nh_.subscribe<std_msgs::Bool>(prefix + "/offboard_command", 10, &ControlFSM::offboard_command_cb, this);
    land_command_sub_ = nh_.subscribe<std_msgs::Bool>(prefix + "/land_command", 10, &ControlFSM::land_command_cb, this);
    hover_command_sub_ = nh_.subscribe<std_msgs::Bool>(prefix + "/hover_command", 10, &ControlFSM::hover_command_cb, this);
    // 切换定位源：topic /uavX/set_location_source，消息内容为定位源名（GPS / Fast_Lio / Vins / Gazebo）；FSM 与 OdometryBridge 均订阅此话题
    set_location_source_sub_ = nh_.subscribe<std_msgs::String>(prefix + "/set_location_source", 10, &ControlFSM::set_location_source_cb, this);
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(prefix + "/mavros/setpoint_raw/local", 10);

    if (!no_RC_)
        rc_sub_ = nh_.subscribe<mavros_msgs::RCIn>(prefix + "/mavros/rc/in", 10, &ControlFSM::rc_cb, this);
    else
        ROS_WARN("[uav_control_fsm] UAV%d: no_RC=true", uav_id_);

    ROS_INFO("[uav_control_fsm] UAV%d: RC ch%d 3-pos switch: low(<%.2f)=hold, mid=exit, high(>%.2f)=enter",
             uav_id_, rc_offboard_channel_, rc_offboard_exit_threshold_,
             rc_offboard_enter_threshold_);

    last_request_ = ros::Time::now();
    last_set_hover_pose_time_ = ros::Time::now();
    takeoff_land_time_ = ros::Time::now();
    last_status_print_time_ = ros::Time::now();
    cmd_timeout_ = ros::Time(0);
    odom_timeout_ = ros::Time(0);
    rc_timeout_ = ros::Time(0);
    hover_pose_.setZero();
    takeoff_start_pose_.setZero();
    //landed_ = true;
    control_mode_kind_ = ControlModeKind::move_mode;

    position_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                                 mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
}

void ControlFSM::waitForFCU()
{
    ros::Rate rate(10.0);
    while (ros::ok() && !current_state_.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("[uav_control_fsm] UAV%d Mavros connected", uav_id_);
    ros::Duration(1.0).sleep();
    for (int i = 0; i < 30 && ros::ok(); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    position_target_.position.x = current_odom_.pose.pose.position.x;
    position_target_.position.y = current_odom_.pose.pose.position.y;
    position_target_.position.z = takeoff_height_;
    position_target_.yaw = tf::getYaw(current_odom_.pose.pose.orientation);

    // 启动阶段应用默认定位源（若配置了 default_location_source），
    // 要求：当前在 MANUAL_CTRL 且飞控未解锁。
    std::string default_profile;
    group_nh_.getParam("default_location_source", default_profile);
    std::cout << "default_location_source: " << default_profile << std::endl;
    if (!default_profile.empty() && !bench_disarmed_)
    {
        if (main_state_ != MainState::MANUAL_CTRL || current_state_.armed)
        {
            ROS_WARN("[uav_control_fsm] UAV%d: skip default location source '%s' (need MANUAL_CTRL and disarmed)",
                     uav_id_, default_profile.c_str());
        }
        else
        {
            applyLocationSource(default_profile, "DEFAULT");
        }
    }
}
// 日志回调函数，对应 log_cb，可通过 applyParamsFromLocationProfileIfNeeded 来调用
void ControlFSM::px4_param_logger(const std::string& paramName, bool success) {
    if (success) {
        std::cout << "参数 " << paramName << " 设置成功" << std::endl;  
    } else {
        std::cout << "参数 " << paramName << " 设置失败" << std::endl;
    }
}

// ================= 回调 =================
void ControlFSM::state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state_ = *msg; bench_state_received_ = ros::Time::now(); }
void ControlFSM::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
    extended_state_ = *msg;
    has_extended_state_ = true;
}

void ControlFSM::current_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odom_ = *msg;
    odom_timeout_ = ros::Time::now();
    if (tf_manager_) tf_manager_->update(current_odom_);
}

void ControlFSM::rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
    rc_data_ = *msg;
    rc_timeout_ = ros::Time::now();
}

void ControlFSM::move_command_cb(const uav_control_fsm::uav_command::ConstPtr& msg)
{
    control_command_ = *msg;
    cmd_timeout_ = ros::Time::now();
    if (msg->Move_mode >= 0 && msg->Move_mode <= 9)
        control_mode_kind_ = ControlModeKind::move_mode;
    else if (msg->Move_mode == 10)
        control_mode_kind_ = ControlModeKind::traj_mode;
    control_manager_.control_command_set(position_target_, control_command_, current_odom_);
}

void ControlFSM::offboard_command_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data) return;
    enter_offboard_mode(ros::Time::now(), "offboard_command");
}

void ControlFSM::land_command_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if (bench_disarmed_) return;
    if (!msg->data) return;
    const bool fsm_can_land =
        main_state_ == MainState::OFFBOARD_CTRL &&
        (offboard_sub_state_ == OffboardSubState::AUTO_HOVER ||
         offboard_sub_state_ == OffboardSubState::CMD_CTRL);
    // 真机常见：RC 已把 PX4 切到 OFFBOARD，但 FSM 仍认为可降；允许在 PX4 OFFBOARD 时执行降落
    const bool px4_in_offboard = (current_state_.mode == "OFFBOARD");
    if (!fsm_can_land && !px4_in_offboard)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: Cannot land in current state (FSM main=%d sub=%d PX4=%s)",
                 uav_id_, static_cast<int>(main_state_), static_cast<int>(offboard_sub_state_),
                 current_state_.mode.c_str());
        return;
    }
    // 直接切到飞控 AUTO.LAND，由 PX4 负责降落与着陆后 disarm，本节点不再发降落 setpoint
    arm_offboard_->set_px4_mode("AUTO.LAND");
    main_state_ = MainState::MANUAL_CTRL;
    offboard_sub_state_ = OffboardSubState::AUTO_HOVER;  // 下次再进 Offboard 时从 HOVER 起
    offboard_enabled_ = false;
    // 拨杆若仍在高档，禁止立刻被 RC 再次拉回 Offboard（否则 AUTO.LAND 会被取消）
    block_rc_offboard_enter_ = true;
    ROS_INFO("[uav_control_fsm] UAV%d: land cmd -> PX4 AUTO.LAND, FSM -> MANUAL_CTRL "
             "(请将 Offboard 拨杆离开高档，否则飞控可能拒降)",
             uav_id_);
}

void ControlFSM::hover_command_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data) return;
    set_hover_pose();
    if (main_state_ == MainState::MANUAL_CTRL)
    {
        ROS_INFO("[uav_control_fsm] UAV%d: Current state = MANUAL_CTRL ,cannot enter hover mode", uav_id_);
        // main_state_ = MainState::OFFBOARD_CTRL;
        // offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
        // arm_offboard_->set_px4_mode("OFFBOARD");
        // ROS_INFO("[uav_control_fsm] UAV%d: MANUAL_CTRL -> OFFBOARD/AUTO_HOVER (hover cmd)", uav_id_);
    }
    else if (main_state_ == MainState::OFFBOARD_CTRL)
    {
        offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
        offboard_enabled_ = false;
        ROS_INFO("[uav_control_fsm] UAV%d: OFFBOARD/CMD_CTRL -> OFFBOARD/AUTO_HOVER (hover cmd)", uav_id_);
    }
}

// 动态切换定位 set_location_source：统一切换 OdometryBridge 与 PX4 参数（先读后比，按需 reboot）
void ControlFSM::set_location_source_cb(const std_msgs::String::ConstPtr& msg)
{
    if (bench_disarmed_) return;
    if (main_state_ != MainState::MANUAL_CTRL || current_state_.armed)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: reject location source change (need MANUAL_CTRL and disarmed)", uav_id_);
        return;
    }
    applyLocationSource(msg->data, "command");
}

void ControlFSM::applyLocationSource(const std::string& source, const char* context)
{
    if (source.empty())
    {
        ROS_WARN("[uav_control_fsm] UAV%d: empty location source name, ignore", uav_id_);
        return;
    }

    if (odometry_bridge_)
        odometry_bridge_->setLocationSource(source);

    active_location_source_ = source;

    if (!px4_param_setter_)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: PX4ParamSetter not initialized, switched OdometryBridge only for '%s'",
                 uav_id_, source.c_str());
        return;
    }

    if (main_state_ != MainState::MANUAL_CTRL || current_state_.armed)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: switched OdometryBridge to '%s', skip PX4 params (need MANUAL_CTRL and disarmed)",
                 uav_id_, source.c_str());
        return;
    }

    if (!px4_param_setter_->waitForService(3.0))
    {
        ROS_WARN("[uav_control_fsm] UAV%d: mavros param/get or param/set not available, cannot apply location source '%s'",
                 uav_id_, source.c_str());
        return;
    }
    // 查看是否能设置 COM_ARM_WO_GPS 参数
    {
        const ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
        int64_t probe = 0;
        while (ros::ok() && ros::Time::now() < deadline)
        {
            if (px4_param_setter_->getParamInt("COM_ARM_WO_GPS", probe))
                break;
            ros::Duration(0.5).sleep();
        }
    }

    auto log_cb = [this](const std::string& param_name, bool ok)
    {
        this->px4_param_logger(param_name, ok);
    };

    const Px4ApplyResult result =
        px4_param_setter_->applyParamsFromLocationProfileIfNeeded(group_nh_, source, log_cb);

    if (!result.profile_valid)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: location profile '%s' invalid (check YAML and param server)",
                 uav_id_, source.c_str());
        return;
    }

    if (result.all_matched())
    {
        ROS_INFO("[uav_control_fsm] UAV%d: location source '%s' PX4 params already match (%d checked), skip write and reboot",
                 uav_id_, source.c_str(), result.params_checked);
        return;
    }

    if (result.params_changed > 0)
    {
        ROS_INFO("[uav_control_fsm] UAV%d: applied %s location source '%s', %d PX4 params changed (%d unchanged, %d failed)",
                 uav_id_, context, source.c_str(),
                 result.params_changed, result.params_unchanged, result.params_failed);

        if (no_RC_)
        {
            ROS_INFO("[uav_control_fsm] UAV%d: no_RC=true, skip FCU reboot", uav_id_);
        }
        else
        {
            const bool reboot_ok = px4_param_setter_->rebootFCU();
            if (reboot_ok)
            {
                ROS_WARN("[uav_control_fsm] UAV%d: FCU reboot command sent, connection will be lost shortly",
                         uav_id_);
            }
            else
            {
                ROS_WARN("[uav_control_fsm] UAV%d: failed to send FCU reboot command, please power cycle FCU manually",
                         uav_id_);
            }
        }
        return;
    }

    if (result.params_failed > 0)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: location source '%s' PX4 param apply failed (%d failed, %d checked)",
                 uav_id_, source.c_str(), result.params_failed, result.params_checked);
    }
    else if (result.params_checked == 0)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: location source '%s' has no PX4 params to check (check YAML)",
                 uav_id_, source.c_str());
    }
}

// ================= 模式切换 根据主状态切换 PX4 模式（降落由 land_command_cb 直接切 AUTO.LAND） =================
void ControlFSM::px4_mode_switch(MainState main_state, OffboardSubState sub_state)
{
    if (bench_disarmed_) return;
    (void)sub_state;
    if (main_state == MainState::OFFBOARD_CTRL)
        arm_offboard_->set_px4_mode("OFFBOARD");
    else
        arm_offboard_->set_px4_mode("POSCTL");
}

// 里程计是否超时
bool ControlFSM::odom_is_received(const ros::Time& now_time)
{
    return (now_time - odom_timeout_).toSec() < odom_timeout_sec_;
}

// 控制命令是否超时
bool ControlFSM::cmd_is_received(const ros::Time& now_time)
{
    return (now_time - cmd_timeout_).toSec() < cmd_timeout_sec_;
}

// 遥控器是否超时
bool ControlFSM::rc_is_received(const ros::Time& now_time)
{
    if (no_RC_) return true;
    return (now_time - rc_timeout_).toSec() < rc_timeout_sec_;
}

bool ControlFSM::rcChannelValid(int channel_1based) const
{
    return channel_1based >= 1 &&
           static_cast<size_t>(channel_1based) <= rc_data_.channels.size();
}

float ControlFSM::rcChannelNormalized(int channel_1based) const
{
    return (rc_data_.channels[channel_1based - 1] - rc_channel_mid_) / rc_channel_scale_;
}

bool ControlFSM::rcOffboardEnterTriggered() const
{
    return rcChannelValid(rc_offboard_channel_) &&
           rcChannelNormalized(rc_offboard_channel_) > rc_offboard_enter_threshold_;
}

bool ControlFSM::rcOffboardExitTriggered() const
{
    // 中档退出 Offboard：exit_threshold ≤ norm ≤ enter_threshold；低档为保持
    if (!rcChannelValid(rc_offboard_channel_))
        return false;
    const float norm = rcChannelNormalized(rc_offboard_channel_);
    return norm >= rc_offboard_exit_threshold_ &&
           norm <= rc_offboard_enter_threshold_;
}

// 设置悬停位置
void ControlFSM::set_hover_pose()
{
    hover_pose_.head<3>() = Eigen::Vector3d(
        current_odom_.pose.pose.position.x,
        current_odom_.pose.pose.position.y,
        current_odom_.pose.pose.position.z);
    hover_pose_(3) = tf::getYaw(current_odom_.pose.pose.orientation);
    last_set_hover_pose_time_ = ros::Time::now();
    control_manager_.resetPosSmooth();
}

bool ControlFSM::enter_offboard_mode(const ros::Time& now, const char* source)
{
    if (bench_disarmed_)
    {
        ROS_WARN("BENCH: use explicit MAVROS set_mode; automatic entry/takeoff is disabled");
        return false;
    }
    if (!current_state_.connected)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: Cannot enter offboard (%s), FCU not connected", uav_id_, source);
        return false;
    }
    if (!odom_is_received(now))
    {
        ROS_WARN("[uav_control_fsm] UAV%d: Cannot enter offboard (%s), odom timeout", uav_id_, source);
        return false;
    }
    if ((now - takeoff_land_time_).toSec() < 0.8)
    {
        ROS_WARN("[uav_control_fsm] UAV%d: Ignore duplicated offboard cmd (%s)", uav_id_, source);
        return false;
    }
    if (main_state_ != MainState::MANUAL_CTRL &&
        !(main_state_ == MainState::OFFBOARD_CTRL && offboard_sub_state_ == OffboardSubState::AUTO_HOVER))
    {
        ROS_WARN("[uav_control_fsm] UAV%d: Cannot enter offboard (%s) in current state", uav_id_, source);
        return false;
    }

    const double x = current_odom_.pose.pose.position.x;
    const double y = current_odom_.pose.pose.position.y;
    const double current_z = current_odom_.pose.pose.position.z;
    const double yaw = tf::getYaw(current_odom_.pose.pose.orientation);
    const bool in_air_by_extended_state =
        has_extended_state_ &&
        extended_state_.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;

    takeoff_land_time_ = now;
    main_state_ = MainState::OFFBOARD_CTRL;
    offboard_enabled_ = false;

    if (current_state_.armed && in_air_by_extended_state)
    {
        offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
        hover_pose_.head<3>() = Eigen::Vector3d(x, y, current_z);
        hover_pose_(3) = yaw;
        last_set_hover_pose_time_ = now;
        arm_offboard_->set_px4_mode("OFFBOARD");
        ROS_INFO("[uav_control_fsm] UAV%d: %s -> OFFBOARD/AUTO_HOVER (z=%.2f)", uav_id_, source, current_z);
        return true;
    }

    const double target_z = std::max(takeoff_height_, current_z);
    takeoff_start_pose_.head<3>() = Eigen::Vector3d(x, y, target_z);
    takeoff_start_pose_(3) = yaw;
    offboard_sub_state_ = OffboardSubState::AUTO_TAKEOFF;
    arm_offboard_->set_px4_mode("OFFBOARD");
    ROS_INFO("[uav_control_fsm] UAV%d: %s -> OFFBOARD/AUTO_TAKEOFF, target_height=%.2f", uav_id_, source, target_z);
    return true;
}

// 主流程
void ControlFSM::process()
{
    ros::Time now_time = ros::Time::now();
    if (bench_disarmed_)
    {
        bench_publish_ready_ = current_state_.connected && (!current_state_.armed || benchArmedLeaseValid()) &&
            bench_state_received_ > ros::Time(0) && (now_time - bench_state_received_).toSec() < 2.0 &&
            odom_timeout_ > ros::Time(0) && odom_is_received(now_time);
        if (!bench_publish_ready_) return;
        const auto& p = current_odom_.pose.pose.position;
        const double yaw = tf::getYaw(current_odom_.pose.pose.orientation);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(yaw))
        { bench_publish_ready_ = false; return; }
        // Pre-stream current-position hold before the explicit OFFBOARD request.
        hover_pose_ << p.x, p.y, p.z, yaw;
        control_manager_.hover_setpoint_set(position_target_, hover_pose_);
        if (current_state_.mode == "OFFBOARD" && cmd_timeout_ > ros::Time(0) && cmd_is_received(now_time))
        {
            const auto& c = control_command_;
            const double dx = c.pos_ref.x - p.x, dy = c.pos_ref.y - p.y, dz = c.pos_ref.z - p.z;
            const double yaw_error = std::remainder(c.yaw_ref * M_PI / 180.0 - yaw, 2.0 * M_PI);
            const double age = (now_time - c.header.stamp).toSec();
            if (c.Move_mode == 0 && c.header.frame_id == "uav" + std::to_string(uav_id_) + "_map" &&
                std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz) && std::isfinite(yaw_error) &&
                age >= 0.0 && age <= cmd_timeout_sec_ &&
                std::hypot(dx, dy) <= (current_state_.armed ? 0.03 : 0.15) &&
                std::abs(dz) <= (current_state_.armed ? 0.005 : 0.03) &&
                std::abs(yaw_error) <= (current_state_.armed ? 0.02 : 0.105))
                control_manager_.control_command_set(position_target_, c, current_odom_);
            else
                ROS_WARN_THROTTLE(1.0, "BENCH: command rejected by frame/age/step limits; holding current pose");
        }
        tf_manager_->update(current_odom_);
        return; // Never reaches the normal FSM's arm(true), RC entry or AUTO_TAKEOFF.
    }
    // 每 10 秒打印一次本机状态（主状态/子状态/PX4 模式/解锁/着陆）
    // if ((now_time - last_status_print_time_).toSec() >= 10.0)
    // {
    //     const char* sub_name = "";
    //     if (main_state_ == MainState::OFFBOARD_CTRL)
    //     {
    //         switch (offboard_sub_state_)
    //         {
    //             case OffboardSubState::AUTO_TAKEOFF: sub_name = "AUTO_TAKEOFF"; break;
    //             case OffboardSubState::AUTO_HOVER:   sub_name = "AUTO_HOVER";   break;
    //             case OffboardSubState::CMD_CTRL:     sub_name = "CMD_CTRL";     break;
    //         }
    //     }
    //     ROS_INFO("[uav_control_fsm] UAV%d Main=%s Sub=%s PX4=%s Armed=%s",
    //              uav_id_, main_state_ == MainState::MANUAL_CTRL ? "MANUAL_CTRL" : "OFFBOARD_CTRL",
    //              main_state_ == MainState::OFFBOARD_CTRL ? sub_name : "-",
    //              current_state_.mode.c_str(), current_state_.armed ? "true" : "false");
    //     last_status_print_time_ = now_time;
    // }    

    // 手动控制模式
    if (main_state_ == MainState::MANUAL_CTRL)
    {
        // land 后拨杆仍在高档：先等拨杆离开高档，再允许 RC 重新进 Offboard
        if (block_rc_offboard_enter_)
        {
            if (!rc_is_received(now_time) || !rcOffboardEnterTriggered())
            {
                block_rc_offboard_enter_ = false;
                ROS_INFO("[uav_control_fsm] UAV%d: RC offboard enter re-armed (stick left high)",
                         uav_id_);
            }
        }

        bool should_enter_hover = false;
        if (!block_rc_offboard_enter_ && !no_RC_ && rc_is_received(now_time) &&
            rcOffboardEnterTriggered())
            should_enter_hover = true;
        if (should_enter_hover)
        {
            const double vx = current_odom_.twist.twist.linear.x;
            const double vy = current_odom_.twist.twist.linear.y;
            if (vx * vx + vy * vy > 9.0)
                ROS_ERROR("[uav_control_fsm] UAV%d: Vel too high", uav_id_);
            else
                enter_offboard_mode(now_time, "RC enter channel");
        }
    }
    else // Offboard 控制模式
    {
        if (!no_RC_ && rc_is_received(now_time) && rcOffboardExitTriggered())
        {
            main_state_ = MainState::MANUAL_CTRL;
            offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
            offboard_enabled_ = false;
            px4_mode_switch(main_state_, offboard_sub_state_);
            ROS_INFO("[uav_control_fsm] UAV%d: RC ch%d mid -> MANUAL_CTRL/POSCTL (exit OFFBOARD)",
                     uav_id_, rc_offboard_channel_);
            return;
        }

        switch (offboard_sub_state_)
        {
        case OffboardSubState::AUTO_TAKEOFF:// 自动起飞状态（odom 为 ENU：z 向上为正，目标 z=takeoff_start_pose_(2)）
            if (current_odom_.pose.pose.position.z >= takeoff_start_pose_(2) - 1e-6)
            {
                offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
                set_hover_pose();
                ROS_INFO("[uav_control_fsm] UAV%d: AUTO_TAKEOFF -> AUTO_HOVER (reached height)", uav_id_);
            }
            else
            {
                takeoff_start_pose_(3) = tf::getYaw(current_odom_.pose.pose.orientation);
                control_manager_.takeoff_setpoint_set(position_target_, takeoff_start_pose_);
                // z 已由 takeoff_start_pose_(2) 设好（= max(takeoff_height_, 当前z)），仅在还需爬升时给速度
                if (current_odom_.pose.pose.position.z < takeoff_start_pose_(2) - 1e-6)
                    position_target_.velocity.z = 0.5;  // ENU：向上为正
                else
                    position_target_.velocity.z = 0.0;
            }
            break;
        case OffboardSubState::AUTO_HOVER:
        {
            // RC 异常则退回手动：无 RC 时仅看通道是否拨到中档（退出）；有 RC 时看超时
            bool rc_fail = false;
            if (no_RC_)
            {
                if (rcOffboardExitTriggered())
                    rc_fail = true;
            }
            else
            {
                if (!rc_is_received(now_time)) rc_fail = true;
            }
            if (rc_fail) { main_state_ = MainState::MANUAL_CTRL; px4_mode_switch(main_state_, offboard_sub_state_); break; }
            // 里程计超时也退回手动（Position 模式），避免在无定位下继续 Offboard
            if (!odom_is_received(now_time)) { main_state_ = MainState::MANUAL_CTRL; px4_mode_switch(main_state_, offboard_sub_state_); break; }
            // 有控制指令且 PX4 已在 OFFBOARD 则切到 CMD_CTRL；否则维持悬停 setpoint
            if (cmd_is_received(now_time) && (control_mode_kind_ == ControlModeKind::move_mode || control_mode_kind_ == ControlModeKind::traj_mode) && current_state_.mode == "OFFBOARD")
            {
                offboard_sub_state_ = OffboardSubState::CMD_CTRL;
                ROS_INFO("[uav_control_fsm] UAV%d: AUTO_HOVER -> CMD_CTRL", uav_id_);
            }
            else
                control_manager_.hover_setpoint_set(position_target_, hover_pose_);
            break;
        }
        case OffboardSubState::CMD_CTRL:
        {
            // no_RC_：无 RC 时仅看里程计超时；有 RC 时看 RC 超时或里程计超时
            bool exit_manual = no_RC_ ? !odom_is_received(now_time) : (!rc_is_received(now_time) || !odom_is_received(now_time));
            if (exit_manual) { main_state_ = MainState::MANUAL_CTRL; px4_mode_switch(main_state_, offboard_sub_state_); break; }
            // 控制指令超时：停用最后一条速度/轨迹指令，锁定当前位姿悬停（不定降）
            if (!cmd_is_received(now_time))
            {
                set_hover_pose();
                control_manager_.hover_setpoint_set(position_target_, hover_pose_);
                offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
                ROS_WARN("[uav_control_fsm] UAV%d: CMD_CTRL -> AUTO_HOVER (cmd timeout %.2fs)",
                         uav_id_, cmd_timeout_sec_);
                break;
            }
            // 每帧用当前 control_command_ 和 odom 更新 setpoint（机体系等依赖当前位姿）
            control_manager_.control_command_set(position_target_, control_command_, current_odom_);
            break;
        }
        }
    }

    if (main_state_ == MainState::OFFBOARD_CTRL)
    {
        // 一旦曾经进入过 OFFBOARD，又检测到当前模式已不是 OFFBOARD，
        // 认为用户/飞控主动退出，FSM 同步回 MANUAL，避免被自动拉回 OFFBOARD。
        if (current_state_.mode == "OFFBOARD")
            offboard_enabled_ = true;
        else if (offboard_enabled_)
        {
            main_state_ = MainState::MANUAL_CTRL;
            offboard_sub_state_ = OffboardSubState::AUTO_HOVER;
            offboard_enabled_ = false;
            ROS_WARN("[uav_control_fsm] UAV%d: detected OFFBOARD exit from FCU/QGC, FSM -> MANUAL_CTRL", uav_id_);
            return;
        }

        if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
        {
            arm_offboard_->request_offboard();
            last_request_ = ros::Time::now();
        }
        else if (current_state_.mode == "OFFBOARD" && !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
        {
            if (arm_offboard_->arm(true))
                ROS_INFO("[uav_control_fsm] UAV%d Arming OK", uav_id_);
            last_request_ = ros::Time::now();
        }
    }

    // land_detector();
}

// 只有 Offboard 控制模式才发布 setpoint
bool ControlFSM::shouldPublishSetpoint() const
{
    if (bench_disarmed_) return bench_publish_ready_ && (!current_state_.armed || benchArmedLeaseValid());
    return main_state_ == MainState::OFFBOARD_CTRL;
}

// 发布 setpoint
void ControlFSM::publishSetpoint()
{
    position_target_.header.stamp = ros::Time::now();
    position_target_.header.frame_id = "uav" + std::to_string(uav_id_) + "_map";
    setpoint_pub_.publish(position_target_);
}

} // namespace uav_control_fsm

#endif
