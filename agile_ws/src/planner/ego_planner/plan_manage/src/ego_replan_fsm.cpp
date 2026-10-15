
#include <plan_manage/ego_replan_fsm.h>
#include <cmath>

namespace ego_planner
{
  namespace
  {
    constexpr double kRad2Deg = 180.0 / M_PI;

    double normalizeAngle(double a)
    {
      while (a > M_PI)
        a -= 2.0 * M_PI;
      while (a < -M_PI)
        a += 2.0 * M_PI;
      return a;
    }

    double quatToYaw(const geometry_msgs::Quaternion &q)
    {
      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      return std::atan2(siny_cosp, cosy_cosp);
    }
  } // namespace

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;
    odom_yaw_ = 0.0;
    yaw_align_active_ = false;
    yaw_align_hold_done_ = false;
    yaw_align_next_state_ = WAIT_TARGET;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);// 是否启用地形感知,会感知前方2m地面高度，用于地形感知
    nh.param("grid_map/virtual_ground", virtual_ground_, -0.1);

    double yaw_align_threshold_deg = 20.0;
    double yaw_align_tolerance_deg = 10.0;
    nh.param("fsm/enable_yaw_align", enable_yaw_align_, true);
    nh.param("fsm/yaw_align_threshold_deg", yaw_align_threshold_deg, 20.0);
    nh.param("fsm/yaw_align_tolerance_deg", yaw_align_tolerance_deg, 10.0);
    nh.param("fsm/yaw_align_hold_sec", yaw_align_hold_sec_, 0.5);
    yaw_align_threshold_rad_ = yaw_align_threshold_deg * M_PI / 180.0;
    yaw_align_tolerance_rad_ = yaw_align_tolerance_deg * M_PI / 180.0;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }


    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_;
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);// 执行FSM回调函数，用于控制无人机的行为和状态
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);// 碰撞检测回调函数，用于检查无人机是否与障碍物发生碰撞

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);// 订阅机器人里程计信息，用于获取机器人的位置和速度
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);// 用于强制停止执行规划动作
    resume_planning_sub_ = nh.subscribe("resume_planning", 1, &EGOReplanFSM::resumePlanningCallback, this);// 强制停止后恢复规划（切回 WAIT_TARGET，再发新目标即可）

    /* Use MINCO trajectory to minimize the message size in wireless communication 使用 MINCO 轨迹来最小化无线通信中的消息大小*/
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    // heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);

    int uav_id = 0;
    nh.param("manager/uav_id", uav_id, 0);
    uav_cmd_pub_ = nh.advertise<uav_control_fsm::uav_command>(
        "/uav" + std::to_string(uav_id) + "/move_command", 10);
    yaw_align_active_pub_ = nh.advertise<std_msgs::Bool>("planning/yaw_align_active", 1, true);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    // std_msgs::Empty heartbeat_msg;
    // heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        gotoPlanOrAlign(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case YAW_ALIGN:
    {
      if (!yaw_align_hold_done_)
      {
        if ((ros::Time::now() - yaw_align_enter_time_).toSec() < yaw_align_hold_sec_)
        {
          // traj_server 已暂停：先悬停当前位姿/yaw，等待 0.5s 再转向
          publishYawAlignCmd(false);
          break;
        }
        yaw_align_hold_done_ = true;
        ROS_INFO("[YAW_ALIGN] hold %.2fs done, start turning", yaw_align_hold_sec_);
      }
      publishYawAlignCmd(true);
      if (isYawAligned())
        changeFSMExecState(yaw_align_next_state_, "Yaw aligned");
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with uav_id = 1
    {
      if (planner_manager_->pp_.uav_id <= 1 || (planner_manager_->pp_.uav_id >= 2 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:// 在紧急情况下，重新规划全局轨迹
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromLocalTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan 确认是否需要重新规划 */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;// 获取局部轨迹数据
      double t_cur = ros::Time::now().toSec() - info->start_time;// 计算当前轨迹时间
      t_cur = min(info->duration, t_cur);// 限制当前轨迹时间不超过轨迹总时间
      Eigen::Vector3d pos = info->traj.getPos(t_cur);// 获取当前轨迹位置
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);// 判断是否到达目标点

      const PtsChk_t* chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;// 获取存储轨迹上需要检查碰撞的采样点
      /******************************************************
      chk_ptr->size() >= 1：至少有一个轨迹段
      chk_ptr->back().size() >= 1：最后一个轨迹段至少有一个检查点,
      chk_ptr->back().back().first - t_cur < emergency_time_：最后一个检查点的时间与当前轨迹时间的差值小于紧急时间，则认为接近轨迹终点。
      */
      bool close_to_current_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? chk_ptr->back().back().first - t_cur < emergency_time_ : 0; // In case of empty vector

      if (mondifyInCollisionFinalGoal()) // case 1: find that current goal is in obstacles,如果当前目标点在障碍物中，则修改目标点
      {
        // pass
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) // case 2: assign the next waypoint
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
      {
        have_target_ = false;
        have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          // prepare for next round
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) // case 3: time to perform next replan
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "YAW_ALIGN"};
    int pre_s = int(exec_state_);
    if (exec_state_ == YAW_ALIGN && new_state != YAW_ALIGN)
      setYawAlignActive(false);
    exec_state_ = new_state;
    if (new_state == YAW_ALIGN)
    {
      yaw_align_enter_time_ = ros::Time::now();
      yaw_align_hold_done_ = false;
      setYawAlignActive(true);
    }
    cout << "[" + pos_call + "]"
         << "UAV:" << planner_manager_->pp_.uav_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::gotoPlanOrAlign(FSM_EXEC_STATE next_state, const string &pos_call)
  {
    if (enable_yaw_align_ && needYawAlign())
    {
      yaw_align_next_state_ = next_state;
      changeFSMExecState(YAW_ALIGN, pos_call);
    }
    else
    {
      changeFSMExecState(next_state, pos_call);
    }
  }

  bool EGOReplanFSM::needYawAlign() const
  {
    return std::fabs(normalizeAngle(computeYawToGoal() - odom_yaw_)) > yaw_align_threshold_rad_;
  }

  bool EGOReplanFSM::isYawAligned() const
  {
    return std::fabs(normalizeAngle(computeYawToGoal() - odom_yaw_)) < yaw_align_tolerance_rad_;
  }

  double EGOReplanFSM::computeYawToGoal() const
  {
    return std::atan2(final_goal_(1) - odom_pos_(1), final_goal_(0) - odom_pos_(0));
  }

  void EGOReplanFSM::publishYawAlignCmd(bool turn_to_goal)
  {
    uav_control_fsm::uav_command cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.Move_mode = uav_control_fsm::uav_command::pos_xyz;
    cmd.pos_ref.x = odom_pos_(0);
    cmd.pos_ref.y = odom_pos_(1);
    cmd.pos_ref.z = odom_pos_(2);
    cmd.vel_ref.x = 0.0;
    cmd.vel_ref.y = 0.0;
    cmd.vel_ref.z = 0.0;
    cmd.acc_ref.x = 0.0;
    cmd.acc_ref.y = 0.0;
    cmd.acc_ref.z = 0.0;
    cmd.yaw_ref = static_cast<float>((turn_to_goal ? computeYawToGoal() : odom_yaw_) * kRad2Deg);
    cmd.yaw_rate_ref = 0.0f;
    uav_cmd_pub_.publish(cmd);
  }

  void EGOReplanFSM::setYawAlignActive(bool active)
  {
    yaw_align_active_ = active;
    std_msgs::Bool msg;
    msg.data = active;
    yaw_align_active_pub_.publish(msg);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[9] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START", "YAW_ALIGN"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", UAV:" << planner_manager_->pp_.uav_id;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.uav_id >= 2 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.uav_id >= 2 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    const double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || exec_state_ == YAW_ALIGN || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    static bool depth_odom_timeout_active = false;
    const bool depth_timeout = map->getOdomDepthTimeout();
    if (depth_timeout)
    {
      if (!depth_odom_timeout_active)
      {
        ROS_ERROR("Depth Lost! EMERGENCY_STOP");
        enable_fail_safe_ = false;
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        depth_odom_timeout_active = true;
      }
    }
    else if (depth_odom_timeout_active)
    {
      depth_odom_timeout_active = false;
      if (!mandatory_stop_)
      {
        enable_fail_safe_ = true;
        ROS_WARN("Depth/odom recovered. Planner resumed (WAIT_TARGET; send goal to replan).");
        if (exec_state_ == EMERGENCY_STOP)
          changeFSMExecState(WAIT_TARGET, "Depth recovered");
      }
    }

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= (int)pts_chk.size())
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).uav_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).uav_id == planner_manager_->pp_.uav_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            if (dist < allowed_dist)
            {
              ROS_WARN("swarm distance between UAV %d and UAV %d is %f, too close!",
                       planner_manager_->pp_.uav_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }
  /***************************************************************************
    flag_use_poly_init:决定使用哪种初始化方式
      true:使用多项式初始化（从起点到终点的简单轨迹）; 
      false：使用上一次最优轨迹初始化（从之前的轨迹中提取中间点）;
    flag_randomPolyTraj:在多项式初始化时，决定是否使用随机中间点（用于增加轨迹的多样性）
      false：单段轨迹（无中间点，直接从起点到终点）
      true：随机中间点（生成两段轨迹，中间点随机偏移）*/
  //局部重规划函数，用于从当前状态重新规划到局部目标点的轨迹
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {

      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo 最多尝试 trial_times 次重规划，默认10次
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;// 第一次规划时，使用多项式初始化
    else
      flag_random_poly_init = true;// 非第一次规划时，使用随机初始化

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    const double passed_t = ros::Time::now().toSec() - info->start_time;
    const bool local_traj_expired =
        info->duration < 1e-3 || passed_t >= info->duration - 1e-3;

    if (local_traj_expired)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }
    else
    {
      double t_cur = passed_t;
      if (t_cur < 0.0)
        t_cur = 0.0;
      else if (t_cur > info->duration)
        t_cur = info->duration;
#ifdef EGO_USE_TRAJ_INFO_AS_LOCAL_REPLAN_START
      start_pt_ = info->traj.getPos(t_cur);
      start_vel_ = info->traj.getVel(t_cur);
      start_acc_ = info->traj.getAcc(t_cur);
#else
      start_pt_ = odom_pos_;
      start_vel_ = info->traj.getVel(t_cur);
      start_acc_ = info->traj.getAcc(t_cur);
#endif
    }

    bool success = false;
    if (!local_traj_expired)
      success = callReboundReplan(false, false);

    if (!success)
    {
      if (local_traj_expired)
      {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();
      }
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
          return false;
      }
    }

    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(// 生成全局但不考虑碰撞的轨迹
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      final_goal_ = next_wp;
      /*** display 将这条轨迹通过点的方式显示出来***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      /* WAIT_TARGET / EMERGENCY_STOP / YAW_ALIGN：不卡在 while；YAW_ALIGN 仅更新 final_goal_ 继续转向 */
      if (exec_state_ != WAIT_TARGET &&
          exec_state_ != EMERGENCY_STOP &&
          exec_state_ != YAW_ALIGN)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        gotoPlanOrAlign(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  bool EGOReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;// 计算移动一个分辨率需要的时间
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step) // 当全局终点在障碍物中时，沿全局轨迹从终点向前搜索无碰撞点
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          if (planNextWaypoint(pt)) // final_goal_=pt inside if success，如果成功，则更新局部目标点
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= t_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    // 目标高度须高于 virtual_ground（与 grid_map 虚拟地面判定一致：z <= virtual_ground 不可飞）
    if (msg->goal[2] <= virtual_ground_)
    {
      ROS_WARN("UAV%d: Reject goal z=%.2f (must be > virtual_ground=%.2f)",
               planner_manager_->pp_.uav_id, msg->goal[2], virtual_ground_);
      return;
    }

    ROS_INFO("UAV%d Received goal: %f, %f, %f", planner_manager_->pp_.uav_id, msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp))
    {
      have_trigger_ = true;
      // 若当前处于强制停止状态，收到新目标后自动恢复规划
      if (exec_state_ == EMERGENCY_STOP)
      {
        mandatory_stop_ = false;
        enable_fail_safe_ = true;
        gotoPlanOrAlign(GEN_NEW_TRAJ, "New goal after mandatory stop");
      }
    }
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::resumePlanningCallback(const std_msgs::Empty &msg)
  {
    if (exec_state_ != EMERGENCY_STOP)
      return;
    mandatory_stop_ = false;
    enable_fail_safe_ = true;
    changeFSMExecState(WAIT_TARGET, "Resume planning");
    ROS_WARN("Resume planning: now in WAIT_TARGET. Send a new goal (e.g. 2D Nav Goal) to replan.");
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_yaw_ = quatToYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }
  /*接收其他飞机的轨迹，并检查是否发生碰撞，如果发生碰撞，则触发重新规划*/
  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->uav_id;
    if ((int)recv_id == planner_manager_->pp_.uav_id) // myself
      return;

    if (msg->uav_id < 0)
    {
      ROS_ERROR("uav_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id && // swarm_traj.size() > recv_id 说明已经收到了该飞机的轨迹，
        planner_manager_->traj_.swarm_traj[recv_id].uav_id == (int)recv_id && // uav_id 和 recv_id 相同，说明是同一架飞机
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)// 如果新消息的时间 ≤ 已存储的时间，说明是过时或重复的轨迹
    {
      ROS_WARN("Received UAV %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->uav_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->uav_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer ，当swarm_traj.size() <= recv_id时，动态扩展swarm_traj数组，防止下标越界*/
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.uav_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if ( msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data 解析和存储数据，将MINCOTraj消息中的数据解析为轨迹*/

    int piece_nums = msg->duration.size(); // 获取轨迹的段数
    Eigen::Matrix<double, 3, 3> headState, tailState; // 初始状态和末尾状态
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0], // 初始状态
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0], // 末尾状态
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1); // 中间点
    Eigen::VectorXd durations(piece_nums); // 每段轨迹所需时间
    for (int i = 0; i < piece_nums - 1; i++) // 中间点
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++) // 每段轨迹所需时间
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO; // 最小Jerk优化器
    MJO.reset(headState, tailState, piece_nums);// 重置优化器，设置初始状态和末尾状态，以及段数
    MJO.generate(innerPts, durations);// 生成轨迹

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true; // 是否远离我的飞机
    for (int i = 0; i < cps_chk.cols(); ++i)// 快速判断其他无人机的轨迹是否与当前无人机相关，如果所有采样点都远离，可以忽略该轨迹。
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // 检查采样点是否接近当前无人机,
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // 如果还没收到前面无人机的轨迹，也接受远离的轨迹
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].uav_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (planner_manager_->checkCollision(recv_id))
      {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)
      {
        // 检查是否收到了所有编号更小的无人机的轨迹
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.uav_id)
        {
          for (int i = 1; i < planner_manager_->pp_.uav_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].uav_id != i)
            {
              break;// 还没收到编号 i 的轨迹
            }

            have_recv_pre_agent_ = true;// 所有前面的都收到了
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].uav_id = -1; // Means this trajectory is invalid
    }
  }
 // 将计算的轨迹转换为ROS消息（poly_traj<用于执行> 和 MINCOTraj<用于广播>）
  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.uav_id = planner_manager_->pp_.uav_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.uav_id = planner_manager_->pp_.uav_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)// 用于测量前方地面高度，用于地形感知
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // 检查是否有有效的轨迹数据，如果检查点少于3个，说明规划未开始，返回失败
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;// 获取局部轨迹
    auto map = planner_manager_->grid_map_;// 获取地图
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m 计算向前2米所需时间
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t; // 计算当前轨迹时间 + 向前2米所需时间
    if (traj_t <= traj->duration) // 检查是否在轨迹时间内
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t); // 获取当前轨迹位置

      double reso = map->getResolution(); // 获取地图分辨率
      for (;; forward_p(2) -= reso) // 沿Z轴向下搜索，直到找到地面
      {
        int ret = map->getOccupancy(forward_p); // 获取地图占用情况 ，1:找到地面（占用），记录高度并发布，返回成功，-1:到达地图底部，未找到地面，返回失败
        if (ret == -1) // reach map bottom，接触到地图底部，返回失败
        {
          return false;
        }
        if (ret == 1) // reach the ground，到达地面，返回成功
        {
          height = forward_p(2); // 记录地面高度

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner
