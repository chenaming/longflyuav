// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/random_init_vertical_scale", pp_.random_init_vertical_scale_, 0.15);
    nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs, false);
    nh.param("manager/uav_id", pp_.uav_id, -1);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setUavId(pp_.uav_id);
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal)
  {
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    static int count = 0;
    cout << "\033[47;30m\n[" << t_start << "] UAV " << pp_.uav_id << " Replan " << count++ << "\033[0m" << endl;
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;
    // if ((start_pt - local_target_pt).norm() < 0.2)
    //   cout << "Close to goal" << endl;

    /*** STEP 1: INIT ***/
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;

    // ROS_ERROR("BBBB");

    if (pp_.use_multitopology_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;

      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        ploy_traj_opt_->setUseMultitopologyTrajs(true);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(), final_cost))
        {
          success[i] = true;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
      }

      t_opt = ros::Time::now() - t_start;

      if (trajs.size() > 1)
      {
        cout << "\033[1;33m"
             << "multi-trajs=" << trajs.size() << ",\033[1;0m"
             << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
      }

      visualization_->displayMultiOptimalPathList(vis_trajs, 0.1); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(), final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();

      t_opt = ros::Time::now() - t_start;
    }

    /*** STEP 3: Store and display results ***/
    cout << "Success=" << (flag_success ? "yes" : "no") << endl;
    if (flag_success)
    {
      static double sum_time = 0;
      static int count_success = 0;
      sum_time += (t_init + t_opt).toSec();
      count_success++;
      printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
             (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);
      // cout << "total time:\033[42m" << (t_init + t_opt).toSec()
      //      << "\033[0m,init:" << t_init.toSec()
      //      << ",optimize:" << t_opt.toSec()
      //      << ",avg_time=" << sum_time / count_success << endl;

      setLocalTrajFromOpt(best_MJO, touch_goal);
      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayOptimalList(cstr_pts, 0);

      continous_failures_count_ = 0;
    }
    else
    {
      cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      visualization_->displayFailedList(cstr_pts, 0);

      continous_failures_count_++;
    }

    return flag_success;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * pp_.random_init_vertical_scale_ *
                      (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }
/***************************************************************************
  获取局部目标点
  参数：
    planning_horizen: 规划范围
    start_pt: 起点
    global_end_pt: 全局终点
    local_target_pos: 局部目标点(输出)
    local_target_vel: 局部目标速度(输出)
    touch_goal: 是否到达目标点(输出)
  ***************************************************************************/
  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;// 保存上一次局部目标对应的全局轨迹时间，用于后续从历史轨迹初始化。

    double t_step = planning_horizen / 20 / pp_.max_vel_;// 规划范围/20/最大速度，计算时间步长,用于在全局轨迹上采样搜索
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);// 在全局轨迹上采样搜索，直到找到局部目标点 or 从上次局部目标时间开始，沿全局轨迹向前搜索，直到全局轨迹结束。
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);// 获取全局轨迹上时间t对应的位置
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)// 如果距离大于规划设定范围，则认为到达目标点，记录当前局部目标对应的全局轨迹时间，并退出循环。
      {
        local_target_pos = pos_t;             // 记录当前局部目标点。
        traj_.global_traj.glb_t_of_lc_tgt = t;// 记录当前局部目标对应的全局轨迹时间。
        break;
      }
    }
    // 计算局部目标点坐标
    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point，当全局轨迹时间t到达全局轨迹结束时间时，认为到达目标点。
    {
      local_target_pos = global_end_pt;// 记录全局终点作为局部目标点。
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;// 记录全局终点对应的全局轨迹时间。
      touch_goal = true;// 设置为到达目标点。
    }
    // 计算局部目标点速度
    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))// 如果全局终点与局部目标点的距离小于刹车距离，则认为局部目标点速度为0。这样做的目的是为了防止无人机在到达目标点时，速度过快，导致无人机无法及时刹车，从而发生碰撞。
    {
      local_target_vel = Eigen::Vector3d::Zero();// 设置局部目标点速度为0。
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
    {
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());
    }

    return ret;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }
  // 检查是否与其他飞机发生碰撞
  bool EGOPlannerManager::checkCollision(int uav_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;
    if (traj_.swarm_traj[uav_id].uav_id != uav_id) // The trajectory is invalid
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;// 当前飞机的轨迹起始时间
    double other_traj_start_time = traj_.swarm_traj[uav_id].start_time;// 其他飞机的轨迹起始时间

    double t_start = max(my_traj_start_time, other_traj_start_time);// 起始时间最大值
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[uav_id].duration);// 对比自身轨迹的2/3和其他飞机的整个轨迹

    for (double t = t_start; t < t_end; t += 0.03)// 在起始时间和结束时间之间，每隔0.03秒检查一次
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[uav_id].traj.getPos(t - other_traj_start_time))
              .norm() < (getSwarmClearance() + traj_.swarm_traj[uav_id].des_clearance) )// 如果同一时刻，自身的轨迹与其他飞机的轨迹距离小于期望的与其他飞机之间的最小距离，认为发生碰撞
      {
        return true;
      }
    }

    return false;
  }
  // 生成全局但不考虑碰撞的轨迹
  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];// 存储中间路径点（不包括最后一个路径点）
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5; // 期望速度，初始为最大速度的1/1.5
    Eigen::VectorXd time_vec(waypoints.size()); // 存储每段轨迹所需时间,大小为路径点数量

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i) // 计算每段轨迹所需时间
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel // 第一段轨迹所需时间
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel; // 其他段轨迹所需时间
      }

      globalMJO.generate(innerPts, time_vec); // 生成轨迹

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)// 这个永远不会执行
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5; // 再次减小期望速度
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
