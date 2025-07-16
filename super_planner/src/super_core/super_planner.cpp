/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <super_core/super_planner.h>
#include <memory>
#include <super_utils/scope_timer.hpp>
#include <fmt/color.h>

using namespace super_utils;

namespace super_planner {
    SuperPlanner::SuperPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)), ros_ptr_(ros_ptr), map_ptr_(map_ptr) {

        // 配置可视化选项
        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        // 初始化轨迹优化模块
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);
        // 获取地图配置
        const auto &rog_map_cfg = map_ptr_->getMapConfig();
        // 初始化A*搜索模块
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_);
        // 初始化凸分解模块
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num);
        // 配置凸分解用到的种子线段在其附近多大范围内不能有障碍
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        // 初始化记录时间消耗的向量
        time_consuming_.resize(8);

        // 初始化机器人状态为未知
        robot_state_.rcv = false;
        // 初始规划器开始时刻为当前时刻
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        // 初始化基于 FOV 的安全走廊生成模块
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        // 设置平移量, 将查找某个点的邻居时令查找范围覆盖半径为 robot_r 的一个范围
        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    RET_CODE
    SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal) {
        // 干活之前先上锁, 不能与重规划同时运行
        std::lock_guard<std::mutex> guard(replan_lock_);
        // 清除上一次规划的结果
        latest_replan.reset();
        // 设置目标
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        if (robot_state_.rcv == false) {
            // 如果没有有效的里程计消息, 则直接返回失败
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
            return FAILED;
        }
        // 先把目标点和航向设置到全局信息中
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;

        // FIXME: vec_Vec3f viz_pts 应该放到大括号里面
        // 画一条从当前位置指向目标位置的线, 同时记录时间消耗
        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space.
        // 调整起始位置到一个附近的自由栅格上
        Vec3f local_star_pt;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        last_exp_traj_info_.setEmpty();
        local_start_p_ = local_star_pt;
        // 生成探索轨迹
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            // 生成失败就返回失败
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            // 成功的话就打印一条日志
            ros_ptr_->info(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        back_traj_info.setEmpty();
        // 生成备份轨迹
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;

        if (back_ret_code == SUCCESS) {
            // 备份轨迹也生成成功的话就进行如下处理
            if (cfg_.print_log) {
                // 如果开启日志功能就打印一条日志
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            // 生成提交轨迹
            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            // 记录最近一次生成的探索轨迹
            last_exp_traj_info_ = exp_traj_info;
            // 由于刚生成完新的轨迹, 此时机器人处于新的探索轨迹上,
            //  所以先把机器人状态设置为不在备份轨迹上
            robot_on_backup_traj_ = false;
            // 设置全局信息中的新目标标志为 false
            gi_.new_goal = false;

            // For visualization
            // 可视化提交轨迹
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                // 设置规划结果的详细信息
                // FIXME: latest_replan.setRetCode 根可视化也没关系, 为啥不放外面?
                latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            // 如果备份轨迹生成结果为 FINISH 或 NO_NEED
            if (cfg_.print_log) {
                // 打印一条日志
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            // 设置机器人为 [不在备份轨迹上]
            robot_on_backup_traj_ = false;
            // 提交轨迹只包含探索轨迹
            cmd_traj_info_.setTrajectory(exp_traj_info);
            // 记录最近一次生成的探索轨迹
            last_exp_traj_info_ = exp_traj_info;
            // 设置全局信息中的新目标标志为 false
            gi_.new_goal = false;

            // For visualization
            // 可视化提交轨迹
            // FIXME: TimeConsuming t_viz 为啥不跟上面一样放在大括号里面?
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            // 设置规划结果的详细信息为 SUPER_SUCCESS_NO_BACKUP
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // 如果备份轨迹生成结果为 FAILED, 则打印日志并返回 FAILED
        ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }


    RET_CODE
    SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal) {
        // 直接对全部的重规划过程进行计时
        TimeConsuming replan_total_t("ReplanOnce", false);
        // 干活之前先上锁, 不能与规划同时运行
        std::lock_guard<std::mutex> guard(replan_lock_);

        // 更新目标位置为这一次重规划的目标位置
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        // 清除上一次规划的结果
        latest_replan.reset();
        // 设置最新的重规划目标
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        // 这里根 PlanFromRest 一样
        // FIXME: vec_Vec3f viz_pts 应该放到大括号里面
        // 画一条从当前位置指向目标位置的线, 同时记录时间消耗
        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        // 为生成探索轨迹计时
        TimeConsuming t_exp("t_exp", false);
        // 生成探索轨迹
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            // 如果生成探索轨迹失败, 则打印日志并返回 FAILED
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            // 如果生成探索轨迹结果为 NEW_TRAJ, 则打印日志并返回 NEW_TRAJ
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            // 如果生成探索轨迹结果为 EMER, 则打印日志并返回 EMER
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            // 如果生成探索轨迹结果为 SUCCESS, 则打印日志
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            // 如果生成探索轨迹结果为 NO_NEED, 则打印日志并返回 NO_NEED
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
        }

        // 可视化探索轨迹中的朝向部分
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        TimeConsuming t_back("t_back", false);
        // 生成备份轨迹
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        // 统计前端规划时间 和 后端优化时间
        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        // 重规划流程计时结束
        double replan_dt = replan_total_t.stop();
        if (replan_dt > cfg_.replan_forward_dt * 0.9) {
            // 如果重规划时间超过了配置的重规划单步时间的 90%，则打印警告日志并返回 FAILED
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            return FAILED;
        }

        // 若果备份轨迹生成结果为 SUCCESS, 则进行如下处理
        if (back_ret_code == SUCCESS) {
            // 生成提交轨迹
            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            // 记录最近一次生成的探索轨迹
            last_exp_traj_info_ = exp_traj_info;
            // 设置状态 [未在备份轨迹上]
            robot_on_backup_traj_ = false;
            // 设置状态 [没有新目标]
            gi_.new_goal = false;

            // 可视化提交轨迹
            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            // 设置详细状态为 SUPER_SUCCESS_WITH_BACKUP
            latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            // 如果生成备份轨迹结果为 NO_NEED, 则进行如下处理
            // 与上面一样设置状态
            robot_on_backup_traj_ = false;
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;


            // 可视化提交轨迹中的探索轨迹, 没有备份轨迹
            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            // 设置最新的重规划结果为 SUPER_SUCCESS_NO_BACKUP
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // 如果备份轨迹生成结果为 FINISH, 则进行如下处理
            // Which means the exp traj is all in known free, no need for backup traj
            // 设置提交轨迹只包含探索轨迹
            cmd_traj_info_.setTrajectory(exp_traj_info);
            // 设置状态, 与上面相同
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            // 可视化提交轨迹
            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            
            // 设置最新的重规划结果为 SUPER_SUCCESS_NO_BACKUP
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        // 如果备份轨迹生成结果为 FAILED, 则打印日志并返回 FAILED
        ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    void SuperPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime();
        if (cmd_traj_info_.backupTrajAvilibale() && eval_t > cmd_traj_info_.getBackupTrajStartTT()) {
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
    }

    Trajectory SuperPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory SuperPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        cmd_traj_info_.lock();
        const double &cur_t = ros_ptr_->getSimTime();
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
        on_backup_traj = robot_on_backup_traj_;

        pvaj = cmd_traj_info_.posTraj().getState(eval_t);


        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;

        yaw = cmd_traj_info_.getYaw((eval_t))[0];
        yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
    }


    void SuperPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }

    /**
     * @brief 生成探索轨迹（Exp Traj），用于无人机路径规划的主要方法。
     * 
     * 该方法根据当前机器人状态、目标点、历史轨迹等信息，生成一条新的探索轨迹。
     * 主要流程包括：
     * 1. 检查上一代轨迹是否可用，若不可用则重新生成。
     * 2. 对上一条轨迹进行碰撞检测，判断是否需要重新规划, 获取安全的引导轨迹.
     * 3. 在引导轨迹基础上继续搜索, 得到完整的, 连到目标点的轨迹.
     * 4. 生成安全飞行走廊（SFC）。
     * 5. 对轨迹进行优化，得到最终的多项式轨迹。
     * 6. 生成对应的yaw轨迹。
     * 7. 计算备份轨迹的在新轨迹上的切换时间
     * 8. 返回生成的探索轨迹信息。
     * 
     * @param last_exp_traj_info 上一次的探索轨迹信息
     * @param out_exp_traj_info 输出的新探索轨迹信息
     * @return RET_CODE 返回轨迹生成的状态码
     */
    RET_CODE SuperPlanner::generateExpTraj(ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info) {
        /* 1) Log the exp traj frontend time*/
        // 1) 记录探索轨迹前端耗时
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        // 位置初末状态
        StatePVAJ pos_init_state, pos_fina_state;
        // 安全飞行走廊 Safe Flight Corridor
        PolytopeVec sfc;
        // 引导路径（A*搜索得到的路径）
        vec_Vec3f guide_path;
        // the guide_stamp saves a TT
        // 时间戳，记录每个引导路径点的时间
        vector<double> guide_stamp;
        // 末端速度
        double guide_path_end_vel{0.0};
        // 初始化路径点数量： 规划距离 / 地图分辨率 * 1.2
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        // 初末航向状态
        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        // WT: 指实际时间，从 clock() 方法读上来的
        // 当前规划开始的实际时间
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        
        // TT: 指相对于轨迹的时间
        // replan_process_start_TT: 当前规划开始时的相对于轨迹的时间
        // replan_state_TT: 将要规划的目标状态在相对轨迹的时间戳
        double replan_process_start_TT, replan_state_TT;

        /* 2) Check last exp traj */
        
        //////////////////////////////////////
        // 1. 从上一代提交轨迹中获取无碰撞路点 //
        //////////////////////////////////////
        if (last_exp_traj_info.empty()) {
            /* 2.1) Perform rest2rest exp traj generation */
            // just skip the first part of the guide trajectory
            // 如果没有之前规划过的轨迹, 则以当前位置为起点开始规划
            pos_init_state.setZero();
            // 设置为机器人当前位置
            pos_init_state.col(0) = local_start_p_;
            // 由于轨迹不存在, 所以时间戳什么的都没有
            // 没有规划开始时间戳
            replan_process_start_TT = -1;
            // 也没有相对估计的规划目标的时间戳
            replan_state_TT = -1;
        } else {
            // !!!!!! 1.1 获取上一代轨迹信息

            // cmd_traj_info_ 就是论文中的 commite trajectory
            // 提交轨迹中获取的路径时探索路径与备份路径的融合
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            // yaw 轨迹也是融合
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            // 这个获取的时上一轮的完整的探索轨迹
            last_exp_traj = last_exp_traj_info.posTraj();

            // !!!!!! 1.2 在轨迹时间坐标系下计算规划开始时刻的时间戳, 规划起点位置的时间戳

            // 当前规划开始的相对于轨迹的时间 = 当前规划开始的实际时间 - 轨迹开始的实际时间
            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            // (轨迹时间坐标系)这一轮规划起点的时间戳 = 本轮规划开始时间 + 路点时间间隔
            // 因为规划开始时间戳对应的当前位置必不可能被占用, 所以往前推进一个单位的时间作为规划起始点
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;

            // 保存无碰撞的 <时间戳,路点> 列表
            vector<TimePosPair> last_exp_traj_time_pos;
            // 保存时间戳对应的速度
            vector<double> last_exp_traj_vel;


            // !!!!!! 1.3 检查规划起点的时间戳是否合法, 路径的起始位置是否离目标点近到可以不用规划
            // 1.3.1 如果要规划的时间点超过了提交轨迹需要的总时间,则不再规划
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (robot_on_backup_traj_) {
                    // 如果超时时飞机还处在备份轨迹上, 则返回 FAILED
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                // 如果超时 && 飞机在探索轨迹上, 则返回 NO_NEED
                return NO_NEED;
            }

            // 如果上一代的探索轨迹规划出来了, 则进行更多的检查
            if (!last_exp_traj_info.empty()) {
                // 1.3.2 如果重规划起点的时刻超过探索路径总时间, 则直接返回
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                // 1.3.3 如果探索路径终点就在目标附近, 直接退出
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                // 1.3.4 如果规划起点的位置就在目标附近, 直接退出
                if (!gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
                    // Return if the traj close to goal
                    out_exp_traj_info = last_exp_traj_info;
                    // 同时设置 GoalConnected 标记
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [SUPER] Replan, close to goal and return NONEED.");
                    if (robot_on_backup_traj_) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            // 如果走到这里, 说明轨迹无论如何没有与目标点连接
            out_exp_traj_info.setGoalConnectedFlag(false);
            // ****** 1.3 结束后规划相关的时间戳得到了保证, 同时上一代探索路径的起始位置也不会离目标太近

            // !!!!!! 1.4 对上一代提交轨迹做碰撞检测

            // 以 replan_state_TT 为起点开始检测, 将所有安全的路点加入 last_exp_traj_*
            // 开始规划的起始时刻
            double eval_t = replan_state_TT; //replan_process_start_TT;
            // 轨迹总时间
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            // FIXME: 明明是在对提交轨迹做碰撞检测, 怎么能直接设置整条探索路径为安全的呢?
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            // 这个 id 看起来没什么用
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
                // 获取当前时刻在轨迹上的位置
                temp_pt = guide_pos_traj.getPos(eval_t);
                // 如果与上一个点的距离过近, 则检测跳过这个点
                // FIXME: 但这里我觉得还是有连个点位于同一个栅格内的可能, 这种地图是按照绝对距离划分的,不是相对距离
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                // 检测当前点是否被占用
                rog_map::GridType temp_grid = map_ptr_->getInfGridType(temp_pt);

                // 如果栅格状态异常则直接停止检测, 同时设置整条路径不再是安全的
                if (temp_grid == rog_map::GridType::OCCUPIED || temp_grid == rog_map::GridType::OUT_OF_MAP) {
                    last_exp_traj_info.setWholeTrajKnownFreeFlag(false);
                    break;
                }

                // FIXME: replan_id 没用, 可以拿掉
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                // 保存无碰撞的点位, 时间, 和速度
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                // 保存当前采样点
                last_sample_pt = temp_pt;
            }


            // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
            // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
            // *    is given, we should only receiding a small distance and replan new trajectory ASAP
            
            // !!!!!! 1.5 切割轨迹, 只保留 split_dis 范围之内的
            // 这一步完事将得到 guide_path, guide_stamp, guide_path_end_vel

            // 设置保存路点相对于当前位置的最大距离 (超过距离的路点就不要了)
            double split_dis = cfg_.receding_dis;
            if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
                // 当整条轨迹是安全的 并且 没有新目标 并且 配置的重规划距离>0 时
                // 把分割距离设置为无限大 (几乎是这样)
                // 其实就是不做分割, 直接复用整条轨迹
                split_dis = std::numeric_limits<double>::max();
            }


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            // 获取规划起始时刻的状态作为初始状态
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                // 没拿到那肯定完犊子了
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                // 如果分割长度小于0 || 没有安全的路点
                // 说明不需要轨迹复用, 此时只将初始状态写入引导轨迹
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                // 因为指导路径只有一个起始点, 所以末端速度设置为初始状态的速度
                guide_path_end_vel = robot_state_.v.norm();
            } else {
                // 如果分割长度大于0 且 有安全的路点
                // 删掉所有超过 split_dis 距离之外的路点, 同时那些被占用的且排在末端的路点也会被丢掉
                // 不过 last_exp_traj_time_pos 保存的本来就是安全的路点, 所以这里不会有被占用的情况
                // FIXME: 拿掉 isOccupiedInflate 的判断, 看看对性能有多大影响
                temp_pt = last_exp_traj_time_pos.back().second;
                // * 8) Pop all evaluated pts after the sampled point.
                while (map_ptr_->isOccupiedInflate(temp_pt) ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    // 防止 last_exp_traj_time_pos 为空导致访问越界
                    if (last_exp_traj_time_pos.empty()) {
                        ros_ptr_->warn(" -- [SUPER] WARN, all traj is collide in INF2");
                        break;
                    }
                    temp_pt = last_exp_traj_time_pos.back().second;
                }

                if (!last_exp_traj_time_pos.empty()) {
                    // 将安全的路点加入 guid_path guid_stamp
                    for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                        guide_path.push_back(last_exp_traj_time_pos[i].second);
                        // 此时的时间坐标系转换为相对于 last_exp_traj_time_pos.front() 的时间坐标系
                        guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                        // FIXME: 为啥不放在最后, last_exp_traj_vel.back() ? 这个也不会越界
                        guide_path_end_vel = last_exp_traj_vel[i];
                    }
                } else {
                    // 如果路点都被删干净了, 那么 guid_* 只配置重规划的起始状态
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = robot_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================


        //////////////////////////
        // 2. 生成完整的引导路径 //
        /////////////////////////
        
        // 已有的引导路径长度
        double guide_path_length = geometry_utils::computePathLength(guide_path);
        // 剩余需要搜索路径的长度 = 最大规划长度 - 已有的引导路径长度
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        // FIXME: 用不到了, 可以删掉
        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        // 如果引导路径为空，或者引导路径的第一个点与初始状态的距离大于1cm，则将当前位置添加到引导路径开头
        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // 当从上一代获取的指导轨迹长度不够时, 需要再搜索后面的部分, 最终连上目标点
        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            // !!!!!! 2.1 如果引导路径的最后一个点与目标点的距离足够近 (小于分辨率的5倍)，则将目标点添加到引导路径中
            if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {
                // 时间消耗的计算假设飞机以最大速度计算
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                // project goal within the planning horizon
                // const Vec3f dir = (gi_.goal_p - robot_state_.p).normalized();
                // const double dis2goal = (gi_.goal_p - robot_state_.p).norm();
                // Vec3f cadi_p = gi_.goal_p;
                // if(dis2goal > cfg_.planning_horizon) {
                //     double proj_l = cfg_.planning_horizon;
                //     Vec3f cadi_p = robot_state_.p + dir * proj_l;
                //     int max_iter = 100;
                //     while(map_ptr_->isOccupiedInflate(cadi_p) && max_iter-- > 0) {
                //         if(map_ptr_->getNearestInfCellNot(OCCUPIED, cadi_p, cadi_p, 1.0)) {
                //             break;
                //         }
                //         proj_l -= 2.0;
                //         if(proj_l < 1){
                //             ros_ptr_->warn(" -- [SUPER] Project goal failed");
                //             gi_.goal_valid = false;
                //             return FAILED;
                //         }
                //         cadi_p = robot_state_.p + dir * proj_l;
                //     }
                //     if(max_iter <= 0) {
                //         ros_ptr_->warn(" -- [SUPER] Project goal failed");
                //         gi_.goal_valid = false;
                //         return FAILED;
                //     }
                // }

                // !!!!!! 2.2 如果引导路径的结束位置距离目标点还有一段距离, 则找一条从引导路径
                // 最后一个点到目标点的路径，限制路径长度为 temp_horizon
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }
                // 即使找到路径, 但是如果新路径的长度小于2个点, 也认为路径搜索失败
                if (new_path.size() < 2) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }

                // compute total dis
                // backward compute dis for all points
                // 计算新路径上所有路径点之间的距离, 以及总距离
                double total_dis{0.0};
                vector<double> dis(new_path.size());
                Vec3f last_p = new_path.back();
                for (int i = new_path.size() - 2; i >= 0; i--) {
                    auto d = (new_path[i] - last_p).norm();
                    total_dis += d;
                    dis[i+1] = total_dis;
                    last_p = new_path[i];
                }
                total_dis += (new_path.front() - guide_path.back()).norm();
                dis[0] = total_dis;
                //  for (int i = 0; i < dis.size(); i++) {
                //      cout << dis[i] << " ";
                //  }
                //  cout << endl;
                // !!!!!! 2.3 计算每个路径点的时间戳
                vector<double> stamps(new_path.size(), 0);
                vector<double> dt(new_path.size(), 0);
                double last_stamp = 0;
                for (int i = dis.size() - 1; i >= 0; i--) {
                    double vel;
                    // 计算每个路径点上应该达到的速度和时间戳
                    geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                          guide_path_end_vel,
                                                          total_dis,
                                                          dis[i], stamps[i], vel);
                    dt[i] = stamps[i] - last_stamp;
                    last_stamp = stamps[i];
                }
                double time_stamp = guide_stamp.back();

                //  for (int i = 0; i < stamps.size(); i++) {
                //      cout << stamps[i] << " ";
                //  }
                //  cout << endl;
                //
                //  for (int i = 0; i < dt.size(); i++) {
                //      cout << dt[i] << " ";
                //  }
                //  cout << endl;

                // 更新引导路径和时间戳
                for (long unsigned int i = 1; i < new_path.size(); i++) {
                    double t = dt[i];
                    time_stamp += t;
                    guide_path.emplace_back(new_path[i]);
                    guide_stamp.emplace_back(time_stamp);
                }
            }
        }

        // 如果路径最后一点的位置与目标位置在 xoy 平面上足够近 (小于两倍栅格尺寸), 则认为该路径已与目标相连
        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        ////////////////////////
        // 3. 计算安全飞行走廊 //
        ////////////////////////

        // 将 A* 搜索到路径转化为多个多边形，也叫安全飞行走廊
        // sfc：Safe Flight Corridor
        sfc.clear();
        {
            // 可视化路径
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut);

        if (!bool_ret_code) {
            ros_ptr_->warn(" -- [SUPER] SearchPolytopeOnPath for new path failed");
            return FAILED;
        }
        {
            // 可视化飞行走廊
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        ////////////////////
        // 4. 设置最终状态 //
        ////////////////////
        
        pos_fina_state.setZero();
        // 默认将最终位置为引导路径的最后一点所在的位置
        pos_fina_state.col(0) = guide_path.back();
        // 如果配置要求到达最终位置时是有速度的 (goal_vel_en 为真)，且目标位置到当前位置的距离还够调整速度(最大规划距离的一半)
        // 就将目标位置的速度设置为最大速度的一半
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        // 如果目标位置与最终位置的距离很近 (小于两倍的地图分辨率)
        //  就将最终位置的速度设置为0，并将最终位置设置为目标位置 (默认是设置为引导路径的最后一点, 不一样的)
        if ((pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
            pos_fina_state.col(1).setZero();
            pos_fina_state.col(0) = gi_.goal_p;
        }

        ////////////////////
        // 5. 求解最终轨迹 //
        ////////////////////
        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);
        
        // 记录时间消耗
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            // 记录优化相关信息
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        
        // 如果优化直接失败则返回失败
        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }

        // 如果规划时间超过了预计的规划时间, 也算失败
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (replan_total_t > cfg_.replan_forward_dt) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({})!!!! Return FAILED", replan_total_t);
            return FAILED;
        }

        {
            // 可视化规划轨迹
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;

        // 重新计算规划起始时刻在轨迹时间戳坐标系下的时间戳
        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;

        // 尝试获取历史轨迹
        if (!last_exp_traj_info_.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }

        // 设置返回轨迹中的安全飞行走廊
        out_exp_traj_info.setSFC(sfc);
        // 设置返回轨迹中的探索轨迹 = 上一代轨迹(从当前规划起始时刻开始) + 新规划的轨迹
        temp_exp_traj = temp_exp_traj + out_traj;
        // 设置返回轨迹中探索轨迹的起始时间戳 = 当前规划开始的实际时间
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;


        ////////////////////
        // 6. 优化朝向轨迹 //
        ////////////////////

        // 尝试获取历史轨迹中的朝向作为初始朝向
        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        // 当配置要求控制最终的朝向 && 最终朝向是一个正常值 && 这条轨迹与目标位置相连
        //  则设置最终位置的 yaw 的值
        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        // 规划 yaw 的轨迹
        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        // 若参考了上一代轨迹, 则获取之前那一段的 yaw 轨迹,
        //  没拿到就认为失败
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        // 设置 yaw 的轨迹
        const auto temp_yaw_traj = old_traj + new_traj;

        /////////////////////////////////////////////////
        // 7. 计算备份轨迹切换时刻在新轨迹时间坐标系下的值 //
        /////////////////////////////////////////////////

        // 如果规划起始时刻 > 上一代轨迹的备份起始时刻, 则说明这次规划是基于备份轨迹的
        // 重新计算备份轨迹起始时刻与结束时刻
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }

        /////////////////////////
        // 8. 设置输出的轨迹信息 //
        /////////////////////////

        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);
        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE SuperPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            // Compute initial
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));
            const double min_dis =
                    cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                             : cfg_.safe_corridor_line_max_length;
            if (!map_ptr_->isLineFree(back_traj_info.getRobotPos(),
                                      temp_point,
                                      min_dis,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        ros_ptr_->vizBackupSfc(temp_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [SUPER] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [SUPER] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        auto sfc0 = back_traj_info.getSFC();
        bool temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_dur,
                                                 back_traj_info.getSFC(),
                                                 temp_pos_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, te,
                                             back_traj_info.getSFC());
            Trajectory traj;
            double out_ts;
            back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                     t0,
                                     te,
                                     init_ts,
                                     sfc0,
                                     init_times,
                                     init_ps,
                                     traj,
                                     out_ts
            );

        }

        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                         temp_yaw_traj, 3, false, free_end)) {
                ros_ptr_->error(" -- [SUPER] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ros_ptr_->error(" -- [SUPER] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
            if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                ros_ptr_->error(" -- [SUPER] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizBackupTraj(temp_pos_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            latest_replan.setBackupTraj(temp_pos_traj);
            latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] Cannot find backup traj start point.");
        return FAILED;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [SUPER] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = map_ptr_->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            ros_ptr_->warn(
                    " -- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [SUPER] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                               path);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [SUPER] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [SUPER] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }
        return true;
    }


    void SuperPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_ptr_->getRobotState();
        out = robot_state_;
    }
}
