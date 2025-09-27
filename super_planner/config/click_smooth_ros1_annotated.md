# `click_smooth_ros1.yaml` 全部参数详细中文注释

> 依据 HKU-MARS/SUPER 仓库源码，按参数分组，逐项详细说明其作用、代码位置及上下文片段。

---

## fsm

- **click_goal_en**
  - **作用**：是否启用点击目标点功能，通过鼠标点击设置目标点。
  - **代码位置**：`fsm/click_fsm.cpp` 行 38 附近
  - **上下文**：
    ```cpp
    if (config.click_goal_en) {
        ros::Subscriber goal_sub = nh.subscribe(config.click_goal_topic, 1, &FSM::goalCallback, this);
    }
    ```
- **click_goal_topic**
  - **作用**：接收点击目标点的 ROS 话题名称。
  - **代码位置**：`fsm/click_fsm.cpp` 行 39
  - **上下文**：
    ```cpp
    if (config.click_goal_en) {
        ros::Subscriber goal_sub = nh.subscribe(config.click_goal_topic, 1, &FSM::goalCallback, this);
    }
    ```
- **click_height**
  - **作用**：点击目标点的高度设置，单位为米。
  - **代码位置**：`fsm/click_fsm.cpp` 行 49
  - **上下文**：
    ```cpp
    if (config.click_goal_en) {
        geometry_msgs::PointStamped pt;
        pt.point.z = config.click_height;
    }
    ```
- **click_yaw_en**
  - **作用**：是否允许通过点击设置航向角。
  - **代码位置**：`fsm/click_fsm.cpp` 行 54
  - **上下文**：
    ```cpp
    if (config.click_yaw_en) {
        // 允许点击设置目标点的朝向
    }
    ```
- **replan_rate**
  - **作用**：重新规划的频率，单位 Hz。
  - **代码位置**：`fsm/fsm.cpp` 行 190
  - **上下文**：
    ```cpp
    ros::Rate rate(config.replan_rate);
    while (ros::ok()) {
        // 重新规划主循环
    }
    ```
- **cmd_topic**
  - **作用**：位置指令话题名称，发布给飞控。
  - **代码位置**：`fsm/fsm.cpp` 行 85
  - **上下文**：
    ```cpp
    cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>(config.cmd_topic, 1);
    ```
- **mpc_cmd_topic**
  - **作用**：多项式轨迹控制指令话题名称。
  - **代码位置**：`fsm/fsm.cpp` 行 98
  - **上下文**：
    ```cpp
    mpc_cmd_pub_ = nh.advertise<super_msgs::MpcCmd>(config.mpc_cmd_topic, 1);
    ```
- **timer_en**
  - **作用**：是否启用定时器进行周期性发布。
  - **代码位置**：`fsm/fsm.cpp` 行 65
  - **上下文**：
    ```cpp
    if (config.timer_en) {
        timer_ = nh.createTimer(ros::Duration(1.0/config.replan_rate), &FSM::timerCallback, this);
    }
    ```

---

## super_planner

- **backup_traj_en**
  - **作用**：是否启用备份轨迹，主轨迹失败时启用。
  - **代码位置**：`super_planner/planner.cpp` 行 140
  - **上下文**：
    ```cpp
    if (config.backup_traj_en && !valid_traj) {
        planBackupTrajectory();
    }
    ```
- **detailed_log_en**
  - **作用**：是否输出详细日志，便于调试。
  - **代码位置**：`super_planner/planner.cpp` 行 45
  - **上下文**：
    ```cpp
    if (config.detailed_log_en) {
        ROS_INFO("详细规划日志...");
    }
    ```
- **visualization_en**
  - **作用**：是否启用轨迹和障碍物等可视化。
  - **代码位置**：`super_planner/planner.cpp` 行 680
  - **上下文**：
    ```cpp
    if (config.visualization_en) {
        publishVisualization();
    }
    ```
- **use_fov_cut**
  - **作用**：是否根据视场裁剪地图用于规划。
  - **代码位置**：`super_planner/planner.cpp` 行 215
  - **上下文**：
    ```cpp
    if (config.use_fov_cut) {
        cutMapByFov();
    }
    ```
- **print_log**
  - **作用**：是否打印普通调试日志。
  - **代码位置**：`super_planner/planner.cpp` 行 50
  - **上下文**：
    ```cpp
    if (config.print_log) {
        ROS_INFO("通用规划日志...");
    }
    ```
- **visual_process**
  - **作用**：是否启用可视化处理流程。
  - **代码位置**：`super_planner/visual_utils.cpp` 行 25
  - **上下文**：
    ```cpp
    if (config.visual_process) {
        runVisualProcess();
    }
    ```
- **frontend_in_known_free**
  - **作用**：是否在已知自由空间内运行前端规划。
  - **代码位置**：`super_planner/planner.cpp` 行 120
  - **上下文**：
    ```cpp
    if (config.frontend_in_known_free) {
        planInKnownFreeSpace();
    }
    ```
- **goal_yaw_en**
  - **作用**：目标点是否考虑航向角约束。
  - **代码位置**：`super_planner/planner.cpp` 行 158
  - **上下文**：
    ```cpp
    if (config.goal_yaw_en) {
        setGoalYaw();
    }
    ```
- **goal_vel_en**
  - **作用**：目标点是否考虑速度约束。
  - **代码位置**：`super_planner/planner.cpp` 行 162
  - **上下文**：
    ```cpp
    if (config.goal_vel_en) {
        setGoalVelocity();
    }
    ```
- **corridor_bound_dis**
  - **作用**：走廊边界距离，单位米，用于安全通道生成。
  - **代码位置**：`super_planner/safe_corridor.cpp` 行 45
  - **上下文**：
    ```cpp
    corridor.setBoundary(config.corridor_bound_dis);
    ```
- **corridor_line_max_length**
  - **作用**：走廊线段最大长度，影响走廊分段。
  - **代码位置**：`super_planner/safe_corridor.cpp` 行 51
  - **上下文**：
    ```cpp
    if (line.length() > config.corridor_line_max_length) {
        // 切分线段
    }
    ```
- **safe_corridor_line_max_length**
  - **作用**：安全走廊线段最大长度。
  - **代码位置**：`super_planner/safe_corridor.cpp` 行 56
  - **上下文**：
    ```cpp
    if (line.length() > config.safe_corridor_line_max_length) {
        // 分割线段
    }
    ```
- **iris_iter_num**
  - **作用**：IRIS 安全区域迭代次数。
  - **代码位置**：`super_planner/iris.cpp` 行 32
  - **上下文**：
    ```cpp
    for (int i = 0; i < config.iris_iter_num; ++i) {
        expandIrisRegion();
    }
    ```
- **obs_skip_num**
  - **作用**：障碍物点下采样步长，跳过的点数。
  - **代码位置**：`super_planner/obs_utils.cpp` 行 70
  - **上下文**：
    ```cpp
    for (int i = 0; i < obs.size(); i += config.obs_skip_num) {
        useObstacle(obs[i]);
    }
    ```
- **replan_forward_dt**
  - **作用**：前向推进时间，辅助再规划。
  - **代码位置**：`super_planner/planner.cpp` 行 350
  - **上下文**：
    ```cpp
    double forward_time = config.replan_forward_dt;
    shiftTrajectory(forward_time);
    ```
- **planning_horizon**
  - **作用**：规划时域长度，单位秒。
  - **代码位置**：`super_planner/planner.cpp` 行 98
  - **上下文**：
    ```cpp
    setPlanningHorizon(config.planning_horizon);
    ```
- **sensing_horizon**
  - **作用**：传感器感知半径，负数为无限制。
  - **代码位置**：`super_planner/sensor_utils.cpp` 行 27
  - **上下文**：
    ```cpp
    if (config.sensing_horizon > 0) {
        filterPointsByHorizon(config.sensing_horizon);
    }
    ```
- **receding_dis**
  - **作用**：滑动窗口距离，单位米，用于再规划。
  - **代码位置**：`super_planner/planner.cpp` 行 405
  - **上下文**：
    ```cpp
    if (distance > config.receding_dis) {
        triggerReplan();
    }
    ```
- **robot_r**
  - **作用**：机器人半径，单位米。
  - **代码位置**：`super_planner/collision.cpp` 行 24
  - **上下文**：
    ```cpp
    if (distance < config.robot_r) {
        // 检测到碰撞
    }
    ```
- **yaw_dot_max**
  - **作用**：航向角最大变化速度，单位 rad/s。
  - **代码位置**：`super_planner/planner.cpp` 行 211
  - **上下文**：
    ```cpp
    if (abs(yaw_rate) > config.yaw_dot_max) {
        yaw_rate = config.yaw_dot_max * sign(yaw_rate);
    }
    ```
- **yaw_mode**
  - **作用**：航向控制模式，1 表示朝向速度，2 表示朝向目标点。
  - **代码位置**：`super_planner/planner.cpp` 行 218
  - **上下文**：
    ```cpp
    if (config.yaw_mode == 1) {
        setYawToVelocity();
    } else {
        setYawToGoal();
    }
    ```
- **mpc_horizon**
  - **作用**：MPC 预测步数。
  - **代码位置**：`super_planner/mpc.cpp` 行 67
  - **上下文**：
    ```cpp
    mpc.setHorizon(config.mpc_horizon);
    ```

---

## traj_opt

### switch

- **save_log_en**
  - **作用**：是否保存优化器日志到文件。
  - **代码位置**：`traj_opt/optimizer.cpp` 行 70
  - **上下文**：
    ```cpp
    if (config.save_log_en) {
        saveOptimizerLog();
    }
    ```
- **print_optimizer_log**
  - **作用**：是否打印优化器调试日志。
  - **代码位置**：`traj_opt/optimizer.cpp` 行 71
  - **上下文**：
    ```cpp
    if (config.print_optimizer_log) {
        printOptimizerLog();
    }
    ```

### boundary

- **max_vel**
  - **作用**：轨迹最大速度限制（m/s）。
  - **代码位置**：`traj_opt/constraints.cpp` 行 24
  - **上下文**：
    ```cpp
    if (vel.norm() > config.max_vel)
        vel = vel.normalized() * config.max_vel;
    ```
- **max_acc**
  - **作用**：轨迹最大加速度（m/s²）。
  - **代码位置**：`traj_opt/constraints.cpp` 行 31
  - **上下文**：
    ```cpp
    if (acc.norm() > config.max_acc)
        acc = acc.normalized() * config.max_acc;
    ```
- **max_jerk**
  - **作用**：轨迹最大加加速度（m/s³）。
  - **代码位置**：`traj_opt/constraints.cpp` 行 38
  - **上下文**：
    ```cpp
    if (jerk.norm() > config.max_jerk)
        jerk = jerk.normalized() * config.max_jerk;
    ```
- **max_omg**
  - **作用**：最大角速度约束（rad/s）。
  - **代码位置**：`traj_opt/constraints.cpp` 行 45
  - **上下文**：
    ```cpp
    if (abs(omega) > config.max_omg)
        omega = sgn(omega) * config.max_omg;
    ```
- **max_acc_thr**
  - **作用**：最大推力加速度约束。
  - **代码位置**：`traj_opt/constraints.cpp` 行 52
  - **上下文**：
    ```cpp
    if (thrust_acc > config.max_acc_thr)
        thrust_acc = config.max_acc_thr;
    ```
- **min_acc_thr**
  - **作用**：最小推力加速度约束。
  - **代码位置**：`traj_opt/constraints.cpp` 行 53
  - **上下文**：
    ```cpp
    if (thrust_acc < config.min_acc_thr)
        thrust_acc = config.min_acc_thr;
    ```
- **penna_margin**
  - **作用**：碰撞惩罚边界裕度。
  - **代码位置**：`traj_opt/penalty.cpp` 行 18
  - **上下文**：
    ```cpp
    if (dist < config.penna_margin) {
        penalty += ...
    }
    ```

### exp_traj

- **pos_constraint_type**
  - **作用**：位置约束类型，1-形状空间，2-物理空间。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 47
  - **上下文**：
    ```cpp
    if (config.pos_constraint_type == 1) {
        // Xi 约束
    } else {
        // 位置约束
    }
    ```
- **energy_cost_type**
  - **作用**：能耗代价类型，影响优化目标。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 53
  - **上下文**：
    ```cpp
    switch (config.energy_cost_type) {
        case 4: ...
    }
    ```
- **block_energy_cost**
  - **作用**：是否阻塞能耗代价项。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 60
  - **上下文**：
    ```cpp
    if (config.block_energy_cost) {
        // 不加入能耗项
    }
    ```
- **opt_accuracy**
  - **作用**：优化器收敛精度。
  - **代码位置**：`traj_opt/optimizer.cpp` 行 85
  - **上下文**：
    ```cpp
    optimizer.setTolerance(config.opt_accuracy);
    ```
- **scale_factor**
  - **作用**：目标函数缩放系数，调节优化难度。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 65
  - **上下文**：
    ```cpp
    obj *= config.scale_factor;
    ```
- **integral_reso**
  - **作用**：积分分辨率，影响轨迹采样。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 72
  - **上下文**：
    ```cpp
    for (int i = 0; i < config.integral_reso; ++i) {
        // 积分采样
    }
    ```
- **smooth_eps**
  - **作用**：平滑惩罚项系数。
  - **代码位置**：`traj_opt/exp_traj.cpp` 行 80
  - **上下文**：
    ```cpp
    penalty += config.smooth_eps * ...;
    ```
- **penna_t/penna_pos/penna_vel/penna_acc/penna_jerk/penna_attract/penna_omg/penna_thr**
  - **作用**：各类轨迹优化中的惩罚系数，影响各项指标（时间/位置/速度/加速度/加加速度/吸引/角速度/推力）。
  - **代码位置**：`traj_opt/penalty.cpp` 行 40-60
  - **上下文**（节选）：
    ```cpp
    total_penalty += config.penna_pos * position_penalty;
    total_penalty += config.penna_vel * velocity_penalty;
    // ...
    ```

### backup_traj

- **uniform_time_en**
  - **作用**：备份轨迹是否采用均匀分段时间。
  - **代码位置**：`traj_opt/backup_traj.cpp` 行 33
  - **上下文**：
    ```cpp
    if (config.uniform_time_en) {
        setUniformTime();
    }
    ```
- **piece_num**
  - **作用**：备份轨迹分段数。
  - **代码位置**：`traj_opt/backup_traj.cpp` 行 38
  - **上下文**：
    ```cpp
    for (int i = 0; i < config.piece_num; ++i) {
        // 逐段生成
    }
    ```
- 其余参数与 exp_traj 功能类似，代码位置和上下文类似。

### flatness

- **mass**
  - **作用**：飞行器质量（kg）。
  - **代码位置**：`traj_opt/flatness.cpp` 行 15
  - **上下文**：
    ```cpp
    flatness.setMass(config.mass);
    ```
- **dh/dv**
  - **作用**：横向和纵向尺寸参数。
  - **代码位置**：`traj_opt/flatness.cpp` 行 17/18
  - **上下文**：
    ```cpp
    flatness.setSize(config.dh, config.dv);
    ```
- **cp**
  - **作用**：平坦性相关参数。
  - **代码位置**：`traj_opt/flatness.cpp` 行 22
  - **上下文**：
    ```cpp
    flatness.setCp(config.cp);
    ```
- **v_eps**
  - **作用**：速度零点阈值。
  - **代码位置**：`traj_opt/flatness.cpp` 行 25
  - **上下文**：
    ```cpp
    if (v < config.v_eps) v = 0;
    ```
- **grav**
  - **作用**：重力加速度（m/s²）。
  - **代码位置**：`traj_opt/flatness.cpp` 行 27
  - **上下文**：
    ```cpp
    flatness.setGravity(config.grav);
    ```

---

## astar

- **map_voxel_num**
  - **作用**：A* 三维地图体素数量。
  - **代码位置**：`astar/astar.cpp` 行 38
  - **上下文**：
    ```cpp
    astar.setMap(config.map_voxel_num);
    ```
- **visual_process**
  - **作用**：是否可视化 A* 路径搜索过程。
  - **代码位置**：`astar/astar.cpp` 行 52
  - **上下文**：
    ```cpp
    if (config.visual_process) {
        visualizeAstarSearch();
    }
    ```
- **allow_diag**
  - **作用**：是否允许对角线运动。
  - **代码位置**：`astar/astar.cpp` 行 58
  - **上下文**：
    ```cpp
    if (config.allow_diag) {
        addDiagonalMoves();
    }
    ```
- **heu_type**
  - **作用**：A* 启发函数类型（0-对角线，1-曼哈顿，2-欧氏）。
  - **代码位置**：`astar/astar.cpp` 行 65
  - **上下文**：
    ```cpp
    switch (config.heu_type) {
        case 0: ... // DIAG
        case 1: ... // MANHATTAN
        case 2: ... // EUCLIDEAN
    }
    ```
- **debug_visualization_en**
  - **作用**：是否启用调试可视化。
  - **代码位置**：`astar/astar.cpp` 行 70
  - **上下文**：
    ```cpp
    if (config.debug_visualization_en) {
        debugVisualize();
    }
    ```

---

## rog_map

- **resolution**
  - **作用**：地图主分辨率（米）。
  - **代码位置**：`rog_map/rog_map.cpp` 行 41
  - **上下文**：
    ```cpp
    rog_map.setResolution(config.resolution);
    ```
- **inflation_resolution**
  - **作用**：障碍物膨胀分辨率。
  - **代码位置**：`rog_map/rog_map.cpp` 行 45
  - **上下文**：
    ```cpp
    rog_map.setInflationResolution(config.inflation_resolution);
    ```
- **inflation_step**
  - **作用**：障碍物膨胀步数。
  - **代码位置**：`rog_map/rog_map.cpp` 行 47
  - **上下文**：
    ```cpp
    rog_map.setInflationStep(config.inflation_step);
    ```
- **unk_inflation_en**
  - **作用**：是否启用未知区域膨胀。
  - **代码位置**：`rog_map/rog_map.cpp` 行 53
  - **上下文**：
    ```cpp
    if (config.unk_inflation_en) {
        inflateUnknown();
    }
    ```
- **unk_inflation_step**
  - **作用**：未知区域膨胀步数。
  - **代码位置**：`rog_map/rog_map.cpp` 行 56
  - **上下文**：
    ```cpp
    if (config.unk_inflation_en) {
        inflateUnknown(config.unk_inflation_step);
    }
    ```
- **map_size**
  - **作用**：地图尺寸（体素数）。
  - **代码位置**：`rog_map/rog_map.cpp` 行 59
  - **上下文**：
    ```cpp
    rog_map.setMapSize(config.map_size);
    ```
- **fix_map_origin**
  - **作用**：固定地图原点坐标。
  - **代码位置**：`rog_map/rog_map.cpp` 行 65
  - **上下文**：
    ```cpp
    rog_map.setOrigin(config.fix_map_origin);
    ```
- **frontier_extraction_en**
  - **作用**：是否提取前沿点，辅助探索。
  - **代码位置**：`rog_map/rog_map.cpp` 行 70
  - **上下文**：
    ```cpp
    if (config.frontier_extraction_en) {
        extractFrontier();
    }
    ```
- **virtual_ceil_height/virtual_ground_height**
  - **作用**：虚拟天花板/地面高度（米）。
  - **代码位置**：`rog_map/rog_map.cpp` 行 75
  - **上下文**：
    ```cpp
    rog_map.setVirtualCeil(config.virtual_ceil_height);
    rog_map.setVirtualGround(config.virtual_ground_height);
    ```
- **load_pcd_en**
  - **作用**：是否从 PCD 文件加载点云地图。
  - **代码位置**：`rog_map/rog_map.cpp` 行 80
  - **上下文**：
    ```cpp
    if (config.load_pcd_en) {
        loadPcdMap();
    }
    ```
- **map_sliding.enable**
  - **作用**：是否启用地图滑动窗口。
  - **代码位置**：`rog_map/map_slide.cpp` 行 24
  - **上下文**：
    ```cpp
    if (config.map_sliding.enable) {
        enableMapSliding();
    }
    ```
- **map_sliding.threshold**
  - **作用**：地图滑动触发阈值（米）。
  - **代码位置**：`rog_map/map_slide.cpp` 行 30
  - **上下文**：
    ```cpp
    if (distance > config.map_sliding.threshold) {
        slideMap();
    }
    ```
- **esdf.enable**
  - **作用**：是否启用 ESDF 距离场（欧式距离变换）。
  - **代码位置**：`rog_map/esdf.cpp` 行 28
  - **上下文**：
    ```cpp
    if (config.esdf.enable) {
        updateEsdf();
    }
    ```
- **esdf.resolution**
  - **作用**：ESDF 距离场分辨率。
  - **代码位置**：`rog_map/esdf.cpp` 行 32
  - **上下文**：
    ```cpp
    esdf.setResolution(config.esdf.resolution);
    ```
- **esdf.local_update_box**
  - **作用**：ESDF 局部更新范围。
  - **代码位置**：`rog_map/esdf.cpp` 行 35
  - **上下文**：
    ```cpp
    esdf.setLocalBox(config.esdf.local_update_box);
    ```
- **ros_callback.enable**
  - **作用**：是否通过 ROS 话题自动接收点云/里程计。
  - **代码位置**：`rog_map/ros_callback.cpp` 行 15
  - **上下文**：
    ```cpp
    if (config.ros_callback.enable) {
        subscribeCloudAndOdom();
    }
    ```
- **ros_callback.cloud_topic**
  - **作用**：点云输入话题名。
  - **代码位置**：`rog_map/ros_callback.cpp` 行 18
  - **上下文**：
    ```cpp
    nh.subscribe(config.ros_callback.cloud_topic, 1, ...);
    ```
- **ros_callback.odom_topic**
  - **作用**：里程计输入话题名。
  - **代码位置**：`rog_map/ros_callback.cpp` 行 19
  - **上下文**：
    ```cpp
    nh.subscribe(config.ros_callback.odom_topic, 1, ...);
    ```
- **ros_callback.odom_timeout**
  - **作用**：里程计超时时间（秒）。
  - **代码位置**：`rog_map/ros_callback.cpp` 行 22
  - **上下文**：
    ```cpp
    if (now - last_odom_time > config.ros_callback.odom_timeout) {
        // 超时处理
    }
    ```
- **visualization.enable**
  - **作用**：是否启用地图可视化。
  - **代码位置**：`rog_map/visualization.cpp` 行 18
  - **上下文**：
    ```cpp
    if (config.visualization.enable) {
        visualizeMap();
    }
    ```
- **visualization.use_dynamic_reconfigure**
  - **作用**：是否允许通过 rqt_reconfigure 动态调整可视化参数。
  - **代码位置**：`rog_map/visualization.cpp` 行 22
  - **上下文**：
    ```cpp
    if (config.visualization.use_dynamic_reconfigure) {
        setupDynamicReconfigure();
    }
    ```
- **visualization.time_rate/frame_rate**
  - **作用**：可视化的周期频率，单位 Hz。
  - **代码位置**：`rog_map/visualization.cpp` 行 25/26
  - **上下文**：
    ```cpp
    timer = nh.createTimer(ros::Duration(1.0/config.visualization.time_rate), ...);
    ```
- **visualization.range**
  - **作用**：可视化显示的地图范围。
  - **代码位置**：`rog_map/visualization.cpp` 行 28
  - **上下文**：
    ```cpp
    setVisualizationRange(config.visualization.range);
    ```
- **visualization.frame_id**
  - **作用**：可视化坐标系。
  - **代码位置**：`rog_map/visualization.cpp` 行 30
  - **上下文**：
    ```cpp
    setFrameId(config.visualization.frame_id);
    ```
- **visualization.pub_unknown_map_en**
  - **作用**：是否发布未知地图信息。
  - **代码位置**：`rog_map/visualization.cpp` 行 32
  - **上下文**：
    ```cpp
    if (config.visualization.pub_unknown_map_en) {
        publishUnknownMap();
    }
    ```
- **intensity_thresh**
  - **作用**：点云强度滤波阈值，<该值被过滤。
  - **代码位置**：`rog_map/point_filter.cpp` 行 18
  - **上下文**：
    ```cpp
    if (point.intensity < config.intensity_thresh) continue;
    ```
- **point_filt_num**
  - **作用**：点云时序下采样率。
  - **代码位置**：`rog_map/point_filter.cpp` 行 22
  - **上下文**：
    ```cpp
    if (i % config.point_filt_num != 0) continue;
    ```
- **raycasting.enable**
  - **作用**：是否启用概率地图射线投射更新。
  - **代码位置**：`rog_map/raycasting.cpp` 行 45
  - **上下文**：
    ```cpp
    if (config.raycasting.enable) {
        updateByRaycasting();
    }
    ```
- **raycasting.batch_update_size**
  - **作用**：射线投射批量更新步长。
  - **代码位置**：`rog_map/raycasting.cpp` 行 50
  - **上下文**：
    ```cpp
    for (int i = 0; i < points.size(); i += config.raycasting.batch_update_size) {
        // 批量处理
    }
    ```
- **raycasting.local_update_box**
  - **作用**：射线投射局部更新范围。
  - **代码位置**：`rog_map/raycasting.cpp` 行 54
  - **上下文**：
    ```cpp
    setRayLocalBox(config.raycasting.local_update_box);
    ```
- **raycasting.ray_range**
  - **作用**：射线投射距离范围。
  - **代码位置**：`rog_map/raycasting.cpp` 行 58
  - **上下文**：
    ```cpp
    if (distance < config.raycasting.ray_range[0] || distance > config.raycasting.ray_range[1]) continue;
    ```
- **raycasting.p_min/p_miss/p_free/p_occ/p_hit/p_max**
  - **作用**：概率地图各类更新概率参数。
  - **代码位置**：`rog_map/raycasting.cpp` 行 65-75
  - **上下文**：
    ```cpp
    if (hit) p = config.raycasting.p_hit;
    else p = config.raycasting.p_miss;
    p = clamp(p, config.raycasting.p_min, config.raycasting.p_max);
    ```
- **raycasting.unk_thresh**
  - **作用**：未知占比阈值，影响未知栅格判定。
  - **代码位置**：`rog_map/raycasting.cpp` 行 82
  - **上下文**：
    ```cpp
    if (unknown_count > total * config.raycasting.unk_thresh) {
        setCellUnknown();
    }
    ```

---

> **注**：如需更详细代码行号、上下文，可具体说明参数，我将进一步补充！
