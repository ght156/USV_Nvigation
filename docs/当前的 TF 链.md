• 看完了。你当前实船定位是“飞控里程 + 脚本对缝”的模式：ROS 侧没有跑 EKF，也没有
  AMCL/SLAM，整个定位就是三段 TF + 一个 GNSS 对齐节点。

  ## 当前的 TF 链

   TF 段                 谁发的                       输入
  ━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━
   map → odom            src/USV_NAV/                 /mavros/global_position/
                         workspace_ros/scripts/       global + /mavros/
                         gnss_odom_map_tf.py:1        local_position/odom
  ────────────────────  ───────────────────────────  ───────────────────────────
   odom → base_link      MAVROS（PX4 侧，不在本仓     PX4 内部融合结果
                         库）
  ────────────────────  ───────────────────────────  ───────────────────────────
   base_link → 传感器    robot_state_publisher        m_common/urdf/
                                                      usv_cf.xacro

  也就是说，PX4 飞控已经替你做了一层融合（IMU + GNSS 等），通过 /mavros/
  local_position/odom 给出局部里程计；你仓库里只负责把它“挂”到海图上。

  ## map → odom 的具体算法

  src/USV_NAV/workspace_ros/scripts/gnss_odom_map_tf.py:1 做的事很直接：

  1. 从地图 YAML 的 map_origin_ref_key（默认 ref_gnss_10）读出锚点经纬度；
  2. 把当前 GNSS 经纬度换算成相对锚点的 ENU 米制位移（east/north）；
  3. 减去 /mavros/local_position/odom 里的 x/y，得到 map → odom 的平移；
  4. 可选按 map_odom_yaw_deg 旋转，然后广播 TF。

  默认 initialize_once:=true：用首对有效的 GNSS+odom 算一次偏移后锁死，之后只按
  republish_hz（默认 20Hz）刷新时间戳重发同一份 TF。只有把它设成 false 才会持续
  用新的 GNSS 重算。

  所以它本质上是“一次标定 + 周期性重发”，不是滤波估计，RTK/GNSS 在启动之后的变化
  不会再去修正 map→odom。

  ## 下游消费

  src/USV_NAV/workspace_nav/config/nav2_params_real_mavros.yaml:22 里写得很明
  确：

  - bt_navigator、velocity_smoother、mission_bridge 全部用 /mavros/
    local_position/odom 作为里程计；

  - local_costmap 挂在 odom 帧，global_costmap 挂在 map 帧；
  - 传感器是 Livox 点云 /livox/lidar，通过 base_link 挂到 TF 树上。

  任务侧 src/USV_NAV/workspace_nav/workspace_nav/mission_bridge.py:169 用的是同
  一份地图 YAML 的 ref_gnss_10 做航点转换，GPS 状态订阅的是 /mavros/
  global_position/raw/fix——注意它和 gnss_odom_map_tf 用的 /mavros/
  global_position/global 不是同一个话题（一个是原始 fix，一个是 PX4 融合后的
  global 位姿）。

  ## 几个现状要点 / 坑

  1. 没有 ROS 侧状态估计：ekf.yaml、navsat.yaml 都不在实船启动链里，只是留档。
  2. map→odom 锁定后不再纠偏：initialize_once:=true 时 GNSS 漂移、HOME 变化都不
     会反映出来；改成 false 又会直接重算、噪声大。

  3. odom→base_link 完全依赖 MAVROS：如果嵌软没发这段 TF，或者发的是 *_ned 帧
     名，Nav2 的 TF 链就是断的。

  4. 地图文件默认值有点不一致：real_boat_bringup 和 nav2_real_mavros 默认都用
     map.yaml，但单独起 src/USV_NAV/workspace_ros/launch/
     real_boat_mavros_tf.launch.py:31 时默认却是 map_real_boat_hk.yaml，两个锚点
     不同，混用会整体偏移。

  5. 默认假设地图轴和 ENU 对齐：map_odom_yaw_deg:=0，如果海图本身带 yaw，需要手
     动配，没有自动处理 map.yaml 的 origin 旋转。

  总结一句话：你现在是“PX4 负责融合，ROS 负责对图”的分工，可靠性取决于 PX4 的
  local odom 和首帧 GNSS 对齐，缺少 ROS 侧对 map→odom 的持续滤波修正。