# 仿真 → 真 USV 迁移要点（ROS 2 Nav2 + MAVROS / PX4）

**只想查「改哪个文件、哪几个参数/话题」**：直接看 **[`实船配置修改清单.md`](./实船配置修改清单.md)**。

本文档在项目现有说明（[`实船调试.md`](./实船调试.md)、[`PROJECT_ARCHITECTURE_AND_NAV2.md`](./PROJECT_ARCHITECTURE_AND_NAV2.md)）之上，汇总 **控制链差异**、按优先级分层的改动清单，以及 **MAVROS（PX4 插件侧）常用话题语义**。不涉及具体飞控 PID 与地面站校准步骤；那些仍以 PX4/QGC、队里规范为准。

---

## 1. 当前仿真控制链 vs 目标实船链

**本仓库仿真期（推力直连 Gazebo）：**

```text
Nav2 → /cmd_vel_nav → converter.py
     → /roboboat/thrusters/left|right/thrust → ros_gz → 推进器插件
```

核心 `converter.py` 是 **差速双推抽象**（`linear` / `angular` 线性组合），与 **陆地差速** 类比；真船不再有这些 ROS topic。

**真 USV（MAVROS 桥接 PX4）常见目标形态：**

```text
Nav2 →（可选平滑）geometry_msgs/Twist 或自建轨迹/LOS 输出
     → 「船控 / 混控」节点
     → MAVROS（setpoint / RC override / actuator_control 等）
     → PX4 → 电调/舵机/推进器
```

短期联调可把 **Twist 直接送进 MAVROS 速度设定点**；正式航行往往还要 ** LOS / MPC / Mixer**（见下文「架构建议」）。本仓库已提供：`mavros_roboboat_relay` 可选用 **`enable_nav2_to_mavros_cmd_vel`**，以及 **`use_mavros_odometry`** 走 **`/mavros/local_position/odom`**，均属「减少重复造轮子」的过渡手段，不等于最终船控形态。

---

## 1.1 地图 `map` 与 MAVROS 局域原点（不跑 `navsat_transform`）

**问题**：原先 **`datum`/`map.yaml` + `navsat_transform`** 把「参考 GNSS ↔ 制图平面」绑在一起。若 **仅用 `/mavros/local_position/odom`** 且 **不启 `navsat_transform`**，谁承担对齐？

- **`local_position/odom`** 的原点通常是 **PX4 HOME / 局域 origin**，**不会自动**等于 **`map` 栅格 (0,0) 对应的真实角点**。  
- **`map`→`odom` 恒等** 隐含：**飞控局域坐标与 `map` 平面坐标在真实世界已对齐**，否则 RViz 里船在 **`map` 上会偏**。  
- **对策**：**QGC** 设 **HOME** 与地图 **`ref_gnss*`** 角点一致，或 ROS 2 调 **`/mavros/cmd/set_home`**（**`CommandHome`**，指定经纬须 **`current_gps: false`**，见 **`实船调试.md`**「用 ROS 2 服务设置 PX4 HOME」）；或 **非恒等 `map`→`odom`**（AMCL、标定、`navsat_transform` 仅偏置等），或 **回到 EKF+navsat 模式 A**。步骤表见 **`实船调试.md`**「`map` / `odom` / PX4 HOME」专节。**TF 各段谁来发、`static_transform*.yaml` 对照**见同文档 **「TF 坐标系总览与复盘调试」**与本节 **§1.2**。

---

## 1.2 TF 树与仓库约定（复盘）

Nav2 需要 **`global_frame`**（通常为 **`map`**）到 **`robot_base_frame`（本仓库即 `base_link`）**，以及 **`odom`** 链路完整。

- **模式 B（`/mavros/local_position/odom`）**：**`map`→`odom`** 由 **`real_boat_mavros_tf.launch.py`**（默认 **`gnss_odom_map_tf`**，从 **`map_config_yaml` / `map_origin_ref_key`** 读锚点；关 **`use_gnss_map_odom_tf`** 时静态 **`tf2`**；旧入口名 **`real_boat_tf_static.launch.py`**）；**`odom`→`base_link`** 由 **MAVROS** + **`mavros_px4_overrides_usv.yaml`**；**`base_link`→激光/IMU 等** 由 **`static_transform_real_boat.yaml`**（不要用仿真用的 **`static_transform.yaml`** 拓扑接到 **`base_link` 链**）。  
- **仿真 / 模式 A**：**`map`→`odom`** 仍为恒等 **`localization`**；**`odom`→`base_link`** 为 **`ekf_node`**；传感器由 **URDF** + **`static_transform.yaml`**。  

表格、命令 **`view_frames` / `tf2_echo`**、常见问题：**[`实船调试.md`](./实船调试.md)**「TF 坐标系总览与复盘调试」。架构缩表：**[`PROJECT_ARCHITECTURE_AND_NAV2.md`](./PROJECT_ARCHITECTURE_AND_NAV2.md)** §4.0。

---

## 2. 改动清单分层

### 2.1 必须改（否则真船对不上或无控制）

| 项 | 说明 |
|----|------|
| **执行层** | 真船无 **`/roboboat/thrusters/...`**。需 **停用或替换 `converter.py`**：常见通路为 MAVROS **`/mavros/setpoint_velocity/cmd_vel(_unstamped)`**（Offboard）、**`/mavros/rc/override`**、**`/mavros/actuator_control`** 等，与队里 PX4 **机架/混控**一致。**本仓库实船默认**：**`nav2_cmd_vel_to_mavros`**（**`/cmd_vel`→`/mavros/setpoint_velocity/cmd_vel_unstamped`**）。仿真与真机 **不可混用同一套增益**。 |
| **里程计话题** | Nav2 的 **`bt_navigator` / `velocity_smoother`** 的 **`odom_topic`** 必须指向真实可用源：本仓库可走 **`robot_localization`→`/odometry/filtered`** 或 **`/mavros/local_position/odom`**（见 **`nav2_mavros_odom_overlay.yaml`**）。**严禁**在未弄清 TF 的情况下 **EKF + MAVROS local_pose 同时向同一子帧发 `odom`→车体**。 |
| **激光 / 避障** | **仿真**：**`nav2_params.yaml`** → **`ObstacleLayer`** + **`LaserScan`**，约定 **`/roboboat/sensors/lidar/scan`**。**实船（模式 B）**：**`nav2_params_real_mavros.yaml`** → **`VoxelLayer`** + **`PointCloud2`**，默认 **`/livox/lidar`**；须核对 **点云 `frame_id`→`base_link` TF**、**时间戳**、**`use_sim_time=false`**，并按现场标定 **高度/体素/距离** 参数。若坚持用二维激光，需改回 **`ObstacleLayer`** 配置或 **`pointcloud_to_laserscan`** + 原激光话题链（见 **`实船调试.md`**）。 |
| **TF 树** | **`map`→`odom`→`base_link`→传感器**须连贯：**模式 B** 用 **`mavros_px4_overrides_usv.yaml`** + **`static_transform_real_boat.yaml`**；**仿真 / 模式 A** 用 **EKF** + **`static_transform.yaml`** + **URDF `robot_state_publisher`**。**`map`→`odom` 恒等**与海图/Home 对齐见 [`实船调试.md`](./实船调试.md)。复盘：**[`实船调试.md`](./实船调试.md)**「TF 坐标系总览」、 **`PROJECT_ARCHITECTURE_AND_NAV2.md`** §4.0。 |
| **`use_sim_time`** | Gazebo **`/clock` 关停**后，整条栈（Nav2、`robot_localization`、桥接节点）必须为 **系统时间**。 |

### 2.2 建议改（稳定性与可调性明显提升）

| 项 | 说明 |
|----|------|
| **Nav2 运动学参数** | USV **转弯半径大、横漂、不能原地回旋**。`RegulatedPurePursuitController`：`desired_linear_vel`、`min/max_lookahead_dist`、**角速度/加速度限制**、**`xy_goal_tolerance`/`yaw_goal_tolerance`** 需按船速与场地重标定。 |
| **代价地图** | **`inflation_radius`**、**`local_costmap` 窗口大小** 在开阔水面常需 **更大**（船需更大安全余量）；同时注意 **浪涌/反光** 导致 **假障碍**（见 2.3）。 |
| **定位融合** | 若长期不用飞控侧融合、而希望 **统一在 ROS 里融合 GPS/IMU/可选速度**，继续用 **`robot_localization`（`ekf` + `navsat_transform`）** 输出 **`/odometry/filtered`**，并仔细调 **协方差与 `datum`**。 |
| **参数分叉** | 维护 **`nav2_params_real.yaml`**（或 launch 多文件覆盖），与仿真参数分离，避免每次手改大文件。 |

### 2.3 最好重构（从「能跑」到「像船」）

| 项 | 说明 |
|----|------|
| **局部控制范式** | Nav2 默认 **差速/全向地面** 假设较强；USV 上 **Regulated Pure Pursuit** 易出现 **蛇形、过冲、横摆**。工程上常见：**保留 Nav2 全局规划 + costmap**，**局部跟踪改为 LOS / Stanley / MPC**，经 **mixer** 再下 MAVROS。 |
| **全局仍可用** | **`SmacPlanner2D`**（栅格 A* + 平滑）与 **`waypoint_transform` / 地图–GNSS 对齐** 往往可长期保留。 |
| **水面感知** | 对 **LiDAR 水杂波**、**动态障碍** 做 **滤波或层策略**（例如时间滤波、强度/高度过滤）；实船 **VoxelLayer** 仍会把 3D 数据投影到 2D costmap，**`min/max_obstacle_height` 等设不好**时同样会 **满屏障碍或剧烈抖动**。 |

---

## 3. 与「PX4 Rover 船模」相关的现实约束

PX4 对 **水面船架** 的支持与陆地 **Rover** 不同队可能差异很大；**Offboard velocity** 能否直接满足航行品质，需在台架与静水验证。许多队伍采用：

- **ROS 侧 LOS / PID**（艏向 + 推力）  
- **MAVROS `actuator_control` 或 RC override** 映射到左右推/舵  

而不是长期依赖 **Nav2 RPP → 单一 `cmd_vel` → PX4** 的默认链路。

---

## 4. 常见踩坑速查

| 现象 | 可能原因 |
|------|----------|
| Nav2「抽搐」、路径反复重规划 | **GPS 跳变**、里程计与 **map** 时间/坐标不一致；检查 EKF/飞控 **定位健康** 与 **TF**。 |
| Costmap 整体旋转 | **罗盘/IMU 航向**与 **地图北向** 不一致；核对 **ENU/NED** 与 **datum**。 |
| **`map`/`odom`/`base_link` 断开，仅有 `*_ned`** | MAVROS TF 帧名/NED 与本仓库 **`map→odom` 静态链**不匹配；先起 **`mavros_px4_usv.launch.py`** 并启用 **`config/mavros_px4_overrides_usv.yaml`**（见 **`实船调试.md`**）；仍不对再查 MAVROS/PX4 版本与 **`实船调试.md`**「`*_ned` 与 ROS TF」。 |
| 有规划无动作 | 未起 **MAVROS 侧控制链**、模式非 **OFFBOARD**、或与 **`converter`** 两套指令冲突。 |
| 障碍层满屏 | **水面反射**、**雨雾**、**安装角** 导致误检；调 **range**、**滤波**、**传感器高度**。 |

---

## 5. MAVROS（PX4）常用话题说明

下列基于 **MAVROS 为 MAVLink↔ROS2 桥** 的常规定义；**具体消息类型**以本机为准：`ros2 topic info <topic>`。

**TF 帧与 Nav2 链衔接**：默认 **`ros2 launch mavros px4.launch`** 往往**不发布**或未对齐 **`odom`→`base_link`**；建议在 **`workspace_ros`** 使用 **`ros2 launch workspace_ros mavros_px4_usv.launch.py`**（安装后参数加载顺序见 **[`实船调试.md`](./实船调试.md)**），仅维护 **`config/mavros_px4_overrides_usv.yaml`** 小覆盖层。

### 5.1 链路状态与时间

| 话题（示例前缀 `/mavros/`） | 含义 |
|-----------------------------|------|
| **`state`** | 连接状态、`armed`、`guided`、`mode`（字符串）等；Offboard 前必看。 |
| **`sys_status`** | 飞行器/船「系统状态」摘要（传感器 present、battery 过低等 flags）。 |
| **`extended_state`** | 扩展状态（ landed、in_air 等；船可能部分字段不适用）。 |
| **`timesync_status`** | 机载时间与地面时间同步质量。 |
| **`time_reference`** | 时间参考（若使用外部授时）。 |

### 5.2 IMU / 磁力计 / 气压

| 话题 | 含义 |
|------|------|
| **`imu/data`** | **滤波后主 IMU**：`orientation`、`angular_velocity`、`linear_acceleration`（典型给 EKF/`robot_localization`）。base_link |
| **`imu/data_raw`** | 更接近传感器原始采样（滤波较少）。 |
| **`imu/mag`** | 三轴磁力（用于航向/校准；水面金属干扰需注意）。 |
| **`imu/static_pressure` / `diff_pressure` / `temperature_*`** | 静压/差压/温度（定高或空速类；USV 上可能空置）。 |

### 5.3 全球坐标 / GNSS

| 话题 | 含义 |
|------|------|
| **`global_position/raw/fix`** | **原始 GNSS Fix**（`sensor_msgs/NavSatFix` 类）；适合作为 `robot_localization` 输入或与地图 datum 对齐。 |
| **`global_position/global`** | 经 MAVROS/PX4 解释后的全局位姿相关信息（常与高度、经纬度链路相关，类型以 `ros2 interface show` 为准）。map-baselink |
| **`global_position/local`** | **以 local origin 为参考的局地位置**（常配合 `~/odom`/地图使用）。 |
| **`global_position/gp_origin`** / **`set_gp_origin`** | **GNSS 本地原点** 状态 / 设定服务侧话题。 |
| **`global_position/rel_alt`** | 相对高度（对船参考有限）。 |
| **`global_position/compass_hdg`** | 罗经航向一类输出（可作为航向监视）。 |
| **`gpsstatus/gps*/raw`、`gps_rtk/*`** | 多天线/RTK 相关状态（若硬件支持）。 |

### 5.4 局域位姿 / 速度 / 「里程计」

| 话题 | 含义 |
|------|------|
| **`local_position/pose`** | 机体在 **_LOCAL 帧**下的位姿（常 `PoseStamped`）。 |
| **`local_position/pose_cov`** | 带协方差的位姿。 |
| **`local_position/velocity_local`** | **本地系**下线速度（ENU 惯例随配置）。 |
| **`local_position/velocity_body`** | **机体系**速度（常用于控制前馈）。 |
| **`local_position/velocity_*_cov`** | 带协方差的速度。 |
| **`local_position/accel`** | 线加速度估计。 |
| **`local_position/odom`** | **`nav_msgs/Odometry`**：飞控滤波器融合的 **局域位置+速度**，是 **Nav2 `odom_topic` 的常见输入**之一（本项目 overlay 即用此）。map-base_link |

### 5.5 Offboard / 设定值（把 ROS 指令送给 PX4）

| 话题 | 含义 |
|------|------|
| **`setpoint_velocity/cmd_vel`** | **`TwistStamped`**：速度设定（时间戳对齐重要）。 |
| **`setpoint_velocity/cmd_vel_unstamped`** | **`Twist`**：无 header；本项目 **`nav2_cmd_vel_to_mavros`** 默认写入此话题。联调：`ros2 topic echo /mavros/setpoint_velocity/cmd_vel_unstamped`。 |
| **`setpoint_position/local`**、`**setpoint_position/global**` | 位置/姿态类设定点。 |
| **`setpoint_raw/local`** / **`target_local`** 等 | **原始 MAVLink 目标**封装，灵活但更易踩帧与单位坑。 |
| **`setpoint_attitude/cmd_vel` / `thrust`** | 姿态/thrust 通道（旋翼常用；船按混控映射）。 |
| **`setpoint_accel/accel`** | 加速度设定。 |
| **`setpoint_trajectory/desired`、`local`** | 轨迹式接口（依版本与插件）。 |

### 5.6 舵机 / 执行器 / RC

| 话题 | 含义 |
|------|------|
| **`rc/in`、`rc/out`** | 遥控输入链路监视 / **PWM 输出监视**（MAVROS 订阅 MAVLink 后发布）；联调 OFFBOARD 时可与 **`/mavros/setpoint_velocity/cmd_vel_unstamped`** 对照（`ros2 topic echo`）。 |
| **`rc/override`** | **软件写入 RC 通道**（需满足飞控安全检查；映射到推力/舵向依参数）。 |
| **`actuator_control`** | **归一化执行器分组**指令（mixer 前端；适合自写推力分配）。 |
| **`manual_control/send`** | **`ManualControl`** 消息：键盘/手柄经 MAVROS 模拟「手控」链路（本仓库 **`mavros_keyboard_teleop`** 使用 **`/send`**，不是 **`/control`**）。 |
| **`manual_control/control`** | MAVROS→ROS 的人工控制 **状态监视**，不可当作等价「输入口」反复往飞控注入指令。 |

### 5.7 高层任务与 Home

| 话题 | 含义 |
|------|------|
| **`mission/waypoints`、`mission/reached`** | 航线与航点序号（Mission 模式相关）。 |
| **`home_position/home`、`set`** | Home 位姿与设定。 |

### 5.8 调试与杂项

| 话题 | 含义 |
|------|------|
| **`debug_value/*`** | 自定义调试 float/vector 等（联调很实用）。 |
| **`statustext/recv`、`send`** | 飞控 **文本状态**（错误/警告）；排障必看。 |
| **`battery`** | 电源状态。 |
| **`estimator_status`** | 估计器健康（哪些源 active、reject 等）。 |
| **`odometry/in`、`odometry/out`** | 外部里程计 **注入** / **回传**（做融合或对比时用）。 |

**说明**：`/mavros/hil/*`、`sim_state/*` 等多为 **仿真/HIL**；真船一般只作调试对照。表中未列出的条目可用：

```bash
ros2 topic list | grep mavros
ros2 topic echo <topic> --once
ros2 interface show <消息类型全名>
```

---

## 6. 文档关系

| 文档 | 内容侧重 |
|------|----------|
| [`实船配置修改清单.md`](./实船配置修改清单.md) | **按文件列出** 要改的话题、YAML 键、launch 参数（速查表）。 |
| [`船体与导航参数索引.md`](./船体与导航参数索引.md) | **船体几何、Gazebo 流体/推进、Nav2 footprint、地图文件（`boat_parameters_index.yaml` 的 `nav2_maps`）、converter** 等。 |
| [`实船调试.md`](./实船调试.md) | MAVROS 启动、模式 A/B、**TF 树复盘**、`map`/`odom`/HOME（含 **`/mavros/cmd/set_home`/`CommandHome`**）、`nav2_real_mavros`、命令示例。 |
| [`PROJECT_ARCHITECTURE_AND_NAV2.md`](./PROJECT_ARCHITECTURE_AND_NAV2.md) | 仿真包分工、EKF/navsat、`nav2_params`、**§7 地图（仿真 / 实船 HK / 备份）**、航点与行为树–速度链。 |
| **本文** | **迁移优先级**、**架构取舍**、**MAVROS 话题语义表**。 |

---

*文档随工程实践增补；PX4 / MAVROS 大版本变更时请以官方文档与本机 `ros2 topic info` 为准。*
