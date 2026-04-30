# YILDIZ-USV 项目架构与 Nav2 说明

本文档面向复盘与二次开发，梳理 RoboBoat 仿真栈的**包结构、数据流、定位、执行器**以及 **Navigation2 的规划–避障–控制**链路。对应代码路径以工作空间根目录 `yildiz_ws/` 为参照。

**补充（工作空间 `docs/`）**：栅格图与 GNSS / 仿真锚点 / `map` 北向与 `map.yaml` 的对齐结论，见仓库根目录 [`docs/地图与GNSS-Nav2对齐说明.md`](../../../docs/地图与GNSS-Nav2对齐说明.md)。**进度台账**见 [`docs/工作进度汇报.md`](../../../docs/工作进度汇报.md)。

---

## 1. 仓库与 ROS 2 包概览

| 包名 | 角色 |
|------|------|
| `workspace_gz` | Gazebo Garden 仿真：`world`、`roboboat` 模型、`xacro`、`ros_gz` 桥（时钟、传感器、推进器指令） |
| `workspace_ros` | 传感器协方差重发布、`robot_localization`（EKF + `navsat_transform`）、静态 TF、`converter`（速度→推力）、视觉/任务节点（如 `target_buoy`、`kamikaze`） |
| `workspace_nav` | Nav2 参数与地图资源、`nav2.launch.py` 引入上游 `nav2_bringup`，航点 JSON 管线（`workspace_nav/workspace_nav/waypoint_transform.py`、`waypoint_with_state.py` 等） |

三个包在逻辑上形成：**仿真（真值与传感器）→ 定位（odom/map）→ 导航（Nav2）→ 执行（推力）**。

---

## 2. 推荐启动顺序与整体数据流

官方 README 中的典型顺序（仿真场景下 `use_sim_time:=true`）：

1. `ros2 launch workspace_gz simulation.launch.py` — Gazebo + 机器人描述 + `ros_gz_bridge`
2. `ros2 launch workspace_ros localization.launch.py` — IMU/GPS 协方差、EKF、`navsat_transform`、**map→odom 恒等静态 TF**
3. `ros2 launch workspace_nav nav2.launch.py` — Nav2 全栈
4. `ros2 run workspace_ros converter` — **将 Nav2 的速度指令转为左右推力**（见第 5 节）
5. 可选：任务/航点相关节点（`target_buoy`、`waypoint_transform`、`waypoint_with_state` 等）

**数据流（纯文本图，兼容各类 Markdown 阅读器；不依赖 Mermaid）**：

```
  ┌─────────────────────┐
  │  Gazebo + Bridge    │
  └──────────┬──────────┘
             │ 传感器话题 (GPS/IMU/LiDAR/…)
             ▼
  ┌─────────────────────┐
  │  EKF / navsat       │
  └──────────┬──────────┘
             │ /odometry/filtered  +  TF (odom→base_link 等)
             ▼
  ┌─────────────────────┐
  │  Nav2               │
  └──────────┬──────────┘
             │ 速度链: /cmd_vel_nav → (velocity_smoother) → /cmd_vel
             ▼
  ┌─────────────────────┐
  │  converter          │
  └──────────┬──────────┘
             │ /roboboat/thrusters/left|right/thrust
             ▼
  ┌─────────────────────┐
  │  Gazebo 推进器插件   │  ──反馈──▶  (闭环回到仿真位姿/传感器)
  └─────────────────────┘
```

**关键点**：Nav2 `bt_navigator` 使用 **`/odometry/filtered`** 作为里程计话题（见 `nav2_params.yaml`）；仿真里该话题由 **`ekf_node`** 输出。全局 **`map`** 与 **`odom`** 在当前启动里通过 **`static_transform_publisher` 对齐（零位姿）** 连接；若你以后改为 **`amcl`/SLAM 动态 map↔odom** 或实船「重定位」校正 **`map`→`odom`**，需替换该静态 TF 策略。

### 2.1 RViz2「Nav2 Goal」与地面站任务：是否都会出速度指令？

**会。** 两种入口只是**谁来提交导航目标**不同，进入 Nav2 之后**共用同一套规划–控制–速度输出链路**：

| 入口 | 典型 ROS 接口 | 说明 |
|------|----------------|------|
| **RViz2** | `NavigateToPose`（`nav2` 的 action）或等价插件 | 单点目标 → `bt_navigator` 行为树 → `planner_server` + `controller_server` |
| **地面站 + 航点管线** | GCS 发 **`/waypoint`** → `waypoint_transform` 写 JSON → `waypoint_with_state` → **`FollowWaypoints`** | 多点序列同样由 `bt_navigator` / `waypoint_follower` 协调，底层仍是规划器 + `controller_server` |

只要行为树处于**跟线（FollowPath）**等会驱动底座的阶段，`controller_server` 就会按控制频率生成 **`geometry_msgs/Twist`**。在 **`nav2_bringup`** 的默认重映射下（与本栈一致）：

1. **控制器**对外发布到话题 **`/cmd_vel_nav`**（节点内部话题名 `cmd_vel` 被 remap 到该全局名）。  
2. **`velocity_smoother`** 订阅 **`/cmd_vel_nav`**，平滑后发布 **`/cmd_vel`**（供需要「官方平滑后速度」的节点使用）。  
3. 本项目的 **`converter`** 订阅 **`/cmd_vel_nav`**，把角/线速度换算为左右 **`/roboboat/thrusters/.../thrust`**。  

因此：**用 RViz2 设单点目标时，同样会有 `/cmd_vel_nav`（以及通常在跑的 `/cmd_vel`）**；地面站不替代这一层，它主要管**任务/遥测/可视化**（见 §11.5）。

---

## 3. 仿真层（`workspace_gz`）

- **`simulation.launch.py`**：设置 `GZ_SIM_*` / `IGN_GAZEBO_*` 资源路径，启动 `gz sim <world.sdf>`，`robot_state_publisher` + `joint_state_publisher`，`ros_gz_sim create` 生成 `roboboat`。
- **`parameter_bridge`**：将仿真侧话题桥到 ROS 2，主要包括：
  - `/clock`
  - 左右推力：`.../cmd_thrust` ↔ `/roboboat/thrusters/left/thrust`、`/roboboat/thrusters/right/thrust`
  - GPS、`Imu`、`LaserScan`、点云、`Camera`、`CameraInfo`

**改模型/传感器**：优先改 `workspace_gz/description/`、`workspace_gz/models/`，并与 **Nav2 `nav2_params.yaml` 中 `scan` 的话题名**、`workspace_ros/config/static_transform.yaml` 中的链路一致。

---

## 4. 定位层（`workspace_ros`）

### 4.1 `localization.launch.py` 内含节点

| 节点 | 作用 |
|------|------|
| `imu_covariance_repub` / `gps_covariance_repub` | 固定协方差的话题重发布（供下游滤波使用） |
| `navsat_transform_node`（`robot_localization`） | GNSS ↔ `utm`/`map` 相关变换，输出 `/odometry/gps` 等（具体以 `navsat.yaml` 为准） |
| `ekf_node` | 融合 IMU 与 GPS 里程计，输出 **`/odometry/filtered`** 并发布 **odom→base_link** TF |
| `static_transform_publisher`（自定义） | 按 `static_transform.yaml` 发布 **传感器仿真 frame** 到 `*_link` 的静态关系 |
| **另一个** `static_transform_publisher` | **`map` → `odom`** 零变换（固定重叠） |

### 4.2 `config/ekf.yaml`（摘要）

- **二维模式** `two_d_mode: true`
- **世界系** `world_frame: odom`（EKF 在 odom 下积分）
- **IMU** `/imu/fixed_cov`：姿态角速度、线加速度等按 `imu0_config` 接入
- **GPS 里程计** `/odometry/gps`：位置与偏航等按 `odom0_config` 接入

若导航中“车在图上不动/乱飘”，首先查 **`/odometry/filtered`** 与 **TF 树**（`map`→`odom`→`base_link`）。

---

## 5. 执行层：从 Nav2 到螺旋桨（`converter`）

- **节点**：`workspace_ros/scripts/converter.py`，入口 `ros2 run workspace_ros converter`。
- **订阅**：`geometry_msgs/Twist`，话题 **`/cmd_vel_nav`**。
- **发布**：`/roboboat/thrusters/left/thrust`、`/roboboat/thrusters/right/thrust`（`std_msgs/Float64`），经 Gazebo 桥驱动推进器插件。

**与 Nav2 官方启动的衔接**（`nav2_bringup` 的 `navigation_launch.py`）：  
- `controller_server` 将内部 **`cmd_vel` 重映射为对外话题 `cmd_vel_nav`**；  
- `velocity_smoother` 订阅 **`cmd_vel_nav`**，将平滑后的速度发布到 **`cmd_vel`**。  

因此：**未运行 `converter` 时**，即使规划与局部控制在工作，**推力仍可能始终为 0**，仿真中表现为“有路径/无运动”或仅漂移。

**调参提示**：`linear_scale`、`angular_scale` 在 `converter.py` 内硬编码，影响跟线响应与转弯半径，与 Nav2 中 `desired_linear_vel` 等需联合调试。

---

## 6. Nav2 配置详解（`workspace_nav/config/nav2_params.yaml`）

以下说明均基于**当前仓库中的 YAML**；若升级 Nav2 大版本，行为树 XML 路径等可能略有差异。

### 6.1 全局与里程计

- **`bt_navigator`**：`global_frame: map`，`robot_base_frame: base_link`，`odom_topic: "/odometry/filtered"`。  
- 行为树节点插件列表在 `plugin_lib_names` 中显式声明（导航、规划、跟线、恢复行为等均依赖此列表）。

### 6.2 规划器（全局路径）

- **服务器**：`planner_server`
- **插件**：`GridBased` → **`nav2_smac_planner/SmacPlanner2D`**
- **要点**：
  - `allow_unknown: true`：允许在未知栅格上规划（与 `global_costmap` 的 `track_unknown_space` 配合）
  - `cost_travel_multiplier: 2.0`：较高代价区域的通行惩罚
  - 内置 **路径平滑**（`smoother` 段：`w_smooth` / `w_data` 等）

**改“绕路方式/全局路径形状”**：主要动 **Smac 与 global costmap 分辨率/膨胀**。

### 6.3 控制器（局部跟线与“动态避障”倾向）

- **服务器**：`controller_server`
- **插件**：`FollowPath` → **`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`**
- **主要参数含义**（节选）：
  - `desired_linear_vel: 1.2`：期望线速度上限量级
  - `lookahead_time` / `min_lookahead_dist` / `max_lookahead_dist`：前视距离，影响转弯与切弯
  - **`use_collision_detection: true`**：沿前视路径做碰撞时间/距离检测；`min_distance_to_obstacle`、`max_allowed_time_to_collision_up_to_carrot` 与 **局部代价** 共同约束速度
  - **`use_rotate_to_heading: false`**：当前未启用“先转头再直行”类行为，船用差速时可按需要再打开
  - `allow_reversing: false`：**不允许倒车**

**跟线不稳/撞障**：先调 **局部 costmap 尺寸与更新频率**、**footprint**，再调 **Regulated PP** 的 lookahead 与碰撞检测参数。

### 6.4 进度与到达判定

- **`SimpleProgressChecker`**：`required_movement_radius`、`movement_time_allowance` —— 用于检测是否“卡住”以触发恢复行为。
- **`SimpleGoalChecker`**：`xy_goal_tolerance`、`yaw_goal_tolerance` —— 到达目标点的容差。

### 6.5 代价地图与避障语义

**局部 costmap `local_costmap`**

- **坐标系**：`global_frame: odom`，**滚动窗口** `20×15 m`，分辨率 `0.05 m`
- **图层**：`obstacle_layer`（LaserScan）+ `inflation_layer`
- **传感器**：`/roboboat/sensors/lidar/scan`
- **机器人足迹**：`footprint` 多边形（与船体尺寸相关）

**全局 costmap `global_costmap`**

- **坐标系**：`global_frame: map`，**`rolling_window: false`**（覆盖静态地图全幅），分辨率 **`0.5 m`**（与 `map.yaml` 中 PGM 分辨率可不同，由 static 层融合）
- **图层**：`static_layer`（来自 `map_server` 的地图）+ `obstacle_layer`（同一激光）+ `inflation_layer`
- **`track_unknown_space: true`**：区分未知/自由/障碍，与规划器 `allow_unknown` 搭配

**“避障”在 Nav2 中的分工**：

1. **全局**：Smac 在 **global costmap** 上搜索低代价路径（静态图障碍 + 激光标记 + 膨胀带）。  
2. **局部**：Regulated Pure Pursuit 根据 **local costmap** 与 **碰撞检测** 调节速度/路径跟踪，避免贴障过快；**Behavior 行为**（旋转、后退等）处理卡死。

**改激光话题或换传感器**：必须同时更新 **两处 `obstacle_layer` 的 `scan.topic`**，并确认数据在 `map`/`odom` 下时间同步与 TF 正确。

### 6.6 行为恢复（`behavior_server`）

- 插件：`spin`、`backup`、`drive_on_heading`、`wait`、`assisted_teleop`
- `costmap_topic` 指向 **`local_costmap/costmap_raw`**，恢复行为在局部代价上执行

具体**何时触发**哪种恢复由 **行为树（BT）** 决定；本仓库未在 `nav2_params.yaml` 中覆盖 `default_nav_to_pose_bt_xml` 等路径时，使用安装包自带默认 BT（随 Nav2 版本变化）。需要精调“卡住–恢复”策略时，建议用 `ros2 param get` 查看运行中的 BT 路径，并在参数文件中显式指定自定义 XML。

### 6.7 速度平滑（`velocity_smoother`）

- `feedback: "CLOSED_LOOP"`，订阅里程计 **`odometry/filtered`**
- 限制 `max_velocity` / `min_velocity`、加减速度，输出经 `navigation_launch` 重映射后的 **`cmd_vel`**

### 6.8 Waypoint Follower（Nav2 自带服务节点）

- `waypoint_follower`：`wait_at_waypoint` 插件，航点间可暂停 **`waypoint_pause_duration`**（此处为 **50**，单位以插件定义为准，一般为秒或 tick，改前请对照 Nav2 文档）

---

## 7. 地图（`workspace_nav/config/map.yaml`）

- 栅格图：`image: ../map/map.pgm`；**`resolution`、`origin` 以仓库中当前 YAML 为准**（勿死记某一组数字）。  
- Nav2 **global costmap** 使用 **0.5 m** 分辨率，与 **PGM 元数据中的分辨率**可以不同（`static_layer` 会处理尺度差异）；若改 `origin`、分辨率或换图，需重新核对 **RViz 中地图与仿真是否对齐**（详见仓库根目录 [`docs/地图与GNSS-Nav2对齐说明.md`](../../../docs/地图与GNSS-Nav2对齐说明.md)）。

---

## 8. 航点任务管线（`workspace_nav`）

| 模块 | 作用 |
|------|------|
| **`workspace_nav/workspace_nav/waypoint_transform.py`** | 默认从 **`map.yaml`** 与 **`navsat_transform`** 共用 **地图角点 datum**（`map_datum_ref_key`，默认 `ref_gnss_10`）；将 **`/waypoint`** JSON 经纬度转为 **Nav2 `map` 系 `x,y`** 写入 **`waypoints.json`**：**局部平面**默认 **ENU**（`projection:=enu`，可选 `utm`），再叠加 **`map.yaml` 的 `origin`** 平移与旋转；输出含 **`map_frame_meta`**。参数 **`datum_source:=first_gps`** 恢复「首帧 GPS 作原点」且**不**套用 `origin` |
| **`workspace_nav/workspace_nav/waypoint_with_state.py`** | 监控 `waypoints.json`，加载后对 Nav2 的 **`FollowWaypoints` action**（`follow_waypoints`）**逐点发送**；可结合里程计跳过已接近点；全部完成后可触发后续任务（如 `kamikaze` 脚本） |

**开发注意**：`waypoint_with_state` 在首次成功加载航点后会进入“已加载”状态，**不会自动反复监视文件更新**；重复任务往往需 **重启节点** 或扩展逻辑。

---

## 9. 其他 `workspace_ros` 节点（简表）

| 入口 | 用途 |
|------|------|
| `target_buoy` | 视觉检测相关（与比赛科目有关） |
| `kamikaze` | 向 **`/cmd_vel_nav`** 发布速度（与 `converter` 同话题，注意与 Nav2 同时运行时的**指令互抢**） |
| `manual_control` | 键盘控制推力，用于手动测试 |

---

## 10. 修改代码时的快速索引

| 目标 | 建议优先查看 |
|------|----------------|
| 改全局路径/规划器 | `nav2_params.yaml` → `planner_server` / `GridBased` |
| 改跟线手感、碰撞检测 | `nav2_params.yaml` → `controller_server` / `FollowPath` |
| 改障碍物来源、膨胀、局部窗口 | `nav2_params.yaml` → `local_costmap` / `global_costmap` |
| 改到达精度 | `general_goal_checker` 容差、`converter` 比例 |
| 改仿真传感器名字 | `workspace_gz/launch/simulation.launch.py`、`nav2_params.yaml` obstacle_layer、`static_transform.yaml` |
| 改滤波与帧关系 | `ekf.yaml`、`navsat.yaml`、`localization.launch.py` |
| 多航点 JSON 与地面站接口 | `workspace_nav/workspace_nav/waypoint_transform.py`、`waypoint_with_state.py`、`json/waypoints.json`；地面站配合见 §11.5 |
| 改仿真经纬参考点（影响 GPS 话题与地面站显示） | `workspace_gz/worlds/world.sdf` → `<spherical_coordinates>` |
| 地面站航点格式与任务存储 | GCS 仓库 `backend/data/waypoints.json`、ROS 话题 **`/waypoint`** |

---

## 11. 仿真经纬原点、地面站与「白地图」任务（工程必知）

本节汇总与 **改 `world`、改地面站、改 PGM** 相关的依赖，方便做场景迁移（例如换到某片水域附近做联调）。

### 11.1 仿真 GPS 经纬在哪里定？改了会怎样？

- **文件**：`workspace_gz/worlds/world.sdf` 中 **`<spherical_coordinates>`**（`surface_model` 一般为 `EARTH_WGS84`，`world_frame_orientation` 为 `ENU`）。
- **字段**：`latitude_deg`、`longitude_deg` 定义 **仿真世界 ENU 原点对应地球表面哪一点**；`elevation`、`heading_deg` 同步影响语义。
- **效果**：Gazebo **`gz-sim-navsat-system`** 与船上 NavSat 传感器按该原点把 **机体在世界中的位姿 → 经纬度**。船在 **`simulation.launch.py` 中 spawn 在原点附近**（默认 `x=y=z=0`）时，**话题里报告的经纬度就会在「新原点」附近**。  
  因此：**改原点 → 仿真 `/roboboat/sensors/gps/navsat` 的读数会变**。

### 11.2 地面站界面上的「船位经纬」从哪来？改仿真原点会反映在 UI 上吗？

上游 **GROUND CONTROL STATION**（独立仓库，Electron + 后端 ROS 订阅）里，后端订阅 **`/gps/fixed_cov`** 等话题，再通过 HTTP 给前端 Cesium。**本栈中** **`gps_covariance_repub`** 将仿真 **`/roboboat/sensors/gps/navsat`** 重发布后得到 **`/gps/fixed_cov`**。

**结论**：在同一 ROS 域内跑通仿真与地面站后端时，**界面上的船舶经纬度即为当前 ROS 里的 GPS**，会随 **`world.sdf` 球形原点与船位** 更新；仅在 **尚未收到有效 GPS** 时，前端可能仍显示代码里写的 **占位默认坐标**，收到数据后会被覆盖。

**改仿真原点后请务必**：重置或 **按新地理范围重新保存** 地面站 `waypoints.json`（及任务文件中经纬航点）；否则任务点仍贴在旧经纬上，会与 `waypoint_transform` 使用的 datum/投影 **不一致**。默认 **`datum_source:=map_yaml`** 时 **不依赖首帧 `/gps/filtered`** 建原点；若仍使用 **`first_gps`**，改世界原点后应先让滤波/GPS **稳定**，再发航点。

### 11.3 纯白 `map.pgm`、LiDAR 与「GPS 航点导航」各管什么？

| 环节 | 作用 |
|------|------|
| **PGM + `map.yaml`** | **`map_server` 静态层**：几乎全白 ≈ **几乎无静态障碍**，全局路径主要不受「岸线」约束，除非你在图里画上障碍灰度。 |
| **全局/局部 `ObstacleLayer`** | **激光 `/roboboat/sensors/lidar/scan`** 在代价地图上画 **动态障碍**；**膨胀层**推开路径。仿真里若没有有效障碍物，表现会接近「空地绕路」。 |
| **地面站经纬航点 → `waypoint_transform`** | 把 **经纬度** 转为 **`map`/工程平面内的 x,y**，写入 **`workspace_nav/json/waypoints.json`**，再由 **`waypoint_with_state`** 调 Nav2 **`follow_waypoints`**。目标几何 **不是** 在 RViz 里点 PGM 得到，而是 **经纬任务链**。 |

若需要 **真实岸线/禁区** 参与全局规划，需 **更换或编辑 PGM** 并保证 **`map` 原点与 GPS datum/TF 策略一致**；**仅改 `world.sdf` 经纬** 不会自动把「某段真实河道」画进栅格。

### 11.4 规划与避障方法（与第 6 节一致，便于对外说明）

- **全局规划**：**Smac Planner 2D**（`nav2_smac_planner/SmacPlanner2D`），在 **global costmap** 上搜索。  
- **局部跟线**：**Regulated Pure Pursuit**（`nav2_regulated_pure_pursuit_controller`），含与障碍相关的限速/碰撞检测等参数。  
- **避障数据来源**：**代价地图 = 静态层（PGM）+ 激光障碍层 + 膨胀**；另由 **行为树** 触发 **Spin / Backup** 等恢复。

### 11.5 地面站（GROUND CONTROL STATION）与本导航栈的配合

独立仓库 **GROUND CONTROL STATION**（开发目录示例：`/home/ght/GROUND-CONTROL-STATION-dev`；若克隆到其它路径，以本机为准）与当前栈通过 **同一 ROS 2 域**联调。**不启动地面站也可以**用 RViz2 设目标完成导航；地面站提供 **任务编辑、持久化、遥测与 Cesium 可视化**，而不是替代 Nav2 算法。

**与本栈相关的数据流（摘要）**：

```
  ┌──────────────────────────────────────┐
  │  GCS Electron UI                     │
  │  航点编辑 → 保存 backend/data/        │
  │  waypoints.json                      │
  └──────────────┬───────────────────────┘
                 │ POST /api/run_mission（拉起 waypoint_publisher）
                 ▼
  ┌──────────────────────────────────────┐
  │  backend/ros_nodes/waypoint_publisher│
  │  定时发布 std_msgs/String：topic      │
  │  「waypoint」（相对名，默认即 /waypoint）│
  │  载荷为 JSON（含 latitude/longitude）  │
  └──────────────┬───────────────────────┘
                 │
                 ▼
  ┌──────────────────────────────────────┐
  │  waypoint_transform → waypoints.json  │
  │  map.yaml datum + origin → map 系 x,y │
  │  workspace_nav/json/waypoints.json    │
  └──────────────┬───────────────────────┘
                 ▼
  ┌──────────────────────────────────────┐
  │  waypoint_with_state → FollowWaypoints│
  └──────────────────────────────────────┘

  并行：GCS backend/server/ros_subscriber.py 订阅 ROS 遥测，供 HTTP API / 前端：
  · /imu/fixed_cov、/odometry/filtered、/gps/fixed_cov
  · /plan（Nav2 全局路径）
  · 线速度等与 Nav2 相关的量见 GCS README（后端实现以 /cmd_vel_nav 等为准）
```

**使用前注意**：

- 地面站与仿真/真机需在 **同一 `ROS_DOMAIN_ID`**，且 **先起定位与 Nav2**，再 **「保存航点」** 后通过 API **下发任务**（启动 `waypoint_publisher`）。  
- **`waypoint_transform`**：默认 **`datum_source:=map_yaml`**，与 **`/waypoint`**、**`map.yaml`** 的语义一致；**`first_gps`** 时首帧 GPS 作平面原点。改动 **`world.sdf` 球形原点**、**`map.yaml` 的 ref/origin** 或任务区后，应 **清空或重存** 地面站航点并核对 **`map_frame_meta`** / RViz。  
- **地面站罗盘（GCS）**：遥测 **航向显示** 使用 **ENU → 罗经角** 与优先 **`/odometry/filtered`** 的说明见 GCS 仓库 `README` / 提交记录；与航点 map 投影无关。
- HTTP/话题列表见地面站仓库 **`README.md`**；地图–GNSS–Nav2 对齐见 **`docs/地图与GNSS-Nav2对齐说明.md`**。

---

## 12. 衍生工作空间 `wuxihik_navigation`（可选）

若在主目录下复制本工作空间用于二次开发，可使用与 `yildiz_ws` **同级** 的目录 **`/home/ght/wuxihik_navigation`**（仅拷源码与工具，不含 `build`/`install`/`log` 时更干净）。进入后：

```bash
cd /home/ght/wuxihik_navigation
source /opt/ros/humble/setup.bash   # 按本机发行版调整
colcon build --symlink-install
source install/setup.bash
```

若 Nav2 等依赖安装在 **隔离工作空间**，需先 `source` 该工作空间的 `install/setup.bash` 再 `colcon build`。本文档在衍生树中路径仍为 **`src/YILDIZ-USV/docs/PROJECT_ARCHITECTURE_AND_NAV2.md`**（若复制时未改仓库目录名）。

---

## 13. 版本与复盘建议

- 记录每次改动时的 **ROS 2 发行版**、**Nav2 版本**及 `nav2_params.yaml` 的 git 提交说明。  
- 大改 BT 或恢复逻辑时，在 RViz 打开 **ParticleCloud/Trajectory**（若适用）并对照 **`ros2 topic echo /local_costmap`**、**`/plan`** 做联合验证。

本文档随仓库演进可继续补充：**自定义行为树 XML 路径**、**实船与仿真的参数分叉**、`bringup_launch` 中与命名空间相关的重映射表等。
