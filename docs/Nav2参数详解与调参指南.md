# Nav2 参数详解与调参指南（USV 实船）

本文逐一解释 **`workspace_nav/config/nav2_params_real_mavros.yaml`** 中每个参数的含义、原理、对 USV 运动的影响，以及遇到问题时的调参方向。

**适用场景**：MAVROS/PX4 实船、Livox 避障、差动推进，`RegulatedPurePursuitController` + `SmacPlanner2D`。里程计 **`/mavros/local_position/odom`**；速度链 **`/cmd_vel_nav` → `nav2_cmd_vel_to_mavros` → PX4**。

> **与仿真对照**：RPP / lookahead 几何原理与仿真仓 `wuxihik_navigation/docs/Nav2参数详解与调参指南.md` 一致；仿真参数文件为 `nav2_params.yaml`（EKF `/odometry/filtered`）。改参时**只改本仓实船 yaml**，勿改仿真仓。

**实船启动**：`ros2 launch workspace_nav nav2_real_mavros.launch.py`（见 [`项目运行与联调.md`](./项目运行与联调.md)）。

---

## 目录

1. [bt_navigator — 行为树导航器](#1-bt_navigator--行为树导航器)
2. [controller_server — 路径跟踪控制器](#2-controller_server--路径跟踪控制器)
   - [2.8 USV 调 RPP 经验顺序](#28-usv-调-rpp-经验顺序)
   - [2.9 lookahead 与环境尺度：多尺度路径跟踪](#29-lookahead-与环境尺度多尺度路径跟踪)
3. [planner_server — 全局路径规划器](#3-planner_server--全局路径规划器)
4. [local_costmap — 局部代价地图](#4-local_costmap--局部代价地图)
5. [global_costmap — 全局代价地图](#5-global_costmap--全局代价地图)
6. [behavior_server — 恢复行为](#6-behavior_server--恢复行为)
7. [velocity_smoother — 速度平滑器](#7-velocity_smoother--速度平滑器)
8. [waypoint_follower — 航点跟随](#8-waypoint_follower--航点跟随)
9. [常见问题速查表](#9-常见问题速查表)

---

## 1. bt_navigator — 行为树导航器

行为树是 Nav2 的"大脑"，决定什么时候规划、什么时候控制、什么时候恢复。

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map              # 全局坐标系
    robot_base_frame: base_link    # 机器人本体坐标系
    odom_topic: "/mavros/local_position/odom"   # 实船；仿真为 /odometry/filtered
    bt_loop_duration: 10           # 行为树 tick 周期 (ms)
    default_server_timeout: 20     # 动作服务器超时 (ms)
    wait_for_service_timeout: 1000 # 等待服务启动超时 (ms)
```

| 参数 | 含义 | 原理 | 调参 |
|------|------|------|------|
| `global_frame` | 全局坐标系名称 | 所有全局规划、目标点都在这个坐标系下表达。必须与 costmap 的 `global_frame` 一致 | 固定 `map`，不要改 |
| `robot_base_frame` | 机器人本体系 | TF 树中表示机器人位置的坐标系 | 固定 `base_link`，与 URDF 一致 |
| `odom_topic` | 里程计话题 | 行为树用这个判断机器人是否在移动、是否到达目标 | 实船为 **`/mavros/local_position/odom`**（与 `nav2_params_real_mavros.yaml`、`mission_bridge`、`velocity_smoother` 一致） |
| `bt_loop_duration` | 行为树循环周期 (ms) | 每 10ms 检查一次行为树状态，决定下一步动作 | 默认 10ms，一般不需要改。太小增加 CPU，太大响应慢 |
| `default_server_timeout` | 动作服务器超时 (ms) | 向 planner/controller 发请求后，等多久算超时 | 网络差或大地图时可适当加大到 50-100 |
| `wait_for_service_timeout` | 启动等待超时 (ms) | 等待 planner/controller/recovery 等服务就绪 | 默认 1000ms 足够 |

**行为树插件库** (`plugin_lib_names`)：列出了所有可用的 BT 节点。Nav2 默认行为树是 `navigate_to_pose_w_replanning_and_recovery.xml`，流程为：

```
NavigateToPose
  ├─ ComputePathToPose (全局规划)
  ├─ FollowPath (路径跟踪，循环)
  │   ├─ 检测 stuck → recovery
  │   └─ 检测 progress → 继续
  └─ 到达目标
```

**常见问题**：一般不需要调这部分。如果 Nav2 启动后长时间无响应，检查 `odom_topic` 是否有数据。

---

## 2. controller_server — 路径跟踪控制器

这是**对 USV 行为影响最大**的部分。本项目使用 **RegulatedPurePursuitController (RPP)**。

### 2.1 基础参数

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0         # 控制循环频率 (Hz)
    min_x_velocity_threshold: 0.05     # 线速度死区 (m/s)
    min_y_velocity_threshold: 0.05     # 横向速度死区 (m/s)
    min_theta_velocity_threshold: 0.05 # 角速度死区 (rad/s)
```

| 参数 | 原理 | 对 USV 的影响 | 调参 |
|------|------|--------------|------|
| `controller_frequency` | 每秒计算 20 次速度指令 | 频率越高控制越细腻，但对船来说 20Hz 足够 | 如果 CPU 紧张可降到 10，但不建议低于 10 |
| `min_*_velocity_threshold` | 低于此值的速度指令视为 0 | 防止推进器在极低指令下抖动 | 一般不改，如果船有"抽搐"现象可稍微加大 |

### 2.2 RPP 核心原理

**Pure Pursuit 算法**：在路径上找一个"前瞻点"（carrot/lookahead point），然后计算从机器人当前位置到该点的曲率，根据曲率算出角速度，使机器人沿圆弧追上该点。

```
路径
  │
  ├── carrot (lookahead point, 距离 = L)
  │      ↗
  │    ／
  │  ／  曲率 κ = 2·sin(α)/L
  │／    角速度 ω = v·κ
  ★ 机器人
```

**关键公式**：
- lookahead 距离 L = `velocity × lookahead_time`，受 `min/max_lookahead_dist` 限制
- 曲率 κ = 2 × sin(航向偏差) / L
- 角速度 = 线速度 × κ

**对 USV 的影响**：Lookahead 太小 → 船盯着脚底下频繁修舵 → S 线。Lookahead 太大 → 有效转弯半径变大、先直走很久再拐，目标在船后时尤为明显。

#### 前瞻点 (carrot) 与最小转弯半径（调参核心）

RPP 每一周期在路径上取距离约为 **L** 的前瞻点，船朝该点走圆弧。**L 必须与船体能转过的最小半径 \(R_{\min}\) 匹配**，否则会出现你观察到的现象：

| L 相对 \(R_{\min}\) | 几何含义 | 典型表现 |
|---------------------|----------|----------|
| **L 过小**（carrot 落在弯内侧、离船很近） | 船尚未完成当前转弯，前瞻点已滑到船侧/船后附近；航向偏差 α 大且变化快，曲率 κ=2·sin(α)/L 剧烈跳变 | 向目标前进时 **S 弯**、左右反复修舵；严重时指令方向 **与期望相反**（表现为“向反方向走”、在 `allow_reversing: false` 下则是长时间大角速度仍追不上 carrot） |
| **L 过大** | carrot 落在弯道前方很远，船长时间沿路径切线方向直行 | **转弯滞后**、**有效转弯半径过大**；目标在船后方时需 **先向前冲很远** 才开始明显转向 |

经验约束（与 `regulated_linear_scaling_min_radius`、船体尺寸同量级核对）：

```text
min_lookahead_dist  ≥ 船体最小转弯半径 R_min（或略大，避免 carrot 落在弯内侧）
                     ≥ 船头前伸量（实船 base_link 在船尾时，避免 carrot 落在船体内）

min_lookahead_dist  ≪ max_lookahead_dist   # 给速度缩放留区间，勿二者都过小
```

**你的判断是有道理的**：过小导致“前瞻点在转弯半径内侧 → 震荡/S 线/偶发反向感”，过大导致“转弯半径过大、先冲远再拐”，都是 Pure Pursuit 几何与 USV 惯性的直接后果，不单是“参数写错一行”，而是 **L 与 \(R_{\min}\)、航速、路径曲率** 不匹配。

### 2.3 速度与前瞻参数

```yaml
FollowPath:
  desired_linear_vel: 1.2              # 目标线速度 (m/s)
  lookahead_time: 3.0                  # 前瞻时间 (s)
  use_velocity_scaled_lookahead_dist: true
  min_lookahead_dist: 3.0              # 最小前瞻距离 (m)
  max_lookahead_dist: 8.0              # 最大前瞻距离 (m)
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `desired_linear_vel` | 直线段或路径平缓时的目标航速 | **太高**：惯性大，冲过路径，S 线加剧。**太低**：舵效差，转不动 |
| `lookahead_time` | 前瞻距离 = 当前速度 × 该值 | 时间越长，carrot 越远，转弯越"提前温和"；时间越短，转弯越"迟但急" |
| `min_lookahead_dist` | 速度趋近 0 时的最小前瞻距离 | **过小**：carrot 易落在 **转弯半径内侧**，转弯未完成就换目标侧 → **S 线**、剧烈左右修舵，严重时像 **反向**；低速也易画圈 |
| `max_lookahead_dist` | 高速时的最大前瞻距离 | **过大**：有效转弯半径大、**先直走很远再拐**；过小则切弯/反应过激。须与 `min` 拉开 |
| `use_velocity_scaled_lookahead_dist` | 前瞻距离随速度缩放 | 必须开，否则高速时盯着近处修，低速时盯着远处看 |

**调参逻辑**：

```
船 S 线 / 画圈:
  → 降 desired_linear_vel（先降到 0.3~0.5 调通控制器）
  → 加大 min_lookahead_dist（3~5m），让船看远一点
  → 加大 max_lookahead_dist（8~10m）

船转弯太迟、切角：
  → 减小 lookahead_time（让 carrot 更近，反应更快）
  → 或减小 max_lookahead_dist
```

### 2.4 曲率限速参数

```yaml
use_regulated_linear_velocity_scaling: true
regulated_linear_scaling_min_speed: 0.1   # 急弯时的最低速度 (m/s)
regulated_linear_scaling_min_radius: 2.0  # 触发降速的转弯半径阈值 (m)
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `use_regulated_linear_velocity_scaling` | 根据路径曲率（弯曲程度）动态调整速度 | 必须开，否则船会高速冲入急弯 |
| `regulated_linear_scaling_min_radius` | 路径转弯半径 < 此值时，速度从 desired 线性降到 min_speed | **太小**：急弯不减速，船冲出路径。**太大**：稍微有点弯就降速，船跑不起来 |
| `regulated_linear_scaling_min_speed` | 急弯时的最低速度 | **太小（0.1）**：低速舵效差，船在弯道里转不动，原地画圈 |

**曲率-速度缩放逻辑**：

```
当前路径点的转弯半径 R:

如果 R >= min_radius:  速度 = desired_linear_vel
如果 R <  min_radius:  速度 = min_speed + (desired - min_speed) × (R / min_radius)
```

**调参逻辑**：

```
粗分辨率地图 (如 shanxi 7.7m):
  路径不平滑 → 更多段被判定为"急弯" → 速度频繁降到 min_speed
  → 加大 min_radius（容忍更多曲率）
  → 提高 min_speed（保证转弯时有舵效）

船在弯道里转不动:
  → min_speed 从 0.1 提到 0.3（给舵面足够水流）
```

### 2.5 接近目标参数

```yaml
use_approach_vel_scaling: true
min_approach_linear_velocity: 0.1     # 接近目标时的最低速度 (m/s)
approach_velocity_scaling_dist: 2.0   # 距目标多远开始降速 (m)
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `use_approach_vel_scaling` | 接近目标时自动减速 | 必须开，否则冲过目标 |
| `approach_velocity_scaling_dist` | 距目标此距离内开始线性减速 | 对船可以加大（3~5m），提前减速 |
| `min_approach_linear_velocity` | 接近目标时的最低速度 | 太小：最后一段极慢，风/流推偏。太大：停不住 |

**速度曲线**：

```
distance_to_goal ≤ approach_velocity_scaling_dist:
  速度 = min_approach + (desired - min_approach) × (distance / approach_dist)

distance_to_goal → 0: 速度 → min_approach
```

### 2.6 其他 RPP 参数

```yaml
max_angular_accel: 2.0               # 最大角加速度 (rad/s²)
allow_reversing: false               # 是否允许倒车
use_rotate_to_heading: false         # 是否先原地转向对齐路径
use_collision_detection: true        # 是否启用碰撞检测减速
max_allowed_time_to_collision_up_to_carrot: 2.0  # 碰撞时间阈值 (s)
min_distance_to_obstacle: 0.8        # 最小障碍物距离 (m)
cost_scaling_factor: 0.5             # 代价缩放因子
inflation_radius: 1.5                # 膨胀半径 (m)
stateful: true                       # 是否记住上一周期状态
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `max_angular_accel` | 限制角速度变化率 | 船转向有惯性，可设小一点（1~1.5）防止转向过猛 |
| `allow_reversing` | 目标在后方时是否倒车 | USV 一般关掉，船倒车不可控 |
| `use_rotate_to_heading` | 追路径前先原地转对方向 | **单桨船必须关**（无法原地转）。**双体差动船可以开**，但注意这会导致高频原地旋转 |
| `use_collision_detection` | 提前检测是否会撞障碍物 | 必须开。检测到碰撞风险时自动降速甚至停车 |
| `max_allowed_time_to_collision_up_to_carrot` | 预估碰撞时间 < 此值则触发减速 | 减小 → 更激进；加大 → 更保守 |
| `min_distance_to_obstacle` | 与障碍物的安全距离 | 船惯性大，可以设大一点（1~1.5m） |
| `stateful` | 保持上一周期状态，避免控制指令突变 | 一般开 |

### 2.7 进度检查与目标检查

```yaml
progress_checker:
  plugin: "nav2_controller::SimpleProgressChecker"
  required_movement_radius: 0.3      # 在时间窗口内至少移动这么远 (m)
  movement_time_allowance: 12.0      # 时间窗口 (s)

general_goal_checker:
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 1.0             # 位置到达容差 (m)
  yaw_goal_tolerance: 1.0            # 航向到达容差 (rad)
  stateful: true
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `required_movement_radius` | 12 秒内移动距离小于此值 → 判定为 stuck | 水流大时应加大，防止误判 stuck |
| `movement_time_allowance` | 判定 stuck 的时间窗口 | 船加速慢，可加大到 15~20s，防止刚起步就被判 stuck |
| `xy_goal_tolerance` | 距目标此距离内算到达 | USV 水上定位精度有限，1~3m 合理 |
| `yaw_goal_tolerance` | 航向偏差此弧度内算到达 | 1.0 rad ≈ 57°，对船来说很宽松。如果要求精确靠泊，减小到 0.2~0.3 |

### 2.8 USV 调 RPP 经验顺序

**核心原则：先用大 lookahead 让船不画圈，再用曲率限速让船转弯不冲，最后再慢慢提高 `desired_linear_vel`。**

不要一上来全改。按以下顺序逐项调，每步验证。

---

#### 第 1 步：调 lookahead，解决画圈/蛇形

这是最重要的一步。RPP 的 adaptive lookahead 公式：

```
lookahead_dist = clamp(当前速度 × lookahead_time,
                       min_lookahead_dist,
                       max_lookahead_dist)
```

| 参数 | USV 经验范围 | 公式/依据 |
|------|-------------|----------|
| `lookahead_time` | **6.0 ~ 12.0** | 控制 carrot 随速度缩放的比例 |
| `min_lookahead_dist` | **1.5 ~ 3 × 地图分辨率** | 低速时的最小前瞻距离 |
| `max_lookahead_dist` | **2 ~ 4 × min_lookahead_dist** | 高速时的最大前瞻距离 |

**以 `map_shanxi` (7.7m/pixel) 为例**：

```yaml
min_lookahead_dist: 10.0
max_lookahead_dist: 18.0
lookahead_time: 8.0
```

| 现象 | 诊断 | 调参方向 |
|------|------|---------|
| 蛇形/画圈/S 线、向目标走时左右摆 | carrot **太近**，常在弯 **内侧**，船转不完就换向 | **增大 min_lookahead_dist**（≥ \(R_{\min}\)）；必要时 **降速** |
| 像反向走、原地扭 | 同上 + `allow_reversing: false` 无法倒车，大角速度仍追内侧 carrot | **增大 min_lookahead_dist**、**降 desired_linear_vel**、**降 max_angular_accel** |
| 目标在船后先冲很远才拐 | carrot **太远**，沿路径切线直行段长 | **减小 max_lookahead_dist** / **lookahead_time**（勿小于 \(R_{\min}\)） |
| 转弯切角严重 | carrot 太远，船抄近道 | **减小 max_lookahead_dist** |
| 路径跟踪太迟钝 | carrot 太远，反应慢 | **减小 lookahead_time**（注意勿跌入“过小”区） |
| 速度越快越不稳 | 高速时 carrot 缩放不够 | **增大 lookahead_time 或 max_lookahead_dist** |

---

#### 第 2 步：调 desired_linear_vel

先不要追求快。控制闭环没调好前，速度越快问题越严重。

```yaml
desired_linear_vel: 0.5 ~ 0.8   # USV 起步建议
```

| 现象 | 调参方向 |
|------|---------|
| 能跟上路径但太慢 | 逐步加到 1.0 / 1.2 |
| 转弯冲出去 | **降低 desired_linear_vel** |
| 直线稳、弯道不稳 | **不一定降速度，优先调曲率限速（第 3 步）** |

---

#### 第 3 步：调曲率限速

这是最容易被忽略但最影响转弯表现的参数组。

```yaml
use_regulated_linear_velocity_scaling: true
regulated_linear_scaling_min_speed: 0.1 ~ 0.25
regulated_linear_scaling_min_radius: 4.0 ~ 10.0
```

**含义**：
- `min_radius` 越大 → 越容易认为"这是急弯" → 越容易触发降速
- `min_speed` 是降速的下限（急弯时的最低速度）

| 现象 | 诊断 | 调参方向 |
|------|------|---------|
| 一到弯道就几乎不走/原地转 | min_radius 太小，误判为急弯；或 min_speed 太小无舵效 | **min_radius ↓ 或 min_speed ↑** |
| 转弯冲出去 | 弯道没降速或降不够 | **min_radius ↑，min_speed ↓** |
| 低速还能跟，速度一高就甩出去 | min_radius 不够大，高速时没提前降速 | **min_radius ↑** |

**以 `map_shanxi` 为例建议起步值**：

```yaml
regulated_linear_scaling_min_radius: 5.0
regulated_linear_scaling_min_speed: 0.15
```

> 原值 `min_radius: 2.0` 对船偏小——路径稍有弯曲就触发降速，船频繁在 0.1m/s 和 1.2m/s 之间切换，表现为画圈或走走停停。

---

#### 第 4 步：调靠近终点减速

```yaml
use_approach_vel_scaling: true
approach_velocity_scaling_dist: 5.0 ~ 15.0   # USV 不要设太短
min_approach_linear_velocity: 0.05 ~ 0.15
```

| 场景 | 建议值 |
|------|--------|
| 普通导航 | `approach_dist: 5.0`, `min_approach_vel: 0.1` |
| 自动靠泊 | `approach_dist: 10.0`, `min_approach_vel: 0.05` |

---

#### 第 5 步：调障碍物/碰撞检测

```yaml
use_collision_detection: true
max_allowed_time_to_collision_up_to_carrot: 2.0 ~ 4.0
min_distance_to_obstacle: 1.0 ~ 3.0           # 船惯性大，比小车设大
```

| 现象 | 调参方向 |
|------|---------|
| 老是莫名停住（无障碍物） | **减小** `max_allowed_time_to_collision` 或 `min_distance_to_obstacle` |
| 避障反应太晚 | **增大** `max_allowed_time_to_collision` |
| 靠泊阶段误停 | 靠泊时单独切换更小的安全距离 |

---

#### 第 6 步：cost 速度调节先别开

```yaml
use_cost_regulated_linear_velocity_scaling: false
```

**先保持 `false`。** 水面代价地图和膨胀层如果不稳定，开了容易出现速度莫名被压低、船走走停停、靠近虚假障碍物就不动。等路径跟踪稳定后再考虑开启。

---

#### 第 7 步：inflation_radius 和 cost_scaling_factor

```yaml
inflation_radius: ≥ 船宽/2 + 安全余量
cost_scaling_factor: 0.5
```

例如船宽 1m，安全余量 1m → `inflation_radius: 1.5` 合理。

| 现象 | 调参方向 |
|------|---------|
| 路径离障碍物太近 | **增大** inflation_radius |
| 可通行区域被挤没了 | **减小** inflation_radius |

---

#### USV 起步参数（可直接替换现有 FollowPath 段）

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

  desired_linear_vel: 0.8
  max_angular_accel: 1.0
  allow_reversing: false

  # --- lookahead ---
  use_velocity_scaled_lookahead_dist: true
  lookahead_time: 8.0
  min_lookahead_dist: 10.0
  max_lookahead_dist: 18.0

  # --- 曲率限速 ---
  use_regulated_linear_velocity_scaling: true
  regulated_linear_scaling_min_speed: 0.15
  regulated_linear_scaling_min_radius: 5.0

  # --- 接近减速 ---
  use_approach_vel_scaling: true
  approach_velocity_scaling_dist: 10.0
  min_approach_linear_velocity: 0.05

  # --- 碰撞检测 ---
  use_collision_detection: true
  max_allowed_time_to_collision_up_to_carrot: 3.0
  min_distance_to_obstacle: 1.5

  # --- 其他 ---
  use_cost_regulated_linear_velocity_scaling: false
  use_rotate_to_heading: false
  stateful: true
```

> **一句话总结：先用大 lookahead 让船不画圈，再用曲率限速让船转弯不冲，最后再慢慢提高 `desired_linear_vel`。**

### 2.9 lookahead 与环境尺度：多尺度路径跟踪

大 lookahead 参数（10~18m）本质是**大范围巡航参数**，适合大湖、海面、长距离航线。但当水域变窄（小河、港口、靠泊），同样的 lookahead 会让船"看过对岸"，忽略局部路径细节。

#### 核心概念：不是分辨率问题，是环境尺度问题

很多人把"大 lookahead 在小场景跑不好"归因于地图分辨率，但实际上：

| 因素 | 对 lookahead 的影响 | 说明 |
|------|-------------------|------|
| **水域宽度** | **极大** | 15m 宽河道用 18m lookahead → 直接看到对岸，切弯撞岸 |
| **航速** | **极大** | 速度越快，相同 lookahead_time 算出的距离越大 |
| **转弯半径** | **极大** | 小船转弯半径小，大 lookahead 会跨过多个弯 |
| **waypoint 密度** | 很大 | 稀疏 waypoint 适合大 lookahead，密集 waypoint 需要小 lookahead |
| 地图分辨率 | 间接 | 分辨率通过影响路径平滑度间接影响 lookahead 选择 |

#### 不同场景的 lookahead 经验值

| 场景 | 水域特征 | 建议 lookahead 范围 | 说明 |
|------|---------|-------------------|------|
| **大范围巡航** | 湖面、海面，宽 > 100m | `10 ~ 18m` | 当前 shanxi 参数，跟踪稳、不蛇形 |
| **小河/狭窄水道** | 河宽 15~50m，弯道密集 | `3 ~ 6m` | 大 lookahead 会跨过多个弯，丢失路径细节 |
| **港口/靠泊** | 宽 < 20m，低速 | `1 ~ 3m` | 需要精确跟踪，甚至不用 RPP 改用视觉 PID |
| **Docking** | 终端逼近 | `1 ~ 2m` | 配合视觉引导，控制器可切换到纯 PID |

#### 为什么大 lookahead 在小场景会"变笨"

RPP 每周期做的事：

```text
1. 在路径上找到最近点
2. 从最近点向前走 lookahead_dist 距离 → 找到 carrot（前瞻点）
3. 计算从船当前位置到 carrot 的圆弧曲率
4. 输出线速度和角速度
```

当 lookahead 太大时：
- **跨弯**：carrot 跨过了路径上的多个弯道，控制器看不到中间的细节
- **切角**：从船到远处 carrot 的圆弧直接穿过弯道内侧 → cutting corner
- **撞岸**：在狭窄河道中，切角路径可能直接穿过岸线

#### 目标在后方/侧后方的绕圈问题

当目标点在船后方或侧后方时：

```
路径从船当前位置 → 延伸到后方目标点
    船头朝前
    最近路径点可能在船后方
    RPP 找最近点 → 找到后方 → carrot 在船后方
    → 船需要掉头
    → 但 allow_reversing: false 且 use_rotate_to_heading: false
    → 船以最小转弯半径画大弧线绕圈
```

**根因**：RPP 永远在路径上向前找 carrot，如果路径方向与船头方向差 180°，整个几何关系需要一个大半径的掉头。这不是参数问题，而是 Pure Pursuit 算法在"目标在身后"场景下的固有限制。

**缓解方法**：
1. waypoint 不要直接设在船后方，通过中间点引导船先转向
2. 如果必须后方目标，设置一个过渡 waypoint 在船前方
3. 接近目标时确保船头已大致朝向目标方向

#### 工程实践：多控制器状态机

真正的 USV 系统不会**固定一个 lookahead**，也不会**一个 RPP 打天下**。而是：

```
CRUISE (大范围巡航)
  lookahead: 10~18m, desired_vel: 0.8~1.2
  ↓ 进入狭窄区域
NARROW_CHANNEL (狭窄水道)
  lookahead: 3~6m, desired_vel: 0.3~0.5
  ↓ 接近目标
APPROACH (接近)
  lookahead: 1~3m, desired_vel: 0.1~0.3
  ↓ 进入靠泊范围
DOCKING (靠泊)
  切换为视觉 PID, 不再用 RPP
  ↓ 到位
HOLD_POSITION (保持位姿)
  独立的位置保持控制器
```

每个状态不仅参数不同，甚至**控制器类型都可能不同**。

---

## 3. planner_server — 全局路径规划器

本项目使用 **SmacPlanner2D**（基于 A* 的混合 A* 规划器）。

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0   # 期望规划频率 (Hz)
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.125                # 目标点容差 (m)
      cost_travel_multiplier: 2.0     # 路径代价乘数
      downsample_costmap: false       # 是否降采样代价地图
      downsampling_factor: 1
      allow_unknown: true             # 是否允许穿越未知区域
      max_iterations: 1000000         # A* 最大迭代次数
      max_on_approach_iterations: 1000
      max_planning_time: 3.0          # 最大规划时间 (s)
      smoother:                       # 路径平滑器
        max_iterations: 1000
        w_smooth: 0.4                 # 平滑权重（越高越平滑）
        w_data: 0.2                   # 数据保真权重（越高越贴近原始路径）
        tolerance: 1.0e-10
        do_refinement: true           # 是否做精化
        refinement_num: 2             # 精化次数
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `expected_planner_frequency` | 目标规划速率 | 这个只是日志告警阈值，不是硬限制，实际跑不到也不影响功能 |
| `cost_travel_multiplier` | 路径每米代价的乘数 | 增大 → 规划器更倾向短路径；减小 → 更倾向安全路径 |
| `allow_unknown` | 允许路径穿过未知区域 | 水域大范围未知时可开，但船可能规划出"穿岛"路径 |
| `max_planning_time` | 单次规划最多算 3 秒 | 大地图（如 7.7m/pixel、数千像素）可能超时，超时后返回当前最优 |
| `w_smooth` | 平滑权重，越高路径越光滑 | **对船很重要**。粗分辨率地图可加大到 0.5~0.6 |
| `w_data` | 保持路径贴近原始 A* 规划的权重 | 减小 → 更平滑但可能偏离最优；加大 → 更贴近原始但更曲折 |
| `do_refinement` | 平滑后再做锚点精化 | 开启可以让路径点在平滑后重新调整到更合理的位置 |

**规划器告警 `Planner loop missed its desired rate`**：

```
原因: SmacPlanner2D 在本次计算中超时
影响: 不严重，只是下一帧规划延迟
解决: 减小 max_planning_time 或增大 global_costmap 分辨率（减少搜索空间）
```

---

## 4. local_costmap — 局部代价地图

局部代价地图是**以机器人为中心的滚动窗口**，用于避障和局部路径跟踪。

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0           # 更新频率 (Hz)
      publish_frequency: 5.0          # 发布频率 (Hz)
      global_frame: odom              # 坐标系（odom 保证局部连续）
      robot_base_frame: base_link
      rolling_window: true            # 滚动窗口模式
      width: 20                       # 窗口宽度 (m)
      height: 15                      # 窗口高度 (m)
      resolution: 0.05                # 栅格分辨率 (m/pixel)
      footprint: "[[0.5,0.3],[0.5,-0.3],[-0.5,-0.3],[-0.5,0.3]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      always_send_full_costmap: True
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `global_frame: odom` | 局部代价图在 odom 帧（随船移动） | 必须用 `odom` 而不是 `map`，因为滚动窗口需要连续、无跳变的坐标系 |
| `rolling_window` | 窗口随船移动 | 船动窗口动，始终覆盖船周围 width×height 的区域 |
| `width/height` | 窗口大小 (m) | 太小：看不到前方的障碍物。太大：计算量大。20×15 对船来说偏小，建议 30~40m |
| `resolution: 0.05` | 每格 5cm | 这是局部代价图的精度，不是地图精度。0.05m 对 Lidar 避障足够 |

**footprint**：船的物理轮廓，用于碰撞检测。当前为 1.0m×0.6m 矩形。

### 障碍物层

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  observation_sources: scan
  scan:
    topic: /roboboat/sensors/lidar/scan
    max_obstacle_height: 1.0       # 超过此高度的点不视为障碍物
    clearing: True                 # 射线清除（看到空白就清掉旧障碍）
    marking: True                  # 射线标记（看到障碍就标记）
    data_type: "LaserScan"
    obstacle_max_range: 10.0       # 超过此距离的检测忽略
    obstacle_min_range: 0.1        # 小于此距离的检测忽略
    raytrace_max_range: 10.0
    raytrace_min_range: 0.1
```

| 参数 | 原理 | 调参 |
|------|------|------|
| `max_obstacle_height` | Lidar 点高于此值的不算障碍 | 水面场景设小（桥、树冠不要挡路），但浮标、码头要保留 |
| `obstacle_max_range` | 超过此距离的障碍物不标记 | 船速高时加大（15~20m），给更多时间反应 |
| `clearing: True` | 射线从传感器到障碍物之间的空间被标记为自由 | 必须开，否则旧障碍物永远不消 |
| `marking: True` | 射线末端被标记为障碍物 | 必须开 |

### 膨胀层

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 1.0        # 代价衰减速率
  inflation_radius: 1.5           # 膨胀半径 (m)
```

| 参数 | 原理 | 调参 |
|------|------|------|
| `inflation_radius` | 障碍物周围此半径内代价值 > 0，机器人会"绕开" | 船惯性大，建议 1.5~2.5m |
| `cost_scaling_factor` | 越大 → 代价从障碍物向外衰减越快 → 机器人更贴近障碍物 | 默认 1.0 即可 |

**膨胀层的代价公式**：

```
cost = 253 × exp(-cost_scaling_factor × (distance_from_obstacle))
```

在 `inflation_radius` 边界处 cost ≈ 0，在障碍物表面 cost = 253（致命代价）。

---

## 5. global_costmap — 全局代价地图

全局代价地图覆盖整个已知世界，用于全局路径规划。

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      rolling_window: false          # 不滚动，覆盖全图
      footprint: "[[0.5,0.3],[0.5,-0.3],[-0.5,-0.3],[-0.5,0.3]]"
      resolution: 7.8                # 与地图 shanxi 分辨率对齐
      track_unknown_space: True
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `rolling_window: false` | 固定窗口，覆盖整张静态地图 | 全局代价图就是地图的范围 |
| `resolution` | 栅格分辨率 (m/pixel) | **关键参数**。应与加载的地图 yaml 分辨率一致或为其倍数。7.8 对应 shanxi 的 7.71，避免重采样 |
| `track_unknown_space` | 区分"未知"和"自由" | True 时规划器不穿越未知区域（更安全）；False 时未知=自由 |

### 静态地图层

```yaml
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: True  # 使用 TRANSIENT_LOCAL QoS，适应大地图
```

`map_subscribe_transient_local: True`：大地图（如 shanxi）必须开，否则 QoS 不匹配导致收不到地图。

### 全局膨胀层

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 1.0
  inflation_radius: 2.0              # 比局部的大（2.0 vs 1.5）
```

全局膨胀半径比局部大是合理的：全局规划需要更大的安全裕度，局部避障可以更精细。

---

## 6. behavior_server — 恢复行为

当机器人 stuck 或无法到达目标时，执行恢复行为。

```yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    cycle_frequency: 1.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    global_frame: map
    local_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 3.0         # 行为模拟前看时间 (s)
    max_rotational_vel: 1.0          # 恢复行为最大角速度 (rad/s)
    min_rotational_vel: 0.2          # 恢复行为最小角速度 (rad/s)
    rotational_acc_lim: 2.0          # 恢复行为角加速度限制
```

恢复行为优先级（由 Nav2 默认行为树决定）：

1. **Spin**：原地旋转 360°，清出周围空间
2. **BackUp**：后退一段距离
3. **DriveOnHeading**：沿固定方向行驶
4. **Wait**：原地等待

| 参数 | 调参 |
|------|------|
| `simulate_ahead_time` | 模拟"继续按当前速度走 X 秒会不会撞" |
| `max_rotational_vel` | 恢复时最大旋转速度，太大可能在狭小空间旋转时撞到东西 |
| `min_rotational_vel` | 恢复时最小旋转速度，太小转不动 |

---

## 7. velocity_smoother — 速度平滑器

位于控制器输出和实际发送之间，防止速度指令突变。

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0        # 平滑器频率 (Hz)
    scale_velocities: true           # 按比例缩放速度（保持方向）
    feedback: "OPEN_LOOP"            # 实船默认 OPEN_LOOP；仿真常用 CLOSED_LOOP
    max_velocity: [0.5, 0.0, 0.5]    # 实船保守示例；须与 nav2_cmd_vel_to_mavros 上限同量级
    min_velocity: [0.0, 0.0, -0.5]
    max_accel: [0.5, 0.0, 0.5]
    max_decel: [-0.5, 0.0, -0.5]
    odom_topic: "/mavros/local_position/odom"
    odom_duration: 0.1
    deadband_velocity: [0.02, 0.0, 0.02]  # 死区
    velocity_timeout: 1.0             # 速度指令超时 (s)
```

| 参数 | 原理 | 对 USV 的影响 |
|------|------|--------------|
| `feedback: CLOSED_LOOP` | 从 odom 读取实际速度，与目标速度比较，按加速度约束逼近 | **对船很重要**，防止速度指令突变导致船体晃动 |
| `max_accel` | 线加速度上限 [x, y, yaw] | 船加速度小：`[0.5, 0, 0.5]` 更真实。太大 → 推进器指令跳变 |
| `max_decel` | 线减速度上限 | 船减速也慢：`[-0.5, 0, -0.5]` |
| `max_velocity` | 绝对速度上限 | 船速一般不超过 2m/s，但 angular 可以更小（1.0~1.5） |
| `deadband_velocity` | 低于此值的指令忽略 | 防止推进器在零附近持续微动 |
| `velocity_timeout` | 超过此时间没收新指令 → 停车 | 安全机制，一般不改 |

**CLOSED_LOOP 模式**工作方式：

```
目标速度 = min(max(控制器输出, min_velocity), max_velocity)
加速度限制 = 目标速度 - 实际速度（从 odom 读取）
实际发出 = 实际速度 + clamp(加速度限制, max_decel×dt, max_accel×dt)
```

这确保了速度指令不会跳变，对船体平稳运动很重要。

---

## 8. waypoint_follower — 航点跟随

```yaml
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 50    # 每个航点停留时间 (ms)
```

| 参数 | 含义 |
|------|------|
| `stop_on_failure` | 某航点失败是否停止整个任务 |
| `waypoint_pause_duration` | 到达航点后的停留时间。50ms 基本上刚到就走；需要停船观察则加大 |

---

## 9. 常见问题速查表

| 现象 | 最可能原因 | 优先调什么 |
|------|-----------|-----------|
| **船原地画圈** | RPP 曲率限速把速度压到 0.1，舵效差转不动 | `regulated_linear_scaling_min_speed` ↑、`min_radius` ↑ |
| **S 线（左右摆）** | lookahead **过小**，carrot 在转弯半径 **内侧**，船未完成转弯就换修舵方向 | `min_lookahead_dist` ↑（≥ \(R_{\min}\)）、`desired_linear_vel` ↓、`max_angular_accel` ↓ |
| **像反方向走 / 剧烈扭** | 同上，曲率指令翻转；`allow_reversing: false` 时无法倒车只能大角速度“拧” | `min_lookahead_dist` ↑、降速，勿只加大 `max_angular_accel` |
| **转弯半径过大、先冲远再拐** | lookahead **过大**，carrot 在弯前很远 | `max_lookahead_dist` ↓、`lookahead_time` ↓（保持 min ≥ \(R_{\min}\)） |
| **冲过路径/目标** | 接近不减速 或 减速度太小 | `approach_velocity_scaling_dist` ↑、`max_decel` ↑ |
| **规划路径穿岛/穿岸** | `allow_unknown: true` + 未知区域无代价 | `allow_unknown: false` 或 完善静态地图 |
| **Planner 速率告警** | 搜索空间太大，SmacPlanner2D 超时 | 减小 `max_planning_time` 或 增大 `global_costmap/resolution` |
| **频繁触发 recovery** | 进度检查太严格 或 船真的 stuck | `movement_time_allowance` ↑、`required_movement_radius` ↓ |
| **换地图后行为异常** | 地图分辨率变了，RPP 曲率检测不准 | 检查 `regulated_linear_scaling_min_radius` 和 `global_costmap/resolution` |
| **速度指令抖动** | velocity_smoother 加速度限制太松 | `max_accel` ↓、`max_decel` ↓ |
| **船在目标点附近转圈不停** | goal_tolerance 太严，船永远到不了 | `xy_goal_tolerance` ↑、`yaw_goal_tolerance` ↑ |
| **目标在船后方，船绕大圈** | RPP 固有限制：路径方向与船头差 180°，carrot 在身后 | 加过渡 waypoint 引导转向，避免直接设后方目标 |
| **小河/窄水道切弯撞岸** | lookahead 太大，跨过多个弯道，丢失路径细节 | `min_lookahead_dist` ↓ (3~6m)、`max_lookahead_dist` ↓ |
| **大湖跑得稳，小河跑不稳** | 固定 lookahead 不匹配环境尺度变化 | 按场景动态切换 lookahead（见 §2.9） |

---

## 参数间的耦合关系

最重要的耦合链：

```
环境尺度 (水域宽度、弯道密度)
    ↓ 决定
地图分辨率 (global_costmap/resolution)
    ↓ 影响
路径平滑度 (smoother 参数)
    ↓ 影响
RPP 曲率检测 (regulated_linear_scaling_min_radius)
    ↓ 影响
实际航速 (regulated_linear_scaling_min_speed)
    ↓ 影响
nav2_cmd_vel_to_mavros 限幅 (max_linear_x / max_angular_z)
    ↓ 影响
船的运动轨迹 (S 线 / 画圈 / 正常)
```

**调参原则**：
- 换环境（大湖→小河）→ 先调 lookahead（§2.9），再调曲率参数
- 换地图分辨率 → 同步检查 RPP 曲率参数
- 改 RPP 速度参数 → 同步检查 **`nav2_cmd_vel_to_mavros`** 的 `max_linear_x` / `max_angular_z`（默认桥吃 `/cmd_vel_nav`，绕过 smoother）

---

*最后更新：2026-05-29（实船副本；与 `nav2_params_real_mavros.yaml` 同步维护）*
