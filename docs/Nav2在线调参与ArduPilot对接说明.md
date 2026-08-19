# Nav2 在线调参与 ArduPilot 对接说明

> 本文整理自代码审查结论：定位信号质量检测、Nav2 在线调速、膨胀层在线修改、按代价图距离动态降速，以及 Nav2 → ArduPilot 速度链改造的可行性与现状。
> 以代码为准，文档仅作辅助。

## 1. 现状概述

- 定位链路：PX4 / MAVROS 里程计 `odom→base_link`，`gnss_odom_map_tf` 用 GNSS 装订 `map→odom`。
- 速度链：`controller_server(RPP) → /cmd_vel_nav → 速度桥 → 飞控`。
- 旧速度桥 `nav2_cmd_vel_to_mavros` 是 **PX4 专用**（`PositionTarget`，且把 `velocity.y` 当 yaw rate 用）。
- 目标：Nav2 输出直接对接 **ArduPilot**，速度平滑器不接。
- 定位质量：`nav_status_aggregator` 检测并上报，**只观测、不强制执行**。

## 2. 定位信号质量检测

### 2.1 检测逻辑

位置：`workspace_nav/workspace_nav/nav_status_aggregator.py`（`_update_loc_health`）。

| 判定 | 条件 |
| --- | --- |
| LOST | TF `map→base_link` 不可用，或 odom 超过 `odom_timeout`（2s），或位置协方差 x/y/z 最大值 > `cov_lost_threshold`（10 m²） |
| DEGRADED | 位置协方差 > `cov_degraded_threshold`（1 m²），或 GPS 超过 `gps_timeout`（5s），或 odom 频率 < 20Hz，或 GPS status < 3 |
| GOOD | 以上均不满足 |

阈值参数见 `workspace_nav/config/mission_stack.real_boat.yaml` 与 aggregator 参数声明。

结果通过 `/nav_status`（2Hz）的 `localization.overall` 上报，状态跳变时发 `/task_event` 告警（`LOC_LOST` / `LOC_DEGRADED`）。

### 2.2 现状：只上报，不干预

- `nav_status_aggregator` 是 pure observer，不发送任何速度/取消/急停指令。
- `mission_bridge` 发任务前只检查 TF、FollowWaypoints action server，发点时等 odom；**不检查定位质量**。
- 因此“定位质量差就不走/降速”目前没有硬门禁，只有告警。

### 2.3 已知问题：GPS status 判断与 MAVROS 语义不一致

aggregator 用 `gps.status.status < 3` 判 DEGRADED，但 `sensor_msgs/NavSatStatus` 的常量是：

```
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2
```

MAVROS 发布 `/mavros/global_position/raw/fix` 时，`fix_type > 2` 只填 `STATUS_FIX = 0`（见 MAVROS `global_position.cpp`）：

```cpp
if (raw_gps.fix_type > 2) {
  fix.status.status = NavSatStatus::STATUS_FIX;   // 0
} else {
  fix.status.status = NavSatStatus::STATUS_NO_FIX;
}
```

因此 `status < 3` 在有有效 GPS 时恒成立，`localization.overall` 会**一直报 DEGRADED**。代码注释里的“3 = 3D fix”是 PX4 `fix_type` 语义，不是 `NavSatStatus` 语义。

建议改为 `gps.status.status >= NavSatStatus.STATUS_FIX`（即 >= 0）判 GPS 有效；如需区分 RTK/非 RTK，可从 `position_covariance` 或 MAVROS `GPS_RAW_INT` 的 fix_type 链路补。

对比：`gnss_odom_map_tf` 用的是 `msg.status.status < NavSatStatus.STATUS_FIX` 才丢弃，语义正确，启动后 `initialize_once` 锁定首对有效数据。

## 3. Nav2 在线修改速度

### 3.1 RPP 动态参数（目标速度）

Nav2 1.1.19 的 RPP 注册了动态参数回调，`FollowPath.desired_linear_vel` 在线设置立即生效：

```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 1.5
```

可在线改的参数不止速度，还包括 `lookahead_*`、`regulated_linear_scaling_min_*`、`cost_scaling_*`、`max_angular_accel` 等，见 `nav2_regulated_pure_pursuit_controller.cpp` 的 `dynamicParametersCallback`。

### 3.2 `/speed_limit` 话题（运行时限速通道）

controller_server 原生订阅 `nav2_msgs/msg/SpeedLimit`：

- `percentage: true`：按 `base_desired_linear_vel` 的百分比限速；
- `percentage: false`：按绝对速度（m/s）限速；
- `speed_limit: 0.0`：清除限速，恢复默认目标速度。

```bash
ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit \
  "{percentage: false, speed_limit: 1.2}"
```

这是“运行时按障碍物动态限速”的正规通道，控制器内部调用 RPP 的 `setSpeedLimit()`，不经过速度平滑器、不经过飞控桥。

**注意**：当前 `nav2_params_real_mavros.yaml` 未配置 `speed_limit_topic`，controller_server 默认订阅相对名 `speed_limit`（实际为 `/controller_server/speed_limit`）。要用 `/speed_limit`，需在 yaml 的 `controller_server.ros__parameters` 增加：

```yaml
speed_limit_topic: "/speed_limit"
```

### 3.3 velocity_smoother：支持动态改，但当前被绕过

- `velocity_smoother` 支持在线改 `max_velocity` / `max_accel` / `max_decel` 等。
- 但实船桥默认 `cmd_vel_src:=/cmd_vel_nav`，**绕过了 smoother**，改它不会影响实际输出。
- 如果后续要走平滑器，需把桥改为订阅 `/cmd_vel`。

### 3.4 飞控桥的硬限幅

旧 PX4 桥 `nav2_cmd_vel_to_mavros` 每个 20Hz 周期都重新读 `max_linear_x` / `max_angular_z`，在线 `ros2 param set` 立即生效：

```bash
ros2 param set /nav2_cmd_vel_to_mavros max_linear_x 1.5
```

注意当前配置：yaml 里 `desired_linear_vel: 1.2`，但桥 launch 默认 `max_linear_x: 1.0`，**实际封顶 1.0 m/s**。

### 3.5 决策层 speed 字段：目前未接入 Nav2

- `NavActuator::executeMission` 里 `(void)speed;`，决策下发的航线速度没有传给 Nav2。
- `SendWaypoints` 服务本身没有 speed 字段。
- `/decision/mission_param_set` 只写 PX4 参数 `WP_SPEED` / `MIS_DONE_BEHAVE`，走的是 PX4 航点任务，不是 Nav2 FollowWaypoints。

结论：当前“任务带速度、航线中途改速”没有实现；Nav2 侧只能全局改 `desired_linear_vel` 或发 `/speed_limit`。

## 4. 在线修改膨胀层

### 4.1 costmap 膨胀层支持动态参数

Nav2 `InflationLayer` 注册了动态参数回调，支持在线改：

- `inflation_layer.inflation_radius`
- `inflation_layer.cost_scaling_factor`
- `inflation_layer.enabled`

改半径/衰减系数后会置 `need_reinflation_` 整层重算。local 与 global 是两个独立节点，需分别设置：

```bash
ros2 param set /local_costmap  inflation_layer.inflation_radius 2.5
ros2 param set /global_costmap inflation_layer.inflation_radius 2.5
```

### 4.2 注意 RPP 自己的 inflation_radius

RPP 还有独立参数 `FollowPath.inflation_radius`（当前 2.0），用于“接近高代价区时提前减速”的代价距离参考，**不是 costmap 膨胀层**。若目的是让船更早减速，需同步：

```bash
ros2 param set /controller_server FollowPath.inflation_radius 2.5
```

### 4.3 持久化

在线修改只对当前进程生效，重启回到 yaml。长期生效需同步修改 `nav2_params_real_mavros.yaml`。

## 5. 按船与代价图距离动态降速

### 5.1 RPP 内置代价调节速度（已开启）

RPP `applyConstraints` 内置“按代价距离降速”：

```cpp
if (use_cost_regulated_linear_velocity_scaling_ && pose_cost 有效) {
  min_distance_to_obstacle =
    (-1.0 / inflation_cost_scaling_factor_) *
    std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) +
    inscribed_radius;

  if (min_distance_to_obstacle < cost_scaling_dist_) {
    cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
  }
}
```

当前 yaml 已开启：

```yaml
use_cost_regulated_linear_velocity_scaling: true
cost_scaling_dist: 3.0
cost_scaling_gain: 1.0
inflation_cost_scaling_factor: 1.0
regulated_linear_scaling_min_speed: 0.1
```

即船进入膨胀代价区后（约 3m 内），速度随“反解出的障碍距离”线性下降，最低不低于 `regulated_linear_scaling_min_speed`。

### 5.2 碰撞检测（硬停）

`use_collision_detection: true` + `max_allowed_time_to_collision_up_to_carrot: 2.5`：预测前进方向在 2.5s 内会撞到障碍时抛异常停住。

### 5.3 内置方案的局限

- 内置降速依据是“船当前所在栅格 cost”，不是“正前方最近障碍距离”。
- 若需要扇形扫描、自定义距离→速度曲线、滞回等，推荐自写一个小节点：
  1. 订阅 `/local_costmap/costmap_raw`（或使用 costmap 服务/API）；
  2. 计算船前方/周边最近障碍距离；
  3. 发布 `nav2_msgs/msg/SpeedLimit` 到 `/speed_limit`；
  4. controller_server 自动替换 RPP 目标速度，桥无需改动。

### 5.4 调参顺序建议

1. 先标定 `cost_scaling_dist`（减速起始距离）与 `cost_scaling_gain`（减速强度）；
2. 再调 `inflation_radius` / `inflation_cost_scaling_factor`（决定距离反解曲线）；
3. 仍不满足再上自定义 `/speed_limit` 节点。

## 6. Nav2 → ArduPilot 对接

### 6.1 旧 PX4 桥不适用

`nav2_cmd_vel_to_mavros` 是 PX4/MAVROS 专用：

- 发 `PositionTarget` 到 `/mavros/setpoint_raw/local`；
- `velocity.y` 被本船 PX4 固件自定义为 yaw rate；
- OFFBOARD 门控、CMODE 判断等均是 PX4 语义。

ArduPilot 的 `setpoint_raw/local` 中 `velocity.y` 是侧向速度，不能直接沿用。

### 6.2 已复制到 USV_NAV 的 ArduPilot 速度桥

`ardupilot_velocity_bridge` 已从 `/home/ght/usv/src/mqtt/src/boat_control/offboard_control`
复制进本仓，位于 `src/usv_ardupilot_velocity_bridge`，并已接入
`workspace_ros/launch/real_boat_bringup.launch.py`。

特性：

- 输入 `input_cmd_topic`（本仓默认 `/cmd_vel_nav`）；
- 输出 `/mavros/setpoint_velocity/cmd_vel_unstamped`（Twist，linear.x + angular.z）；
- 自动切 `GUIDED` + 解锁（`auto_arm`，可关）；
- 命令超时（`command_timeout_sec`）后发零速；
- `max_linear_x` / `max_angular_z` 限幅，**每次收到指令都重新读参数**，在线 `ros2 param set` 立即生效；
- 输出 publisher 已改为 `SensorDataQoS`（best-effort），与 MAVROS
  `setpoint_velocity/cmd_vel_unstamped` 订阅侧一致（原版默认 reliable 连不上）。

编译与启动：

```bash
cd /home/ght/USV_NAV
colcon build --packages-select usv_ardupilot_velocity_bridge
source install/setup.bash

# 与实船 TF 一起起
ros2 launch workspace_ros real_boat_bringup.launch.py \
  use_sim_time:=false enable_ardupilot_velocity_bridge:=true
```

链路：

```text
/cmd_vel_nav (Twist) → ardupilot_velocity_bridge → /mavros/setpoint_velocity/cmd_vel_unstamped (Twist)
```

单独起节点：

```bash
ros2 run usv_ardupilot_velocity_bridge ardupilot_velocity_bridge --ros-args \
  -p input_cmd_topic:=/cmd_vel_nav \
  -p publish_rate_hz:=20
```

注意点：

- 该节点允许负 `linear.x`（可倒车），船不能倒车需加 `forbid_reverse` 式钳制（当前未加）；
- 桥的 `max_linear_x` / `max_angular_z` 应作为**安全硬上限**，比 Nav2 正常工作范围略宽；
  日常调速度以 Nav2 侧参数为准，避免两处限幅互相打架（见 §7）；
- 参考脚本：`/home/ght/usv/scripts/keyboard_cmd_vel.py`、`guided_random_cmd_vel.py`。

## 7. 结论

能做到，且不需要改 Nav2 本体：

1. **在线改最大速度**：RPP 动态参数（`FollowPath.desired_linear_vel`）+ `/speed_limit` 话题，桥的 `max_linear_x` 做硬上限。
2. **按障碍物距离降速**：RPP 内置 `use_cost_regulated_linear_velocity_scaling` 已实现并开启；需要更精确曲线时加一个发 `/speed_limit` 的小节点。
3. **ArduPilot 对接**：使用本仓 `src/usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge`，输入 `/cmd_vel_nav`，
   通过 `real_boat_bringup.launch.py enable_ardupilot_velocity_bridge:=true` 一起启动；速度平滑器不接不影响。
4. **定位质量**：检测和告警已有，但没接入硬门禁；且 GPS status 判断存在与 MAVROS 语义不一致的问题，建议先修。

## 8. 待办建议

- [ ] 修复 `nav_status_aggregator` 的 GPS status 判断（`< 3` → `>= STATUS_FIX`），避免一直报 DEGRADED。
- [ ] `nav2_params_real_mavros.yaml` 增加 `controller_server.speed_limit_topic: "/speed_limit"`。
- [ ] 标定 RPP 代价降速参数（`cost_scaling_dist` / `cost_scaling_gain` / `inflation_radius`）。
- [ ] 给 `src/usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge` 补 `forbid_reverse`（当前允许负 `linear.x`）；QoS 与 launch 集成已完成。
- [ ] 如需精确“前方最近障碍距离 → 速度”曲线，实现并接入 `/speed_limit` 发布节点。
- [ ] 如需航线级速度下发/中途调速，补齐决策层 speed → Nav2 的链路（当前 `(void)speed`）。
