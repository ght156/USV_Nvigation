# 2026-10-19 巡航到点容差 2 m 与 map→odom TF 卡滞导致任务 RUNNING

## 1. 基本信息

- 日期：2026-10-19
- 船/工作空间：兴飞 MEC-MB002（`ssh hkzl`），`~/usv-test`
- 会话时间段：约 13:30–13:50（导航容差与卡滞）
- 数据来源：
  - `~/usv-test/log/cleaning_boat_20261019_131019_149976687/navigation.log`
  - 同目录 `localization.log`
  - `/mission_bridge/status_detail`、`/controller_server` 参数
- 环境：`ROS_DOMAIN_ID=5`；当前会话使用默认 RMW（新版项目说明为 FastDDS；
  历史记录里的 CycloneDDS 配置仅作旧环境参考）。

## 2. 现象

1. 平台反馈：巡航到点容差 1 m 时船在目标点附近一直转圈，要求改成 2 m。
2. 在线把 `general_goal_checker.xy_goal_tolerance` 改成 2.0 后，下一段任务前 4 个航点
   正常到达；第 5/5 航点前船停住。
3. 导航**没有报失败**：`/mission_bridge/status_detail` 一直为：

```json
{"state":"RUNNING","waypoint_total":5,"waypoint_completed":4,"waypoint_current_index":4,...}
```

4. `controller_server` 每秒 `[follow_path] [ActionServer] Aborting handle`；
   `waypoint_follower` 每 50 ms 打 `Processing waypoint 0...`；`/cmd_vel_nav` 无输出，
   船原地停住。

## 3. 证据

### 3.1 到点容差

修改前：

```text
$ ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
Double value is: 0.5
```

- YAML `nav2_params_xingfei.yaml` 里 `general_goal_checker.xy_goal_tolerance: 1.0`；
- `dock_mission` 启动/出港收尾时 `apply_cruise()` 用
  `cruise_xy_goal_tolerance: 0.5` 在线设置，所以参数服务器显示 0.5；
- `docking_goal_checker` 仍是 `0.5 / 0.15`，未动。

在线修改：

```text
$ ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 2.0
Set parameter successful
$ ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
Double value is: 2.0
```

该版 `nav2_controller::SimpleGoalChecker` 头文件含 `dynamicParametersCallback`，
在线修改会更新 `xy_goal_tolerance_` / 缓存平方值，立即生效。

### 3.2 mission_bridge 读错参数文件

启动日志（`navigation.log:131`）：

```text
[mission_bridge]: waypoint_tolerance_m: 2.00 m
(自动 ← /home/hkzl/usv-test/install/arm/workspace_nav/share/workspace_nav/config/nav2_params_chengfeng.yaml
 的 goal checker xy_goal_tolerance)
```

原因：`mission_bridge.py` 的
`NAV2_PARAMS_DEFAULT_NAME = "nav2_params_chengfeng.yaml"`；
`mission_bridge.launch.py` 的 `nav2_params_path` 默认空，兴飞 launch 没传，所以回退到
澄丰 YAML。兴飞 Nav2 当时是 1.0，而 mission_bridge 是 2.0，两边不同源。

### 3.3 任务卡在 5/5

时间线：

- 13:37:56 `[navigate] accept goal: 5 waypoints`，`STATE -> RUNNING`
- 13:39:19 第 1 点到达；之后 2、3、4 点依次到达
- 13:40:32 `Sending waypoint 5/5 x=133.60, y=128.94, yaw=-166.306`
- 13:40:40–13:40:42：

```text
[tf_help] Transform data too old when converting from map to odom
[tf_help] Data time: 1792417240s 186682215ns, Transform time: 1792417239s 815000000ns
[controller_server] Exception in transformPose: Lookup would require extrapolation into the future.
  Requested time 1792417241.535000 but the latest data is at time 1792417241.435000,
  when looking up transform from frame [odom] to frame [map]
[controller_server] Unable to transform robot pose into global plan's frame
[controller_server] [follow_path] [ActionServer] Aborting handle.
```

- 之后持续 `Aborting handle`（约 1.03 s 一次）；`waypoint_follower` 反复
  `Processing waypoint 0...`；任务不结束。
- 13:45 左右 `status_detail` 仍为 `RUNNING, waypoint_completed=4, current_index=4`。
- 船位：`/mavros/gps_input/local` 约 `(28.06, -2.17)`；`mavros` `GUIDED/armed=true`；
  `/cmd_vel_nav` 无消息。

### 3.4 CPU / odom

```text
load average: 11.78, 10.07, 9.14
mavros_node 100% CPU
component_container_isolated 60.3% / 32.0%
mqtt_bridgev2_n 25.4%
dock_mission_node 19.8%
docking_pose_estimator 19.4%
mission_bridge 18.6%
docking_fsm 17.9%
livox_ros_driver 15.5%
nav_status_aggregator 15.1%
docking_motion_controller 13.5%
decision_node 11.7%
```

`nav_status_aggregator` 记录 `odom_hz=7.9`；日志中大量
`Control loop missed its desired rate of 20.0000Hz`、
`BehaviorTreeEngine tick rate 100.00 was exceeded!`；13:41:38 还出现
`controller_server get_parameters timeout`。

> 注：上表 `%CPU` 来自 `ps -eo pcpu`，是进程启动以来的**平均值**，不是瞬时值；
> load average 是 1/5/15 分钟。船恢复后用 `top -b -n1 -H` /
> `pidstat -t -p <pid> 1 5` 看瞬时线程占用，再定论。

### 3.5 为什么 progress checker 没报失败

`nav2_params_xingfei.yaml`：

```yaml
progress_checker:
  plugin: "nav2_controller::SimpleProgressChecker"
  required_movement_radius: 2.0
  movement_time_allowance: 60.0
```

按设计，单个 `FollowPath` goal 内 60 s 平面位移不足 2.0 m 应判
`Failed to make progress`，上层会收到失败。但本次：

- `controller_server` 因 TF 异常约每 1 s abort 一次；
- `waypoint_follower` 立刻重发同一个 `FollowPath` goal；
- 每个新 goal 都会 reset progress checker，60 s 窗口永远累计不起来；
- `mission_bridge` 只等 `FollowWaypoints` 的 result，没有整任务超时，所以一直
  `RUNNING`。

补充：如果船是**绕圈**（半径使平面位移超过 2 m），progress checker 也可能不触发；
它只看位移，不看角运动。

### 3.6 Nav2 频率与局部体素层

当前基线频率：

```text
bt_navigator.bt_loop_duration: 10        # 100 Hz
controller_server.controller_frequency: 20.0
velocity_smoother.smoothing_frequency: 20.0
local_costmap.update_frequency: 5.0 / publish_frequency: 5.0
global_costmap.update_frequency: 1.0 / publish_frequency: 1.0
```

日志里 `BehaviorTreeEngine tick rate 100.00 was exceeded!` 与
`Control loop missed its desired rate of 20.0000Hz` 同时出现，说明当前 CPU 已经饱和；
**这不靠降控制频率解决，必须查 CPU 根因**（控制频率保持 20 Hz）。

局部代价图：

- 主机船端仓 `/home/ght/usv` 的 commit `9da7953`（2026-10-19 16:38）已把
  `local_costmap.voxel_layer.enabled` 从 `true` 改为 `false`，理由是“整船集成栈避免
  重复消费点云”；
- 但船端 13:10 起栈在 commit 之前，`navigation.log` 里
  `local_costmap.local_costmap: Message Filter dropping message: frame 'livox_frame'`
  说明船端局部代价图仍在消费点云，**船端大概率还是 `enabled: true`**；
- 导航仓 `/home/ght/USV_NAV` 的同一文件仍保留 `enabled: true`，这是给独立导航做
  局部避障用的，不要直接同步成 `false`。

### 3.7 空闲子系统配置的循环频率（静态检查）

从配置里读到的循环频率：

| 节点 | 参数 | 频率 |
|---|---|---:|
| `dock_mission_node` | `create_timer(0.2)` | 5 Hz |
| `docking_fsm` | `state_rate` | 10 Hz |
| `docking_pose_estimator` | `publish_rate` | 20 Hz |
| `docking_motion_controller` | `control_rate` | 20 Hz |
| `docking_safety` | `check_rate` | 20 Hz |
| `decision_node` | `tick_period_ms: 50` | 20 Hz |
| `nav_status_aggregator` | `publish_rate` | 2 Hz（Nav2 参数轮询 ≥2 s） |
| `mission_bridge` | status 0.5 s / TF 1 s / feedback 0.5 s | 2 Hz / 1 Hz / 2 Hz |

这些节点在导航-only 测试时仍按上述频率运行；docking 五个节点合计约 80% 的
`ps` 平均值，但需要瞬时值确认。根因方向是“IDLE 时仍在做全量定时器/TF/参数工作”，
优先做状态门控或导航-only 不启动，而不是简单降控制频率。

## 4. 根因

- **2 m 容差本身已生效**：在线 `general_goal_checker.xy_goal_tolerance=2.0`，
  `docking_goal_checker` 保持 0.5/0.15。
- **卡滞根因（待验证/高度怀疑）**：`usv_map_odom_tf_node` 发布的是固定
  `map→odom`，但 `on_timer()` 用最近一帧 odom 的 `last_stamp_` 作为 TF 时间戳：

```cpp
tf.header.stamp = have_stamp_ ? last_stamp_ : this->now();
```

  在 CPU 高负载、odom 约 7.9 Hz 时，`last_stamp_` 比当前时间滞后 0.3–0.4 s；
  costmap `transform_tolerance=0.3` 不够，`controller_server` 的 `transformPose`
  抛外推异常并 abort。
- **mission_bridge 读错 YAML** 是独立问题：默认回退到
  `nav2_params_chengfeng.yaml`，导致 waypoint 容差与兴飞 Nav2 不同源；已修。
- `FollowPath.transform_tolerance` 在线 0.5、costmap 在线 0.6 未让当前任务恢复；
  需要确认 costmap 动态参数是否真正生效，且当前任务已经进入 abort/retry 循环，
  通常要 cancel/restart 才能干净恢复。

## 5. 已做修改

### 5.1 已落盘（船端 source + install，主机两仓）

- `nav2_params_xingfei.yaml`：`general_goal_checker.xy_goal_tolerance` 1.0 → **2.0**；
  `docking_goal_checker` 保持 0.5/0.15。
- `dock_mission/config/dock_mission.yaml`：`cruise_xy_goal_tolerance` 0.5 → **2.0**。
- `dock_mission/nav2_goal_checker.py`：`CRUISE_PROFILE.xy_goal_tolerance` 1.0 → **2.0**。
- `dock_mission/dock_mission_node.py`：`cruise_xy_goal_tolerance` 默认 1.0 → **2.0**。
- `workspace_nav/launch/nav2_boat.launch.py`：向 `mission_bridge.launch.py` 传
  `'nav2_params_path': params_file`，让 `waypoint_tolerance_m` 与 Nav2 实际 YAML 同源。
- 船端备份后缀：`.bak_20261019_2m`。

主机改动：

- `/home/ght/usv`：上述 5 个文件（含 launch）。
- `/home/ght/USV_NAV`：`nav2_params_xingfei.yaml`、`nav2_boat.launch.py`
  （工作仓没有 dock_mission 包）。
- **未提交/未推送**，按约定由用户手动 push。

### 5.2 在线修改（未落盘）

```bash
ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 2.0
ros2 param set /controller_server FollowPath.transform_tolerance 0.5
ros2 param set /local_costmap/local_costmap transform_tolerance 0.6
ros2 param set /global_costmap/global_costmap transform_tolerance 0.6
ros2 param set /mission_bridge nav2_params_path \
  /home/hkzl/usv-test/install/arm/workspace_nav/share/workspace_nav/config/nav2_params_xingfei.yaml
```

- `general_goal_checker` 和 `FollowPath` 的在线修改有动态回调依据；costmap 的 0.6
  在线值未确认在运行插件内生效。
- **注意**：`FollowPath.transform_tolerance` 和 costmap `transform_tolerance` 尚未写入
  YAML；重启后会回到 0.2 / 0.3。

### 5.3 控制频率保持 20 Hz（不降频）

2026-10-19 曾把 `controller_frequency` 20→10、`bt_loop_duration` 10→20、
`smoothing_frequency` 20→10 作为临时降负载方案；**用户明确否决：控制频率不能降，
必须找 CPU 根因**。上述三个值已全部回滚：

```yaml
controller_frequency: 20.0
bt_loop_duration: 10
smoothing_frequency: 20.0
```

船端从未部署过降频值（当时 SSH 已超时）。保留的降载动作只有：

- 船端 `local_costmap.voxel_layer.enabled: false`（deploy 仓 commit `9da7953`，
  待部署）——这是去掉重复点云消费，不是降控制频率；
- 主机导航仓 `/home/ght/USV_NAV` 保留 `enabled: true`，供独立导航局部避障使用。

## 6. 待验证 / 下一步（明天）

1. **先结束卡住的任务**（不要直接杀节点）：
   - 发布 `/gcs_mission/cancel`（`std_msgs/msg/Empty`），或调用
     `/mission_bridge/cancel_mission`（`m_common/srv/CancelMission`）；
   - 确认 `status_detail` 变成取消/失败，`waypoint_follower` 停止重试。
2. **TF 根因修复（推荐）**：
   - 修改 `~/usv-test/src/USV_NAV/usv_localization/src/usv_map_odom_tf_node.cpp`
     的 `on_timer()`，把固定 `map→odom` 的时间戳改成 `this->now()`（或改用
     `StaticTransformBroadcaster` 发 `/tf_static`）；重建 `usv_localization` 单包
     （新 build base，避免 `make pkgd-r` 的 GUARD_CACHE 清空共享 install）。
   - 同时把 `FollowPath.transform_tolerance` 0.5、local/global costmap
     `transform_tolerance` 0.6（或 0.8）写入 `nav2_params_xingfei.yaml` 作为兜底。
3. **重启并验证**：
   - 重启 `usv-test` 整栈（按现场脚本；supervisor 可能在任一 launch 退出时收整栈）；
   - `ros2 param get` 确认 2.0 / 0.5 / 0.6；
   - 下一单任务日志应出现
     `waypoint_tolerance_m: 2.00 m (自动 ← ...nav2_params_xingfei.yaml...)`；
   - `navigation.log` 不应再出现 `tf_help`、`Aborting handle`；应出现
     `Reached the goal!` / `Goal succeeded`。
4. **CPU / odom 现状**：
   - `mavros_node` 100% CPU、overall load 11.8、`odom_hz≈7.9`；
   - 具体根因排查见第 6 项；**不以降低控制频率或代价图频率代替根因分析**。
5. **归港相关**：`undock_unconfirmed_as_success` 在船端 install 仍为 `true`，
   后视相机恢复后必须改回 `false`（见同日另一篇记录）。
6. **CPU 根因排查（控制频率保持 20 Hz，不降频）**：
   - 先部署 `local_costmap.voxel_layer.enabled: false`（deploy 仓 commit `9da7953`），
     对比部署前后 `top` / `load average` / `Control loop missed`；
   - 查 `mavros_node` 100% CPU：`top -H -p <mavros_pid>`、`/mavros/imu/data` 与
     `/mavros/gps_input/local` 频率、MAVROS 插件/日志，确认是哪个线程/消息在烧 CPU；
   - 确认两个 `component_container_isolated` 分别对应 nav2 还是 perception
     AI/apriltag 容器（`ps -eo pid,args` + `ros2 node list`），查 32% 那个在跑什么；
   - 查 docking 空闲节点约 80% CPU：确认 IDLE 时定时器是否仍做全量 TF/参数工作，
     优先状态门控；导航-only 是否停掉 dock/docking 窗口由平台决定；
   - 查 `mission_bridge` / `nav_status_aggregator` / `decision` / `mqtt_bridgev2_n`
     的 CPU 来源；
   - 修复 `map→odom` 时间戳（§6.2）；控制频率保持 20 Hz，验证
     `Control loop missed its desired rate of 20.0000Hz` 是否消失。
7. **记录当前状态**：截至 2026-10-19 13:50（船端）/ 22:15（主机整理），任务仍
   `RUNNING`，船停，未取消未重启；按用户要求留到明天处理。
