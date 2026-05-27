# 导航模块 — 任务管理对接接口文档

> version: 1.0 | 基于 `mission_bridge` 节点 (workspace_nav)

## 1. 架构概览

```
┌─────────────────────────────────────────────────────────┐
│ 上层任务管理 (GCS / 任务编排器)                          │
│                                                         │
│  输入 → [/waypoint] [/color_code]                       │
│        [/gcs_mission/start] [/gcs_mission/cancel]       │
│                                                         │
│  输出 ← [/mission_bridge/state]                         │
│        [Nav2 内部状态: BT status, planner/controller]    │
└──────────────┬──────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────┐
│ mission_bridge (ROS 2 Node)                             │
│                                                         │
│  · 经纬度 → map 坐标转换 (ENU/UTM)                       │
│  · 航点文件 (waypoints.json)                             │
│  · 任务状态机 (MissionState)                             │
│  · 逐点调 Nav2 FollowWaypoints action                   │
│  · 目标颜色感知对接 (target_buoy.json)                   │
└──────────────┬──────────────────────────────────────────┘
               │ FollowWaypoints action
┌──────────────▼──────────────────────────────────────────┐
│ Nav2 (bt_navigator + planner + controller + recovery)   │
└─────────────────────────────────────────────────────────┘
```

## 2. 导航接收的数据（上层 → 导航）

### 2.1 航点任务 `/waypoint`

**类型**: `std_msgs/String` (JSON)

**JSON 格式**:

```json
{
  "waypoints": [
    {"latitude": 22.345678, "longitude": 114.123456},
    {"latitude": 22.345900, "longitude": 114.123800}
  ],
  "mission_id": "task_2026_0527_001",
  "explicit_replan": true,
  "command": "start"
}
```

**字段说明**:

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `waypoints` | array | **是** | 航点列表，每个元素 `{latitude, longitude}` 或 `[lat, lon]` 二元组 |
| `mission_id` | string | 否 | 任务唯一标识；相同 hash 的重复航点会被拒绝（可配） |
| `explicit_replan` | bool | 否 | 强制重规划。任务运行中收到时 cancel 当前任务并执行新航线 |
| `command` | string | 否 | `"start"` / `"replan"` / `"restart"` — 等价于 `explicit_replan: true` |

**航点格式兼容**:

```json
// 方式 1：对象数组（推荐）
{"waypoints": [{"latitude": 22.3, "longitude": 114.1}]}

// 方式 2：二元组数组
{"waypoints": [[22.3, 114.1], [22.4, 114.2]]}

// 方式 3：纯数组（无外层 key）
[{"latitude": 22.3, "longitude": 114.1}]
```

**校验规则**:
- `(0, 0)` 坐标会被拒绝
- 经纬度超出 `[-90, 90]` / `[-180, 180]` 会被拒绝
- 空数组不触发任务

### 2.2 目标颜色 `/color_code`

**类型**: `std_msgs/String`

**格式**: 十六进制颜色码或语义色名

```
"#FF0000"   → red
"#00FF00"   → green
"#000000"   → black
"red"       → red
"green"     → green
"black"     → black
```

**行为**: 接收到颜色后写入 `target_buoy.json`；默认仅在颜色变化时写盘（可配 `target_buoy_force_rewrite:=true` 强制每次写）。

### 2.3 任务控制指令

#### 启动脉冲 `/gcs_mission/start`

**类型**: `std_msgs/Empty`

仅在 `waypoint_command_mode=start_pulse` 时生效。上层先通过 `/waypoint` 推送航线，再发空消息脉冲启动。

**典型流程**:
```
上层 → /waypoint          (推送航线，缓冲)
上层 → /gcs_mission/start (空消息，触发执行)
```

#### 取消任务 `/gcs_mission/cancel`

**类型**: `std_msgs/Empty`

取消当前运行中的任务，清空缓冲区。默认会抑制后续不带 `explicit_replan` 的被动 `/waypoint`（可配 `suppress_passive_waypoints_after_cancel:=false` 关闭）。

### 2.4 坐标系统

| 项目 | 值 |
|---|---|
| 输入坐标 | WGS84 经纬度 (latitude, longitude) |
| 投影方式 | ENU（默认），以 map.yaml 的 `ref_gnss_10` 为 datum |
| 输出坐标 | Nav2 map 坐标系 (x, y 米)，已应用 origin 平移与旋转 |
| 全局帧 | `map` |
| 车体帧 | `base_link` |

### 2.5 命令模式

通过参数 `waypoint_command_mode` 配置：

| 模式 | 行为 |
|---|---|
| `immediate` | 收到 `/waypoint` 立刻执行（默认） |
| `debounce` | 防抖模式，收到后等待 `waypoint_commit_delay_sec`（默认 0.45s）再执行 |
| `start_pulse` | 缓冲模式，收到 `/waypoint` 仅暂存，需 `/gcs_mission/start` 脉冲触发 |

## 3. 导航反馈的状态（导航 → 上层）

### 3.1 任务状态 `/mission_bridge/state`

**类型**: `std_msgs/String`

| 状态 | 含义 | 触发条件 |
|---|---|---|
| `WAITING_SYSTEM` | 系统等待就绪 | 启动中，等待 TF（map→base_link）和 FollowWaypoints action server |
| `IDLE` | 空闲 | 无任务、TF OK、action server OK |
| `RUNNING` | 任务执行中 | 航点已提交给 Nav2，正在导航 |
| `COMPLETED` | 任务完成 | 所有航点到达，waypoints.json 已清空 |
| `FAILED` | 任务失败 | Nav2 FollowWaypoints 返回非 SUCCEEDED 状态 |

**状态转换图**:

```
WAITING_SYSTEM ──(TF+action ready)──→ IDLE
IDLE ──(新航点)──→ RUNNING
RUNNING ──(全部到达)──→ COMPLETED ──→ IDLE
RUNNING ──(航点失败)──→ FAILED ──→ IDLE
任意状态 ──(TF丢失)──→ WAITING_SYSTEM
```

### 3.2 可暴露的 Nav2 内部状态（建议增强）

当前 `mission_bridge` 仅上报自身状态机的状态。以下 Nav2 原生机制可向上层暴露更细粒度的导航状态：

#### A. 规划失败

| 来源 | 话题/Action | 说明 |
|---|---|---|
| `planner_server` | `ComputePathToPose` action 失败 | 全局路径规划失败，返回非 SUCCEEDED |
| `controller_server` | `FollowPath` action 失败 | 路径跟踪失败（超时、卡住等） |
| `bt_navigator` | `/behavior_tree_log` | BT 节点执行日志，可观察到具体哪个节点失败 |

**建议反馈码**:
- `PLAN_FAILED` — 全局规划器找不到路径（如目标点被障碍物包围）
- `CONTROL_FAILED` — 控制器无法跟踪路径（速度/转向异常）
- `STUCK` — 进步检查器判定卡住（一段时间内移动距离 < 阈值）
- `RECOVERY_FAILED` — 恢复行为（如 Spin/BackUp）全部尝试后仍失败

#### B. 定位状态

| 来源 | 话题 | 说明 |
|---|---|---|
| TF 系统 | `tf2` can_transform check | map→base_link 不可用则退回 WAITING_SYSTEM |
| EKF | `/odometry/filtered` | 里程计方差过大、丢失 GPS 时协方差会增长 |
| AMCL | `/amcl_pose` | 粒子云分散 → 定位不确定 |

**建议反馈码**:
- `LOCALIZATION_LOST` — TF 丢失，map→base_link 不存在
- `LOCALIZATION_DEGRADED` — 协方差超过阈值，定位精度下降
- `LOCALIZATION_OK` — 定位正常

#### C. Nav2 Behavior Tree 状态

| 话题 | 类型 | 内容 |
|---|---|---|
| `/behavior_tree_status` | `BehaviorTreeStatusChange` | 当前运行的 BT 节点及状态（RUNNING/SUCCESS/FAILURE） |

### 3.3 建议的增强状态上报设计

如果上层任务管理需要完整的状态感知，建议在 `mission_bridge` 中新增一个结构化状态话题：

**建议新增 Topic**: `/mission_bridge/status` (自定义消息)

```yaml
mission_state: string        # WAITING_SYSTEM / IDLE / RUNNING / COMPLETED / FAILED
mission_id: string           # 当前任务 ID
current_waypoint: uint32     # 当前航点序号 (1-based)
total_waypoints: uint32      # 总航点数
nav_state: string            # PLANNING / FOLLOWING / STUCK / RECOVERING / IDLE
localization_state: string   # OK / DEGRADED / LOST
planner_healthy: bool        # planner_server 是否正常
controller_healthy: bool     # controller_server 是否正常
error_code: string           # 最后一次错误码 (PLAN_FAILED / CONTROL_FAILED / STUCK / RECOVERY_FAILED / NONE)
error_detail: string         # 可读错误详情
```

### 3.4 FollowWaypoints Action 的反馈

Nav2 的 `FollowWaypoints` action 本身提供逐点反馈：

```
# Feedback
uint32 current_waypoint    # 当前正在走向的航点索引

# Result
int32[] missed_waypoints   # 未成功到达的航点索引列表
```

`mission_bridge` 内部逐点调用 `FollowWaypoints`，每个航点的 goal 结果在 `_goal_result_cb` 中处理——**只有 status != SUCCEEDED 时触发 FAILED**。

## 4. 错误处理机制

### 4.1 mission_bridge 已有的错误处理

| 场景 | 行为 |
|---|---|
| waypoints.json 校验失败 | 记录 error，不启动任务 |
| FollowWaypoints server 不可用 | 记录 error，进入 FAILED |
| goal 被拒绝 | 记录 error，进入 FAILED |
| goal 返回非 SUCCEEDED | 记录 error+status，进入 FAILED（然后自动转 IDLE） |
| TF 不可用 | 退回 WAITING_SYSTEM，周期检查恢复 |
| 解析失败 | 记录 warning，丢弃消息 |
| 运行中收到重复任务 | 根据 `allow_replace_running_mission` 决定拒绝或 cancel 重跑 |
| 完成相同 hash 的航线 | 根据 `allow_repeat_identical_route` 决定忽略或重跑 |

### 4.2 上层任务管理应关注的错误

| 错误类型 | 判断方式 | 建议上层动作 |
|---|---|---|
| 规划失败 | `/mission_bridge/state` = FAILED 且 log 含 "plan" | 检查航点是否可达（陆地/障碍物），调整航点 |
| 航点不可达 | waypoint goal 返回 REJECTED | 调整目标点位置 |
| 导航卡住 | 长时间 RUNNING 但 current_index 不变 | 发 Cancel 后重试，或启用 recovery |
| 定位丢失 | `/mission_bridge/state` = WAITING_SYSTEM | 等待 GPS/EKF 恢复，或人工介入 |
| 执行超时 | RUNNING 超时未 COMPLETED | 上层超时检测 + Cancel |

## 5. 典型交互时序

### 5.1 标准任务流程 (immediate 模式)

```
上层                     mission_bridge              Nav2
 │                           │                        │
 │── /waypoint (JSON) ──────→│                        │
 │                           │── /mission_bridge/state │
 │                           │   = "RUNNING"          │
 │                           │── FollowWaypoints ────→│
 │                           │   (waypoint 1)         │
 │                           │←── result=SUCCEEDED ──│
 │   ← "COMPLETED" ─────────┤                        │
 │                           │── FollowWaypoints ────→│
 │                           │   (waypoint 2)         │
 │                           │←── result=SUCCEEDED ──│
 │   ← "COMPLETED" ─────────┤ waypoints.json 清空     │
```

### 5.2 中途取消流程

```
上层                     mission_bridge              Nav2
 │                           │                        │
 │── /gcs_mission/cancel ───→│                        │
 │                           │── cancel_goal_async ──→│
 │                           │── waypoints.json 清空  │
 │   ← "IDLE" ───────────────┤                        │
```

### 5.3 start_pulse 流程

```
上层                     mission_bridge              Nav2
 │                           │                        │
 │── /waypoint (JSON) ──────→│ (缓冲，不执行)          │
 │                           │ log: "staged X waypoints,
 │                           │       pulse to start"
 │── /gcs_mission/start ────→│                        │
 │                           │── /mission_bridge/state │
 │                           │   = "RUNNING"          │
 │                           │── FollowWaypoints ────→│
```

## 6. 可配置参数摘要

| 参数 | 默认值 | 说明 |
|---|---|---|
| `waypoint_topic` | `/waypoint` | 航点输入话题 |
| `color_topic` | `/color_code` | 目标颜色输入话题 |
| `odom_topic` | `/odometry/filtered` | 里程计话题 |
| `follow_waypoints_action` | `follow_waypoints` | Nav2 action 名称 |
| `waypoint_tolerance_m` | `1.5` | 航点抵达容差 (米) |
| `waypoint_command_mode` | `debounce` | immediate / debounce / start_pulse |
| `waypoint_commit_delay_sec` | `0.45` | debounce 模式静默窗口 |
| `mission_start_topic` | `""` | start_pulse 启动话题 (如 `/gcs_mission/start`) |
| `mission_cancel_topic` | `/gcs_mission/cancel` | 取消话题 |
| `allow_replace_running_mission` | `false` | 运行中是否允许新航点抢占 |
| `allow_repeat_identical_route` | `false` | 是否允许相同航线重跑 |
| `suppress_passive_waypoints_after_cancel` | `true` | Cancel 后抑制被动航点 |
| `global_frame` | `map` | TF 全局帧 |
| `robot_frame` | `base_link` | TF 车体帧 |
| `datum_source` | `map_yaml` | 坐标基准来源 |
| `projection` | `enu` | 投影方式 (enu/utm) |
| `debug_mode` | `false` | 打印原始消息负载 |
