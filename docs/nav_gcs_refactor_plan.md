# 导航状态聚合 — 架构重构方案

> version 4.0 | 2026-05-27 | 第一阶段落地范围定稿

---

## 1. 背景与目标

当前 GCS 和导航之间的状态反馈只有一个粗粒度的 `/mission_bridge/state` (String, 5 个状态值)，且 GCS 内部维护了一套重复的状态机。这导致：

- 上层任务管理系统无法感知导航内部状态（正在规划/跟随/卡住/恢复中）
- 定位质量无从得知（协方差爆炸、GPS 丢星等）
- 任务失败时不知道原因（规划失败？控制失败？目标点在障碍层？）
- GCS 状态与导航状态可能不同步（靠 5 秒看门狗兜底）
- 上层只能轮询，无法感知"刚刚发生了什么"

**目标**：新增 `nav_status_aggregator` 观测节点，建立 **状态快照 + 事件反馈** 双通道输出，统一对 GCS 和上层任务管理系统提供服务。

## 2. 架构

### 2.1 三通道输出设计

```
                 nav_status_aggregator
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
  /nav_status      /task_event     /system_alarm
  (2Hz 快照)       (事件，即时)     (告警，即时)
  当前状态全貌      任务生命周期      严重异常通知
  轮询用           事件驱动用        告警用
```

| 通道 | Topic | 频率 | 用途 | 消费方 |
|------|-------|------|------|--------|
| 状态快照 | `/nav_status` | 2 Hz | 当前导航健康、位姿、任务进度 — 轮询读取 | GCS 前端、任务管理 |
| 任务事件 | `/task_event` | 事件触发 | 任务开始/完成/失败/取消/暂停 — "刚刚发生了什么" | 任务管理、日志记录 |
| 系统告警 | `/system_alarm` | 事件触发 | 严重异常（定位丢失、碰撞风险、EMERGENCY） | GCS 告警、任务管理 |

### 2.2 整体分层

```
┌─────────────────────────────────────────────────────────┐
│                 上层任务管理系统                          │
│                                                         │
│   订阅: /nav_status (轮询) + /task_event (事件)           │
│          + /system_alarm (告警)                          │
└──────────────────────┬──────────────────────────────────┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
   /nav_status    /task_event   /system_alarm
        │              │              │
┌───────┴──────────────┴──────────────┴──────────────────┐
│              nav_status_aggregator (新建)                │
│              纯观察者，不发送任何控制指令                   │
│                                                         │
│  健康判定: 规划失败 / 控制异常 / 定位退化 / 卡死 / 恢复    │
│  事件生成: 任务开始/完成/失败/取消                         │
│  心跳看门狗: odom / GPS / mission_bridge                 │
└────────┬──────────┬──────────────┬──────────────────────┘
         │          │              │
    ┌────▼───┐ ┌───▼────┐ ┌──────▼──────────┐
    │mission │ │ Nav2   │ │ 定位/EKF/GPS/TF │
    │_bridge │ │planner │ │                 │
    │Executor│ │control │ │ /odom /gps /tf  │
    └────────┘ └────────┘ └─────────────────┘
```

### 2.3 职责边界

| 节点 | 角色 | 做什么 | 不做什么 |
|------|------|--------|----------|
| `mission_bridge` | **Mission Executor** | 接收入站指令、坐标转换、状态机、驱动 Nav2 | 不对外上报详细状态 |
| `nav_status_aggregator` | **Status Reporter** | 订阅所有相关 topic、健康判定、三通道发布 | 不发送任何 goal/action |
| GCS `mission_service` | **Dispatch Proxy** | 把用户指令转发到 ROS、跟踪子进程生命周期 | 不维护任务状态机 |
| GCS `ros_subscriber` | **Telemetry Bridge** | 订阅高频遥测 + `/nav_status` + `/task_event` | — |

## 3. 对外接口 — 状态快照 `/nav_status`

### 3.1 基本信息

| 属性 | 值 |
|------|-----|
| Topic 名 | `/nav_status` |
| 消息类型 | `std_msgs/String` (JSON)，长期规划自定义 `NavStatus.msg` |
| QoS | RELIABLE + TRANSIENT_LOCAL, depth=10 |
| 发布频率 | 2 Hz |
| Publisher | `nav_status_aggregator` |

### 3.2 JSON Schema

```json
{
  "schema_version": 1,
  "stamp": { "sec": 1717000000, "nanosec": 123456789 },
  "vehicle_id": "usv_001",

  "task": {
    "state": "RUNNING",
    "task_id": "task_2026_0527_001",
    "command_id": "cmd_xxx",
    "nav_phase": "TRACKING",
    "current_waypoint": 2,
    "total_waypoints": 5,
    "progress_percent": 40.0,
    "elapsed_sec": 45.2,
    "distance_to_goal_m": 23.4,
    "eta_sec": 120,
    "last_error": null
  },

  "planner": {
    "status": "OK",
    "last_plan_time_ms": 350,
    "last_error": null
  },

  "controller": {
    "status": "OK",
    "tracking_error_m": 0.35,
    "last_error": null
  },

  "localization": {
    "overall": "GOOD",
    "position_cov_max": 0.15,
    "orientation_cov_max": 0.02,
    "gps_fix": 4,
    "tf_ok": true,
    "odom_hz": 29.5
  },

  "pose": {
    "x": 12.34,
    "y": 56.78,
    "yaw": 1.234,
    "v": 1.2,
    "w": 0.05
  },

  "flags": {
    "manual_override": false,
    "emergency_stop": false,
    "recovery_active": false
  },

  "alerts": {
    "odom_stale": false,
    "gps_stale": false,
    "mission_bridge_alive": true,
    "planner_error": false,
    "controller_error": false
  }
}
```

### 3.3 字段说明

#### task — 对外任务状态（与上层对齐）

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 任务状态：`IDLE` / `RUNNING` / `PAUSED` / `COMPLETED` / `FAILED` / `CANCELED` / `EMERGENCY` |
| `task_id` | string\|null | 上层分配的任务 ID |
| `command_id` | string\|null | 上层分配的本条指令 ID（防串台） |
| `nav_phase` | string | 导航内部阶段：`IDLE` / `PLANNING` / `TRACKING` / `RECOVERY` / `STUCK` |
| `current_waypoint` | int | 当前航点索引 (0-based) |
| `total_waypoints` | int | 总航点数 |
| `progress_percent` | float | 任务进度百分比 (0–100) |
| `elapsed_sec` | float | 已执行时间 |
| `distance_to_goal_m` | float | 距最终目标距离（米） |
| `eta_sec` | float\|null | 预计剩余时间 |
| `last_error` | string\|null | 最后一次错误码 |

**设计原则**：`task.state` 保持 7 个稳定值，与上层任务管理语义对齐。导航内部的 `PLANNING / TRACKING / RECOVERY / STUCK` 放在 `task.nav_phase` 里，不与任务状态混在一起。

#### planner

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `OK` / `FAILED` / `TIMEOUT` |
| `last_plan_time_ms` | float | 最近一次规划耗时（毫秒） |
| `last_error` | string\|null | 最后一次规划错误码 |

#### controller

| 字段 | 类型 | 说明 |
|------|------|------|
| `status` | string | `OK` / `FAILED` / `STUCK` |
| `tracking_error_m` | float | 当前跟踪误差（米） |
| `last_error` | string\|null | 最后一次控制错误码 |

#### localization

| 字段 | 类型 | 说明 |
|------|------|------|
| `overall` | string | `GOOD` / `DEGRADED` / `LOST` |
| `position_cov_max` | float | 位置协方差对角线最大值（m²） |
| `orientation_cov_max` | float | 姿态协方差对角线最大值（rad²） |
| `gps_fix` | int | 0=NO_FIX, 2=2D, 3=3D, 4=RTK_FIXED, 5=RTK_FLOAT |
| `tf_ok` | bool | map→base_link 变换是否可用 |
| `odom_hz` | float | 里程计实际接收频率 |

#### flags

| 字段 | 类型 | 说明 |
|------|------|------|
| `manual_override` | bool | 人工接管 |
| `emergency_stop` | bool | 急停激活 |
| `recovery_active` | bool | Nav2 恢复行为执行中 |

#### alerts — 心跳告警汇总

每个 bool 为 true 即表示该信号异常。

## 4. 对外接口 — 任务事件 `/task_event`

### 4.1 基本信息

| 属性 | 值 |
|------|-----|
| Topic 名 | `/task_event` |
| 消息类型 | `std_msgs/String` (JSON)，长期规划自定义 `TaskEvent.msg` |
| QoS | RELIABLE, depth=50（事件流不做 TRANSIENT_LOCAL，由消费端自行缓存历史） |
| 发布方式 | 事件触发，状态变迁时即时发布（不轮询） |
| Publisher | `nav_status_aggregator` |

> **QoS 说明**: `/nav_status` 适合 TRANSIENT_LOCAL（晚加入者需要最新快照），`/task_event` 是事件流，更适合 RELIABLE + depth=50，由 GCS 端缓存最近 N 条。

### 4.2 JSON Schema

```json
{
  "schema_version": 1,
  "stamp": { "sec": 1717000000, "nanosec": 123456789 },
  "vehicle_id": "usv_001",
  "task_id": "task_2026_0527_001",
  "command_id": "cmd_xxx",
  "event": "WAYPOINT_REACHED",
  "detail": {
    "waypoint_index": 2,
    "waypoint_total": 5,
    "position": { "x": 12.34, "y": 56.78 }
  }
}
```

### 4.3 事件类型枚举

| 事件 | 含义 | detail 携带信息 |
|------|------|----------------|
| `TASK_STARTED` | 任务开始执行 | task_id, total_waypoints, first_goal |
| `TASK_COMPLETED` | 所有航点到达 | task_id, elapsed_sec, final_pose |
| `TASK_FAILED` | 任务失败 | task_id, error_code, failed_waypoint_index, reason |
| `TASK_CANCELLED` | 任务被取消 | task_id, source (GCS/system) |
| `TASK_PAUSED` | 任务暂停 | task_id, current_waypoint |
| `TASK_RESUMED` | 任务恢复 | task_id, current_waypoint |
| `WAYPOINT_STARTED` | 开始驶向某航点 | waypoint_index, goal_position |
| `WAYPOINT_REACHED` | 到达某航点 | waypoint_index, elapsed_to_waypoint |
| `WAYPOINT_SKIPPED` | 航点被跳过（已在容差内） | waypoint_index, distance |
| `WAYPOINT_FAILED` | 某航点不可达 | waypoint_index, error_code, reason |
| `NAV_PHASE_CHANGED` | 导航内部阶段切换 | from_phase, to_phase (PLANNING→TRACKING 等) |
| `RECOVERY_STARTED` | 恢复行为触发 | recovery_type (spin/backup) |
| `RECOVERY_ENDED` | 恢复行为结束 | success (bool), next_action |
| `EMERGENCY_STOP` | 急停触发 | source, reason |
| `EMERGENCY_CLEARED` | 急停清除 | — |
| `ALARM_RAISED` | 告警激活（阶段 1 替代 /system_alarm） | alarm_code, level, message, suggested_action |
| `ALARM_CLEARED` | 告警清除 | alarm_code |

### 4.4 事件 vs 快照的使用场景

```
上层任务管理:

  轮询 /nav_status (2s 一次)          订阅 /task_event (事件驱动)
  ├── 刷新 UI 状态面板                ├── "任务开始了" → 记录日志，通知其他模块
  ├── 更新进度条                      ├── "航点 3 失败" → 触发异常处理流程
  └── 健康指示灯                      └── "急停触发" → 立即告警，暂停所有任务
```

## 5. 对外接口 — 系统告警 `/system_alarm`

### 5.1 基本信息

| 属性 | 值 |
|------|-----|
| Topic 名 | `/system_alarm` |
| 消息类型 | `std_msgs/String` (JSON)，长期规划自定义 `Alarm.msg` |
| QoS | RELIABLE + TRANSIENT_LOCAL, depth=10 |
| 发布方式 | 告警触发/清除时即时发布 |
| Publisher | `nav_status_aggregator` |

### 5.2 JSON Schema

```json
{
  "schema_version": 1,
  "stamp": { "sec": 1717000000, "nanosec": 123456789 },
  "vehicle_id": "usv_001",
  "alarm_id": "alarm_loc_lost_001",
  "level": "ERROR",
  "active": true,
  "code": "LOC_LOST",
  "message": "定位丢失: TF map→base_link 中断 3.2s, cov_max=15.3m²",
  "suggested_action": "检查 GPS/EKF 状态，确认传感器数据正常"
}
```

### 5.3 告警级别与类型

| 级别 | 触发条件示例 |
|------|------------|
| `WARN` | 定位 DEGRADED、GPS 降级 2D、规划耗时异常增长 |
| `ERROR` | 定位 LOST、规划连续失败、控制器 STUCK、mission_bridge 失联 |
| `FATAL` | EMERGENCY STOP 激活、碰撞风险、系统级故障 |

告警激活 (`active: true`) 和清除 (`active: false`) 各发一次。

## 6. 消息格式演进路线

```
阶段 1 (当前 → 稳定运行前):
  三通道都用 std_msgs/String(JSON)
  优点: GCS/上层无需编译 ROS msg，直接解析 JSON
  风险: 字段拼错、类型不一致运行时才发现

阶段 2 (稳定后):
  定义自定义 ROS 2 msg:
    workspace_nav/msg/NavStatus.msg
    workspace_nav/msg/TaskEvent.msg
    workspace_nav/msg/Alarm.msg
  优点: 编译期类型校验、ros2 topic echo 可读、多语言支持
  过渡方式: aggregator 同时发布 String(JSON) + 自定义 msg，消费者渐进迁移
```

## 7. mission_bridge/status_detail — 关键透传通道

aggregator 的 task_id / command_id / waypoint_index / mission_state / elapsed_sec 全部从 mission_bridge 透传，aggregator 不自己猜任务状态。

### 7.1 消息格式

Topic: /mission_bridge/status_detail
类型: std_msgs/String (JSON)
QoS: RELIABLE + TRANSIENT_LOCAL, depth=5
频率: 状态变迁时即时发布 + 2Hz 心跳保活

```json
{
  "state": "RUNNING",
  "task_id": "task_2026_0527_001",
  "command_id": "cmd_xxx",
  "waypoint_total": 5,
  "waypoint_completed": 2,
  "waypoint_current_index": 2,
  "elapsed_sec": 45.2,
  "error_code": null
}
```

### 7.2 发布时机

| 触发条件 | 发布内容 |
|----------|----------|
| _execute_mission_atomic 进入 RUNNING | state=RUNNING, task_id, command_id, total waypoints |
| _goal_result_cb 航点到达 | 更新 waypoint_completed, waypoint_current_index |
| _finish_all_waypoints_success | state=COMPLETED, elapsed_sec |
| _on_nav_fatal / _on_nav_failed | state=FAILED, error_code |
| _cb_mission_cancel | state=IDLE（取消后） |
| 定时 2Hz | 心跳保活（如果 RUNNING 则更新 elapsed_sec） |

### 7.3 mission_bridge 代码改动量

在现有 _transition_state() 调用处增加 _publish_status_detail() 调用，约 +35 行。


## 8. 错误码体系（分阶段实现）

### 8.1 第一阶段（必须实现）

**只做 5 个粗粒度错误码**，不涉及 costmap 查询等复杂诊断：

| 错误码 | 类别 | 触发条件 |
|--------|------|----------|
| `PLAN_FAILED` | 规划 | Nav2 ComputePathToPose 返回非 SUCCEEDED（含超时、无路径、goal 不可达等，不做细分） |
| `CTRL_STUCK` | 控制 | progress_checker 报告无进展 > 12s，或 FollowPath 返回失败 |
| `LOC_LOST` | 定位 | TF 断链 or position_cov_max > 10.0 m² or odom 超时 |
| `LOC_DEGRADED` | 定位 | position_cov_max > 1.0 m² or GPS fix < 3 or odom_hz < 20 |
| `MISSION_FAILED` | 任务 | FollowWaypoints goal 被拒或返回失败 |

### 8.2 第二阶段（后续增强）

在第一阶段稳定后，逐步增加细粒度诊断：

| 错误码 | 第一阶段的父类 | 额外需要的判定信息 |
|--------|--------------|------------------|
| `PLAN_GOAL_IN_OBSTACLE` | PLAN_FAILED | 查询 costmap 在目标坐标的值 |
| `PLAN_GOAL_OUTSIDE_MAP` | PLAN_FAILED | 判断目标坐标与 costmap 边界的关系 |
| `PLAN_NO_PATH` | PLAN_FAILED | 规划器搜索完成但无路径 |
| `CTRL_DEVIATED` | CTRL_STUCK | 计算跟踪误差是否 > 阈值 |
| `CTRL_OBSTACLE_BLOCKED` | CTRL_STUCK | 检查局部 costmap 前方区域 |
| `LOC_GPS_LOST` | LOC_LOST | GPS fix_type = 0 |
| `LOC_COV_EXPLODED` | LOC_LOST | cov_max 持续增大 |

### 8.3 错误日志流转

```
Nav2 / mission_bridge            aggregator                    GCS / 上层
─────────────────────            ──────────                    ──────────

① 异常发生                       ② 判定+分类                    ③ 接收反馈
  planner 返回 FAILED              第一阶段: PLAN_FAILED          /task_event:
  原因: goal in obstacle           第二阶段: PLAN_GOAL_IN_         event=TASK_FAILED
                                   OBS                           error_code=PLAN_GOAL_IN_
                                  │                              OBS
                                  │                              /nav_status:
                                  │  ④ 写入 ROS logger             task.last_error=...
                                  │  [WARN] planner FAILED        planner.last_error=...
                                  │  PLAN_GOAL_IN_OBSTACLE
                                  │  goal=(12.3,45.6)            ⑤ GCS 前端展示:
                                  │                               "规划失败: 目标点在障碍层"
                                  │  ⑥ 发布 /task_event +           "建议检查目标坐标"
                                  │     更新 /nav_status
```

**aggregator 内部 ROS 日志格式**:

```
[WARN] [nav_status_aggregator]: planner FAILED | goal=(12.3, 45.6)
[ERROR] [nav_status_aggregator]: controller STUCK | no_progress=12.5s last_move=0.15m
[WARN] [nav_status_aggregator]: localization DEGRADED | cov_max=2.3m² gps_fix=2 odom_hz=12.1
[ERROR] [nav_status_aggregator]: localization LOST | tf_broken=3.2s cov_max=15.3m²
[FATAL] [nav_status_aggregator]: EMERGENCY_STOP active | source=manual_override
```

### 8.4 错误恢复与清除

| 条件 | 行为 |
|------|------|
| 新任务启动 | 清空 `task.last_error`；发布 `TASK_STARTED` event |
| planner 下次规划成功 | `planner.status`→OK，`last_error` 清空 |
| controller 恢复跟踪 | `controller.status`→OK，`last_error` 清空 |
| 定位恢复 | `localization.overall`→GOOD；发布 alarm 清除 |
| EMERGENCY 清除 | 发布 `EMERGENCY_CLEARED` event + alarm 清除 |

## 9. 状态判定逻辑

### 9.1 定位健康

```
每收到 odometry 消息时判定:

cov_max = max(pose.covariance[0], pose.covariance[7], pose.covariance[14])

if tf_chain broken or odom_timeout > 5s:
    → LOST  (触发 /system_alarm level=ERROR)
elif cov_max > 10.0:
    → LOST  (LOC_COV_EXPLODED)
elif cov_max > 1.0 or gps_fix < 3 or odom_hz < 20:
    → DEGRADED  (触发 /system_alarm level=WARN)
else:
    → GOOD  (如有活跃告警则清除)
```

### 9.2 规划健康

```
通过监听 planner_server action status:

if action server 不可达:
    → FAILED, PLAN_SERVER_UNREACHABLE
elif last result != SUCCEEDED:
    → PLAN_FAILED（阶段 1）
    → PLAN_GOAL_IN_OBSTACLE / PLAN_GOAL_OUTSIDE_MAP / PLAN_NO_PATH（阶段 2）
else:
    → OK
```

### 9.3 控制器健康

```
通过 FollowPath action status + progress_checker:

if action server 不可达:
    → FAILED
elif progress_checker 报告无进展 > 12s:
    → STUCK  (发布 /system_alarm)
elif tracking_error > 5m:
    → STUCK
elif last result != SUCCEEDED:
    → FAILED
else:
    → OK
```

### 9.4 任务阶段推导 `nav_phase`

```
if mission.state == IDLE:
    nav_phase = "IDLE"
elif recovery 行为激活:
    nav_phase = "RECOVERY"
elif progress_checker 无进展:
    nav_phase = "STUCK"
elif FollowPath goal 已提交但未返回:
    nav_phase = "TRACKING"
elif ComputePathToPose goal 已提交但未返回:
    nav_phase = "PLANNING"
else:
    nav_phase = "IDLE"
```

### 9.5 心跳看门狗

| 信号源 | 超时阈值 | 超时后行为 |
|--------|----------|-----------|
| `/odometry/filtered` | 2s | `alerts.odom_stale=true`，定位降级为 LOST |
| `/gps/fixed_cov` | 5s | `alerts.gps_stale=true` |
| `/mission_bridge/status_detail` | 5s | `alerts.mission_bridge_alive=false`，发布 `/system_alarm` |
| aggregator 自身 | — | GCS 端检测 `/nav_status` 超过 5s 未更新 → stale |

## 10. 改动清单

### 10.1 导航侧 (wuxihik_navigation)

| 文件 | 操作 | 说明 |
|------|------|------|
| `workspace_nav/nav_status_aggregator.py` | **新建** | aggregator 完整实现，三通道发布 |
| `workspace_nav/mission_bridge.py` | **修改** | 新增 `_status_detail_pub` + `_publish_status_detail()`；传递 task_id/command_id |
| `launch/mission_bridge.launch.py` | **修改** | aggregator Node + launch args |
| `setup.py` | **修改** | entry_point 注册 |
| `msg/NavStatus.msg` | **阶段 2 新建** | 自定义消息 |
| `msg/TaskEvent.msg` | **阶段 2 新建** | 自定义消息 |
| `msg/Alarm.msg` | **阶段 2 新建** | 自定义消息 |

### 10.2 GCS 侧 (GROUND-CONTROL-STATION-dev)

| 文件 | 操作 | 说明 |
|------|------|------|
| `backend/server/ros_subscriber.py` | **修改** | 新增 3 个订阅：`/nav_status` + `/task_event` + `/system_alarm` |
| `backend/server/data_store.py` | **修改** | 新增 3 个数据存储 + JSON 解析 + stale 检测 |
| `backend/server/services/mission_service.py` | **修改** | 移除重复状态机 + 看门狗；仅保留 dispatch |
| `backend/server/api_server.py` | **修改** | 新增 `GET /api/nav_status` + `/api/task_events`；改造 `/api/mission_status` |
| `src/render/.../types/index.ts` | **修改** | 新增 TS 接口 |
| `src/render/.../components/MissionControls.tsx` | **修改** | 展示错误码；移除不一致警告 |
| `src/render/.../components/NavHealthIndicator.tsx` | **新建(可选)** | 健康指示灯 |

### 10.3 不需要改的文件

**导航侧**: `gps_map_conversion.py`, `waypoint_with_state.py`, `waypoint_transform.py`, `nav2_params.yaml`, `package.xml`

**GCS 侧**: `waypoint_publisher.py`, `cancel_publisher.py`, `color_code_publisher.py`, `config_service.py`, `settings.py`

## 11. 过渡策略

```
阶段 1: aggregator 部署，多轨运行
  ├── /nav_status        ← aggregator (新增)
  ├── /task_event        ← aggregator (新增)
  ├── /system_alarm      ← aggregator (新增)
  ├── /mission_bridge/state           (保留，不删)
  └── /mission_bridge/status_detail  ← mission_bridge (新增)

阶段 2: GCS 切换消费
  ├── 前端: 优先读 /nav_status + /task_event
  ├── /api/mission_status: 优先 /nav_status，stale 回退 /mission_bridge/state
  ├── /api/task_events: 缓存最近 N 条 /task_event
  └── mission_service: 保留旧逻辑但不写状态，兼容回滚

阶段 3: 稳定后清旧代码
  ├── GCS 移除 /mission_bridge/state 订阅
  ├── GCS mission_service 删除重复状态机
  └── (可选) 定义自定义 ROS msg，渐进迁移
```

## 12. GCS 前端展示设计

### 12.1 状态面板

```
┌─────────────────────────────────────────────┐
│  任务: RUNNING          进度: ████░░ 40%     │
│  ID: task_2026_0527_001  航点: 2/5          │
│  阶段: TRACKING          距目标: 23.4m       │
│  预计剩余: 2min          已用: 45s           │
└─────────────────────────────────────────────┘
```

### 12.2 错误/告警展示

```
┌─────────────────────────────────────────────┐
│  ⚠ 规划失败 — PLAN_GOAL_IN_OBSTACLE         │
│  目标点 (22.3456, 114.1234) 在障碍层内       │
│  建议: 检查目标坐标，确认不在陆地区域          │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│  导航健康                                    │
│  规划器: ❌ FAILED   控制器: ✅ OK            │
│  定位:   ⚠ DEGRADED (cov=2.3m², GPS 2D)    │
│  告警:   ⚠ LOC_DEGRADED (active)            │
└─────────────────────────────────────────────┘
```

### 12.3 事件时间线（建议新增）

```
[14:32:01] TASK_STARTED      — 任务开始，5 个航点
[14:32:15] WAYPOINT_REACHED  — 航点 1 到达
[14:33:02] WAYPOINT_REACHED  — 航点 2 到达
[14:33:45] NAV_PHASE_CHANGED — TRACKING → STUCK
[14:33:50] RECOVERY_STARTED  — spin 恢复
[14:34:02] RECOVERY_ENDED    — 恢复成功
[14:34:05] NAV_PHASE_CHANGED — STUCK → TRACKING
```

## 13. 风险与缓解

| 风险 | 缓解 |
|------|------|
| aggregator 崩溃导致状态丢失 | 不影响 mission_bridge 执行；GCS 5s stale 检测 + 回退旧接口 |
| JSON 格式版本不兼容 | `schema_version` 字段保障向前兼容；阶段 2 换自定义 msg |
| Nav2 action status topic 路径变动 | topic 名可配置，避免硬编码 |
| 过渡期 GCS 两套状态并存 | 优先 aggregator，stale 回退，逐步收敛 |
| 字段拼写/类型错误（JSON 阶段） | aggregator 内 schema 自检 + GCS 端宽松解析 + 日志警告 |
| task_id/command_id 串台 | aggregator 从 mission_bridge status_detail 透传，不做二次生成 |


---

## 14. 第一阶段落地范围（定稿）

| 交付物 | 内容 |
|--------|------|
| /nav_status (2Hz) | 状态快照：task + planner + controller + localization + pose + flags + alerts |
| /task_event (事件) | 15 种任务事件 + ALARM_RAISED / ALARM_CLEARED 告警事件 |
| /system_alarm | 阶段 2 再独立，阶段 1 告警走 /task_event |
| /mission_bridge/status_detail | mission_bridge 新增，透传 task_id/command_id/waypoint 进度 |
| 错误码 | 仅 5 个粗粒度：PLAN_FAILED / CTRL_STUCK / LOC_LOST / LOC_DEGRADED / MISSION_FAILED |
| 自定义 ROS msg | 阶段 2，阶段 1 全部用 String(JSON) |
| GCS 改动 | ros_subscriber + data_store + mission_service + api_server + 前端 |
| 过渡兼容 | /mission_bridge/state 保留，GCS 优先 /nav_status，stale 回退 |

**不纳入阶段 1**：
- PLAN_GOAL_IN_OBSTACLE 等细粒度错误（需要 costmap 查询）
- /system_alarm 独立 topic
- 自定义 .msg 文件
- 前端事件时间线组件（可选，后续加）

---

## 16. 实施记录 (2026-05-27)

### 导航侧 (wuxihik_navigation)

| 文件 | 改动 |
|------|------|
| `nav_status_aggregator.py` (**新建**) | 双通道节点 `/nav_status` (2Hz) + `/task_event` (事件)。订阅 `/mission_bridge/status_detail`、`/odometry/filtered`、`/gps/fixed_cov`、`/compute_path_to_pose/_action/status`、`/rosout`。定位/规划/控制健康判定，5 错误码，`recent_logs` 日志捕获（rosout + 自观测合成） |
| `mission_bridge.py` | +`/mission_bridge/status_detail` 发布，透传 task_id/command_id/waypoint 进度。修复 `_goal_result_cb`：检查 FollowWaypoints result 的 `missed_waypoints`，航点被跳过时标记 FAILED 而非 COMPLETED |
| `mission_bridge.launch.py` | aggregator Node 并列启动 + launch args |
| `setup.py` | entry_point 注册 |

### GCS 侧 (GROUND-CONTROL-STATION-dev)

| 文件 | 改动 |
|------|------|
| `data_store.py` | +`json` import，+`nav_status_data`/`task_events` 存储，+4 方法 |
| `ros_subscriber.py` | +`/nav_status` + `/task_event` 订阅 |
| `mission_service.py` | 状态机简化为 `{IDLE, DISPATCHING}`，删除 watchdog，`get_mission_status()` 改为 nav_status → mission_bridge_state 二级 fallback，透传 planner/controller/localization/alerts/recent_logs |
| `api_server.py` | +`GET /api/nav_status`，+`GET /api/task_events`，改造 `/api/mission_status` |
| `MissionControls.tsx` | 新增 Nav2 Log 面板（固定 240px 高度、自动滚底、自动换行），`nav_phase` 显示，失败时显示 error_panel |
| `WaypointEditor.tsx` | 修复 waypoint 地图只显示最后一个的 bug（缺 `id` 字段），修复 remove 传 index 而非 id，修复 TXT 导入返回值类型 |
| `types/index.ts` | 更新 `MissionStatus` 接口，新增 planner/controller/localization/alerts/recent_logs 字段 |
| `useMissionHandlers.ts` | 轮询间隔 2s → 1s |

### 未完成 / 待解决

- `/rosout` 订阅 QoS 匹配不稳定（`qos_profile_system_default` 待验证），当前同时依赖自观测 `_push_log` 合成日志
- `/system_alarm` 独立 topic 推迟到阶段 2
- 前端事件时间线组件未实现
