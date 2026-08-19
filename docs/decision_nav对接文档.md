# 导航与上层状态机对接接口

> **发给上层状态机（decision）**。本文定义导航与上层之间的对外接口：上层以 **Service** 下发，导航以 **Topic** 反馈。  
> **冲突以导航已实现契约为准**；decision 有、导航尚无的能力标 **待实现**。  
> 实现：`mission_bridge` + `nav_status_aggregator`。

---

## 一、对接关系

### 1.1 已实现接口总览

```
上层状态机                          导航栈【已实现】
──────────                          ──────────────
/mission_bridge/send_waypoints  ──►  下发航线并立即执行
/mission_bridge/set_pause       ──►  暂停 / 继续
/mission_bridge/emergency_stop  ──►  急停
/mission_bridge/cancel_mission  ──►  取消任务 / 退出急停

/nav_status                     ◄──  周期状态快照（约 2 Hz）
/task_event                     ◄──  事件与告警（事件驱动）
```

| 方向 | 形式 | 说明 |
|------|------|------|
| 上层 → 导航 | **4 个 Service**【已实现】 | 同步返回 `success` / `message`（**仅受理**，≠ 到点） |
| 导航 → 上层 | **2 个 Topic**【已实现】 | `/nav_status` 看过程；`/task_event` 看终态与告警 |

> Service 调用即开始，无需额外「开始导航」指令。  
> 任务成败以 `/task_event` 的 `TASK_COMPLETED` / `TASK_FAILED` / `TASK_CANCELLED` 为准，辅以 `/nav_status.task.state`。

### 1.2 单点导航与多点导航：同一接口，只差列表长度【已实现】

本导航栈**原生支持多点航线**（底层 Nav2 `FollowWaypoints`），**没有单独的「单点服务」**。

| 上层语义 | 怎么发 | 说明 |
|----------|--------|------|
| **单点导航** | `waypoints` 长度 = **1** | 去一个目标点；到点后发 `TASK_COMPLETED` |
| **多点导航** | `waypoints` 长度 **≥ 2** | 按顺序逐点执行；全部到点后发 `TASK_COMPLETED` |

二者共用同一个 Service、同一套状态机与同一套反馈 Topic，**区别只是列表里有几个点**。

> 对照 decision：其 `/nav/way_point` 与 `/nav/mission_execute` 均映射到本接口（冲突以导航为准，不拆两套对外通道）。

### 1.3 飞控 GUIDED 与解锁【约定：导航自管｜实现：待实现】

| 项目 | 说明 |
|------|------|
| **约定** | 任务开始前由**导航**切 ArduPilot **GUIDED** 并 **arm**；结束/取消/急停后由导航 disarm 或交回 |
| **实现状态** | **待实现**（当前 `mission_bridge` 尚未内置 set_mode / arm） |
| **上层** | 正式联调前勿抢写 mavros 运动控制 / arming；过渡方案需双方确认 |

### 1.4 航点：经纬度 + 航向

| 项目 | 说明 |
|------|------|
| **上层下发** | WGS84 **`latitude` / `longitude`（度）** + **`yaw`（弧度，到点航向）** |
| **导航内部** | 转换为 map 坐标后交给 Nav2；不要求上层填 map x/y |
| **`yaw`** | 与 Nav2 map/ENU 航向一致；不需要指定朝向时填 `0` |

---

## 二、上层需要调用的 Service【已实现】

### 2.1 `/mission_bridge/send_waypoints` — 下发航线（单点 / 多点通用）

| 项目 | 要求 |
|------|------|
| **服务名** | `/mission_bridge/send_waypoints` |
| **类型** | `m_common/srv/SendWaypoints` |
| **航点类型** | `m_common/msg/MissionWaypoint` |
| **状态** | **已实现** |

**请求字段：**

```
m_common/MissionWaypoint[] waypoints   # 必填，非空；1 个=单点，≥2=多点
string mission_id                      # 任务 ID，原样出现在反馈中
string command_id                      # 指令 ID（可选）
```

**`MissionWaypoint` 字段：**

```
float64 latitude     # 纬度（度），(-90, 90)；禁止 (0, 0)
float64 longitude    # 经度（度），(-180, 180)
float64 yaw          # 到点航向（rad）；不需要朝向时填 0
```

**响应字段：**

```
bool success      # true=已受理并开始；false=拒绝
string message    # 原因说明
```

**行为：**

| 当前 `task.state` | 结果 |
|------------------|------|
| `IDLE` | 立即开始 → `RUNNING` |
| `RUNNING` / `PAUSED` | **自动抢占**旧任务，执行新航线（不发 `TASK_CANCELLED`，只发 `TASK_STARTED`） |
| `WAITING_SYSTEM` | `success=false`（系统未就绪） |
| `EMERGENCY` | `success=false`（须先 `cancel_mission`） |

**调用示例（单点）：**

```bash
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [{latitude: 31.4892, longitude: 120.3678, yaw: 1.57}], mission_id: 'goto_001'}"
```

**调用示例（多点）：**

```bash
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [
     {latitude: 31.4892, longitude: 120.3678, yaw: 0.0},
     {latitude: 31.4880, longitude: 120.3700, yaw: 1.57},
     {latitude: 31.4865, longitude: 120.3720, yaw: 3.14}
   ], mission_id: 'mission_001'}"
```

### 2.2 `/mission_bridge/set_pause` — 暂停 / 继续

| 项目 | 要求 |
|------|------|
| **服务名** | `/mission_bridge/set_pause` |
| **类型** | `m_common/srv/SetPause` |
| **状态** | **已实现** |

**请求 / 响应：**

```
bool pause          # true=暂停，false=继续
---
bool success
string message
```

| 操作 | 前置状态 | 行为 | 新状态 | 事件 |
|------|----------|------|--------|------|
| `pause: true` | `RUNNING` | cancel 当前 goal，**保存**剩余航点与 index | `PAUSED` | `TASK_PAUSED` |
| `pause: false` | `PAUSED` | 从断点恢复 | `RUNNING` | `TASK_RESUMED` |
| 其他组合 | — | `success=false`（幂等除外） | 不变 | — |

**调用示例：**

```bash
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: true}"
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: false}"
```

### 2.3 `/mission_bridge/emergency_stop` — 急停

| 项目 | 要求 |
|------|------|
| **服务名** | `/mission_bridge/emergency_stop` |
| **类型** | `m_common/srv/EmergencyStop` |
| **状态** | **已实现** |

**请求为空；响应 `bool success` + `string message`。**

- 任意非 `EMERGENCY` → 清空航点/缓冲，cancel goal → `EMERGENCY`，发 `EMERGENCY_STOP`
- 已 `EMERGENCY` → 幂等 `success=true`
- **不会自动回 IDLE**；退出急停须调 `cancel_mission`
- 导航急停**不**自动 disarm / 切模式（GUIDED/arm 自管落地后按导航约定处理）

**调用示例：**

```bash
ros2 service call /mission_bridge/emergency_stop m_common/srv/EmergencyStop
```

> decision 侧若带 `reason` 字段：导航当前无此字段，**冲突以导航为准**（空请求即可）。

### 2.4 `/mission_bridge/cancel_mission` — 取消 / 退出急停

| 项目 | 要求 |
|------|------|
| **服务名** | `/mission_bridge/cancel_mission` |
| **类型** | `m_common/srv/CancelMission` |
| **状态** | **已实现** |

**请求为空；响应 `bool success` + `string message`。**

| 当前状态 | 行为 | 新状态 | 事件 |
|----------|------|--------|------|
| `RUNNING` | 停导航、清航线 | `IDLE` | `TASK_CANCELLED` |
| `PAUSED` | 丢弃已保存断点 | `IDLE` | `TASK_CANCELLED` |
| `EMERGENCY` | 清除急停标志 | `IDLE` | — |
| `IDLE` | 幂等 | `IDLE` | — |
| `WAITING_SYSTEM` | `success=false` | 不变 | — |

**调用示例：**

```bash
ros2 service call /mission_bridge/cancel_mission m_common/srv/CancelMission
```

---

### 2.5 导航区域：作业区 / 禁止区 / 硬边界【已实现】

详见 `docs/作业区与电子围栏.md`。三个服务均由 `zone_manager` 节点提供：

| 项目 | 内容 |
|------|------|
| **服务名** | `/mission_bridge/set_nav_zones` |
| **类型** | `m_common/srv/SetNavZones`（区域 `m_common/msg/NavZones`，WGS84 经纬度） |
| **语义** | 全量替换：本次下发覆盖此前全部区域 |
| **配套** | `/mission_bridge/clear_nav_zones`（`std_srvs/srv/Trigger`，清除全部）；`/mission_bridge/get_nav_zones`（`m_common/srv/GetNavZones`，查询当前生效区域） |

- 作业区：闭合多边形，仅允许内部作业；禁止区：闭合多边形列表，不可进入；硬边界：不闭合折线列表，不可穿越。
- 生效手段：Nav2 KeepoutFilter 禁行层（规划不穿越）+ 航点下发预校验（越界航点拒绝）+ 运行时越界自动急停。
- 区域随 `zone_manager` 重启清空，需重新下发。

---

## 三、导航反馈给上层的 Topic【已实现】

### 3.1 `/nav_status` — 状态快照（主通道）

| 项目 | 要求 |
|------|------|
| **话题名** | `/nav_status` |
| **运行时类型** | `std_msgs/String`（UTF-8 JSON） |
| **字段 schema** | `m_common/msg/NavStatus`（与 JSON 字段一一对应，供上层对照；当前 Topic **仍发 String**） |
| **频率** | 约 2 Hz |
| **QoS** | **RELIABLE + TRANSIENT_LOCAL**，depth ≥ 10 |
| **状态** | **已实现** |

> 订阅端 QoS 必须与发布端一致（TRANSIENT_LOCAL），否则晚订阅拿不到最新状态。

**完整 JSON 结构：**

```json
{
  "schema_version": 1,
  "stamp": {"sec": 1716900000, "nanosec": 500000000},
  "vehicle_id": "usv_001",

  "task": {
    "state": "RUNNING",
    "task_id": "m001",
    "command_id": "cmd_a1b2c3",
    "nav_phase": "TRACKING",
    "current_waypoint": 2,
    "total_waypoints": 5,
    "progress_percent": 40.0,
    "elapsed_sec": 12.5,
    "distance_to_goal_m": 35.2,
    "eta_sec": null,
    "last_error": null
  },

  "planner": {
    "status": "OK",
    "last_plan_time_ms": 120.0,
    "last_error": null
  },

  "controller": {
    "status": "OK",
    "tracking_error_m": 0.15,
    "last_error": null
  },

  "localization": {
    "overall": "GOOD",
    "position_cov_max": 0.023,
    "orientation_cov_max": 0.001,
    "gps_fix": 3,
    "tf_ok": true,
    "odom_hz": 50.0
  },

  "pose": {
    "x": 12.34,
    "y": -5.67,
    "yaw": 1.57,
    "v": 0.5,
    "w": 0.0
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
    "controller_error": false,
    "emergency_active": false,
    "mission_paused": false
  },

  "recent_logs": [
    {"stamp": 1716900000.1, "level": "INFO", "node": "mission_bridge", "message": "Sending waypoint 3/5"}
  ]
}
```

**上层必看字段：**

| 路径 | 类型 | 说明 |
|------|------|------|
| `task.state` | string | 任务生命周期，**状态机只判断此字段** |
| `task.task_id` | string\|null | 对应下发的 `mission_id` |
| `task.command_id` | string\|null | 对应下发的 `command_id` |
| `task.nav_phase` | string | 辅助显示，**不做业务分支** |
| `task.current_waypoint` | int | 已完成航点数 |
| `task.total_waypoints` | int | 总航点数（单点时为 1） |
| `task.progress_percent` | float | 0.0 ~ 100.0 |
| `task.elapsed_sec` | float | 已运行秒数 |
| `task.distance_to_goal_m` | float | 到当前目标距离 (m) |
| `task.eta_sec` | float\|null | 预计剩余时间 |
| `task.last_error` | string\|null | 最新错误码 |
| `planner.status` | string | `"OK"` / `"FAILED"` |
| `controller.status` | string | `"OK"` / `"STUCK"` |
| `localization.overall` | string | `"GOOD"` / `"DEGRADED"` / `"LOST"` |
| `pose.x` / `pose.y` / `pose.yaw` | float | 当前 map 系位姿 |
| `pose.v` / `pose.w` | float | 线速度 (m/s)、角速度 (rad/s) |
| `alerts.*` | bool | 快速告警扫描 |

**`task.state` 取值：**

| 值 | 含义 |
|----|------|
| `WAITING_SYSTEM` | TF 或 Nav2 未就绪，此时 `send_waypoints` 会被拒 |
| `IDLE` | 空闲，可下发新任务 |
| `RUNNING` | 航线执行中 |
| `PAUSED` | 已暂停（不会自动回 IDLE） |
| `COMPLETED` | 全部到达（约 0.05s 后变 `IDLE`） |
| `FAILED` | 任务失败（约 0.05s 后变 `IDLE`） |
| `EMERGENCY` | 急停中（不会自动回 IDLE） |

**`nav_phase` 取值（仅辅助显示）：**  
`IDLE` / `TRACKING` / `STUCK` / `RECOVERY` / `PAUSED` / `EMERGENCY`

**`alerts` 字段：**

| 路径 | 含义 |
|------|------|
| `alerts.odom_stale` | odom 超时未更新 |
| `alerts.gps_stale` | GPS 超时未更新 |
| `alerts.mission_bridge_alive` | mission_bridge 心跳正常 |
| `alerts.planner_error` | `planner.status == "FAILED"` |
| `alerts.controller_error` | 控制器卡住/失败 |
| `alerts.emergency_active` | 急停中 |
| `alerts.mission_paused` | 暂停中 |

> **不要**解析 `recent_logs[].message` 做 if/else；用 `task.state`、`planner.status`、`alerts`、事件 `error_code`。  
> `RUNNING` 期间 `planner.status` 可能短暂为 `FAILED`（Nav2 内部重试），**勿仅凭 planner 失败判任务结束**。

### 3.2 `/task_event` — 事件（强烈建议订阅）

| 项目 | 要求 |
|------|------|
| **话题名** | `/task_event` |
| **运行时类型** | `std_msgs/String`（UTF-8 JSON） |
| **字段 schema** | `m_common/msg/NavTaskEvent`（与 JSON 字段一一对应；当前 Topic **仍发 String**） |
| **QoS** | RELIABLE，depth ≥ 50 |
| **触发** | 状态跳变 / 告警产生或解除（非周期） |
| **状态** | **已实现** |

**完整 JSON 结构：**

```json
{
  "schema_version": 1,
  "stamp": {"sec": 1716900000, "nanosec": 800000000},
  "vehicle_id": "usv_001",
  "task_id": "m001",
  "command_id": "cmd_a1b2c3",
  "event": "TASK_FAILED",
  "detail": {
    "error_code": "MISSION_FAILED",
    "failed_waypoint_index": 2,
    "reason": "Mission failed with error: PLAN_FAILED",
    "nav2_error_code": 0,
    "nav2_error_msg": ""
  }
}
```

**顶层字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `event` | string | 事件类型（见下表） |
| `task_id` | string | 关联任务 ID（=`mission_id`） |
| `command_id` | string | 关联指令 ID |
| `detail` | object | 事件载荷，随 `event` 变化 |

**事件类型：**

| `event` | 触发 | `detail` 关键字段 |
|---------|------|-------------------|
| `TASK_STARTED` | 开始执行 | `total_waypoints` |
| `TASK_COMPLETED` | 整条航线跑完（单点=到那一点；多点=全部到点） | `elapsed_sec` |
| `TASK_FAILED` | 任务失败 | `error_code`, `failed_waypoint_index`, `reason`, `nav2_error_code`, `nav2_error_msg` |
| `TASK_CANCELLED` | 外部取消 | `source` |
| `TASK_PAUSED` | 暂停 | `waypoint_index` |
| `TASK_RESUMED` | 恢复 | `waypoint_index` |
| `EMERGENCY_STOP` | 急停 | — |
| `ALARM_RAISED` | 告警触发 | `alarm_code`, `level`, `message`, `suggested_action` |
| `ALARM_CLEARED` | 告警解除 | `alarm_code` |

**告警 / 错误码：**

| `alarm_code` / `error_code` | level | 含义 |
|-----------------------------|-------|------|
| `PLAN_FAILED` | WARN | 规划找不到有效路径 |
| `CTRL_STUCK` | ERROR | 控制器超时无位移进展 |
| `LOC_LOST` | ERROR | 定位丢失 |
| `LOC_DEGRADED` | WARN | 定位精度下降 |
| `MISSION_FAILED` | ERROR | FollowWaypoints 整体失败 |

> 对照 decision 的 `arrived` / `unreachable` / `stuck`：导航对外统一用上表 `TASK_*`（冲突以导航为准）。

---

## 四、状态机与推荐流程【已实现】

### 4.1 状态转换

```
WAITING_SYSTEM ──(TF+Nav2 就绪)──► IDLE
IDLE ──(send_waypoints)──► RUNNING
RUNNING ──(全部到达)──► COMPLETED ──(≈0.05s)──► IDLE
RUNNING ──(失败)──► FAILED ──(≈0.05s)──► IDLE
RUNNING ──(cancel_mission)──► IDLE
RUNNING ──(set_pause true)──► PAUSED ──(set_pause false)──► RUNNING
PAUSED  ──(cancel_mission)──► IDLE
PAUSED / RUNNING ──(send_waypoints)──► RUNNING   # 抢占换线
任意非 EMERGENCY ──(emergency_stop)──► EMERGENCY
EMERGENCY ──(cancel_mission)──► IDLE
```

### 4.2 推荐使用流程

1. **启动前**：订阅 `/nav_status`（TRANSIENT_LOCAL）与 `/task_event`；确认 `task.state == IDLE`，且 `localization.overall != LOST`。
2. **飞控**：GUIDED + arm 由导航自管（**待实现**；过渡方案双方确认）。
3. **下发任务**：调 `send_waypoints`（1 点=单点，≥2=多点），检查 Response `success`；听 `TASK_STARTED` 或 `task.state → RUNNING`。
4. **运行中**：用 `/nav_status` 看 `progress_percent` / `current_waypoint`；用 `/task_event` 处理告警与终态。
5. **暂停/继续**：`set_pause`。
6. **急停**：`emergency_stop` → 之后必须 `cancel_mission` 才能再发航线。
7. **结束判定**：

| 结果 | `task.state` | `/task_event` |
|------|------------|---------------|
| 成功 | `COMPLETED`（随后 → `IDLE`） | `TASK_COMPLETED` |
| 失败 | `FAILED`（随后 → `IDLE`） | `TASK_FAILED` |
| 取消 | `IDLE` | `TASK_CANCELLED` |
| 暂停 | `PAUSED`（不自动恢复） | `TASK_PAUSED` |
| 急停 | `EMERGENCY`（不自动恢复） | `EMERGENCY_STOP` |

---

## 五、时序示例

```
上层 ──Service──► send_waypoints {waypoints: [P1, P2, P3], mission_id: "m001"}
        Response: {success: true, message: "Mission started: 3 waypoints..."}

导航 ──► /nav_status  task.state="RUNNING", total_waypoints=3
导航 ──► /task_event  TASK_STARTED { task_id:"m001", total_waypoints:3 }

        … 逐点执行，progress_percent 增长 …

上层 ──Service──► set_pause {pause: true}
导航 ──► /nav_status  task.state="PAUSED"
导航 ──► /task_event  TASK_PAUSED

上层 ──Service──► set_pause {pause: false}
导航 ──► /nav_status  task.state="RUNNING"
导航 ──► /task_event  TASK_RESUMED

        … 急停 …

上层 ──Service──► emergency_stop
导航 ──► /nav_status  task.state="EMERGENCY"
导航 ──► /task_event  EMERGENCY_STOP

上层 ──Service──► cancel_mission
导航 ──► /nav_status  task.state="IDLE"

上层 ──Service──► send_waypoints {waypoints: [P], mission_id: "goto_002"}   # 单点
导航 ──► /task_event  TASK_STARTED { total_waypoints:1 }
        …
导航 ──► /task_event  TASK_COMPLETED
导航 ──► /nav_status  task.state="COMPLETED" → "IDLE"
```

---

## 六、接口汇总

### 上层 → 导航（Service）【已实现】

| 接口 | 类型 | 用途 | 状态 |
|------|------|------|------|
| `/mission_bridge/send_waypoints` | `m_common/srv/SendWaypoints`（航点 `m_common/msg/MissionWaypoint`） | 下发航线（经纬度+航向；1 点=单点，≥2=多点） | **已实现** |
| `/mission_bridge/set_pause` | `m_common/srv/SetPause` | 暂停 / 继续 | **已实现** |
| `/mission_bridge/emergency_stop` | `m_common/srv/EmergencyStop` | 急停 | **已实现** |
| `/mission_bridge/cancel_mission` | `m_common/srv/CancelMission` | 取消 / 退出急停 | **已实现** |
| `/mission_bridge/set_nav_zones` | `m_common/srv/SetNavZones`（区域 `m_common/msg/NavZones`） | 全量设置作业区/禁止区/硬边界 | **已实现** |
| `/mission_bridge/clear_nav_zones` | `std_srvs/srv/Trigger` | 清除全部导航区域 | **已实现** |
| `/mission_bridge/get_nav_zones` | `m_common/srv/GetNavZones` | 查询当前生效导航区域 | **已实现** |

### 导航 → 上层（Topic）【已实现】

| 接口 | 运行时类型 | 字段 schema | 频率 | 用途 | 状态 |
|------|------------|-------------|------|------|------|
| `/nav_status` | `std_msgs/String` (JSON) | `m_common/msg/NavStatus` | ≈2 Hz | 任务状态快照（主通道） | **已实现** |
| `/task_event` | `std_msgs/String` (JSON) | `m_common/msg/NavTaskEvent` | 事件驱动 | 终态、暂停/恢复、急停、告警 | **已实现** |
| `/nav_zones/current` | `std_msgs/String` (JSON) | 见 `docs/作业区与电子围栏.md` | 变更时 | 当前生效导航区域（latched） | **已实现** |

> 可编译定义：`src/m_common/srv/{SendWaypoints,SetPause,EmergencyStop,CancelMission,SetNavZones,GetNavZones}.srv`，`src/m_common/msg/{MissionWaypoint,NavStatus,NavTaskEvent,GeoPolygon,NavZones}.msg`。

---

## 七、待实现（decision 有、导航暂无）

未完成前上层**勿依赖**。

| 能力 | decision 侧名称 | 状态 | 备注 |
|------|-----------------|------|------|
| 返航专用 | `/nav/return_home` | **待实现** | |
| 归港专用（坞点自存） | `/nav/return_dock` | **待实现** | |
| 只装载不执行 | `/nav/mission_upload` | **待实现** | 当前下发即执行 |
| Hold（停住可续） | `/nav/gcs_mission/cancel` (`NavHold`) | **待实现** | ≠ `cancel_mission`（会清任务） |
| 航点动作 | `Waypoint.actions`（HOVER 等） | **待实现** | |
| 结束动作 | `finish_action` | **待实现** | |
| 导航自管 GUIDED + arm | — | **待实现** | |
| 拆成四 Topic / `arrived` 事件词 | decision 4 Topic | **不采用** | 统一 `/nav_status` + `/task_event`（`TASK_*`） |

### 与 decision 命名对照（冲突→导航为准）

| decision | 导航（请用这个） |
|----------|------------------|
| `/nav/way_point` | `send_waypoints`（1 点） |
| `/nav/mission_execute` | `send_waypoints`（≥1 点） |
| `/nav/mission_control` pause/resume | `set_pause` |
| `/nav/mission_control` cancel | `cancel_mission` |
| `/nav/emergency_stop` | `/mission_bridge/emergency_stop` |
| `/nav/mission_status`、`/nav/nav_status` | `/nav_status` |
| `/nav/mission_event`、`arrived` 等 | `/task_event`（`TASK_*`） |

---

## 八、对接检查清单

| # | 项 |
|---|-----|
| 1 | 控制只用上述 4 个 Service；反馈订 `/nav_status` + `/task_event` |
| 2 | `/nav_status` 订阅 QoS = **TRANSIENT_LOCAL + RELIABLE** |
| 3 | 单点发 1 个航点、多点发 ≥2 个，**同一** `send_waypoints` |
| 4 | `success` 只表示受理；完成看 `TASK_*` |
| 5 | 急停后先 `cancel_mission` 再 `send_waypoints` |
| 6 | 业务判断用 `task.state`、`error_code`、`planner.status`，不用 `recent_logs` 原文 |
| 7 | 航点为 **经纬度 + yaw**（`m_common/msg/MissionWaypoint`），不发 map x/y |
| 8 | 勿依赖 §七 中标 **待实现** 的能力 |

---

## 九、反模式

| 不要 | 应该 |
|------|------|
| 为单点另找专用接口 | 同一 `send_waypoints`，列表长度=1 |
| 等「开始导航」额外指令 | `send_waypoints` 调用即开始 |
| `/nav_status` 用默认 VOLATILE 订阅 | TRANSIENT_LOCAL + RELIABLE |
| 用 `planner.status==FAILED` 直接判任务失败 | 等 `TASK_FAILED` 或 `task.state==FAILED` |
| 在 `EMERGENCY` 下直接发新航点 | 先 `cancel_mission` → `IDLE` 再发 |
| 把 Service `success` 当成「已到达」 | `success` 仅受理；到达看 `TASK_COMPLETED` |
| 要求上层发 map 系 x/y | 发 `latitude` / `longitude` / `yaw` |
| 把 `cancel_mission` 当成 Hold | Hold **待实现**；cancel 会清任务 |
| 依赖尚未实现的返航/归港专用服务 | 见 §七 |
