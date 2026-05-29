# 导航模块 — 上层任务系统对接说明

> **version: 4.1** | 供**第三方上层**（任务编排、竞赛主控等）对接本导航栈  
> 本文只写**对外契约**：Service 名、反馈 Topic、JSON 格式、状态机行为。  
> **上层控制一律走 Service**；**上层反馈一律订 Topic**（`/nav_status`、`/task_event`）。  
> 船端 launch 参数、里程计/GPS 话题名、YAML 配置属**船端集成**，不在本文。  
> **srv 源文件**：`src/m_common/srv/`（`SendWaypoints`、`SetPause`、`EmergencyStop`、`CancelMission`）

---

## 1. 对接关系

本导航栈**同时支持两路控制**，互不关闭、可并行存在：

| 角色 | 控制方式 | 反馈方式 |
|------|----------|----------|
| **第三方上层**（竞赛主控等） | 四个 **Service**（§5.1） | 订 **`/nav_status`**、**`/task_event`** |
| **地面站 GCS**（船东自用） | **Topic**（§5.2，经纬度 JSON 等） | 可订 `/nav_status`；可选 `/mission_bridge/state` |

```
第三方上层（Service 下发）              mission_bridge
──────────────────────              ────────
/mission_bridge/send_waypoints  ──►  map 坐标航线
/mission_bridge/set_pause       ──►  暂停 / 继续
/mission_bridge/emergency_stop  ──►  急停
/mission_bridge/cancel_mission  ──►  取消 / 退出急停

地面站 GCS（Topic 下发，保留）          mission_bridge
──────────────────────              ────────
/waypoint（可配）                 ──►  WGS84 JSON 航线（debounce/immediate/start_pulse）
/color_code（可配）               ──►  目标色 → target_buoy.json
/gcs_mission/cancel（可配）       ──►  取消 / 退出急停（与 cancel_mission 同逻辑）
/gcs_mission/start（可配）        ──►  仅 start_pulse 模式下的「开始」脉冲

上层 + GCS（Topic 订阅反馈）            nav_status_aggregator
──────────────────────              ────────
/nav_status                     ◄──  2Hz 状态快照（TRANSIENT_LOCAL）
/task_event                     ◄──  事件与告警
/mission_bridge/state（可选）     ◄──  GCS 粗粒度状态回退
```

- **上层请勿使用 GCS Topic 发任务**；**GCS 请勿依赖上层 Service**（各用各的入口即可）。
- **`cancel_mission` Service 与 `/gcs_mission/cancel` Topic 等效**，共用同一套取消/清急停逻辑；地面站继续用 Topic 即可，无需改 Service。
- 飞控解锁、OFFBOARD、速度桥等由**飞控/船端其它模块**处理，**不属于**本文导航对接范围。

### 1.1 内部架构（上层无需关心，仅供理解数据来源）

```
上层 ─── Service ───► mission_bridge ─── FollowWaypoints action ───► Nav2
GCS  ─── Topic ─────►      ▲
                          │  /mission_bridge/status_detail（内部）
                          ▼
              nav_status_aggregator
              发布: /nav_status + /task_event
```

aggregator 是纯观察者，**不上发任何控制指令**。

### 1.2 反馈消息的两种使用方式

导航反馈（`/nav_status`、`/task_event`）运行时以 **`std_msgs/String`（UTF-8 JSON）** 发布。上层有两种对接方式：

| 方式 | 订阅类型 | 解析 | 依赖 | 适用 |
|------|----------|------|------|------|
| **A. JSON 字符串**（当前） | `std_msgs/String` | `json.loads(msg.data)` 手动解析 | 零依赖 | GCS、快速对接 |
| **B. 编译型 msg**（可选） | `m_common/msg/NavStatus` 等 | ROS2 自动反序列化，字段安全 | 需 `find_package(m_common)` | 上层 C++/Python 节点 |

```python
# 方式 A（当前运行时）：订阅 String，手动解析 JSON
import json
from std_msgs.msg import String
sub = node.create_subscription(String, '/nav_status', lambda m: handle(json.loads(m.data)), 10)

# 方式 B（可选，如后续增加 typed topic）：直接使用编译型 msg
# from m_common.msg import NavStatus
# sub = node.create_subscription(NavStatus, '/nav_status', callback, 10)
```

> **当前**：`nav_status_aggregator` 只发布 `std_msgs/String`。`m_common/msg/NavStatus.msg` 和 `NavTaskEvent.msg` 是**已编译的类型定义**，字段与 §5.3/§5.4 的 JSON 结构一一对应，供上层做 schema 参考。上层对接用方式 A 即可（零依赖，不用装 `m_common`）。

JSON 字段说明见 `docs/msg/` 下 4 个文件；编译型定义见 `src/m_common/msg/NavStatus.msg`、`NavTaskEvent.msg`。

---

## 2. 坐标系统

| 入口 | 坐标 |
|------|------|
| **地面站** `/waypoint` JSON | WGS84 经纬度；导航栈内部转换为 map |
| **上层** `send_waypoints` Service | map 系 `PoseStamped`（`frame_id: map`） |

| 项目 | 值 |
|------|-----|
| 输入坐标 | WGS84 经纬度 `(latitude, longitude)`，如 `22.345678, 114.123456` |
| 投影方式 | ENU（默认），以 map.yaml 中 datum 锚点为原点 |
| 全局帧 | `map` |
| 车体帧 | `base_link` |

**注意**：`(0, 0)` 坐标、经度超 `[-180,180]`、纬度超 `[-90,90]` 会被拒绝，空数组不触发任务。

---

## 3. 地面站 GCS：要不要发「开始导航」？

> **本节仅适用于地面站 Topic `/waypoint`**，与上层 Service `send_waypoints` 无关（Service **调用即开始**，无 debounce/start_pulse）。

取决于 launch 参数 **`waypoint_command_mode`**（默认 **`debounce`**）。

| 模式 | GCS 发 `/waypoint` 后 | 是否还要发「开始」？ |
|------|----------------------|---------------------|
| **`debounce`**（launch 默认） | 写入航线；**静默约 0.45s**（无新 `/waypoint`）后**自动开始** | **不需要** |
| **`immediate`** | **立即**开始执行 | **不需要** |
| **`start_pulse`** | **只缓存**航线，**不**开走 | **需要**再发 **`/gcs_mission/start`**（`Empty`） |

**结论（GCS）**：

- **`debounce` / `immediate`**：只发 `/waypoint` JSON 即可。
- **`start_pulse`**：**先发 `/waypoint` → 再发 `/gcs_mission/start`**。

`start_pulse` 模式下若未配置 `mission_start_topic`，节点会启动失败；对接前必须向船端确认该话题名（默认约定为 `/gcs_mission/start`）。

> **注意**：以上模式仅适用于 Topic 接口 `/waypoint`。使用 **Service 接口 `send_waypoints`** 时无需关心 `waypoint_command_mode` —— Service 调用后立即执行。

---

## 4. 上层如何使用（推荐流程）

### 4.1 启动前

1. 订阅 **`/nav_status`**（QoS 必须为 **RELIABLE + TRANSIENT_LOCAL**，与发布端一致）。
2. （建议）订阅 **`/task_event`**，用于告警与状态跳变。
3. 确认 **`/nav_status.task.state`** 为 **`IDLE`**（若为 **`WAITING_SYSTEM`**，表示 TF 或 Nav2 未就绪，`send_waypoints` 会 `success=false`）。
4. （建议）确认 **`/nav_status.localization.overall`** 不为 `LOST`。

**`WAITING_SYSTEM` 说明**：启动后等待 TF（map→base_link）与 FollowWaypoints action（默认每 1s 检查）。运行中若 TF 或 action 丢失，状态会**回退**到 `WAITING_SYSTEM` 并取消当前 Nav2 goal。就绪后自动回到 `IDLE`。**仅在 `IDLE` 时下发新任务**（`RUNNING` 可用 `send_waypoints` 抢占换线）。

### 4.2 开始任务

**推荐方式（Service）**：

1. 调 **`/mission_bridge/send_waypoints`** Service（见 §5.1），传入 `PoseStamped[]` 航点。
2. 检查 Response `success`：`true` 表示航线已被接受，`false` 时读 `message` 获知原因。
3. 监听 **`/nav_status`**：`task.state` → `RUNNING`；或收 **`TASK_STARTED`** 事件。

**地面站方式（Topic，§5.2）** — 非上层接口，船东 GCS 仍可按原流程使用 `/waypoint` 等话题。

### 4.3 运行中（Service 行为摘要）

| Service | 请求 | 行为 | `/nav_status.task.state` | `/task_event` |
|---------|------|------|--------------------------|---------------|
| `set_pause` | `pause: true` | 仅 **`RUNNING`**：cancel 当前 goal，**保存**剩余航点与 index | → `PAUSED` | `TASK_PAUSED` |
| `set_pause` | `pause: false` | 仅 **`PAUSED`**：从断点恢复导航 | → `RUNNING` | `TASK_RESUMED` |
| `emergency_stop` | （空） | 任意非 `EMERGENCY`：清空航点/缓冲，cancel goal | → `EMERGENCY` | `EMERGENCY_STOP` |
| `send_waypoints` | 新航线 | **`RUNNING`/`PAUSED` 时自动抢占**旧任务（不触发 `TASK_CANCELLED`） | → `RUNNING` | `TASK_STARTED` |
| `cancel_mission` | （空） | `RUNNING`：停止并清航线；`PAUSED`：丢弃暂停进度；`EMERGENCY`：退出急停 | → `IDLE` | `TASK_CANCELLED` |

- 用 **`/nav_status`** 看 `progress_percent`、`planner.status`、`alerts`（**勿**解析 `recent_logs` 做分支）。
- 用 **`/task_event`** 的 `ALARM_RAISED` 处理中途规划/定位/卡住告警；**任务成败**以 `TASK_COMPLETED` / `TASK_FAILED` 为准。
- **`task.state` 仍为 `RUNNING` 时 `planner.status` 可能为 `FAILED`**（Nav2 内部重试），勿仅凭 planner 失败判任务结束。

### 4.4 取消与急停恢复

调 **`/mission_bridge/cancel_mission`**（`m_common/srv/CancelMission`）：

- **`RUNNING`**：取消 Nav2 goal，清空当前航线 → `IDLE`
- **`PAUSED`**：丢弃已保存断点 → `IDLE`
- **`EMERGENCY`**：清除急停标志 → `IDLE`（之后可再 `send_waypoints`）
- **`IDLE`**：幂等 `success=true`

### 4.5 取消 / 急停后再次出发

直接调 **`send_waypoints`**（无需 `explicit_replan`；Service 等价于强制换线）。

### 4.6 结束判定

| 结果 | `/nav_status.task.state` | `/task_event` |
|------|--------------------------|---------------|
| 成功 | `COMPLETED`（随后约 0.05s 变 `IDLE`） | `TASK_COMPLETED` |
| 失败 | `FAILED`（随后约 0.05s 变 `IDLE`） | `TASK_FAILED`，看 `detail.error_code` |
| 取消 | `IDLE` | `TASK_CANCELLED` |
| 暂停 | `PAUSED`（**不会自动回 IDLE**） | `TASK_PAUSED` |
| 急停 | `EMERGENCY`（**不会自动回 IDLE**） | `EMERGENCY_STOP` |

失败瞬间请以 **`TASK_FAILED`** 或当时 `/nav_status` 快照为准（`FAILED` 会很快回到 `IDLE`）。  
**PAUSED/EMERGENCY 不会自动恢复**，需显式操作（resume / cancel）。

---

## 5. 接口与消息（固定契约名）

### 5.1 Service 接口（上层 → 导航，推荐）

#### 5.1.1 `/mission_bridge/send_waypoints` — 下发航线

| 项目 | 说明 |
|------|------|
| 类型 | `m_common/srv/SendWaypoints` |
| 请求 | `geometry_msgs/PoseStamped[] waypoints` + `string mission_id` + `string command_id`（可选） |
| 响应 | `bool success` + `string message` |

**坐标**：航点使用 map 坐标系 PoseStamped（`frame_id: map`），由上层自行完成经纬度→map 转换（或使用船端提供的转换节点）。

**行为**：
- `WAITING_SYSTEM` → `success=false`（系统未就绪）
- `EMERGENCY` → `success=false`（须先 `cancel_mission`）
- `IDLE` / 刚结束的 `COMPLETED`/`FAILED`（已回 `IDLE`）→ 立即开始
- **`RUNNING` / `PAUSED` → 自动抢占**当前任务并执行新航线（**不**发 `TASK_CANCELLED`，仅 `TASK_STARTED`）
- `mission_id` / `command_id` 原样出现在 `/nav_status.task` 与 `/task_event`

**调用示例**（`ros2 service call`）：
```bash
ros2 service call /mission_bridge/send_waypoints m_common/srv/SendWaypoints \
  "{waypoints: [{header: {frame_id: map}, pose: {position: {x: 10.0, y: 20.0}, orientation: {w: 1.0}}}], mission_id: 'm001'}"
```

#### 5.1.2 `/mission_bridge/set_pause` — 暂停/继续

| 项目 | 说明 |
|------|------|
| 类型 | `m_common/srv/SetPause` |
| 请求 | `bool pause`（true=暂停，false=继续） |
| 响应 | `bool success` + `string message` |

- `pause=true` + `RUNNING` → 保存剩余航点，cancel goal，切 `PAUSED`
- `pause=false` + `PAUSED` → 从断点恢复，切 `RUNNING`
- 其他状态 → `success=false`

**调用示例**：
```bash
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: true}"
ros2 service call /mission_bridge/set_pause m_common/srv/SetPause "{pause: false}"
```

#### 5.1.3 `/mission_bridge/emergency_stop` — 急停

| 项目 | 说明 |
|------|------|
| 类型 | `m_common/srv/EmergencyStop` |
| 请求 | 无 |
| 响应 | `bool success` + `string message` |

- 任意非 `EMERGENCY` 状态 → cancel goal，清空所有航点/缓冲/timer，切 `EMERGENCY`
- 已 `EMERGENCY` → 幂等返回 `success=true`
- **`WAITING_SYSTEM` 也可急停**（进入 `EMERGENCY`，便于统一拉闸）
- **退出 EMERGENCY**：调 **`/mission_bridge/cancel_mission`**

**调用示例**：
```bash
ros2 service call /mission_bridge/emergency_stop m_common/srv/EmergencyStop
```

#### 5.1.4 `/mission_bridge/cancel_mission` — 取消 / 退出急停

| 项目 | 说明 |
|------|------|
| 类型 | `m_common/srv/CancelMission` |
| 请求 | 无 |
| 响应 | `bool success` + `string message` |

见 §4.4。

**调用示例**：
```bash
ros2 service call /mission_bridge/cancel_mission m_common/srv/CancelMission
```

---

### 5.2 地面站 GCS 专用 Topic（保留，与上层 Service 并行）

> **供船东地面站使用，逻辑与改 Service 前一致，未删减。**  
> 竞赛上层请用 §5.1 Service；地面站继续用本节 Topic，无需改为 Service。  
> GCS Topic **无同步 ACK**，是否接受任务请结合 `/nav_status` 或 `/mission_bridge/state` 观察。  
> 本仓库地面站实现与联调说明：[`GROUND-CONTROL-STATION-dev/docs/NAV_STACK_INTEGRATION.md`](../../GROUND-CONTROL-STATION-dev/docs/NAV_STACK_INTEGRATION.md)（路径相对本机工作区）。

默认话题名（均可通过 launch 参数 `waypoint_topic`、`color_topic`、`mission_cancel_topic`、`mission_start_topic` 修改）：

| 话题 | 类型 | 用途 |
|------|------|------|
| `/waypoint` | `std_msgs/String` | 下发 WGS84 航线 JSON |
| `/color_code` | `std_msgs/String` | 目标色 |
| `/gcs_mission/cancel` | `std_msgs/Empty` | 取消 / 退出急停 |
| `/gcs_mission/start` | `std_msgs/Empty` | 仅 `start_pulse` 时开始已缓存航线 |

#### 5.2.1 `/waypoint` — 下发航线（GCS）

> 坐标为 **WGS84 经纬度**（与 Service 的 map `PoseStamped` 不同）。

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/String` |
| 内容 | UTF-8 JSON |

**标准格式（推荐）**：

```json
{
  "waypoints": [
    {"latitude": 22.345678, "longitude": 114.123456},
    {"latitude": 22.345900, "longitude": 114.123800}
  ],
  "mission_id": "task_2026_0528_001",
  "command_id": "cmd_a1b2c3",
  "explicit_replan": true
}
```

| 字段 | 必填 | 说明 |
|------|------|------|
| `waypoints` | **是** | 经纬度列表；也支持 `[[lat,lon],...]` 或裸数组（见下方兼容格式） |
| `mission_id` | 否 | 任务 ID，原样出现在 `/nav_status.task.task_id` |
| `command_id` | 否 | 单次指令 ID，原样回显在 `/nav_status.task.command_id` |
| `explicit_replan` | 否 | `true`：强制换线；运行中换线、Cancel 后再跑通常需要 |

**兼容格式**（均等效，但不推荐混用）：

```json
// 格式 A：对象数组（推荐，语义清晰）
{"waypoints": [{"latitude": 22.3, "longitude": 114.1}]}

// 格式 B：二元组数组（紧凑）
{"waypoints": [[22.3, 114.1], [22.4, 114.2]]}

// 格式 C：裸数组（无外层 key，简单场景）
[{"latitude": 22.3, "longitude": 114.1}]
```

**校验规则**：`(0, 0)` 坐标拒绝；经纬度超限拒绝；空数组不触发任务。

#### 5.2.2 `/color_code` — 目标颜色（可选）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/String` |
| 内容 | `"#FF0000"` / `"red"` 等，供感知选靶 |

颜色映射：

| 输入 | 语义 |
|------|------|
| `"#FF0000"` / `"#ff0000"` / `"red"` | 红色目标 |
| `"#00FF00"` / `"#00ff00"` / `"green"` | 绿色目标 |
| `"#000000"` / `"black"` | 黑色目标 |

与航线独立；不发不影响导航执行。仅语义色变化时写 `target_buoy.json`（可配强制写入）。

#### 5.2.3 `/gcs_mission/cancel` — 取消 / 清除急停（GCS）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/Empty` |
| 作用 | RUNNING/PAUSED：停止任务、清缓冲；EMERGENCY：清除急停标志→IDLE；抑制后续被动航点（若 `suppress_passive_waypoints_after_cancel=true`） |

#### 5.2.4 `/gcs_mission/start` — 开始（仅 `start_pulse`）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/Empty` |
| 作用 | 执行已缓存的 `/waypoint` 航线 |

**`debounce` / `immediate` 模式下不要依赖此话题。**

---

### 5.3 `/nav_status` — 状态快照（导航 → 上层，**主通道**）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/String` (JSON) |
| 频率 | 约 2 Hz（可配 `publish_rate`，默认 2.0） |
| QoS | **RELIABLE + TRANSIENT_LOCAL**，depth≥10 |

**上层必订**；用于 UI、状态机、是否允许发下一任务等。TRANSIENT_LOCAL 保证上层任意时刻订阅都能立即获得最新状态。

#### 完整 JSON 结构

```json
{
  "schema_version": 1,
  "stamp": {"sec": 1716900000, "nanosec": 500000000},
  "vehicle_id": "usv_001",

  "task": {
    "state": "RUNNING",
    "task_id": "task_2026_0528_001",
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
    "controller_error": false
  },

  "recent_logs": [
    {"stamp": 1716900000.1, "level": "INFO",  "node": "mission_bridge",    "message": "Sending waypoint 3/5"},
    {"stamp": 1716900000.5, "level": "WARN",  "node": "planner_server",   "message": "GridBased: failed to create plan, no valid path found."}
  ]
}
```

#### 字段详解

**`task` — 任务状态**：

| 路径 | 类型 | 说明 |
|------|------|------|
| `task.state` | string | `WAITING_SYSTEM` / `IDLE` / `RUNNING` / `PAUSED` / `COMPLETED` / `FAILED` / `EMERGENCY`（§6.1） |
| `task.task_id` | string\|null | 与 `/waypoint` 下发的 `mission_id` 对应 |
| `task.command_id` | string\|null | 与 `/waypoint` 下发的 `command_id` 对应 |
| `task.nav_phase` | string | `IDLE` / `TRACKING` / `STUCK` / `RECOVERY` / `PAUSED` / `EMERGENCY`（辅助显示，§6.2） |
| `task.current_waypoint` | int | 已完成的航点数 |
| `task.total_waypoints` | int | 总航点数 |
| `task.progress_percent` | float | 0.0 ~ 100.0 |
| `task.elapsed_sec` | float | 任务已运行秒数 |
| `task.distance_to_goal_m` | float | 到当前目标点距离（米，Phase 2 完善） |
| `task.eta_sec` | float\|null | 预计剩余时间（Phase 2 完善） |
| `task.last_error` | string\|null | 最新错误码（§6.2） |

**`planner` — 规划器健康**：

| 路径 | 值 | 含义 |
|------|-----|------|
| `planner.status` | `"OK"` | 规划正常 |
| | `"FAILED"` | planner action 最近一次 ABORTED（目标可能在障碍物内或不在地图内） |
| `planner.last_error` | string\|null | `"PLAN_FAILED"` 等 |

**`controller` — 控制器健康**：

| 路径 | 值 | 含义 |
|------|-----|------|
| `controller.status` | `"OK"` | 正常 |
| | `"STUCK"` | 超过 12s（默认）无位移进展 |
| `controller.last_error` | string\|null | `"CTRL_STUCK"` 等 |

**`localization` — 定位健康**：

| 路径 | 值 | 判断依据 |
|------|-----|----------|
| `localization.overall` | `"GOOD"` | TF OK + cov < 阈值 + GPS fix ≥ 3 |
| | `"DEGRADED"` | cov 偏高 / GPS fix < 3 / odom 频率低 |
| | `"LOST"` | TF 丢失 / odom 超时 / cov 超阈值 |
| `localization.position_cov_max` | float | 位置协方差最大值 (m²) |
| `localization.gps_fix` | int | GPS 定位类型（0=无, 3=3D fix） |
| `localization.tf_ok` | bool | map→base_link TF 是否可用 |
| `localization.odom_hz`| float | 里程计实际频率 |

**`pose` — 船位（map 坐标系）**：

| 路径 | 类型 | 说明 |
|------|------|------|
| `pose.x` / `pose.y` | float | map 坐标 (m) |
| `pose.yaw` | float | 朝向角 (rad) |
| `pose.v` | float | 线速度 (m/s) |
| `pose.w` | float | 角速度 (rad/s) |

**`alerts` — 布尔告警（快速扫描用）**：

| 路径 | 含义 |
|------|------|
| `alerts.odom_stale` | odom 超时未更新 |
| `alerts.gps_stale` | GPS 超时未更新 |
| `alerts.mission_bridge_alive` | mission_bridge 心跳正常 |
| `alerts.planner_error` | 等同于 `planner.status == "FAILED"` |
| `alerts.controller_error` | 等同于 `controller.status == "STUCK"` |
| `alerts.emergency_active` | 急停激活中（`task.state == "EMERGENCY"`） |
| `alerts.mission_paused` | 任务暂停中（`task.state == "PAUSED"`） |

**`recent_logs[]` — 日志（仅展示）**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | float | UNIX 时间戳 |
| `level` | string | `INFO` / `WARN` / `ERROR` / `FATAL` |
| `node` | string | 来源节点（`planner_server`, `controller_server`, `mission_bridge` 等） |
| `message` | string | 日志原文 |

最多 200 条（环形缓冲）。**上层不应解析 `message` 做 if/else 分支**——用 `task.last_error`、`planner.status`、`alerts` 等结构化字段做业务判断。

---

### 5.4 `/task_event` — 事件（导航 → 上层，**强烈建议**）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/String` (JSON) |
| QoS | RELIABLE, depth=50 |
| 触发 | 事件驱动（状态转换、告警产生/解除） |

#### 完整 JSON 结构

```json
{
  "schema_version": 1,
  "stamp": {"sec": 1716900000, "nanosec": 800000000},
  "vehicle_id": "usv_001",
  "task_id": "task_2026_0528_001",
  "command_id": "cmd_a1b2c3",
  "event": "TASK_FAILED",
  "detail": {
    "error_code": "PLAN_FAILED",
    "failed_waypoint_index": 2,
    "reason": "Mission failed with error: PLAN_FAILED",
    "nav2_error_code": 0,
    "nav2_error_msg": ""
  }
}
```

| 顶层字段 | 类型 | 说明 |
|----------|------|------|
| `event` | string | 事件类型（见下表） |
| `task_id` | string | 关联任务 ID |
| `command_id` | string | 关联指令 ID |
| `detail` | object | 事件载荷，字段随 `event` 变化 |

#### 事件类型

| `event` | 触发条件 | `detail` 关键字段 |
|---------|----------|-------------------|
| `TASK_STARTED` | IDLE → RUNNING | `task_id`, `total_waypoints` |
| `TASK_COMPLETED` | → COMPLETED | `task_id`, `elapsed_sec` |
| `TASK_FAILED` | → FAILED | `error_code`, `failed_waypoint_index`, `reason`, `nav2_error_code`, `nav2_error_msg` |
| `TASK_CANCELLED` | RUNNING → IDLE（外部取消） | `task_id`, `source` |
| `TASK_PAUSED` | → PAUSED | `task_id`, `waypoint_index` |
| `TASK_RESUMED` | PAUSED → RUNNING | `task_id`, `waypoint_index` |
| `EMERGENCY_STOP` | → EMERGENCY | `task_id` |
| `ALARM_RAISED` | 告警触发 | `alarm_code`, `level` (`WARN`/`ERROR`), `message`, `suggested_action` |
| `ALARM_CLEARED` | 告警解除 | `alarm_code` |

**`ALARM_RAISED` detail 示例**：

```json
{
  "alarm_code": "PLAN_FAILED",
  "level": "WARN",
  "message": "Planner failed to find valid path — target may be in obstacle or outside map",
  "suggested_action": "检查目标航点坐标，确认不在障碍层内"
}
```

#### 告警码完整列表

| `alarm_code` | level | 含义 | `suggested_action` |
|--------------|-------|------|---------------------|
| `PLAN_FAILED` | WARN | 规划器找不到有效路径 | 检查目标航点坐标，确认不在障碍层内 |
| `CTRL_STUCK` | ERROR | 控制器超时无位移进展 | 检查推进器状态，确认无障碍物卡死 |
| `LOC_LOST` | ERROR | 定位丢失 | 检查 GPS/EKF 状态，确认传感器数据正常 |
| `LOC_DEGRADED` | WARN | 定位精度下降 | 检查 GPS 信号强度，确认定位数据质量 |
| `MISSION_FAILED` | ERROR | FollowWaypoints 整体失败 | 检查导航系统状态，确认任务参数正确 |

分支请用 **`detail.error_code`** / **`detail.alarm_code`**，不要解析 `message` / `suggested_action` 英文字符串。

---

### 5.5 `/mission_bridge/state` — 回退（可选）

| 项目 | 说明 |
|------|------|
| 类型 | `std_msgs/String` |
| 值 | `IDLE` / `RUNNING` / `PAUSED` / `COMPLETED` / `FAILED` / `EMERGENCY` / `WAITING_SYSTEM` |

仅当 **`/nav_status` 超过约 5s 无更新** 时作粗粒度回退；正常勿用其做主逻辑。

---

## 6. 状态、错误码与 Nav2 错误链路

### 6.1 `task.state` — 7 值状态机

| 值 | 含义 | 典型触发 |
|----|------|----------|
| `WAITING_SYSTEM` | 等待就绪 | TF 不可用或 FollowWaypoints action server 未就绪 |
| `IDLE` | 空闲 | 就绪、无任务 |
| `RUNNING` | 执行中 | 航点已提交 Nav2，正在导航 |
| `PAUSED` | 已暂停 | 上层调 `set_pause` {pause: true}；保存进度，可恢复 |
| `COMPLETED` | 全部到达 | 所有航点成功到达 |
| `FAILED` | 任务失败 | Nav2 FollowWaypoints 返回非 SUCCEEDED 或 missed_waypoints |
| `EMERGENCY` | 急停 | 上层调 `emergency_stop`；清空一切，需显式取消退出 |

**状态转换图**：

```
WAITING_SYSTEM ──(TF+action ready)──► IDLE
IDLE ──(新航点)──► RUNNING
RUNNING ──(全部到达)──► COMPLETED ──(0.05s)──► IDLE
RUNNING ──(航点失败)──► FAILED ──(0.05s)──► IDLE
RUNNING ──(外部取消)──► IDLE
RUNNING ──(pause)──► PAUSED ──(resume)──► RUNNING
PAUSED  ──(外部取消)──► IDLE
PAUSED  ──(新航点+explicit_replan)──► RUNNING
任意非EMERGENCY ──(emergency_stop)──► EMERGENCY
EMERGENCY ──(cancel topic)──► IDLE
IDLE/RUNNING/PAUSED ──(TF或action丢失)──► WAITING_SYSTEM ──(恢复)──► IDLE
```

### 6.2 `task.state` 与 `nav_phase`

- **`task.state`**：任务生命周期，**上层状态机应只判断此字段**。
- **`nav_phase`**：导航内部阶段（`IDLE` / `TRACKING` / `STUCK` / `RECOVERY`），**仅供辅助显示**，不做业务分支。

### 6.3 错误码（`task.last_error` / 事件 `detail.error_code`）

| 错误码 | 触发来源 | 含义 | 上层可做什么 |
|--------|----------|------|--------------|
| `PLAN_FAILED` | planner_server action ABORTED | 全局规划失败，目标可能在障碍物内或不在地图范围内 | 调整航点坐标 |
| `CTRL_STUCK` | 控制器超 12s（可配）无位移 | 机器人被卡住或无法跟踪路径 | 告警、人工介入 |
| `LOC_LOST` | TF 丢失 / odom 超时 / cov 超阈值 | 定位丢失 | 暂缓发新任务，等待恢复 |
| `LOC_DEGRADED` | cov 偏高 / GPS fix < 3 / odom 频率低 | 定位精度下降 | 暂缓发新任务 |
| `MISSION_FAILED` | FollowWaypoints 非 SUCCEEDED 或 missed_waypoints | 任务级失败 | 读 `TASK_FAILED.detail` 获取详情 |

### 6.4 Nav2 FollowWaypoints 错误 → 上层可见信号的链路

当 Nav2 内部发生错误时，错误信息通过**两条互补路径**到达上层：

**路径 A — aggregator 的 planner/controller 实时监控**：

```
planner_server FAILED (action ABORTED)
  │
  └─► /compute_path_to_pose/_action/status
        │
        └─► aggregator._cb_planner_status (检测 ABORTED)
              ├─► _push_log("WARN", "planner_server", "GridBased: failed to create plan...")
              │     └─► /nav_status.recent_logs[]  ← 前端 Nav2 Log 面板可见
              │
              ├─► _fire_alarm("PLAN_FAILED", ...)
              │     └─► /task_event ALARM_RAISED
              │
              └─► /nav_status.planner.status = "FAILED"
```

**路径 B — mission_bridge 的 FollowWaypoints result 回传**：

```
FollowWaypoints result (missed_waypoints + error_code + error_msg)
  │
  └─► mission_bridge._goal_result_cb
        │
        ├─► (运行中) /mission_bridge/status_detail → aggregator 更新 last_error
        │
        └─► (最终 FAILED) status_detail(state=FAILED, nav2_error_code=..., nav2_error_msg=...)
              │
              └─► aggregator._detect_mission_transition
                    ├─► _push_log("ERROR", "mission_bridge", "Nav2 FollowWaypoints error...")
                    │     └─► /nav_status.recent_logs[]
                    │
                    └─► /task_event TASK_FAILED
                          detail: { error_code, failed_waypoint_index, nav2_error_code, nav2_error_msg }
```

**关键**：`TASK_FAILED.detail` 中的 `nav2_error_code` 和 `nav2_error_msg` 来自 Nav2 FollowWaypoints action result 的原生字段，是**最权威**的错误来源。`failed_waypoint_index` 来自 `missed_waypoints[0].waypoint_index`。

---

## 7. 可选：路径与位姿可视化

**不属于**导航任务契约；若上层要做地图显示，向**船端集成方**索取实际话题名（随船而异），常见包括：

| 用途 | 常见话题名（以船端为准） |
|------|-------------------------|
| 规划路径折线 | `/plan` (`nav_msgs/Path`) |
| 船位与速度 | 船端定位输出的 odometry 话题 |
| 经纬度 | 船端 GPS 话题 |

任务成败**仍只看** `/nav_status` + `/task_event`，**不要**用「是否有 `/plan`」判断任务是否成功。

---

## 8. 时序示例

### 8.1 Service 方式（推荐）

```
上层 ──Service──► /mission_bridge/send_waypoints {waypoints: [...], mission_id: "m001"}
        Response: {success: true, message: "Mission started: 3 waypoints..."}

导航 ──► /nav_status  task.state="RUNNING"
导航 ──► /task_event  TASK_STARTED { task_id:"m001", total_waypoints:3 }

        … 逐点执行 …

上层 ──Service──► /mission_bridge/set_pause {pause: true}
        Response: {success: true, message: "Mission paused"}

导航 ──► /nav_status  task.state="PAUSED", nav_phase="PAUSED"
导航 ──► /task_event  TASK_PAUSED { task_id:"m001", waypoint_index:2 }

上层 ──Service──► /mission_bridge/set_pause {pause: false}
        Response: {success: true, message: "Mission resumed"}

导航 ──► /nav_status  task.state="RUNNING"
导航 ──► /task_event  TASK_RESUMED { task_id:"m001", waypoint_index:2 }

        … 急停 …

上层 ──Service──► /mission_bridge/emergency_stop
        Response: {success: true, message: "Emergency stop executed"}

导航 ──► /nav_status  task.state="EMERGENCY", flags.emergency_stop=true
导航 ──► /task_event  EMERGENCY_STOP { task_id:"m001" }

上层 ──Service──► /mission_bridge/cancel_mission
导航 ──► /nav_status  task.state="IDLE"
```

### 8.2 Topic 方式 debounce（默认：只发航点）

```
上层 ──► /waypoint (JSON)
        {
          "waypoints": [{"latitude":22.3,"longitude":114.1}, ...],
          "mission_id": "m001",
          "explicit_replan": true
        }
        （静默 ~0.45s，防抖）

导航 ──► /nav_status  task.state="RUNNING"
导航 ──► /task_event  TASK_STARTED { task_id:"m001", total_waypoints:3 }

        … 逐点执行，/nav_status.progress_percent 逐步增长 …
        （中途 planner 失败）
导航 ──► /task_event  ALARM_RAISED { alarm_code:"PLAN_FAILED" }
导航 ──► /nav_status  planner.status="FAILED", alerts.planner_error=true
        （Nav2 内部重试，/nav_status 中出现 recent_logs: [{level:"WARN", node:"planner_server",
         message:"GridBased: failed to create plan, no valid path found."}]）

        （最终 FollowWaypoints 返回 failed + missed_waypoints）
导航 ──► /task_event  TASK_FAILED {
          error_code:"MISSION_FAILED",
          failed_waypoint_index:2,
          nav2_error_msg:"..."
        }
导航 ──► /nav_status  task.state="FAILED", task.last_error="MISSION_FAILED"
        （0.05s 后）task.state="IDLE"
```

### 8.3 Topic 方式 start_pulse（发航点 + 开始）

```
上层 ──► /waypoint (JSON)     # 仅缓存，不执行
上层 ──► /gcs_mission/start   # Empty，真正开始
导航 ──► /nav_status  task.state="RUNNING"
导航 ──► /task_event  TASK_STARTED
        … 执行 …
```

### 8.4 Topic 方式取消后再跑

```
上层 ──► /gcs_mission/cancel  # Empty
导航 ──► /nav_status  task.state="IDLE"
导航 ──► /task_event  TASK_CANCELLED

上层 ──► /waypoint { ..., "explicit_replan": true }
导航 ──► /nav_status  task.state="RUNNING"
```

---

## 9. 对接检查清单（给第三方）

| # | 项 |
|---|-----|
| 1 | 已向船端确认 `waypoint_command_mode`（debounce / immediate / start_pulse）— 仅 Topic 方式需要，Service 方式无需 |
| 2 | 已订阅 `/nav_status`，QoS = TRANSIENT_LOCAL + RELIABLE |
| 3 | 已订阅 `/task_event`（建议） |
| 4 | 控制仅用四个 Service；反馈订 `/nav_status` + `/task_event` |
| 5 | 新任务在 `IDLE` 下发；`RUNNING` 换线用 `send_waypoints` 抢占 |
| 6 | 急停后须 `cancel_mission` 再 `send_waypoints` |
| 7 | 取消用 `cancel_mission`，勿依赖 GCS Topic |
| 8 | 业务逻辑用 `task.state`、`planner.status`、`error_code`，不用 `recent_logs` 原文 |
| 9 | Topic 方式坐标使用 WGS84 经纬度；Service 方式使用 map 坐标系 PoseStamped |

---

## 10. 船端内部（非上层接口）

以下内容由**船东 / USV_NAV 集成**在 launch 与 yaml 中配置，**第三方上层无需实现**：

- 里程计、GPS、地图 yaml、`params_file` 等
- `mission_bridge` 与 `nav_status_aggregator` 是否同机启动
- Nav2 参数与 `/plan`、`/cmd_vel_nav` 的 remap
- 坐标转换（经纬度 → ENU → map 坐标系）

集成说明见仓库内 `mission_bridge.launch.py`、`config/mission_stack.example.yaml`（**不发给上层厂商**）。

---

## 11. 附录：发给上层的文件清单

完整打包说明（必发 / 建议发 / 不要发）见 **[`UPPER_LAYER_DELIVERY.md`](UPPER_LAYER_DELIVERY.md)**。  
本地 Service 联调测试见 **[`test/README.md`](../test/README.md)**（**不要**发给上层）。

> `/nav_status`、`/task_event` 在 ROS 中为 **`std_msgs/String`**；`docs/msg/*.msg` 仅描述 JSON 结构，不是可编译 ROS 类型。

---

## 12. 反模式

| 不要 | 应该 |
|------|------|
| 假设必须发「开始导航」才走 | 先确认 §3 的模式；Service 方式无需关心 |
| 用 `/rosout` 判断任务状态 | `/nav_status` + `/task_event` |
| Cancel 后直接发航点且不带 `explicit_replan` | 带 `true` 或走 start_pulse；Service 方式自带抢占语义 |
| `/nav_status` 用默认 VOLATILE 订阅 | TRANSIENT_LOCAL + RELIABLE |
| 解析 `recent_logs[].message` 做 if/else | 用 `error_code`、`planner.status` |
| 依赖 `/mission_bridge/state` 做主逻辑 | 优先 `/nav_status`，仅在其陈旧（>5s）时回退 |
| 用「是否有 `/plan`」或「planner.status==FAILED」直接判定任务失败 | 等 `task.state==FAILED` 或 `TASK_FAILED` 事件 |
| 用 Topic 发航点后轮询确认是否被接受 | **用 Service 接口，直接读 Response success/message** |
| 在 EMERGENCY 状态下发新航点 | 先 `cancel_mission` 退出 EMERGENCY → IDLE |
