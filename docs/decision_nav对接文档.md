# 导航与上层 Decision 状态机对接接口

> **发给上层状态机（Decision）**。本文定义导航侧（`src/USV_NAV/workspace_nav`）对外的全部接口，以当前代码实际状态为准（2026-08-19 基线）。
> **Decision 的主任务接口是 `NavigateTask` Action**；Service 仅用于暂停/急停/围栏配置；反馈以 Action Feedback/Result + `/nav_status` 为准。
> 涉及节点：`mission_bridge`（任务）、`nav_status_aggregator`（状态）、`zone_manager`（围栏管理）、`zone_monitor`（运行时越界监控）。
> 与《decision_nav对接修改说明》（2026-08-19，下称「对接说明」）的对齐情况见 §九。

---

## 一、对接关系总览

### 1.1 接口一览

```
Decision                              导航栈
────────                              ──────────────
/mission_bridge/navigate        ──►   NavigateTask Action：单点 / 无动作航线（主任务接口）
/mission_bridge/set_pause       ──►   暂停 / 恢复（Service）
Action 标准 Cancel              ──►   取消当前任务
/mission_bridge/emergency_stop  ──►   急停（Service）
/mission_bridge/cancel_mission  ──►   退出 EMERGENCY（Service，仅此用途）
/mission_bridge/set_geofence    ──►   写入完整电子围栏快照（Service）
/mission_bridge/get_geofence    ──►   查询当前围栏快照（Service）

NavigateTask Feedback/Result    ◄──   航点进度 + 唯一任务终态
/nav_status                     ◄──   持续状态快照（m_common/msg/NavStatus，2 Hz）
/mission_bridge/safety_event    ◄──   无活动任务时的越界等异步安全事件
```

| 方向 | 形式 | 说明 |
|------|------|------|
| Decision → 导航 | **1 个 Action + 5 个 Service** | Action Goal 受理即开始执行，无需额外「开始导航」指令 |
| 导航 → Decision | **Action 内置通道 + 2 个 Topic** | Feedback 看过程；**Result 是唯一终态依据**；`/nav_status` 看持续状态；`/safety_event` 看无任务时的越界 |

> 任务成败**只以 Action Result 为准**。`/nav_status.task_state`、`/task_event` 均不能代替 Result（对接说明 §1.3）。

### 1.2 单点与无动作航线：同一 Action，只差列表长度

| `waypoints[]` 长度 | 语义 | 处理 |
|---:|---|---|
| 0 | 非法 | Goal 请求阶段拒绝（无 Result） |
| 1 | 单点导航 | 到点并满足停车条件后返回 `RESULT_SUCCESS` |
| ≥ 2 | 无动作航线 | 按数组顺序逐点执行，全部到点后返回 `RESULT_SUCCESS` |

- 导航不接收 `task_type`、速度、海拔、航点动作等字段；有动作航线由 Decision 拆成多个单点 Goal 下发。
- 返回 Home 不作为特殊任务：Decision 把 Home 坐标当普通单点发即可。

---

## 二、航点数据约定：`MissionWaypoint`

可编译定义：`src/m_common/msg/MissionWaypoint.msg`

```
float64 latitude     # WGS84 纬度，度，[-90, 90]
float64 longitude    # WGS84 经度，度，[-180, 180]
float64 yaw          # 到点航向，弧度（ENU/map 语义，与 Nav2 一致）
int32   seq          # 航点在原始航线中的序号
```

| 字段 | 约定 |
|------|------|
| `latitude` / `longitude` | 必须有限、在合法范围内；`(0, 0)` 视为非法并在 Goal 阶段拒绝 |
| `yaw` | 合法范围 **[-2π, 2π]**；**超出该范围的值（如 Decision 下发的 65536）= 不指定朝向**，导航取行进方向作为到点朝向（实现：`mission_bridge.py` `yaw_is_specified` / `YAW_UNSPECIFIED=65536.0`） |
| `seq` | 由 Decision 下发，**导航不重新编号，原样透传**到 Feedback 的 `current_seq`/`reached_seq` 和 Result 的 `final_current_seq`/`final_reached_seq` |

导航内部把经纬度转为 map 坐标后交给 Nav2 `FollowWaypoints`；Decision 不需要、也不应发送 map 系 x/y。

---

## 三、`NavigateTask` Action —— 主任务接口

可编译定义：`src/m_common/action/NavigateTask.action`

| 项目 | 内容 |
|------|------|
| **Action 名** | `/mission_bridge/navigate` |
| **类型** | `m_common/action/NavigateTask` |
| **节点** | `mission_bridge` |

### 3.1 接口定义

```text
# Goal
m_common/MissionWaypoint[] waypoints
---
# Result
int32 RESULT_SUCCESS=0
int32 RESULT_INVALID_GOAL=1
int32 RESULT_BUSY=2
int32 RESULT_NOT_READY=3
int32 RESULT_CANCELED=4
int32 RESULT_PLANNING_FAILED=5
int32 RESULT_CONTROLLER_FAILED=6
int32 RESULT_STUCK=7
int32 RESULT_LOCALIZATION_LOST=8
int32 RESULT_GEOFENCE_VIOLATION=9
int32 RESULT_EMERGENCY_STOP=10
int32 RESULT_INTERNAL_ERROR=99

int32 result
string error_code
string message
int32 final_current_seq
int32 final_reached_seq
---
# Feedback
uint8 PHASE_VALIDATING=1
uint8 PHASE_PLANNING=2
uint8 PHASE_TRACKING=3
uint8 PHASE_RECOVERY=4
uint8 PHASE_PAUSED=5

uint8 phase
int32 current_seq
int32 reached_seq
float32 distance_remaining_m
string message
```

### 3.2 Goal 处理规则

**Goal 请求阶段直接拒绝（不产生 Result）的情形：**

- `waypoints[]` 为空；
- 任一航点经纬度非有限数、超出 `[-90,90]`/`[-180,180]`、或为 `(0, 0)`；
- 系统未就绪（`WAITING_SYSTEM`，TF/Nav2 未起来）；
- 处于 `EMERGENCY`；
- 已有任务在执行或暂停（`RUNNING`/`PAUSED`，**不自动抢占**，Decision 须先 Cancel 旧 Goal）；
- 已有另一个活动 NavigateTask Goal；
- 航点违反电子围栏（下发阶段预校验，直接拒绝，不发越界事件）。

**接受后才发现运行条件失效**（如竞态下状态已变化）：返回 `RESULT_NOT_READY` / `RESULT_BUSY`（Action 状态 `ABORTED`）。

### 3.3 Feedback（2 Hz 周期发布）

| 字段 | 口径 |
|------|------|
| `phase` | `VALIDATING` → `PLANNING`（等待 Nav2 首个 feedback）→ `TRACKING`；暂停期间持续发 `PAUSED` 以证明 Goal 存活。`RECOVERY` 在定义中保留，当前实现不上报 |
| `current_seq` | 当前正在前往的航点 `seq`（Decision 下发的原值） |
| `reached_seq` | 最近一个已到点的航点 `seq`；尚未到达任何点时 `-1` |
| `distance_remaining_m` | **当前船位到当前目标航点的距离**（已按对接说明 §9.3 落实，见 `mission_bridge.py` `_nt_publish_feedback`） |
| `message` | 辅助字符串，勿解析 |

### 3.4 Result —— 唯一终态

- **每个被接受的 Goal 恰好返回一次 Result**（实现上由终止事件「先发先赢」保证，见 `_nt_terminate`）。
- 成功：单点须到点并满足停车条件；多点须全部到点 → `RESULT_SUCCESS`（Action 状态 `SUCCEEDED`）。
- 取消：`RESULT_CANCELED`（Action 状态 `CANCELED`）。
- 失败：`ABORTED` + 对应结果码与 `error_code`。

**结果码与触发场景：**

| `result` | `error_code` | 场景 |
|---|---|---|
| `RESULT_SUCCESS` (0) | `""` | 全部航点到达 |
| `RESULT_CANCELED` (4) | `CANCELED` | Decision 标准 Cancel，或老通道取消波及 |
| `RESULT_PLANNING_FAILED` (5) | — | Nav2 规划类失败（见下方注意） |
| `RESULT_CONTROLLER_FAILED` (6) | — | Nav2 控制/跟随类失败（见下方注意） |
| `RESULT_LOCALIZATION_LOST` (8) | — | 定位丢失看门狗终止任务 |
| `RESULT_GEOFENCE_VIOLATION` (9) | `GEOFENCE_VIOLATION` | 任务执行中越界：先停车，再 `ABORTED`（对接说明 §6.1） |
| `RESULT_EMERGENCY_STOP` (10) | `EMERGENCY_STOP` | 任务执行中收到急停 |
| `RESULT_NOT_READY` (3) / `RESULT_BUSY` (2) | `STATE_*` / `BUSY` | 接受后竞态下无法启动 |
| `RESULT_INTERNAL_ERROR` (99) | `INTERNAL_ERROR` | 其他无法归类的失败 |

`RESULT_INVALID_GOAL` (1)、`RESULT_STUCK` (7) 在定义中保留（非法请求在 Goal 阶段已拒绝、不产 Result；STUCK 当前实现不上报）。

> **注意：** `PLANNING_FAILED` 与 `CONTROLLER_FAILED` 的区分依赖 Nav2 错误消息关键词（Humble 的 FollowWaypoints Result 没有 error_code 字段，恒为 0；见 `_map_nav2_failure`）。**Decision 不要依赖这两者的精确区分**，需要精确原因时看 `message` 和 `/nav_status.last_error`。

### 3.5 Cancel 语义（标准 Action Cancel）

- Decision 用 ROS 2 Action 标准 Cancel 取消当前 Goal，**不再有自定义取消服务**用于任务取消。
- 导航收到 Cancel 后：先标记 `RESULT_CANCELED`，再安全停车（取消 Nav2 goal、清控制输出），随后返回 `CANCELED`。
- 暂停中（`PAUSED`）的 Goal 同样可被 Cancel。

### 3.6 调用示例

```bash
# 单点导航（带 feedback 观察）
ros2 action send_goal /mission_bridge/navigate m_common/action/NavigateTask \
  "{waypoints: [{latitude: 31.4892, longitude: 120.3678, yaw: 65536.0, seq: 0}]}" --feedback

# 无动作航线（多点）
ros2 action send_goal /mission_bridge/navigate m_common/action/NavigateTask \
  "{waypoints: [
     {latitude: 31.4892, longitude: 120.3678, yaw: 65536.0, seq: 0},
     {latitude: 31.4880, longitude: 120.3700, yaw: 1.57,    seq: 1},
     {latitude: 31.4865, longitude: 120.3720, yaw: 65536.0, seq: 2}
   ]}" --feedback
```

---

## 四、暂停 / 恢复 / 急停

### 4.1 `/mission_bridge/set_pause` — 暂停与恢复

| 项目 | 内容 |
|------|------|
| **类型** | `m_common/srv/SetPause`（`bool pause` → `bool success` + `string message`），定义：`src/m_common/srv/SetPause.srv` |

- `pause=true`（要求 `RUNNING`）：保存剩余航点与断点 index，取消当前 Nav2 goal → `PAUSED`；**不结束 Action Goal**，Feedback 持续发 `PHASE_PAUSED`。
- `pause=false`（要求 `PAUSED`）：从断点恢复 → `RUNNING`；**不创建新 Goal**。
- 重复暂停 / 重复恢复：幂等返回 `success=true`。
- `EMERGENCY` / `WAITING_SYSTEM` / `IDLE` 下调用：`success=false`。

### 4.2 `/mission_bridge/emergency_stop` — 急停

| 项目 | 内容 |
|------|------|
| **类型** | `m_common/srv/EmergencyStop`（空请求 → `bool success` + `string message`），定义：`src/m_common/srv/EmergencyStop.srv` |

- 任意非 `EMERGENCY` 状态：清空航点/断点/缓冲，取消 Nav2 goal → `EMERGENCY`；活动 NavigateTask Goal 收到 `ABORTED + RESULT_EMERGENCY_STOP`（若由越界触发则优先报 `RESULT_GEOFENCE_VIOLATION`）。
- 已 `EMERGENCY`：幂等 `success=true`。
- **不会自动回 IDLE**；退出急停必须调 `cancel_mission`。

### 4.3 `/mission_bridge/cancel_mission` — 仅用于退出 EMERGENCY

| 项目 | 内容 |
|------|------|
| **类型** | `m_common/srv/CancelMission`（空请求 → `bool success` + `string message`），定义：`src/m_common/srv/CancelMission.srv` |

- `EMERGENCY` → 清除急停标志 → `IDLE`（这是 Decision 侧对该服务的**唯一**用途）。
- 该服务历史上兼具「取消任务」能力（`RUNNING`/`PAUSED` → `IDLE`，会波及活动 Goal 使其返回 `RESULT_CANCELED`）；**Decision 取消任务请一律用 Action 标准 Cancel**，不要调本服务。

---

## 五、电子围栏（摘要）

详档见 `docs/作业区与电子围栏.md`，本节只写 Decision 需要的契约。围栏服务由 `zone_manager` 节点提供。

### 5.1 `/mission_bridge/set_geofence` — 写入完整围栏快照

| 项目 | 内容 |
|------|------|
| **类型** | `m_common/srv/SetGeoFence`（`m_common/GeoFence[] fences` → `bool success` + `string message`），定义：`src/m_common/srv/SetGeoFence.srv`、`src/m_common/msg/GeoFence.msg` |
| **语义** | **全量快照覆盖**（不用增量）；空数组 = 清除全部围栏；校验失败返回 `success=false` 并**保留上一次有效围栏**；成功保存后**本地持久化**（`workspace_nav/json/nav_zones.json`），重启自动恢复 |
| **调用时机** | 开机初始化一次（等待 Service 可用后调用）；之后仅在云平台更新围栏时再次调用 |

`GeoFence` 字段：`fence_id`（平台围栏 ID）/ `type`（`exclusion`=禁航区，不得进入；`inclusion`=作业区，不得驶出，**多个 inclusion 取并集**）/ `shape`（`polygon`：`points` 至少 3 个不重复点；`circle`：`center` + `radius_m > 0`）。

配套查询：`/mission_bridge/get_geofence`（`m_common/srv/GetGeoFence`）返回当前保存的快照。

### 5.2 围栏的生效手段

1. **任务下发预校验**：航点落在禁航区内或作业区外 → Goal 直接拒绝 / Service 返回 `success=false`，不发越界事件；
2. **Nav2 KeepoutFilter 禁行层**：围栏即代价地图，规划不穿越；
3. **运行时监控**（`zone_monitor`）：越界自动调 `emergency_stop` 急停（锁存，区域更新或回到合法区后解除）：
   - **有活动任务** → 任务 Goal 收 `ABORTED + RESULT_GEOFENCE_VIOLATION`（`error_code="GEOFENCE_VIOLATION"`），该 Result 是越界失败的唯一依据；
   - **无活动任务** → 发布 `/mission_bridge/safety_event`（`m_common/msg/NavSafetyEvent`，定义：`src/m_common/msg/NavSafetyEvent.msg`；`event_code="GEOFENCE_VIOLATION"`，`fence_type`=exclusion/inclusion，`transition`=ENTER/EXIT，`enabled`=true 触发 / false 解除）。

> **硬边界（不闭合折线，本平台扩展）**：对接说明未定义，经 GCS `/nav_zones` JSON 话题或旧接口下发；硬边界**不做航点预校验**（由规划器绕行），只进代价地图。

### 5.3 调用示例

```bash
# 写入一个禁航多边形 + 一个圆形作业区
ros2 service call /mission_bridge/set_geofence m_common/srv/SetGeoFence \
  "{fences: [
     {fence_id: 'excl_01', type: 'exclusion', shape: 'polygon',
      points: [{lat: 31.4890, lon: 120.3670}, {lat: 31.4890, lon: 120.3680},
               {lat: 31.4880, lon: 120.3680}], center: {lat: 0.0, lon: 0.0}, radius_m: 0.0},
     {fence_id: 'incl_01', type: 'inclusion', shape: 'circle',
      points: [], center: {lat: 31.4890, lon: 120.3680}, radius_m: 500.0}
   ]}"

# 清除全部围栏
ros2 service call /mission_bridge/set_geofence m_common/srv/SetGeoFence "{fences: []}"

# 查询当前快照
ros2 service call /mission_bridge/get_geofence m_common/srv/GetGeoFence
```

---

## 六、导航反馈 Topic

### 6.1 `/nav_status` — 持续状态快照（已按 Decision 要求落实）

| 项目 | 内容 |
|------|------|
| **类型** | **`m_common/msg/NavStatus`（强类型）**，定义：`src/m_common/msg/NavStatus.msg` |
| **频率 / QoS** | 2 Hz；**RELIABLE + TRANSIENT_LOCAL，depth=1**（对接说明 §1.3/§9.5） |
| **定位** | 只发布不依赖某个 Goal 的持续状态：导航就绪、定位、规划器/控制器、急停、是否有活动任务/暂停等；**不用于宣布任务终态** |

> 订阅端 QoS 必须匹配（TRANSIENT_LOCAL + RELIABLE），否则晚订阅拿不到最新快照。

关键字段（详见 msg 定义注释）：

| 字段 | 取值 / 说明 |
|------|------------|
| `task_state` | `WAITING_SYSTEM` / `IDLE` / `RUNNING` / `PAUSED` / `COMPLETED` / `FAILED` / `EMERGENCY`；`COMPLETED`/`FAILED` 为瞬态（约 0.05 s 后回 `IDLE`） |
| `task_id` / `command_id` | 对应下发 ID（老通道 Service 携带；NavigateTask 通道为空） |
| `nav_phase` | 辅助显示（`IDLE`/`TRACKING`/`STUCK`/`RECOVERY`/`PAUSED`/`EMERGENCY`），勿做业务分支 |
| `current_waypoint` / `total_waypoints` / `progress_percent` / `elapsed_sec` / `distance_to_goal_m` / `eta_sec` | 任务进度（`eta_sec` 未知为 -1） |
| `planner_status` / `controller_status` | `OK`/`FAILED`、`OK`/`STUCK`；RUNNING 中 planner 可能短暂 FAILED（Nav2 内部重试），**勿据此判终态** |
| `localization_overall` | `GOOD` / `DEGRADED` / `LOST` |
| `pose_x/y/yaw/v/w` | 当前 map 系位姿与速度 |
| `alerts_*` | 快速告警布尔（odom/gps 超时、planner/controller 错误、急停、暂停等） |
| `last_error` | 最新错误码，无错误为空 |

### 6.2 `/mission_bridge/safety_event` — 异步安全事件

`m_common/msg/NavSafetyEvent`，事件驱动，QoS RELIABLE depth=1。**只在无活动任务时**上报越界；有任务时越界由 Action Result 承担，不重复发。字段与取值见 §5.2 与 msg 定义。

### 6.3 `/task_event` — 保留给 GCS（Decision 勿依赖）

`std_msgs/String`（UTF-8 JSON，schema 对应 `m_common/msg/NavTaskEvent`），RELIABLE depth=50，事件驱动（`TASK_STARTED` / `TASK_COMPLETED` / `TASK_FAILED` / `TASK_CANCELLED` / `TASK_PAUSED` / `TASK_RESUMED` / `EMERGENCY_STOP` / `ALARM_*`）。该话题为 GCS 老通道保留；**Decision 的任务终态一律以 Action Result 为准**，勿解析本话题做终态判定。

---

## 七、状态机与时序

### 7.1 任务状态机（`mission_bridge`）

```
WAITING_SYSTEM ──(TF+Nav2 就绪)──► IDLE
IDLE ──(NavigateTask Goal 接受 / send_waypoints)──► RUNNING
RUNNING ──(全部到达)──► COMPLETED ──(≈0.05s)──► IDLE
RUNNING ──(失败/越界/失联)──► FAILED ──(≈0.05s)──► IDLE
RUNNING ──(Action Cancel)──► 停车 ──► IDLE
RUNNING ──(set_pause true)──► PAUSED ──(set_pause false)──► RUNNING
PAUSED  ──(Action Cancel)──► IDLE
任意非 EMERGENCY ──(emergency_stop / 越界)──► EMERGENCY
EMERGENCY ──(cancel_mission)──► IDLE
```

注意：**NavigateTask 通道不抢占**——`RUNNING`/`PAUSED` 时新 Goal 直接拒绝，须先 Cancel 旧 Goal。（`send_waypoints` 老通道仍有抢占语义，见 §8.2。）

### 7.2 时序示例（Action 流程）

```
Decision ──Action Goal──► /mission_bridge/navigate {waypoints: [P1(seq0), P2(seq1), P3(seq2)]}
        ◄── Goal Response: ACCEPT

导航 ──► Feedback  phase=VALIDATING → PLANNING → TRACKING
                   current_seq=0, reached_seq=-1, distance_remaining_m=…   （2 Hz）
导航 ──► /nav_status  task_state="RUNNING", total_waypoints=3

        … P1 到点：Feedback reached_seq=0, current_seq=1 …

Decision ──Service──► set_pause {pause: true}
        ◄── {success: true}
导航 ──► Feedback  phase=PAUSED（持续，证明 Goal 存活）
导航 ──► /nav_status  task_state="PAUSED"

Decision ──Service──► set_pause {pause: false}
导航 ──► Feedback  phase=TRACKING
导航 ──► /nav_status  task_state="RUNNING"

        … 全部到点 …

导航 ──► Result  result=RESULT_SUCCESS(0), final_current_seq=2, final_reached_seq=2
导航 ──► /nav_status  task_state="COMPLETED" → "IDLE"
```

取消与急停：

```
Decision ──Action Cancel──► 当前 Goal
导航   先安全停车 ──► Result  result=RESULT_CANCELED(4)（Action 状态 CANCELED）

任意时刻：
Decision ──Service──► emergency_stop
导航 ──► Result（如有活动 Goal） result=RESULT_EMERGENCY_STOP(10)（越界触发则为 9）
导航 ──► /nav_status  task_state="EMERGENCY"
Decision ──Service──► cancel_mission        # 退出急停
导航 ──► /nav_status  task_state="IDLE"
```

越界（无任务时）：

```
（船在 IDLE 下被风浪推出作业区）
导航 ──► /mission_bridge/safety_event  event_code="GEOFENCE_VIOLATION",
         fence_type="inclusion", transition="EXIT", enabled=true
（回到合法区或围栏更新后） enabled=false
```

---

## 八、兼容保留通道（非 Decision 新链路）

### 8.1 GCS 话题

`/waypoint` 等 GCS JSON 控制话题保留运行；GCS 围栏走 `/nav_zones` JSON 话题，回读 `/nav_zones/current`（latched）。与 Decision 链路并行互不影响，但同时使用时的任务抢占按先到先得处理。

### 8.2 `/mission_bridge/send_waypoints`（Service 老通道）

| 项目 | 内容 |
|------|------|
| **类型** | `m_common/srv/SendWaypoints`（`MissionWaypoint[] waypoints` + `mission_id` + `command_id` → `success`/`message`），定义：`src/m_common/srv/SendWaypoints.srv` |
| **语义** | 下发即执行；`RUNNING`/`PAUSED` 下**自动抢占**旧任务（与 NavigateTask 的拒绝语义不同）；同样的经纬度校验与围栏预校验 |
| **定位** | GCS/调试兼容保留；**Decision 新链路一律用 NavigateTask**。若活动任务是 NavigateTask Goal，被本服务抢占时该 Goal 收到 `ABORTED + RESULT_CANCELED`（`error_code="PREEMPTED"`） |

---

## 九、与《decision_nav对接修改说明》(2026-08-19) 的对齐情况

### 9.1 已按 Decision 要求落实

- `NavigateTask` Action（§3）：Goal 只收 `MissionWaypoint[]`；空/非法/未就绪/EMERGENCY/围栏违规/已有活动 Goal 一律拒绝；标准 Cancel（先停车再 CANCELED）；Result 唯一终态；Feedback 2 Hz 带 phase/seq/剩余距离。
- `MissionWaypoint.msg` 增加 `seq`，导航原样透传不重新编号。
- **`yaw` 哨兵**：超出合法范围 `[-2π, 2π]` 的值（含 Decision 下发的 65536）= 不指定朝向，取行进方向。Decision 可直接按 65536 下发。
- **`/nav_status`**：已切到 `m_common/msg/NavStatus` 强类型，QoS RELIABLE + TRANSIENT_LOCAL **depth=1**。
- **`distance_remaining_m` 口径**：当前船位到当前目标航点的距离（对接说明 §9.3）。
- `SetGeoFence`/`GetGeoFence`（§6.3/§6.4）：全量快照、空数组清除、校验失败保留旧值、polygon+circle、exclusion/inclusion、本地持久化。
- `NavSafetyEvent`（§6.2）：无活动任务时越界上报；有任务时由 Action Result（`RESULT_GEOFENCE_VIOLATION`）承担。
- 任务下发阶段违反围栏直接拒绝，不发越界事件（§6.4 末条）。

### 9.2 保留差异 / 未实施

| 项 | 状态 | 说明 |
|---|---|---|
| `ReturnDock` Action、`/mission_bridge/set_pre_dock_point` | **不在本项目，保留待实现** | 归港在另外的项目里，落地前 Decision 勿调用 |
| `/task_event` 终态上报 | 保留 | GCS 在用；Decision 以 Action Result 为唯一终态依据即可（对接说明 §8.13 是对 Decision 侧的要求） |
| `cancel_mission` 服务 | 保留 | 仅用于退出 EMERGENCY；任务取消用 Action 标准 Cancel |
| `send_waypoints` 服务、GCS `/waypoint` 话题 | 保留兼容 | 老通道行为不变（含抢占语义）；Decision 新链路不用 |
| 硬边界（不闭合折线） | 本平台扩展 | 对接说明未定义；经 `/nav_zones` JSON 或旧接口下发；不做航点预校验（规划器绕行） |
| 任务执行期间围栏热更新 | 与对接说明 §6.3「任务内固定快照」不同 | 本实现围栏即代价地图，更新立即对当前任务生效（更安全）；如需严格快照语义需再改 |
| `PLANNING_FAILED` / `CONTROLLER_FAILED` 精确区分 | 尽力而为 | 依赖 Nav2 错误消息关键词（Humble FollowWaypoints Result 无 error_code 字段），Decision 勿依赖精确区分 |

### 9.3 待 Decision / 云平台确认

1. **`seq` 从 0 还是 1 开始**（对接说明 §9.2）：导航只透传、不重新编号，两种均可；请与云平台确认后告知，联调时按统一口径验证 Feedback/Result 中的 seq。

---

## 十、接口汇总

### 10.1 Action（Decision 主任务接口）

| 接口 | 类型 | 用途 | 定义文件 |
|------|------|------|----------|
| `/mission_bridge/navigate` | `m_common/action/NavigateTask` | 单点/无动作航线；标准 Cancel；Result 唯一终态 | `src/m_common/action/NavigateTask.action` |

### 10.2 Service

| 接口 | 类型 | 用途 | 定义文件 |
|------|------|------|----------|
| `/mission_bridge/set_pause` | `m_common/srv/SetPause` | 暂停 / 恢复当前任务 | `src/m_common/srv/SetPause.srv` |
| `/mission_bridge/emergency_stop` | `m_common/srv/EmergencyStop` | 急停 | `src/m_common/srv/EmergencyStop.srv` |
| `/mission_bridge/cancel_mission` | `m_common/srv/CancelMission` | 退出 EMERGENCY（仅此用途） | `src/m_common/srv/CancelMission.srv` |
| `/mission_bridge/set_geofence` | `m_common/srv/SetGeoFence` | 写入完整围栏快照（全量覆盖，持久化） | `src/m_common/srv/SetGeoFence.srv`、`src/m_common/msg/GeoFence.msg` |
| `/mission_bridge/get_geofence` | `m_common/srv/GetGeoFence` | 查询当前围栏快照 | `src/m_common/srv/GetGeoFence.srv` |
| `/mission_bridge/send_waypoints` | `m_common/srv/SendWaypoints` | 【兼容】老通道下发航线（有抢占语义） | `src/m_common/srv/SendWaypoints.srv` |
| `/mission_bridge/set_nav_zones` / `get_nav_zones` | `m_common/srv/SetNavZones` / `GetNavZones` | 【兼容】旧区域接口（自动转围栏） | `src/m_common/srv/` |
| `/mission_bridge/clear_nav_zones` | `std_srvs/srv/Trigger` | 【兼容】清除全部围栏+硬边界 | — |

### 10.3 Topic（导航 → 上层）

| 接口 | 类型 | QoS | 频率 | 用途 |
|------|------|-----|------|------|
| `/nav_status` | `m_common/msg/NavStatus` | RELIABLE + TRANSIENT_LOCAL, depth=1 | 2 Hz | 持续状态快照（非终态） |
| `/mission_bridge/safety_event` | `m_common/msg/NavSafetyEvent` | RELIABLE, depth=1 | 事件驱动 | 无活动任务时的越界等安全事件 |
| `/task_event` | `std_msgs/String` (JSON) | RELIABLE, depth=50 | 事件驱动 | 【保留给 GCS】任务事件与告警，Decision 勿依赖 |
| `/nav_zones/current` | `std_msgs/String` (JSON) | latched | 变更时 | 【GCS】当前生效围栏+硬边界回读 |

消息定义：`src/m_common/msg/{MissionWaypoint,NavStatus,NavSafetyEvent,GeoFence,GeoPoint,NavTaskEvent}.msg`。

---

## 十一、对接检查清单

| # | 项 |
|---|-----|
| 1 | 任务下发只走 `/mission_bridge/navigate`（NavigateTask），单点 1 个航点、无动作航线 ≥2 个 |
| 2 | 终态判定只认 Action Result；`/nav_status`、`/task_event` 不作终态依据 |
| 3 | `/nav_status` 订阅 QoS = TRANSIENT_LOCAL + RELIABLE（depth=1 发布端） |
| 4 | 取消任务用 Action 标准 Cancel；`cancel_mission` 只用于退出 EMERGENCY |
| 5 | 已有活动 Goal 时新 Goal 会被拒：先 Cancel 再下发，不要指望抢占 |
| 6 | 急停后先 `cancel_mission` 回 `IDLE` 再发新任务 |
| 7 | 航点只发 `latitude/longitude/yaw/seq`；不指定朝向时 yaw 填超范围值（如 65536）；不发 map x/y |
| 8 | 围栏开机初始化一次、云平台更新时再发全量快照；空数组=清除；校验失败旧值仍在 |
| 9 | 不依赖 `PLANNING_FAILED`/`CONTROLLER_FAILED` 的精确区分；不解析 Feedback `message` 或日志字符串做分支 |
| 10 | 勿调用 `ReturnDock` / `set_pre_dock_point`（不在本项目，保留待实现） |

---

## 十二、反模式

| 不要 | 应该 |
|------|------|
| 为单点另找专用接口 | 同一 NavigateTask，`waypoints` 长度=1 |
| 等「开始导航」额外指令 | Goal 被接受即开始执行 |
| 用 `/task_event` 或 `/nav_status` 判任务终态 | 只认 Action Result |
| 用自定义 Service 取消任务 | Action 标准 Cancel；`cancel_mission` 仅退出 EMERGENCY |
| 指望新 Goal 抢占执行中的任务 | 先 Cancel 旧 Goal，再发新 Goal |
| 在 `EMERGENCY` 下直接发新 Goal | 先 `cancel_mission` → `IDLE` |
| 不指定朝向时把 yaw 填 0 | 0 是合法航向（朝东）；不指定请用超范围值（如 65536） |
| 修改/重编航点 `seq` 的预期 | `seq` 由 Decision 定义，导航原样透传 |
| 增量更新围栏 | 每次全量快照；空数组清除 |
| 依赖 `PLANNING_FAILED`/`CONTROLLER_FAILED` 精确区分 | 区分依赖 Nav2 错误消息关键词，仅供参考；需要原因看 `message` |
| `/nav_status` 用默认 VOLATILE 订阅 | TRANSIENT_LOCAL + RELIABLE |
| 用 `planner_status==FAILED` 判任务失败 | RUNNING 中可能短暂 FAILED；终态只看 Result |
| 解析 Feedback `message` / 日志做业务分支 | 用 `phase`、`result`、`error_code` 等结构化字段 |
| 要求上层发 map 系 x/y | 发 WGS84 `latitude`/`longitude` |
| 调用 `ReturnDock` / `set_pre_dock_point` | 不在本项目，归港保留待实现 |
