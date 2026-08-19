# Decision 与导航节点对接说明（导航侧）

更新时间：2026-08-19

本文只保留导航节点需要实现和确认的内容。MQTT、云平台字段、用户 ID、载荷动作以及 Decision 内部任务编排不属于导航接口。

## 1. 全部接口总表

### 1.1 Decision 给导航的任务/控制只有 6 类

| 序号 | 业务含义 | ROS 接口 | 导航如何判断/处理 |
|---|---|---|---|
| 1 | 单点导航 | `/mission_bridge/navigate`，`NavigateTask` Action | `waypoints[]` 长度等于 1 |
| 2 | 无动作航线 | `/mission_bridge/navigate`，`NavigateTask` Action | `waypoints[]` 长度大于等于 2，按数组顺序连续执行 |
| 3 | 返回船坞 | `/mission_bridge/return_dock`，`ReturnDock` Action | 执行导航内部已有的返坞流程 |
| 4 | 暂停任务 | `/mission_bridge/set_pause`，`SetPause` Service | `pause=true`，暂停当前 Action |
| 5 | 取消任务 | 当前 Action 的标准 Cancel | 安全停止并将当前 Goal 置为 `CANCELED` |
| 6 | 恢复任务 | `/mission_bridge/set_pause`，`SetPause` Service | `pause=false`，恢复当前暂停的 Action |

导航不需要支持其他业务任务类型：

- 不需要单独的单点和多点接口，二者只通过 `waypoints[]` 长度区分。
- 不接收航点动作；拍照、云台、采水、悬停等动作由 Decision 自己执行。
- 不接收返回 Home 任务；Decision 会把 Home 坐标作为一个普通单点发给导航。
- 不接收 HOLD 任务。
- 不接收虚拟遥控；虚拟遥控由 `sub_decision` 自行处理。
- 不接收驶离船坞指令，而是接受decision发布的导航任务之后，自己判断当前是否在船坞内、与卡扣对接、离开船坞、单点导航。

### 1.2 Decision 调用的非任务配置接口

| Service | 类型 | 调用方 | 服务方 | 调用时机和用途 |
|---|---|---|---|---|
| `/mission_bridge/set_geofence` | `m_common/srv/SetGeoFence` | Decision | 导航 | 开机初始化一次；云平台围栏更新后再次写入完整围栏快照 |
| `/mission_bridge/set_pre_dock_point` | `m_common/srv/SetPreDockPoint` | Decision | 导航 | 开机初始化一次；云平台预泊点更新后再次写入最新预泊点 |

这两个 Service 都不属于第 1.1 节的任务/控制，只负责更新导航保存的配置，不会创建导航任务。

### 1.3 导航发布给 Decision 的接口

| 接口 | 类型 | 用途 | 能否作为任务终态 |
|---|---|---|---|
| `NavigateTask` Feedback/Result | Action 内置通道 | 上报航点进度以及成功、失败、取消 | 是，Result 是导航任务终态 |
| `ReturnDock` Goal/Result，可选 Feedback | Action 内置通道 | Goal 是否接受立即响应；返坞完成后再返回成功、失败或取消 Result | 是，Result 是返坞任务终态 |
| `/nav_status` | `m_common/msg/NavStatus` | 上报导航是否就绪、定位和传感器等持续状态 | 否 |
| `/mission_bridge/safety_event` | `m_common/msg/NavSafetyEvent` | 没有活动 Action 时，上报电子围栏越界等异步安全事件 | 否 |

不再使用 `/task_event` 重复上报任务完成、失败或取消，任务终态统一通过 Action Result 返回。

有任务发生电子围栏越界时，通过 Action内置通道 明确反馈围栏越界故障。
无任务发生电子围栏越界时，通过/mission_bridge/safety_event 明确反馈围栏越界故障。

## 2. 导航接收的航点数据

### 2.1 修改现有 `MissionWaypoint.msg`

复用 `m_common/msg/MissionWaypoint`，并将其字段修改为：

```text
float64 latitude
float64 longitude
float64 yaw
int32 seq
```

字段含义：

| 字段 | 含义 |
|---|---|
| `latitude` | WGS84 纬度，单位度，范围 `[-90, 90]` |
| `longitude` | WGS84 经度，单位度，范围 `[-180, 180]` |
| `yaw` | 到点航向，单位弧度，使用导航约定的 ENU/map 航向语义；不指定时填 `0` |
| `seq` | 航点在原始航线中的序号 |

`MissionWaypoint.msg` 只是 ROS 数组元素的结构，不是额外任务，也不会增加一次接口调用。Action Goal 中实际接收的是一个 `MissionWaypoint[]`。

### 2.2 单点和无动作航线的判断规则

| `waypoints[]` 长度 | 导航语义 | 处理结果 |
|---:|---|---|
| `0` | 非法请求 | 在 Goal 请求阶段拒绝，不开始导航 |
| `1` | 单点导航 | 到达该点并满足停车条件后返回成功 |
| `>= 2` | 无动作航线 | 按数组顺序执行，全部航点到达后返回成功 |

导航不需要判断原始航线是否包含动作。Decision 保证：只有无动作航线才会一次发送多个点；有动作航线会被 Decision 拆成多个独立的单点 Goal。

## 3. 单点导航/无动作航线 `NavigateTask.action`

建议定义为：

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

### 3.1 Goal 处理规则

- `waypoints[]` 为空时在 Goal 请求阶段拒绝；因为 Goal 没有被接受，所以不会产生 Result。
- 每个航点只读取 `latitude`、`longitude`、`yaw`、`seq`。
- 经纬度不是有限数、超出合法范围或为 `(0, 0)` 时在 Goal 请求阶段拒绝。
- `seq` 必须保持 Decision 下发的原值，导航不得重新编号。
- 导航未就绪时拒绝新 Goal；已经接受后才发现运行条件失效时，使用 `RESULT_NOT_READY` 结束。
- 已有活动 Goal 时拒绝新 Goal；旧任务由 Decision 先 Cancel，导航不自动静默抢占。

### 3.2 Feedback 规则

- `current_seq`：当前正在前往的航点 `seq`。
- `reached_seq`：最近一个已经满足到点条件的航点 `seq`；尚未到达任何点时填 `-1`。
- `distance_remaining_m`：当前点或整条剩余航线的剩余距离，双方联调时固定口径。
- 暂停期间继续发送 `PHASE_PAUSED` Feedback，证明 Goal 仍然存活。

### 3.3 Result 规则

- 每个被接受的 Goal 必须且只能产生一次 Result。
- 单点只有到点并满足停车条件后才能返回 `RESULT_SUCCESS`。
- 多点只有全部航点完成后才能返回 `RESULT_SUCCESS`。
- 被 Decision Cancel 的 Goal 返回 `RESULT_CANCELED`，Action 状态为 `CANCELED`。
- 导航自身故障导致任务失败时，Action 状态为 `ABORTED`，并返回对应结果码和稳定的 `error_code`。
- Topic 和 Action Feedback 都不能代替 Action Result。

## 4. 返回船坞 `ReturnDock.action`

返坞目标和船坞配置由导航内部管理时，Goal 不需要业务字段：

```text
# Goal
---
# Result
int32 RESULT_SUCCESS=0
int32 RESULT_BUSY=1
int32 RESULT_NOT_READY=2
int32 RESULT_CANCELED=3
int32 RESULT_DOCK_NOT_FOUND=4
int32 RESULT_APPROACH_FAILED=5
int32 RESULT_LOCK_FAILED=6
int32 RESULT_GEOFENCE_VIOLATION=7
int32 RESULT_EMERGENCY_STOP=8
int32 RESULT_INTERNAL_ERROR=99

int32 result
string error_code
string message
bool dock_locked
---
# Feedback
uint8 STATE_RUNNING=1
uint8 STATE_PAUSED=2

uint8 state
string message
```

导航只实现“返回并进入船坞”，不实现驶离船坞。

返回船坞可能持续较长时间，而且中间没有可用于计算航点百分比的航点，因此：

- Decision 发送 Action Goal 后，导航必须尽快返回“接受”或“拒绝”，不能等返坞完成才响应 Goal。
- Goal 被接受后保持 `EXECUTING`，实际进入船坞并确认成功后再返回最终 Result。
- `ReturnDock` 不要求航点序号、剩余距离或百分比进度。
- Feedback 是可选的。导航可以只在运行、暂停等状态变化时发布，也可以低频发送心跳；没有可用进度时不需要虚构进度值。
- 最终成功、失败、取消仍然必须通过 Action Result 返回。
- Service 虽然也能立即回复“已受理”，但没有内置的长期 Result 和标准 Cancel；因此返回船坞仍使用 Action，不改成长时间阻塞的 Service。

### 4.1 预泊点来源

预泊点不放进 `ReturnDock` Goal，由 Decision 通过配置 Service 写入导航：

```text
云平台 → Decision → /mission_bridge/set_pre_dock_point Service → 导航本地保存

Service: /mission_bridge/set_pre_dock_point
Type:    m_common/srv/SetPreDockPoint
```

建议 Service 定义：

```text
# Request
m_common/MissionWaypoint point
---
# Response
bool success
string message
```

- 预泊点复用 `MissionWaypoint`，只包含 `latitude/longitude/yaw/seq`。
- 开机时，Decision 等待导航 Service 可用，然后主动调用一次，把当前预泊点写入导航。
- 之后只有云平台更新预泊点时，Decision 才再次调用该 Service；没有更新时不重复调用。
- 导航收到请求后校验并保存最新预泊点，`success=true` 表示已经成功保存，不表示开始返坞。
- 校验失败时返回 `success=false`，并保留上一次有效预泊点。
- 导航在 `ReturnDock` 开始时直接复制自己已经保存的预泊点到当前任务上下文。
- 返坞任务执行期间固定使用任务开始时复制的预泊点，不需要再次读取 Service。
- 返坞过程中如果云平台更新预泊点，导航可以保存新值，但新值只供下一次返坞任务使用，不改变当前任务。
- 导航开始返坞前必须校验预泊点经纬度；没有有效预泊点时拒绝 `ReturnDock` Goal。
- `ReturnDock` Goal 保持为空，不再重复携带 `dock_id` 或预泊点。

## 5. 暂停、恢复和取消任务

### 5.1 暂停、恢复使用Service `SetPause.srv`

继续复用现有 `m_common/srv/SetPause`：

```text
bool pause
---
bool success
string message
```

- `pause=true`：暂停当前 `NavigateTask` 或允许暂停阶段的 `ReturnDock`。
- `pause=false`：恢复当前被暂停的 Action。
- 暂停不结束 Action，不清空剩余航点。
- 恢复不创建新 Action Goal，从原任务上下文继续。
- 重复暂停、重复恢复应幂等返回。
- 如果返坞某个安全关键阶段不能暂停，明确返回 `success=false` 和原因。

### 5.2 取消使用Action内置的 Action Cancel

- Decision 使用 ROS 2 Action 标准 Cancel 取消当前 Goal。
- 导航收到 Cancel 后先安全停止控制输出，再将 Goal 置为 `CANCELED`。
- 导航不能只停止运动而不结束 Goal。
- 导航不要保证真的停船并取消任务之后才返回成功。

## 6. 电子围栏越界

电子围栏越界根据当前是否存在任务 Action 分成两种路径。

| 场景 | 导航处理 | 告知 Decision 的接口 |
|---|---|---|
| 有任务 `NavigateTask` 或 `ReturnDock` | 立即停止运动并进入安全状态，处置完成后将 Goal 置为 `ABORTED` | 当前 Action Result |
| 没有任务 Action | 执行本地安全处置 | `/mission_bridge/safety_event` Topic |

### 6.1 有任务 Action 时

导航完成停车等安全处置后返回：

```text
Action state = ABORTED
result = RESULT_GEOFENCE_VIOLATION
error_code = "GEOFENCE_VIOLATION"
message = "electronic geofence violation"
```

该 Action Result 是任务失败原因的唯一依据。同一次越界不要求再向 Decision 重复发布 `NavSafetyEvent`。

### 6.2 没有任务 Action 时

没有 Goal 就无法返回 Action Result，因此导航必须发布Topic：

```text
Topic: /mission_bridge/safety_event
Type:  m_common/msg/NavSafetyEvent
```

建议消息字段：

```text
std_msgs/Header header
uint8 severity
string event_code
string fence_id
string fence_type
string transition
bool enabled
float64 latitude
float64 longitude
float32 distance_to_boundary_m
string message
```

电子围栏事件固定使用：

```text
event_code = "GEOFENCE_VIOLATION"
fence_type = "exclusion" 或 "inclusion"
transition = "ENTER" 或 "EXIT"
enabled = true 表示告警触发
enabled = false 表示告警解除
```

Topic 只负责报告无活动任务时的异步事件，不能创建任务，也不能伪造 Action Result。

### 6.3 围栏数据来源和 Service

电子围栏的数据流固定为：

```text
云平台 → Decision → /mission_bridge/set_geofence Service → 导航本地保存 → 导航内部按需同步给 MAVROS
```

接口约定：

```text
Service: /mission_bridge/set_geofence
Type:    m_common/srv/SetGeoFence
```

开机与更新规则：

- 开机时，Decision 等待导航 Service 可用，然后主动调用一次，把当前完整围栏集合写入导航。
- 之后只有云平台更新围栏时，Decision 才再次调用该 Service；没有更新时不重复调用。
- 每次请求都必须携带完整围栏快照，不使用新增、修改或删除的增量消息。
- 空围栏数组表示清除导航已经保存的围栏。
- 导航校验并保存成功后返回 `success=true`；校验失败返回 `success=false`，并继续保留上一次有效围栏。
- 如果 MAVROS 也需要围栏，由导航在成功保存后通过导航内部接口同步给 MAVROS，Decision 不再额外发布围栏 Topic。
- 导航在任务开始时复制自己已经保存的围栏快照到当前任务上下文。
- 任务执行期间固定使用任务开始时复制的围栏，不需要再次请求或持续读取配置。
- 任务执行期间如果云平台更新围栏，导航可以保存新快照，但新值只供下一次任务使用，不改变当前任务。
- `NavigateTask` Goal 仍然只包含 `MissionWaypoint[] waypoints`，不得把围栏字段塞进 `MissionWaypoint`。

### 6.4 云平台电子围栏字段定义

云平台在 `mission_upload/mission_execute` 中使用 `geo_fences` 表达本次任务携带的围栏数组。本文只复用它的数据结构；Decision 将其转换成 `m_common/GeoFence[]` 后调用 `SetGeoFence`。

| 云平台字段 | 类型 | 必填 | 定义 |
|---|---|:---:|---|
| `geo_fences` | array | 否 | 电子围栏列表；不传或空数组表示不启用围栏 |
| `geo_fences[].fence_id` | string | 是 | 平台围栏 ID，用于日志、失败原因和告警关联 |
| `geo_fences[].type` | string | 是 | `exclusion`=禁航区，船不得进入；`inclusion`=作业区，船不得驶出 |
| `geo_fences[].shape` | string | 否 | `polygon`=多边形；`circle`=圆形；缺省按 `polygon` |
| `geo_fences[].points` | array | 条件 | `shape=polygon` 时必填；至少 3 个不重复的 WGS84 顶点 |
| `geo_fences[].points[].lat` | double | 条件 | 多边形顶点纬度，WGS84 |
| `geo_fences[].points[].lon` | double | 条件 | 多边形顶点经度，WGS84 |
| `geo_fences[].center` | object | 条件 | `shape=circle` 时必填，表示圆心 |
| `geo_fences[].center.lat` | double | 条件 | 圆心纬度，WGS84 |
| `geo_fences[].center.lon` | double | 条件 | 圆心经度，WGS84 |
| `geo_fences[].radius_m` | float | 条件 | `shape=circle` 时必填；半径单位 m，必须大于 `0` |

建议 ROS 消息与云平台结构对应：

```text
# GeoPoint.msg
float64 lat
float64 lon

# GeoFence.msg
string fence_id
string type
string shape
m_common/GeoPoint[] points
m_common/GeoPoint center
float32 radius_m

# SetGeoFence.srv
m_common/GeoFence[] fences
---
bool success
string message
```

转换和校验规则：

- `shape` 缺省时，Decision 转换为明确的 `polygon` 再发给导航。
- 纬度范围 `[-90,90]`，经度范围 `[-180,180]`，所有数值必须有限。
- `polygon` 只使用 `points`，且至少包含 3 个不重复点。
- `circle` 只使用 `center + radius_m`，且 `radius_m>0`。
- `exclusion` 禁止船体进入；`inclusion` 限制船体不得驶出。
- 参数或字段组合非法时返回 `result=1001`；不支持相应围栏形状时返回 `result=1006`。
- 任务下发阶段已经发现航线违反围栏时直接拒绝任务，不发布越界事件。

## 7. `/nav_status`

Action 已经负责任务进度和终态，`/nav_status` 只发布不依赖某个 Goal 的持续状态，例如：

- 导航是否就绪。
- 定位是否有效。
- 规划器、控制器是否可用。
- 激光雷达等导航必要传感器是否正常。
- 是否急停。
- 当前是否存在活动 Goal、是否暂停。

`/nav_status` 不再承担以下功能：

- 宣布任务完成。
- 宣布任务失败或取消。
- 代替 Action Feedback 上报航点进度。

## 8. 导航节点需要修改的内容

1. 将长时间导航执行接口改为 `NavigateTask` Action Server。
2. Action Goal 只接收 `MissionWaypoint[] waypoints`。
3. 为现有 `MissionWaypoint.msg` 增加 `seq`，最终只保留 `latitude/longitude/yaw/seq`。
4. 根据数组长度自动判断单点或无动作航线，不增加 `task_type`。
5. 实现 `ReturnDock` Action Server。
   Goal 接受结果立即返回，最终 Result 等返坞完成后返回；Feedback 不要求进度百分比。
6. 继续提供 `SetPause` Service，实现暂停和恢复。
7. 使用 Action 标准 Cancel 实现取消，不再使用自定义取消 Service。
8. 使用 Action Feedback 上报当前航点和进度，使用 Result 上报唯一终态。
9. 保留 `/nav_status`，但只上报导航持续状态。
10. 新增 `/mission_bridge/safety_event` 的 `NavSafetyEvent` Topic，用于没有活动 Action 时的电子围栏越界。
11. 提供 `/mission_bridge/set_geofence` Service；保存 Decision 开机初始化或云平台更新时写入的完整围栏快照。
12. 提供 `/mission_bridge/set_pre_dock_point` Service；保存 Decision 开机初始化或云平台更新时写入的预泊点。
13. 删除 `/task_event` 对任务成功、失败、取消的重复上报依赖。
14. 不解析航点动作，不调用相机、云台、采水等驱动，不实现虚拟遥控。

## 9. 联调前需要确认

1. `yaw` =65536 表示“不指定朝向”。
2. `seq` 从 0 还是从 1 开始decsion跟云平台确认。
3. 多点 Feedback 的 `distance_remaining_m` 是到当前点的距离。
4. 返坞过程中decision发布时没有暂停只有取消。
5. `/nav_status` 和 `/mission_bridge/safety_event` 的最终 QoS=1。

## 10. 联调验收

- `waypoints=[]`：Goal 被拒绝。
- `waypoints` 只有 1 个点：按单点导航执行。
- `waypoints` 有多个点：按无动作航线连续执行。
- 每个航点只包含 `latitude/longitude/yaw/seq`。
- 导航不接收 `task_type`、速度、海拔、航点动作或云端字段。
- 有动作航线由 Decision 分成多个单点 Goal，导航每次只执行一个点。
- Action Feedback 中的 `current_seq/reached_seq` 与下发 `seq` 一致。
- 成功、失败和取消均只产生一次 Action Result。
- 暂停不结束 Goal，恢复不创建新 Goal。
- Cancel 后导航安全停车并返回 `CANCELED`。
- 有活动 Action 时越界：导航停车后返回 `ABORTED + RESULT_GEOFENCE_VIOLATION`。
- 无活动 Action 时越界：导航发布 `NavSafetyEvent(event_code="GEOFENCE_VIOLATION")`。
- 开机时 Decision 分别调用一次围栏和预泊点配置 Service，导航成功保存并返回 `success=true`。
- 没有云平台更新时，Decision 不重复调用两个配置 Service，导航继续使用已经保存的值。
- 云平台更新围栏后，Decision 调用 `SetGeoFence`；当前任务仍使用启动时的围栏，下一次任务使用新快照。
- `SetGeoFence` 收到空围栏数组后清除旧围栏，下一次任务不再使用旧围栏。
- 云平台更新预泊点后，Decision 调用 `SetPreDockPoint`；当前返坞仍使用启动时的预泊点，下一次返坞使用新值。
- 配置 Service 校验失败时返回 `success=false`，导航继续保留上一次有效值。
- 没有有效预泊点时，导航拒绝 `ReturnDock` Goal。
- `ReturnDock` Goal 能够立即完成接受/拒绝响应，长时间返坞结束后才产生最终 Result。
- `ReturnDock` 不提供百分比或航点进度时仍可正常完成；可选 Feedback 只上报运行、暂停或心跳。
- `/nav_status` 和安全事件 Topic 均不能代替 Action Result。
