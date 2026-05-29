# 导航 Service 接口 + PAUSED/EMERGENCY 状态 实现方案

> 2026-05-28 | 待实施

## 结论：新增 service 不影响话题

同一个 ROS2 Node 可以同时持有 topic subscriber + service server，两者使用不同的 DDS endpoint，完全独立。现有 `/waypoint` 等话题照常工作。

---

## 1. 背景

导航栈目前是纯话题接口（topic-based）。上层任务管理需要**服务接口**用于三件事：
- **下发航线**：同步得到是否接受的 ACK
- **暂停/继续**：暂停当前任务，保存进度，可恢复
- **急停**：立刻切断一切，进入紧急状态

同时需要增加 `PAUSED`、`EMERGENCY` 两个任务状态。

## 2. 涉及仓库

| 仓库 | 改什么 |
|------|--------|
| `wuxihik_navigation` | `m_common` 新增 3 个 srv；`mission_bridge.py` 新增 2 个状态 + 3 个 service server；`nav_status_aggregator.py` 新增事件/phase/flags |
| `GROUND-CONTROL-STATION-dev` | 加 REST API → one-shot 脚本调 ROS2 service；前端加状态标签/颜色/按钮 |

---

## 3. Step 1: srv 定义（wuxihik_navigation）

当前 `workspace_nav`（ament_python）不能直接编译 srv。利用已有的 `m_common`（ament_cmake，已有 `rosidl_generate_interfaces`）新增三个 srv。

### 3.1 `src/m_common/srv/SendWaypoints.srv`

```
# 上层 → 导航：下发航线
geometry_msgs/PoseStamped[] waypoints
string mission_id
---
bool success
string message
```

### 3.2 `src/m_common/srv/SetPause.srv`

```
# 上层 → 导航：暂停/继续
bool pause    # true=暂停，false=继续
---
bool success
string message
```

### 3.3 `src/m_common/srv/EmergencyStop.srv`

```
# 上层 → 导航：急停
---
bool success
string message
```

### 3.4 `src/m_common/CMakeLists.txt`

`rosidl_generate_interfaces` 加三条 srv 路径。

### 3.5 `src/YILDIZ-USV/workspace_nav/package.xml`

加 `<depend>m_common</depend>`。

---

## 4. Step 2: MissionState + service server（mission_bridge.py）

### 4.1 枚举扩展

```python
class MissionState(str, Enum):
    WAITING_SYSTEM = "WAITING_SYSTEM"
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"          # 新增
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    EMERGENCY = "EMERGENCY"    # 新增
```

### 4.2 三个 service server（`__init__` 内）

```python
from m_common.srv import SendWaypoints, SetPause, EmergencyStop

self._srv_cg = MutuallyExclusiveCallbackGroup()
self._srv_send  = self.create_service(SendWaypoints,  'mission_bridge/send_waypoints', self._cb_send_waypoints,  callback_group=self._srv_cg)
self._srv_pause = self.create_service(SetPause,       'mission_bridge/set_pause',      self._cb_set_pause,       callback_group=self._srv_cg)
self._srv_emerg = self.create_service(EmergencyStop,  'mission_bridge/emergency_stop',  self._cb_emergency_stop,   callback_group=self._srv_cg)
```

### 4.3 `_cb_send_waypoints` 逻辑

- 校验 waypoints 非空、坐标合法
- 状态检查：WAITING_SYSTEM → 返回 success=false, message="System not ready"
- RUNNING/PAUSED + allow_replace=false → 返回 success=false, message="Mission already active"
- IDLE/COMPLETED/FAILED → 直接执行
- RUNNING/PAUSED → preempt 后执行
- 返回 `SendWaypoints.Response(success=True, message=...)`

### 4.4 `_cb_set_pause` 逻辑

- `pause=True` + RUNNING → 调 `_pause_mission()`，返回 success=true
- `pause=False` + PAUSED → 调 `_resume_mission()`，返回 success=true
- 其他 → 返回 success=false, message="Cannot pause/resume in current state"

### 4.5 `_cb_emergency_stop` 逻辑

- 任何非 EMERGENCY 状态 → 调 `_emergency_stop()`，返回 success=true
- 已 EMERGENCY → 返回 success=true（幂等）

---

## 5. Step 3: PAUSED 状态内部逻辑

### 5.1 新增字段

```python
self._paused_nav_xy: List[Tuple[float, float]] = []  # 暂停时保存的剩余航点
self._paused_index: int = 0                            # 暂停时保存的当前索引
```

### 5.2 `_pause_mission()`

1. 保存 `_nav_xy` → `_paused_nav_xy`，`current_index` → `_paused_index`
2. `cancel_goal_async()` 取消当前 FollowWaypoints goal
3. 停 `_send_timer`
4. `_mission_token += 1`（废掉旧 goal 回调）
5. 状态切 `PAUSED`，发 `status_detail(state_override="PAUSED")`
6. **不设**自动回 IDLE 定时器（停在 PAUSED 直到显式操作）

### 5.3 `_resume_mission()`

1. 恢复 `_paused_nav_xy` → `_nav_xy`，`_paused_index` → `current_index`
2. 状态切 `RUNNING`
3. `_start_nav_stack()`（从当前航点继续，不是从头）
4. 发 `status_detail`

### 5.4 与其他路径的互操作

- `_cb_mission_cancel`（话题取消）：PAUSED 时清 saved waypoints → IDLE
- `_consume_waypoint_command`：PAUSED 时允许 explicit_replan 抢占（放弃暂停进度，执行新航线）
- `_publish_status_detail` 心跳：PAUSED 时也发（原来仅 RUNNING 发，改为 RUNNING/PAUSED/EMERGENCY 都发）

---

## 6. Step 4: EMERGENCY 状态内部逻辑

### 6.1 `_emergency_stop()`

1. `cancel_goal_async()` 取消当前 goal
2. 清空 `_nav_xy` / `current_index` / `_paused_nav_xy` / `_paused_index`
3. 停所有 timer：`_send_timer` + `_waypoint_commit_timer` + `_delayed_mission_timer`
4. `_mission_token += 1`；`_running_mission_hash = None`
5. `_suppress_passive_waypoints = True`
6. 清 waypoints.json
7. 状态切 `EMERGENCY`，发 `status_detail(state_override="EMERGENCY", error_code="EMERGENCY_STOP")`

### 6.2 退出 EMERGENCY

通过话题 `/gcs_mission/cancel` 发 Empty → `_cb_mission_cancel` 检测当前为 EMERGENCY → 清 suppress 标志 → 切 IDLE。

### 6.3 EMERGENCY 时的行为

- 收 `/waypoint` → 全部忽略（`_consume_waypoint_command` 的 guard 拦截）
- 状态**不会**自动回 IDLE（不在 `_defer_idle` 范围内）
- heartbeat 继续发 status_detail（保持 aggregator 的 `mission_bridge_alive` 不过期）

---

## 7. Step 5: aggregator 改动

### 7.1 `_detect_mission_transition` 新增

| 转换 | 事件 |
|------|------|
| → PAUSED | `TASK_PAUSED` |
| PAUSED → RUNNING | `TASK_RESUMED` |
| → EMERGENCY | `EMERGENCY_STOP` |
| PAUSED → IDLE | `TASK_CANCELLED`（复用） |

### 7.2 `_derive_nav_phase` 新增

```python
if self._mission_state == "PAUSED":    return "PAUSED"
if self._mission_state == "EMERGENCY": return "EMERGENCY"
```

### 7.3 `flags.emergency_stop`

从硬编码 `False` → `self._mission_state == "EMERGENCY"`（动态）。

### 7.4 alerts 新增两项

```python
"emergency_active": self._mission_state == "EMERGENCY",
"mission_paused": self._mission_state == "PAUSED",
```

### 7.5 `_update_stuck_detection`

无需改动 — `PAUSED != "RUNNING"` 和 `EMERGENCY != "RUNNING"` 已会导致跳过。

---

## 8. Step 6: GCS 前端改动

| 文件 | 改动 |
|------|------|
| `MissionControls.tsx` | `STATE_LABELS` 加 PAUSED/EMERGENCY；`stateColors` 加紫色(PAUSED)+深红(EMERGENCY)；error panel 对 EMERGENCY 也显示；加暂停/继续/急停/清除急停按钮 |
| `types/index.ts` | `MissionControlsProps` + `MissionHandlersReturn` 加新 handler 和状态字段 |
| `useMissionHandlers.ts` | 加 `handlePauseMission`/`handleResumeMission`/`handleEmergencyStop`/`handleClearEmergency` |
| `plane_service.tsx` | 加 4 个 service 方法调新 API |
| `baseService.ts` | 加 4 个 API endpoint 枚举值 |

按钮渲染逻辑：
- RUNNING 时：显示 Pause + Emergency 按钮
- PAUSED 时：显示 Resume + Emergency 按钮
- EMERGENCY 时：显示 Clear Emergency 按钮

---

## 9. Step 7: GCS 后端改动

### 9.1 新增 REST API

`api_server.py` 加 `/api/pause_mission`、`/api/resume_mission`、`/api/emergency_stop`、`/api/clear_emergency`。

### 9.2 新增 one-shot ROS2 service client 脚本

按 `cancel_publisher.py` 模式，在 `backend/ros_nodes/` 下创建：

| 脚本 | 调的 service | 参数 |
|------|-------------|------|
| `pause_service_client.py` | `mission_bridge/set_pause` | pause=true |
| `resume_service_client.py` | `mission_bridge/set_pause` | pause=false |
| `emergency_service_client.py` | `mission_bridge/emergency_stop` | — |
| `clear_emergency_client.py` | 发 Empty 到 `/gcs_mission/cancel` | — |

`mission_service.py` 加对应方法（spawn subprocess 调脚本）。

---

## 10. 不改的部分

- `/nav_status` JSON schema 基本不变（仅 `flags.emergency_stop` 改为动态值 + alerts 加两项）
- topic 接口完全保留：`/waypoint`、`/gcs_mission/cancel`、`/gcs_mission/start`、`/color_code` 全部照常
- GCS data_store / ros_subscriber 无需改动（纯透传）
- GCS mission_service `VALID_STATES` 不改（那是 GCS 自己的 dispatch 阶段，与导航状态无关）

---

## 11. 验证

1. `colcon build --packages-select m_common workspace_nav` → 编译通过
2. `ros2 interface show m_common/srv/SetPause` → srv 可见
3. `ros2 service call /mission_bridge/send_waypoints ...` → 航线执行
4. 运行中 `ros2 service call /mission_bridge/set_pause "{pause: true}"` → 状态 "PAUSED"，goal 取消
5. `ros2 service call /mission_bridge/set_pause "{pause: false}"` → 从断点继续
6. 运行/暂停中 `ros2 service call /mission_bridge/emergency_stop` → 状态 "EMERGENCY"，一切清空
7. **回归**：topic `/waypoint` 照常工作；话题 cancel 照常
8. GCS 前端：PAUSED 紫色徽章、EMERGENCY 深红徽章、按钮正确出现/消失
