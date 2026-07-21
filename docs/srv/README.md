# 导航对外接口 — 消息与服务定义

> 供**竞赛上层**对接参考。  
> **完整说明**：[`../nav_task_interface.md`](../nav_task_interface.md)  
> **可编译 ROS 定义**（上层 `find_package(m_common)` 用）：`src/m_common/srv/` 下 4 个 `.srv`

## 架构

```
上层 ──Service──► mission_bridge     （下发，同步 ACK）
上层 ◄──Topic──── nav_status_aggregator  （/nav_status + /task_event）
```

地面站 GCS 仍用 Topic（`/waypoint` 等），与上层 Service **并行**，见主文档 §5.2。

## Service（上层 → 导航，本目录 4 个参考文件）

| 文件 | ROS 类型 | Service 名 |
|------|----------|------------|
| [`nav_send_waypoints.srv`](nav_send_waypoints.srv) | `m_common/srv/SendWaypoints` | `/mission_bridge/send_waypoints` |
| [`nav_set_pause.srv`](nav_set_pause.srv) | `m_common/srv/SetPause` | `/mission_bridge/set_pause` |
| [`nav_emergency_stop.srv`](nav_emergency_stop.srv) | `m_common/srv/EmergencyStop` | `/mission_bridge/emergency_stop` |
| [`nav_cancel_mission.srv`](nav_cancel_mission.srv) | `m_common/srv/CancelMission` | `/mission_bridge/cancel_mission` |

## Topic 反馈（导航 → 上层）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/nav_status` | `std_msgs/String` JSON | 2Hz，**RELIABLE + TRANSIENT_LOCAL** |
| `/task_event` | `std_msgs/String` JSON | 事件驱动 |

> `/nav_status`、`/task_event` **不是**独立 ROS msg 包，payload 为 JSON 字符串。  
> 字段说明见 `../msg/nav_status_snapshot.msg`、`../msg/nav_task_event.msg`（文档用，不参与 rosidl 编译）。

## msg 目录（4 个，均为 JSON  schema 说明）

| 文件 | 用途 |
|------|------|
| [`../msg/nav_status_snapshot.msg`](../msg/nav_status_snapshot.msg) | `/nav_status` JSON 字段 |
| [`../msg/nav_task_event.msg`](../msg/nav_task_event.msg) | `/task_event` JSON 字段 |
| [`../msg/nav_waypoint_gps.msg`](../msg/nav_waypoint_gps.msg) | 地面站 `/waypoint` 经纬度元素（上层 Service 不用） |
| [`../msg/nav_task_result.msg`](../msg/nav_task_result.msg) | 内部拒绝码参考（Topic 兼容场景） |

## 发给上层厂商的文件清单（建议）

1. `docs/nav_task_interface.md` — 主对接文档  
2. `docs/srv/` — 4 个 `nav_*.srv` + 本 README  
3. `docs/msg/` — 4 个 `nav_*.msg`  
4. `src/m_common/srv/SendWaypoints.srv` 等 **4 个可编译 srv**（或提供已安装的 `m_common` deb/仓库）
