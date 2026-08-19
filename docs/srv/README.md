# 导航对外接口 — 消息与服务定义

> 完整说明：[`../导航_上层状态机对接接口.md`](../导航_上层状态机对接接口.md)  
> 本目录为注释副本，**文件名与类型与 `src/m_common` 一致**。

## Service（上层 → 导航）

| ROS 类型 | Service 名 | 本目录文件 |
|----------|------------|------------|
| `m_common/srv/SendWaypoints` | `/mission_bridge/send_waypoints` | `SendWaypoints.srv` |
| `m_common/srv/SetPause` | `/mission_bridge/set_pause` | `SetPause.srv` |
| `m_common/srv/EmergencyStop` | `/mission_bridge/emergency_stop` | `EmergencyStop.srv` |
| `m_common/srv/CancelMission` | `/mission_bridge/cancel_mission` | `CancelMission.srv` |

航点元素：`m_common/msg/MissionWaypoint`（`latitude`, `longitude`, `yaw`）。

## Topic（导航 → 上层）

| 话题 | 运行时类型 | 字段 schema |
|------|------------|-------------|
| `/nav_status` | `std_msgs/String` JSON | `m_common/msg/NavStatus` |
| `/task_event` | `std_msgs/String` JSON | `m_common/msg/NavTaskEvent` |
