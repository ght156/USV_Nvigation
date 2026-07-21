# 导航 → 上层：/task_event 话题 payload 字段说明
# 实现：std_msgs/String，data 为 UTF-8 JSON，事件驱动推送
# QoS：RELIABLE，depth=50
#
# === 完整 JSON 结构 ===
# {
#   "schema_version": 1,
#   "stamp": {"sec": int, "nanosec": int},
#   "vehicle_id": "usv_001",
#   "task_id": "m001",
#   "command_id": null,
#   "event": "TASK_FAILED",
#   "detail": { ... }
# }
#
# === 事件类型与 detail 载荷 ===
# TASK_STARTED:
#   { "task_id": "m001", "total_waypoints": 3 }
#
# TASK_COMPLETED:
#   { "task_id": "m001", "elapsed_sec": 45.2 }
#
# TASK_FAILED:
#   { "error_code": "MISSION_FAILED", "failed_waypoint_index": 2,
#     "reason": "Mission failed with error: MISSION_FAILED",
#     "nav2_error_code": 0, "nav2_error_msg": "" }
#
# TASK_CANCELLED:
#   { "task_id": "m001", "source": "cancel" }
#
# TASK_PAUSED:
#   { "task_id": "m001", "waypoint_index": 2 }
#
# TASK_RESUMED:
#   { "task_id": "m001", "waypoint_index": 2 }
#
# EMERGENCY_STOP:
#   { "task_id": "m001" }
#
# ALARM_RAISED:
#   { "alarm_code": "PLAN_FAILED", "level": "WARN",
#     "message": "Planner failed to find valid path...",
#     "suggested_action": "检查目标航点坐标，确认不在障碍层内" }
#
# ALARM_CLEARED:
#   { "alarm_code": "PLAN_FAILED" }
#
# === alarm_code 枚举 ===
# PLAN_FAILED | CTRL_STUCK | LOC_LOST | LOC_DEGRADED | MISSION_FAILED
#
# 详见 nav_task_interface.md §5.6

string placeholder_not_used_in_ros
