# 导航 → 上层：/nav_status 话题 payload 字段说明
# 实现：std_msgs/String，data 为 UTF-8 JSON，推送 2Hz
# QoS：RELIABLE + TRANSIENT_LOCAL，depth=10
#
# === 完整 JSON 结构 ===
# {
#   "schema_version": 1,
#   "stamp": {"sec": int, "nanosec": int},
#   "vehicle_id": "usv_001",
#
#   "task": {
#     "state": "IDLE",              # WAITING_SYSTEM|IDLE|RUNNING|PAUSED|COMPLETED|FAILED|EMERGENCY
#     "task_id": "m001",            # string|null，send_waypoints.mission_id 或 GCS /waypoint
#     "command_id": null,           # string|null，send_waypoints.command_id 或 GCS /waypoint
#     "nav_phase": "IDLE",          # IDLE|TRACKING|STUCK|RECOVERY|PAUSED|EMERGENCY（辅助显示）
#     "current_waypoint": 0,        # int，已完成的航点数
#     "total_waypoints": 3,         # int
#     "progress_percent": 33.3,     # float，0.0~100.0
#     "elapsed_sec": 5.2,           # float
#     "distance_to_goal_m": 12.3,   # float（Phase 2 完善）
#     "eta_sec": null,              # float|null（Phase 2 完善）
#     "last_error": null            # PLAN_FAILED|CTRL_STUCK|LOC_LOST|LOC_DEGRADED|MISSION_FAILED|null
#   },
#
#   "planner": {
#     "status": "OK",               # OK|FAILED
#     "last_plan_time_ms": 0.0,
#     "last_error": null
#   },
#
#   "controller": {
#     "status": "OK",               # OK|STUCK
#     "tracking_error_m": 0.0,
#     "last_error": null
#   },
#
#   "localization": {
#     "overall": "GOOD",            # GOOD|DEGRADED|LOST
#     "position_cov_max": 0.05,     # float
#     "orientation_cov_max": 0.01,  # float
#     "gps_fix": 3,                 # int，0=无 3=3D fix
#     "tf_ok": true,                # bool
#     "odom_hz": 50.0               # float
#   },
#
#   "pose": {                       # map 坐标系
#     "x": 1.2, "y": 3.4,          # float (m)
#     "yaw": 1.57,                  # float (rad)
#     "v": 0.5,                    # float (m/s)
#     "w": 0.0                     # float (rad/s)
#   },
#
#   "flags": {
#     "manual_override": false,
#     "emergency_stop": false,
#     "recovery_active": false
#   },
#
#   "alerts": {                     # 布尔告警，快速扫描
#     "odom_stale": false,
#     "gps_stale": false,
#     "mission_bridge_alive": true,
#     "planner_error": false,
#     "controller_error": false,
#     "emergency_active": false,     # 急停激活中
#     "mission_paused": false        # 任务暂停中
#   },
#
#   "recent_logs": [               # 最多 200 条环形缓冲，仅展示勿做逻辑
#     {"stamp": 1716900000.1, "level": "WARN", "node": "planner_server",
#      "message": "GridBased: failed to create plan, no valid path found."}
#   ]
# }
#
# 详见 nav_task_interface.md §5.5

string placeholder_not_used_in_ros
