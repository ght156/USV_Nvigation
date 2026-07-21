# 归港 → 上层：/dock_task/event 话题 payload 字段说明
# 实现：std_msgs/String，data 为 UTF-8 JSON，事件驱动
# QoS：RELIABLE，depth=50
#
# 详见 nav_task_interface.md §13.3.4
#
# === 完整 JSON 结构 ===
# {
#   "schema_version": 1,
#   "event": "DOCK_SUCCEEDED",
#   "mission_id": "m001",
#   "command_id": "c1",
#   "detail": {
#     "elapsed_sec": 142.5,
#     "mode": "one_click"
#   }
# }
#
# === 事件类型与 detail 载荷 ===
# DOCK_TASK_ACCEPTED:
#   { "mode": "one_click", "require_camera": true }
#
# DOCK_STAGING_STARTED:
#   （无必填 detail）
#
# DOCK_HANDOFF:
#   （无必填 detail）
#
# DOCK_SUCCEEDED:
#   { "elapsed_sec": 142.5, "mode": "one_click" }
#
# DOCK_FAILED:
#   { "reason": "...", "retry_count": 3,
#     "needs_manual_takeover": true, "needs_reapproach": true }
#
# DOCK_CANCELLED:
#   （无必填 detail）
#
# MANUAL_TAKEOVER_REQUESTED:
#   { "reason": "...", "retry_count": 3 }
#
# UNDOCK_SUCCEEDED / UNDOCK_FAILED:
#   （预留，出泊实现后补充）
#
# === 上层成败判据 ===
# 靠泊成功：以 DOCK_SUCCEEDED 为准（不要解析 /dock/status 的 success 字段）
# 需人工：  MANUAL_TAKEOVER_REQUESTED 或 status.needs_manual_takeover==true

string placeholder_not_used_in_ros
