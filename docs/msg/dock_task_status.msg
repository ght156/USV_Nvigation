# 归港 → 上层：/dock_task/status 话题 payload 字段说明
# 实现：std_msgs/String，data 为 UTF-8 JSON，约 5Hz
# QoS：RELIABLE，depth=10
# 商用单坞；勿用 /dock/status（船内联调全量字段）
#
# 详见 nav_task_interface.md §13.3.3
#
# === 完整 JSON 结构 ===
# {
#   "schema_version": 1,
#   "run_state": "RUNNING",
#   "phase": "DOCKING",
#   "mode": "one_click",
#   "dock_active": true,
#   "retry_count": 1,
#   "retry_max": 3,
#   "needs_manual_takeover": false,
#   "needs_reapproach": false,
#   "camera_ready": true,
#   "error_code": 0,
#   "error_message": null,
#   "mission_id": "m001",
#   "command_id": "c1",
#   "elapsed_sec": 98.5
# }
#
# === run_state 枚举 ===
# IDLE        空闲，无归港会话（dock_active 应为 false）
# RUNNING     归港执行中
# SUCCEEDED   成功（瞬态，随后回 IDLE）
# FAILED      失败（常伴随 needs_manual_takeover=true）
# CANCELLED   已取消（瞬态，随后回 IDLE）
#
# === phase 枚举 ===
# IDLE        无任务
# STAGING     Nav 预泊、停稳、Entry 验收
# DOCKING     精靠泊（usv_docking）
# UNDOCKING   出泊（预留）
#
# === mode 枚举 ===
# one_click   一键归港
# dock_only   仅精靠泊
# undock      出泊
# null        空闲时
#
# === 上层关键判据 ===
# 运行中：     dock_active==true && run_state==RUNNING
# 成功且关闭：  event DOCK_SUCCEEDED && dock_active==false && run_state==IDLE
# 需人工接管：  needs_manual_takeover==true

string placeholder_not_used_in_ros
