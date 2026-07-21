# mission_bridge 内部逻辑拒绝码
#
# 上层下发指令推荐使用 Service 接口（有 ACK），Topic 接口保留兼容。
# Service 返回 success/message 直接确认操作是否被接受；
# 操作生效仍应通过 /nav_status.task.state 变化 + /task_event 确认。
# 以下码值描述 mission_bridge 拒绝指令的逻辑原因，仅供参考。
#
# Service 接口：直接读 Response success/message，不经过本枚举。
# 以下码值主要描述 GCS Topic /waypoint 被拒绝时的逻辑原因。

int32 OK=0                     # 成功（Topic：消息已接收并处理）
int32 ERR_INVALID=1            # 参数非法：(0,0) / 越界经纬度 / 空数组 / 颜色格式不支持
int32 ERR_NOT_READY=2          # WAITING_SYSTEM
int32 ERR_REJECTED_PASSIVE=3   # cancel 后无 explicit_replan 的被动 /waypoint
int32 ERR_REJECTED_BUSY=4      # RUNNING 且 allow_replace=false 且无 explicit_replan（仅 Topic）
int32 ERR_REJECTED_DUPLICATE=5 # 相同 hash 刚完成且不允许重复（仅 Topic）
int32 ERR_EMERGENCY=6          # EMERGENCY 状态（Topic 忽略；Service 返回 success=false）
int32 ERR_INTERNAL=99          # 写盘失败 / 坐标转换失败等
