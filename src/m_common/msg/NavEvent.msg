# nav → decision: 事件流
# event_type: arrived / obstacle_detected / replanned / unreachable / stuck / nav_resumed
#   arrived            → Navigating 子阶段切 Acting
#   obstacle_detected  → 可日志/触发 Fault(可选)
#   unreachable/stuck  → 触发 Fault 或 abort 任务
builtin_interfaces/Time stamp
string event_type
uint32 seq
string detail
