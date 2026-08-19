# nav → decision: 周期状态
# internal_state: idle / navigating / avoiding / arrived / holding / failed
# decision 用它辅助到达判定 + 监控 nav 是否失联(stamp 老化判定)
builtin_interfaces/Time stamp
string internal_state
uint32 current_seq            # 当前正在去的航点序号(由 decision 在 gotoPoint 时约定)
float64 current_target_lat
float64 current_target_lon
