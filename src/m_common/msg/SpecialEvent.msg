# 链路及船体处置事件(docs/特殊情况码.md §6)。
# decision 发布,mqtt_bridgev2 转成 events topic 的 special_event 上报。
builtin_interfaces/Time stamp

# 7001=船体已阻断 / 7004=低电自动返航 / 7005=超低电漂流 / 7006~7008=充电
int32 event_code

# "ACTIVE" 触发 / "CLEARED" 解除
string event_state

# 简要中文说明
string message

# 与任务相关时带上,否则留空
string execution_id

int32 BOAT_BLOCKED = 7001
int32 LOW_BATTERY_AUTO_RETURN_STARTED = 7004
int32 ULTRA_LOW_BATTERY_DRIFTING = 7005
int32 CHARGING_STARTED = 7006
int32 CHARGING_SUCCEEDED = 7007
int32 CHARGING_FAILED = 7008
