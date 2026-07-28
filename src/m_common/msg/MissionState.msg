# 任务进度周期上报,对应 §4.2.2.4 / state topic mission_state
# 由 decision 发布,mqtt_bridgev2 订阅后转 MQTT state
builtin_interfaces/Time stamp
string user_id
string execution_id
string phase
uint32 current_seq
uint32 reached_seq
uint32 waypoint_total
float32 progress
