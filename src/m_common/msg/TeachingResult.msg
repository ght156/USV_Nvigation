# 示教航线结果,对应 §4.1.2.13 teaching_result
# 由 decision 采点汇总后发布,mqtt_bridgev2 订阅后转 MQTT state
string  teaching_id
int32   result             # 0 成功;非 0 或 waypoints 为空视为示教失败
m_common/Waypoint[] waypoints
float32 total_distance_m   # 示教总里程(米)
int64   duration_ms        # 示教总时长(毫秒)
