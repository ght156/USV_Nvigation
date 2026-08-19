# 导航异步安全事件（无活动任务时的电子围栏越界等）
# 对应可编译类型：m_common/msg/NavSafetyEvent
# 话题：/mission_bridge/safety_event（QoS depth=1）
#
# 固定取值：event_code="GEOFENCE_VIOLATION"；fence_type=exclusion|inclusion；
# transition=ENTER（闯入）|EXIT（驶出）；enabled=true 告警触发 / false 告警解除

std_msgs/Header header
uint8 severity
string event_code
string fence_id
string fence_type
string transition
bool enabled
float64 latitude
float64 longitude
float32 distance_to_boundary_m
string message
