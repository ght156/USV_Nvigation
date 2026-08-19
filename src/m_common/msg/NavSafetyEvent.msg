# 导航异步安全事件（无活动任务 Action 时的电子围栏越界等）
# event_code 固定取值：GEOFENCE_VIOLATION
# fence_type：exclusion=禁航区；inclusion=作业区
# transition：ENTER=闯入；EXIT=驶出
# enabled：true=告警触发；false=告警解除
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
