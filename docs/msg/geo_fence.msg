# 电子围栏（与云平台 geo_fences 结构对应）
# 对应可编译类型：m_common/msg/GeoFence、m_common/msg/GeoPoint
#
# type：exclusion=禁航区（不得进入）；inclusion=作业区（不得驶出）
# shape：polygon（points 至少 3 个不重复点，末点自动闭合）；circle（center + radius_m）

# GeoPoint.msg
float64 lat
float64 lon

# GeoFence.msg
string fence_id              # 平台围栏 ID，用于日志/失败原因/告警关联
string type
string shape
m_common/GeoPoint[] points
m_common/GeoPoint center
float32 radius_m
