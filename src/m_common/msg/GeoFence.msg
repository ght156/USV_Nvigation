# 电子围栏（与云平台 geo_fences 结构对应）
# fence_id：平台围栏 ID，用于日志、失败原因和告警关联
# type：exclusion=禁航区（船不得进入）；inclusion=作业区（船不得驶出）
# shape：polygon=多边形（points 至少 3 个不重复点）；circle=圆形（center + radius_m）
string fence_id
string type
string shape
m_common/GeoPoint[] points
m_common/GeoPoint center
float32 radius_m
