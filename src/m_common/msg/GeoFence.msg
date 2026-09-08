# 电子围栏完整定义；字段与云平台 geo_fences[] 对齐。
string fence_id
# exclusion=禁航区，inclusion=作业区
string type
# polygon 或 circle
string shape

# shape=polygon 时使用，至少 3 个不重复点。
m_common/GeoPoint[] points
# shape=circle 时使用。
m_common/GeoPoint center
float32 radius_m
