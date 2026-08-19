# 上层下发航点：WGS84 经纬度 + 航向
# 对应可编译类型：m_common/msg/MissionWaypoint

float64 latitude     # 度，(-90, 90)；禁止 (0,0)
float64 longitude    # 度，(-180, 180)
float64 yaw          # 到点航向（rad）；不需要朝向时填 0
