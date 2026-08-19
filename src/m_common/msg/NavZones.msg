# 导航区域集合（作业区 / 禁止区 / 硬边界），坐标均为 WGS84 经纬度
# work_area：作业区（闭合多边形），仅允许在内部作业；空 = 不限制
# forbidden_zones：禁止区列表（各自闭合），不允许进入
# hard_boundaries：硬边界列表（折线，不闭合），不允许穿越
m_common/GeoPolygon work_area
m_common/GeoPolygon[] forbidden_zones
m_common/GeoPolygon[] hard_boundaries
