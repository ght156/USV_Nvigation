# Decision 的无人船全局配置快照。
# 前三个字段对应云协议 usv_param_state；预泊点和电子围栏作为船端内部配置，供未来云端配置入口写入，并由 Decision 转交导航。

builtin_interfaces/Time stamp
# 低电量返航的阈值
uint8 low_battery_threshold
# 默认航线速度
float32 wp_speed
# 默认航线结束动作 0=返回上次模式 1=悬停 2=返航 3=重复执行 4=进船坞
uint8 finish_action
uint8 return_dock_battery_threshold
m_common/DockBerth[] dock_berths

# 当前预泊点。has_pre_dock_point=false 表示尚未配置有效预泊点。
bool has_pre_dock_point
m_common/MissionWaypoint pre_dock_point

# 当前完整电子围栏快照；空数组表示当前不启用电子围栏。
m_common/GeoFence[] geo_fences
