# 云平台下发的航点定义,对应 §4.2.2.3 waypoints[]
float64 lat
float64 lon
# 目标航向(rad)。云协议航点项不含 yaw,mqtt_bridgev2 与 decision 示教采点均恒填 0;
# 供内部导航接口使用。
float64 yaw
float32 alt
# 原始航线序号。项目约定云端显式传入并从 1 连续递增；Decision
# 拆分带动作航线时保持不变。
int32 seq
m_common/WaypointAction[] actions
