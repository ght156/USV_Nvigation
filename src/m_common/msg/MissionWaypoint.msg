# decision发给导航侧的航点定义（WGS84 + 航向）
# latitude / longitude：度
# yaw：到点航向，度（ENU/map 系航向语义，统一用度；Nav2 侧内部再转弧度）；
# 超出 [-360, 360]（约定使用 65536.0）表示不指定朝向，由导航使用行进方向。
float64 latitude
float64 longitude
float64 yaw
# 原始航线序号；导航不得重编号，须原样写入 Feedback/Result。
int32 seq
