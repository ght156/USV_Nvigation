# 上层下发航点（WGS84 + 航向 + 序号）
# latitude / longitude：度
# yaw：到点航向，弧度（ENU/map 系航向语义，与 Nav2 一致），合法范围 [-2π, 2π]；
#      超出该范围的值（如 65536）= 不指定朝向，导航取行进方向作为到点朝向
# seq：航点在原始航线中的序号，由 Decision 下发，导航不得重新编号
float64 latitude
float64 longitude
float64 yaw
int32 seq
