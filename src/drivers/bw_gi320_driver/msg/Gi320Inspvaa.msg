# INSPVAA 消息定义
# 用于定时发布解析后的 INSPVAA 惯导组合导航数据

# UNIX 时间
builtin_interfaces/Time stamp
# 纬度 (WGS84) [度]
float64 latitude
# 经度 (WGS84) [度]
float64 longitude
# 椭球高 (WGS84) [m]
float64 altitude

# 东向速度 [m/s]（负值为西向）
float64 velocity_e
# 北向速度 [m/s]（负值为南向）
float64 velocity_n
# 天向速度 [m/s]
float64 velocity_u

# 横滚角 [度] 沿 Y 轴右手螺旋，逆时针为正
float64 roll
# 俯仰角 [度] 沿 X 轴右手螺旋，逆时针为正
float64 pitch
# 航向角 [度] 从北向逆时针方向，北偏东为正 由 IMU 陀螺经组合滤波器计算出的惯性方位角
float64 yaw

#累计里程 单位m
float64 odometer

