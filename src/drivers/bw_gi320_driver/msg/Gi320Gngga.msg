# GNGGA 消息定义
# 用于定时发布解析后的 GNGGA GPS 数据


# UNIX 时间
builtin_interfaces/Time stamp
# 纬度
float64 latitude
# 经度
float64 longitude
# 天线海拔高度
float64 altitude

# GPS 质量指示符
uint8 fix_type
# 质量枚举说明：
# 0 = 定位不可用或无效
# 1 = 单点定位
# 2 = 伪距差分或 SBAS 定位
# 4 = RTK 固定解
# 5 = RTK 浮点解
# 6 = 惯导定位
# 7 = 用户设定位置（Fixed Position）

# 使用中的卫星数
uint8 satellite_num

# 水平精度因子 (HDOP)
float32 hdop

#累计里程 单位m
float64 odometer
