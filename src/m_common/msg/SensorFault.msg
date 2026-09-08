# 一条处于活跃状态的传感器故障。见 docs/特殊情况码.md §4。
# 由 mqtt_bridgev2 汇总(心跳超时判离线,类型化回调判内容类故障),
# 再广播给需要据此做业务互锁的节点 —— 目前是 decision 的任务阻断。

# 传感器类型,协议附录 E.12:
#   LIDAR / RTK / IMU / FRONT_CAMERA / REAR_CAMERA / BOAT_CONTROLLER /
#   WATER_QUALITY / GIMBAL_CONTROL / BATTERY / WIRELESS_CHARGING
string type

# 协议附录 E.13:1=正常 2=离线/通信异常 3=故障
uint8 status

# 故障码,非 0。3001~3008 设备离线,3009 RTK 定位丢失,
# 5001~5011 电池,5101~5122 无线充电。
int32 fault_code
