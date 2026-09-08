# 当前所有活跃的传感器故障快照(全量,不是增量)。
#
# 为什么是全量:阻断是"当前是否还有任何阻断条件成立"的判断,增量语义下
# 任何一条丢失都会让船永久停在阻断态或永久漏掉阻断。全量 + transient_local
# 让晚启动的订阅者也能立刻拿到当前真值。
#
# 发布方:mqtt_bridgev2(它是全船传感器故障的唯一汇聚点)。
# 该汇聚只走 ROS,不依赖 MQTT 连接 —— 云平台断线时阻断逻辑必须照常工作。
builtin_interfaces/Time stamp
m_common/SensorFault[] active_faults
