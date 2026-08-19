# §4.2.1.1 遥控
# 10Hz 连发;松手时 direction=STOP, speed=0;船端按 ttl 做超时保护。
# topic: /decision/manual_control (mqtt_bridgev2 → decision)
string direction
float32 speed
