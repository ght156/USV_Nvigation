# boat_control → decision: 任务推进事件
# event_type 决定哪个字段有意义:
#   item_reached       → seq(已到达航点序号)
#   water_sample_done  → bottle_no(完成的采水瓶号)
#   upload_ok / mission_finished / upload_failed → 仅 event_type(+ reason)
builtin_interfaces/Time stamp
string event_type
uint32 seq
uint8  bottle_no
int32  reason             # 失败原因码,0=正常
