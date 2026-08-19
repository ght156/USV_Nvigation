# 航点附加动作,对应 §4.2.2.3 waypoints[].actions[]
# type=HOVER         → 使用 duration_seconds
# type=WATER_SAMPLE  → 使用 bottle_no + volume_ml
string type
float32 duration_seconds
uint8   bottle_no
float32 volume_ml
