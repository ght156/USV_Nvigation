# perception → decision: 检测事件
# event_type: 由 perception 端定义,常见如 fishing_active / person_near_water / target_acquired
# target_class / color / confidence / image_url 为业务字段
builtin_interfaces/Time stamp
string event_type
string target_class
string color
float32 confidence
string image_url
float64 lat
float64 lon
