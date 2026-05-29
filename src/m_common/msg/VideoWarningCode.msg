int32 code_id #编码id   0 正常； 1 钓鱼； 2 人员靠近水域
float32 conf # 置信度
string des #编码描述
string image_url # 告警图片地址（对象存储/HTTP访问地址）
sensor_msgs/Image image