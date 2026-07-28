# 通用目标检测结果列表（单帧）
# header.stamp  : 该帧检测对应的采集时间
# header.frame_id: 列表内所有目标 2D/3D 字段所在坐标系（如 camera / base_link / map）

std_msgs/Header header
DetectionObj[] objects
