# 通用目标检测结果（单个目标）
# 由感知端（2D/3D 检测、跟踪、基准标记）填充，供决策 / 显示等下游使用。
# 所有坐标系信息统一由外层 DetectionObjList.header.frame_id 描述。

# --- 检测来源（下游可据此分流处理）---
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_OBJECT=1              # 通用目标检测（YOLO 等）
uint8 SOURCE_APRILTAG=2           # AprilTag / 基准标记
uint8 source

# --- 分类信息 ---
string label_name                  # 类别名称，如 person / boat / buoy
int32 class_id                     # 类别 ID（模型输出索引），未知填 -1
float32 confidence                 # 置信度 [0.0, 1.0]

# --- 跟踪信息 ---
int32 track_id                     # 多目标跟踪 ID，未跟踪填 -1

# --- 2D 像素包围框（图像左上角为原点，单位：像素）---
# (bbox_x, bbox_y) 为左上角坐标；bbox_width / bbox_height 为框宽高。
bool has_bbox                      # 为 true 时下方 2D 框字段有效
float32 bbox_x
float32 bbox_y
float32 bbox_width
float32 bbox_height

# --- 3D 位姿（相对外层 header.frame_id，位置单位：米）---
bool has_pose                      # 为 true 时 pose / size 有效
geometry_msgs/PoseWithCovariance pose   # 目标中心位姿（含不确定度，未知协方差填 0）

# --- 3D 包围盒尺寸（米，沿 pose 各轴的边长）---
geometry_msgs/Vector3 size         # 无估计时填 0

# --- 运动估计（相对 header.frame_id，单位：m/s, rad/s）---
bool has_velocity                  # 为 true 时 velocity 有效
geometry_msgs/Twist velocity

# --- AprilTag / 基准标记扩展 ---
# source==SOURCE_APRILTAG 时 apriltag 字段有效；其它来源保持默认零值即可忽略。
AprilTagInfo apriltag
