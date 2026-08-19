# AprilTag / 基准标记(fiducial)专有信息
# 作为 DetectionObj 的可选扩展使用：当 DetectionObj.source == SOURCE_APRILTAG 时填充。
# 通用字段（2D 框 / 置信度 / 3D 位姿）仍复用 DetectionObj 的 bbox_* 与 pose。
# 约定：
#   - class_id  存放 tag id（与本结构 id 一致，方便通用消费者直接用）
#   - label_name 建议形如 "apriltag:tag36h11:0"
#   - pose       存放 tag 在 header.frame_id 下的 3D 位姿

string family                  # tag 家族，如 tag36h11 / tagStandard41h12 / tagCustom48h12
int32 id                       # tag 唯一 ID
int32 hamming                  # 纠错位数（汉明距离），越小越可靠
float32 decision_margin        # 检测质量裕度，越大越可靠
float64 pose_error             # estimate_tag_pose 重投影误差，越小越好
float32 tag_size               # tag 物理边长（米）

# 图像像素坐标（左上角为原点）
geometry_msgs/Point center     # tag 像素中心 (det->c)，z 置 0
geometry_msgs/Point[] corners  # 4 个角点像素坐标 (det->p)，顺序固定，z 置 0
