#pragma once
// 感知/视觉链路公共服务与话题名（m_common 维护，调用方用 m_common::perception_names:: 引用）

namespace m_common::perception_names {

////////////////////////////////////// 公共服务名称 //////////////////////////////
// RTSP 推流开关(m_common::srv::RTSPStreamSwitch)
constexpr const char kSrvRtspStreamSwitch[]        = "/rtsp2_multi_source_node/rtsp_stream_switch";
// AI 检测框视频画面叠加开关(std_srvs::srv::SetBool)
constexpr const char kSrvAiDetectorOverlaySwitch[] = "/ai_detector/overlay_switch";
// rtsp2 暂停/恢复节点处理(std_srvs::srv::SetBool)
constexpr const char kSrvRtsp2Pause[]              = "/rtsp2_multi_source_node/pause";

////////////////////////////////////// 公共话题名称 //////////////////////////////
// AI检测框叠加当前开关状态(std_msgs::msg::Bool)
constexpr const char kTopicAiDetectorOverlayStatus[]   = "/ai_detector/overlay_switch_status";
// 海康路叠加可视化图(sensor_msgs::msg::Image，rtsp2 订阅推流)
constexpr const char kTopicAiDetectorHikOverlayImage[] = "/ai_detector/cam_front_hik/overlay_image";
// 前RGBD相机垃圾识别结果(m_common::msg::DetectionObjList)
constexpr const char kTopicAiDetectorRgbdTrashDetsResult[] =
    "/ai_detector/cam_front_rgbd/trash_dets";
// AprilTag 码头/船坞检测结果(m_common::msg::DetectionObjList)
constexpr const char kTopicApriltagDetectionResults[] = "/apriltag_node/detections";

} // namespace m_common::perception_names
