#pragma once
namespace m_common::drivers_names {

////////////////////////// 相机驱动对外话题(astra 大白 DCW) ////////////////////////
// 前Astra深度相机RGB+深度对齐+内参(usv_camera_msg::msg::RGBD，ai_detector RGBD 路输入)
constexpr const char kTopicAstraFrontRgbd[]       = "/camera_front_astra/rgbd";
// 前Astra深度相机彩色/深度图像与内参
constexpr const char kTopicAstraFrontColorImage[] = "/camera_front_astra/color/image_raw";
constexpr const char kTopicAstraFrontDepthImage[] = "/camera_front_astra/depth/image_raw";
constexpr const char kTopicAstraFrontColorInfo[]  = "/camera_front_astra/color/camera_info";
constexpr const char kTopicAstraFrontDepthInfo[]  = "/camera_front_astra/depth/camera_info";
// 前Astra深度相机深度→彩色外参
constexpr const char kTopicAstraExtrinsicD2C[]    = "/camera_front_astra/extrinsic/depth_to_color";

} // namespace m_common::drivers_names
