#include "apriltag_node.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
// 1. 引入占位符 `_1`
#if X86_DEBUG_VIEW
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif
using namespace perception;
tf2::Transform averageTransforms(const std::vector<tf2::Transform> &transforms)
{
  if (transforms.size() == 1)
  {
    return transforms[0];
  }

  // if (transforms.empty())
  // {
  //   throw std::invalid_argument("averageTransforms: input vector is empty");
  // }
  // 1. 平移平均
  tf2::Vector3 mean_trans(0, 0, 0);
  for (const auto &T : transforms)
  {
    mean_trans += T.getOrigin();
  }
  mean_trans /= static_cast<double>(transforms.size());

  // 2. 旋转平均：Markley SVD 方法
  //

  std::vector<Eigen::Vector4d> quat_vecs;
  quat_vecs.reserve(transforms.size());

  // 参考四元数（第一个）
  tf2::Quaternion ref_q = transforms[0].getRotation();
  Eigen::Vector4d ref_eigen(ref_q.w(), ref_q.x(), ref_q.y(), ref_q.z());

  for (const auto &T : transforms)
  {
    tf2::Quaternion q = T.getRotation();
    // 构造 Eigen 向量 (w, x, y, z)
    Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());

    // 确保与参考的点积为正（处理双覆盖）
    if (v.dot(ref_eigen) < 0.0)
    {
      v = -v; // 翻转整个四元数
    }
    quat_vecs.push_back(v);
  }

  // 构造矩阵 M = sum(v * v^T)
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  for (const auto &v : quat_vecs)
  {
    M += v * v.transpose();
  }

  // 求 M 的最大特征值对应的特征向量
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(M);
  Eigen::Vector4d mean_vec = solver.eigenvectors().col(3); // 最大特征值在最后一列

  // 归一化得到平均四元数
  mean_vec.normalize();

  // 转换回 tf2::Quaternion（注意顺序：tf2 内部存储为 x,y,z,w）
  tf2::Quaternion mean_quat(mean_vec(1), mean_vec(2), mean_vec(3), mean_vec(0));

  // 3. 组合结果
  return tf2::Transform(mean_quat, mean_trans);
}
tf2::Transform rpyToTransform(double x, double y, double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw); // 角度为弧度
  tf2::Transform transform(q, tf2::Vector3(x, y, z));
  return transform;
}
void transformToXYZRPY(const tf2::Transform &transform,
                       double               &x,
                       double               &y,
                       double               &z,
                       double               &roll,
                       double               &pitch,
                       double               &yaw)
{
  // 获取平移
  tf2::Vector3 t = transform.getOrigin();
  x              = t.x();
  y              = t.y();
  z              = t.z();

  // 获取旋转四元数并转换为 RPY
  tf2::Quaternion q = transform.getRotation();
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
}
/**
 * 将 apriltag_pose_t 转换为 tf2::Transform
 * @param pose AprilTag 检测得到的位姿（包含旋转矩阵 R 和平移向量 t）
 * @return tf2::Transform 对象，可用于 ROS 坐标系变换
 */
tf2::Transform apriltagPoseToTf2(const apriltag_pose_t &pose)
{
  // 假设 pose.R 和 pose.t 均为有效指针，矩阵大小正确

  // 获取旋转矩阵数据（matd_t 采用行主序存储）
  double        *R_data = pose.R->data; // 按行排列: R00, R01, R02, R10, R11, R12, R20, R21, R22
  tf2::Matrix3x3 rot;
  rot.setValue(R_data[0], R_data[1], R_data[2], R_data[3], R_data[4], R_data[5], R_data[6],
               R_data[7], R_data[8]);

  // 获取平移向量数据（3×1 矩阵，数据按行存储）
  double      *t_data = pose.t->data; // tx, ty, tz
  tf2::Vector3 trans(t_data[0], t_data[1], t_data[2]);

  // 构造 tf2::Transform（旋转 + 平移）
  tf2::Transform tf_transform(rot, trans);
  return tf_transform;
}

Eigen::Affine3d xyzrpyToAffine3d(
    double x, double y, double z, double roll, double pitch, double yaw)
{
  // 平移部分
  Eigen::Translation3d translation(x, y, z);

  // 旋转部分：Z-Y-X 内旋（yaw, pitch, roll）
  Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot_pitch(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot_roll(roll, Eigen::Vector3d::UnitX());

  // 组合旋转（注意顺序：先 yaw，再 pitch，最后 roll）
  Eigen::Quaterniond rotation = rot_yaw * rot_pitch * rot_roll;

  // 构造仿射变换：平移 * 旋转（即先旋转后平移）
  return translation * rotation;
}

AprilTagLocalization::AprilTagLocalization(const std::string &node_name)
    : private_node_ptr_(rclcpp::Node::make_shared(
          node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
      logger_(private_node_ptr_->get_logger()),
      WORKSPACE_DIR(ament_index_cpp::get_package_prefix("apriltag_localization"))
{
  MLOGGER_INFO(
      "=========================AprilTagLocalization Initing=====================================");
  MLOGGER_INFO("WORKSPACE_DIR:[{}]", WORKSPACE_DIR.c_str());
  if (!initConfig())
  {
    MLOGGER_ERROR("initConfig() Failed!!");
    throw std::runtime_error("initConfig() Failed!!");
  }
  if (!initModel())
  {
    MLOGGER_ERROR("initModel() Failed!!");
    throw std::runtime_error("initModel() Failed!!");
  }
  MLOGGER_INFO("initModel() Successfully!!");
  if (!intiNode())
  {
    MLOGGER_ERROR("intiNode() Failed!!");
    throw std::runtime_error("intiNode() Failed!!");
  }
  MLOGGER_INFO(
      "=========================AprilTagLocalization Init "
      "Successfully!!=========================\n");
}

AprilTagLocalization::~AprilTagLocalization()
{
  if (td_ptr_ != nullptr)
  {
    MLOGGER_INFO("Destroying apriltag detector...");
    apriltag_detector_destroy(td_ptr_);
  }
  if (tf_ptr_ != nullptr)
  {
    MLOGGER_INFO("Destroying apriltag family...");
    destory_apriltag_family_t_(tf_ptr_);
  }
  if (detections_ != nullptr)
  {
    MLOGGER_INFO("Destroying apriltag detections...");
    apriltag_detections_destroy(detections_);
  }
  return;
}

bool AprilTagLocalization::initConfig()
{
  // private_node_ptr_->declare_parameter<std::string>("camera_info_topic", "");
  // private_node_ptr_->declare_parameter<std::string>("image_topic", "");
  // private_node_ptr_->declare_parameter<std::string>("apriltag_family_name", "tag25h9");
  // private_node_ptr_->declare_parameter<std::string>("detection_result_topic", "");
  // private_node_ptr_->declare_parameter<double>("tag_size", 500.0);
  // private_node_ptr_->declare_parameter<std::vector<int64_t>>("tag_ids", {});

  camera_info_topic_      = private_node_ptr_->get_parameter("camera_info_topic").as_string();
  image_topic_            = private_node_ptr_->get_parameter("image_topic").as_string();
  detection_result_topic_ = private_node_ptr_->get_parameter("detection_result_topic").as_string();
  tag_size_               = private_node_ptr_->get_parameter("tag_size").as_double();
  current_apriltag_family_name_ =
      private_node_ptr_->get_parameter("apriltag_family_name").as_string();

  if (camera_info_topic_.empty() || image_topic_.empty() || detection_result_topic_.empty())
  {
    MLOGGER_ERROR("One or more parameters are empty! Please check the configuration.");
    return false;
  }

  if (current_apriltag_family_name_.empty())
  {
    MLOGGER_ERROR("AprilTag family name is empty! Please check the configuration.");
    return false;
  }

  if (tag_size_ <= 0)
  {
    MLOGGER_ERROR("Invalid tag size: {}. Tag size must be positive.", tag_size_);
    return false;
  }

  std::vector<int64_t> tag_ids = private_node_ptr_->get_parameter("tag_ids").as_integer_array();
  for (const int64_t &id : tag_ids)
  {
    std::string current_tag_id    = "tag_" + std::to_string(id);
    std::string dock_offset_x     = current_tag_id + ".dock_offset_x";
    std::string dock_offset_y     = current_tag_id + ".dock_offset_y";
    std::string dock_offset_z     = current_tag_id + ".dock_offset_z";
    std::string dock_offset_roll  = current_tag_id + ".dock_offset_roll";
    std::string dock_offset_pitch = current_tag_id + ".dock_offset_pitch";
    std::string dock_offset_yaw   = current_tag_id + ".dock_offset_yaw";
    // 检查是否存在 "dock_offset_x"
    bool check_param = private_node_ptr_->has_parameter(dock_offset_x) &&
                       private_node_ptr_->has_parameter(dock_offset_y) &&
                       private_node_ptr_->has_parameter(dock_offset_z) &&
                       private_node_ptr_->has_parameter(dock_offset_roll) &&
                       private_node_ptr_->has_parameter(dock_offset_pitch) &&
                       private_node_ptr_->has_parameter(dock_offset_yaw);
    if (!check_param)
    {
      MLOGGER_WARN("tag id:{} check_param failed, please check parameters !", id);
      continue;
    }

    // 存在且已声明
    double dock_offset_x_val     = private_node_ptr_->get_parameter(dock_offset_x).as_double();
    double dock_offset_y_val     = private_node_ptr_->get_parameter(dock_offset_y).as_double();
    double dock_offset_z_val     = private_node_ptr_->get_parameter(dock_offset_z).as_double();
    double dock_offset_roll_val  = private_node_ptr_->get_parameter(dock_offset_roll).as_double();
    double dock_offset_pitch_val = private_node_ptr_->get_parameter(dock_offset_pitch).as_double();
    double dock_offset_yaw_val   = private_node_ptr_->get_parameter(dock_offset_yaw).as_double();
    MLOGGER_INFO(
        "Load dock to [tag_{}] parameters:\n dock_offset_x_val: {}\n dock_offset_y_val: {}\n "
        "dock_offset_z_val: {}\n dock_offset_roll_val: {}\n dock_offset_pitch_val: {}\n "
        "dock_offset_yaw_val: {}",
        id, dock_offset_x_val, dock_offset_y_val, dock_offset_z_val, dock_offset_roll_val,
        dock_offset_pitch_val, dock_offset_yaw_val);
    dock_offset_roll_val  = degreesToRadians(dock_offset_roll_val);
    dock_offset_pitch_val = degreesToRadians(dock_offset_pitch_val);
    dock_offset_yaw_val   = degreesToRadians(dock_offset_yaw_val);
    dock_pose_ext_map_[int(id)] =
        rpyToTransform(dock_offset_x_val, dock_offset_y_val, dock_offset_z_val,
                       dock_offset_roll_val, dock_offset_pitch_val, dock_offset_yaw_val);
  }
  // ── 加载入口偏移参数（dock_center → dock_entry） ──
  if (private_node_ptr_->has_parameter("entry_offset_x_dock"))
  {
    entry_offset_x_dock_ = private_node_ptr_->get_parameter("entry_offset_x_dock").as_double();
  }
  if (private_node_ptr_->has_parameter("entry_offset_y_dock"))
  {
    entry_offset_y_dock_ = private_node_ptr_->get_parameter("entry_offset_y_dock").as_double();
  }
  if (private_node_ptr_->has_parameter("entry_offset_yaw_deg"))
  {
    double deg = private_node_ptr_->get_parameter("entry_offset_yaw_deg").as_double();
    entry_offset_yaw_rad_ = degreesToRadians(deg);
  }
  MLOGGER_INFO("Entry offset: x={}, y={}, yaw_deg={} (rad={})",
               entry_offset_x_dock_, entry_offset_y_dock_,
               entry_offset_yaw_rad_ / DEG_TO_RAD, entry_offset_yaw_rad_);

  result_pose_stamp_.header.frame_id = "camera_link";
  if (private_node_ptr_->has_parameter("frame_id"))
  {
    result_pose_stamp_.header.frame_id = private_node_ptr_->get_parameter("frame_id").as_string();
  }
  return true;
}

bool AprilTagLocalization::initModel()
{
  // apriltag_family_t *tf      = NULL;
  const char *famname = current_apriltag_family_name_.c_str();
  if (!strcmp(famname, "tag36h11"))
  {
    tf_ptr_                    = tag36h11_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tag36h11_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tag25h9"))
  {
    tf_ptr_                    = tag25h9_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tag25h9_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tag16h5"))
  {
    tf_ptr_                    = tag16h5_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tag16h5_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tagCircle21h7"))
  {
    tf_ptr_                    = tagCircle21h7_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tagCircle21h7_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tagCircle49h12"))
  {
    tf_ptr_                    = tagCircle49h12_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tagCircle49h12_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tagStandard41h12"))
  {
    tf_ptr_                    = tagStandard41h12_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tagStandard41h12_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tagStandard52h13"))
  {
    tf_ptr_                    = tagStandard52h13_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tagStandard52h13_destroy, std::placeholders::_1));
  } else if (!strcmp(famname, "tagCustom48h12"))
  {
    tf_ptr_                    = tagCustom48h12_create();
    destory_apriltag_family_t_ = std::function<void(apriltag_family_t *)>(
        std::bind(&tagCustom48h12_destroy, std::placeholders::_1));
  } else
  {
    MLOGGER_ERROR("Unrecognized tag family name: {}. Use e.g. \"tag36h11\".",
                  current_apriltag_family_name_.c_str());
    return false;
  }
  td_ptr_ = apriltag_detector_create();
  apriltag_detector_add_family(td_ptr_, tf_ptr_);
  if (errno == ENOMEM)
  {
    MLOGGER_ERROR(
        "Unable to add family to detector due to insufficient memory to allocate the tag-family "
        "decoder with the default maximum hamming value of 2. Try choosing an alternative tag "
        "family.");
    return false;
  }
  td_ptr_->nthreads = 2;
  MLOGGER_INFO("Initialized AprilTag family: {}", current_apriltag_family_name_);
  return tf_ptr_ != NULL && td_ptr_ != NULL;
}
bool AprilTagLocalization::intiNode()
{
  // 订阅相机信息和图像话题
  camera_info_sub_ = private_node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&AprilTagLocalization::cameraInfoCallback, this, std::placeholders::_1));
  MLOGGER_INFO("Subscribed to camera info topic: {}", camera_info_topic_.c_str());

  image_sub_ = private_node_ptr_->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&AprilTagLocalization::imageCallback, this, std::placeholders::_1));
  MLOGGER_INFO("Subscribed to image topic: {}", image_topic_.c_str());

  output_result_array_pose_publisher_ =
      private_node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(detection_result_topic_,
                                                                            10);
  MLOGGER_INFO("interface_msgs::msg::ApriltagPoseList Publisher topic: {}",
               detection_result_topic_.c_str());

  // ── 新增：/dock/perception 发布器（10字段） ──
  dock_perception_publisher_ =
      private_node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/dock/perception", 10);
  MLOGGER_INFO("/dock/perception publisher created.");

  main_loop_ = std::thread(std::bind(&AprilTagLocalization::main_loop, this));
  //
  // std::string test_image_path =
  //     "/home/jzw/MyWorkscpae/HIK/ros2_perception/src/yolo-inference/data/bus.jpg";
  // cv::Mat mat = cv::imread(test_image_path);
  // std_msgs::msg::Header header;
  // std::string encoding = "bgr8";
  // sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, encoding,
  // mat).toImageMsg(); cv_bridge_shared_                           = cv_bridge::toCvShare(msg,
  // "mono8"); is_image_received_.store(true); detect();
  return true;
}
void AprilTagLocalization::run()
{
  rclcpp::spin(private_node_ptr_);
  return;
}

void AprilTagLocalization::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg == nullptr)
  {
    MLOGGER_ERROR("Received null CameraInfo message!");
    return;
  }
  if (is_camera_info_received_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  current_camera_intrinsics_.fx               = msg->k[0];
  current_camera_intrinsics_.fy               = msg->k[4];
  current_camera_intrinsics_.cx               = msg->k[2];
  current_camera_intrinsics_.cy               = msg->k[5];
  current_camera_intrinsics_.distortion_model = msg->distortion_model;
  current_camera_intrinsics_.distortion_coefficients.assign(msg->d.begin(), msg->d.end());
  MLOGGER_INFO(
      "CameraInfo received: fx={}, fy={}, cx={}, cy={}, distortion_model={}, "
      "distortion_coefficients=[{}]",
      current_camera_intrinsics_.fx, current_camera_intrinsics_.fy, current_camera_intrinsics_.cx,
      current_camera_intrinsics_.cy, current_camera_intrinsics_.distortion_model.c_str(),
      fmt::join(current_camera_intrinsics_.distortion_coefficients, ","));
  is_camera_info_received_ = true;
}

void AprilTagLocalization::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg == nullptr)
  {
    MLOGGER_ERROR("Received null Image message!");
    return;
  }
  if (!is_camera_info_received_)
  {
    return;
  }
  // MLOGGER_INFO("imageCallback.....");
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    cv_bridge_shared_ = cv_bridge::toCvShare(msg, "bgr8");
    is_image_received_.store(true);
    cond_var_.notify_one();
  }
}

void AprilTagLocalization::drawResult(cv::Mat &out_image, apriltag_detection_t *det)
{
  // cv::Mat result_mat;
  // cv::line(out_image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0],
  // det->p[1][1]),
  //          cv::Scalar(0, 0xff, 0), 2);
  // cv::line(out_image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[3][0],
  // det->p[3][1]),
  //          cv::Scalar(0, 0, 0xff), 2);
  // cv::line(out_image, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0],
  // det->p[2][1]),
  //          cv::Scalar(0xff, 0, 0), 2);
  // cv::line(out_image, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0],
  // det->p[3][1]),
  //          cv::Scalar(0xff, 0, 0), 2);
  std::vector<cv::Scalar> color_list = {cv::Scalar(0, 0xff, 0), cv::Scalar(0, 0, 0xff),
                                        cv::Scalar(0xff, 0, 0), cv::Scalar(0xff, 0xff, 0)};
  for (int i = 0; i < 4; i++)
  {
    cv::circle(out_image, cv::Point(det->p[i][0], det->p[i][1]), 5, color_list[i], -1);
  }
  std::stringstream ss;
  ss << det->id;
  cv::String text      = ss.str();
  int        fontface  = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double     fontscale = 1.0;
  int        baseline;
  cv::Size   textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
  cv::putText(out_image, text,
              cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2), fontface,
              fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  return;
}

bool AprilTagLocalization::detect()
{
  std_msgs::msg::Header header;
  // image_u8_t           *img = nullptr;
  cv::Mat image_mat, image_gray;
  /**

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    header       = current_image_->header;
    uint8_t *buf = new uint8_t[current_image_->data.size()];
    std::memcpy(buf, current_image_->data.data(), current_image_->data.size());
    image_u8_t im = image_u8{static_cast<int32_t>(current_image_->width),
                             static_cast<int32_t>(current_image_->height),
                             static_cast<int32_t>(current_image_->step), buf};
    img           = &im;
  }  */

  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    header = cv_bridge_shared_->header;
    cv_bridge_shared_->toImageMsg();
    image_mat = cv_bridge_shared_->image.clone();
    is_image_received_.store(false);
  }
  cv::cvtColor(image_mat, image_gray, cv::COLOR_BGR2GRAY);
  image_u8_t img = {image_gray.cols, image_gray.rows, image_gray.cols, image_gray.data};
  detections_    = apriltag_detector_detect(td_ptr_, &img);
  // if (errno == EAGAIN)
  // {
  // MLOGGER_ERROR("Unable to create the {} threads requested, exit.\n", td_ptr_->nthreads);
  // return false;
  // }
  // td_ptr_->wp;
  result_array_pose_msg_.data.clear();
  int                         result_size = zarray_size(detections_);
  std::vector<tf2::Transform> result_list;
  for (int i = 0; i < result_size; i++)
  {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);

    // Do stuff with detections here.
    // drawResult(image_mat, det);

    // First create an apriltag_detection_info_t struct using your known parameters.
    apriltag_detection_info_t info;
    info.det     = det;
    info.tagsize = tag_size_;
    info.fx      = current_camera_intrinsics_.fx;
    info.fy      = current_camera_intrinsics_.fy;
    info.cx      = current_camera_intrinsics_.cx;
    info.cy      = current_camera_intrinsics_.cy;
    // Then call estimate_tag_pose.
    apriltag_pose_t pose;
    double          err = estimate_tag_pose(&info, &pose);
    if (err > 1e-3)
    {
      MLOGGER_INFO("error={}, is too large, skipping..", err);
      continue;
    }
    if (dock_pose_ext_map_.find(det->id) == dock_pose_ext_map_.end())
    {
      // 键不存在 → 跳过，防止 operator[] 自动创建默认条目
      MLOGGER_EVERY_N_WARN(10, "TAG ID {}, is not exist in config map, skipping.", det->id);
      continue;
    }
    const auto &dock_offset = dock_pose_ext_map_[det->id];
    tf2::Transform tf = apriltagPoseToTf2(pose);
    tf                = camera2camera_link * tf * camera_tag2ros_ * dock_offset;
    result_list.emplace_back(tf);
  }

  // if (!image_mat.empty())
  // {
  //   cv::imshow("result", image_mat);
  //   cv::waitKey(1);
  // }
  if (!result_list.empty())
  {
    tf2::Transform result_tf = averageTransforms(result_list);

    result_array_pose_msg_.data.resize(6);
    transformToXYZRPY(result_tf, result_array_pose_msg_.data[0], result_array_pose_msg_.data[1],
                      result_array_pose_msg_.data[2], result_array_pose_msg_.data[3],
                      result_array_pose_msg_.data[4], result_array_pose_msg_.data[5]);
    output_result_array_pose_publisher_->publish(result_array_pose_msg_);
    MLOGGER_INFO("Final dock loc: (x, y, z), (roll, pitch, yaw): ({},{},{}),({},{},{})",
                 result_array_pose_msg_.data[0], result_array_pose_msg_.data[1],
                 result_array_pose_msg_.data[2], result_array_pose_msg_.data[3],
                 result_array_pose_msg_.data[4], result_array_pose_msg_.data[5]);

    if (result_list.size() >= 2)
    {
      MLOGGER_INFO(
          "================================multi poses========================================");
      MLOGGER_INFO("Found {} valid tags:", result_list.size());
      for (const tf2::Transform &tf : result_list)
      {
        double ros_x, ros_y, ros_z, ros_roll, ros_pitch, ros_yaw;
        transformToXYZRPY(tf, ros_x, ros_y, ros_z, ros_roll, ros_pitch, ros_yaw);
        MLOGGER_INFO("dock loc: (x, y, z), (roll, pitch, yaw): ({},{},{}),({},{},{})", ros_x, ros_y,
                     ros_z, ros_roll, ros_pitch, ros_yaw);
      }
      MLOGGER_INFO(
          "===================================================================================");
    }

    // ── 计算入口位姿并发布 /dock/perception（在有效检测分支内） ──
    tf2::Transform dock_center_to_entry;
    dock_center_to_entry.setOrigin(tf2::Vector3(entry_offset_x_dock_, entry_offset_y_dock_, 0.0));
    tf2::Quaternion q_entry;
    q_entry.setRPY(0.0, 0.0, entry_offset_yaw_rad_);
    dock_center_to_entry.setRotation(q_entry);

    tf2::Transform base_to_entry = result_tf * dock_center_to_entry;

    double entry_x, entry_y, entry_z, entry_roll, entry_pitch, entry_yaw;
    double center_x, center_y, center_z, center_roll, center_pitch, center_yaw;
    transformToXYZRPY(base_to_entry, entry_x, entry_y, entry_z, entry_roll, entry_pitch, entry_yaw);
    transformToXYZRPY(result_tf, center_x, center_y, center_z, center_roll, center_pitch, center_yaw);

    int observation_mode = (result_list.size() >= 2) ? 2 : ((result_list.size() == 1) ? 1 : 0);
    int pose_valid = 1;
    double confidence = std::min(1.0, result_list.size() / 2.0);

    dock_perception_msg_.data = {
        entry_x, entry_y, entry_yaw,
        center_x, center_y, center_yaw,
        confidence,
        static_cast<double>(result_list.size()),
        static_cast<double>(observation_mode),
        static_cast<double>(pose_valid)
    };
    dock_perception_publisher_->publish(dock_perception_msg_);
    MLOGGER_INFO("/dock/perception: entry({:.3f},{:.3f},{:.3f}) center({:.3f},{:.3f},{:.3f}) tags={} mode={}",
                 entry_x, entry_y, entry_yaw, center_x, center_y, center_yaw,
                 result_list.size(), observation_mode);
  } else
  {
    output_result_array_pose_publisher_->publish(result_array_pose_msg_);
    // 无检测时发布无效感知消息
    dock_perception_msg_.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    dock_perception_publisher_->publish(dock_perception_msg_);
  }

  apriltag_detections_destroy(detections_);

  return result_size > 0;
}
void AprilTagLocalization::main_loop()
{
  while (true)
  {
    std::unique_lock<std::mutex> lock(image_mutex_);
    if (!cond_var_.wait_for(lock, std::chrono::milliseconds(2000),
                            [&] { return is_image_received_.load(); }))
    {
      MLOGGER_WARN("New image not received yet in {}ms.", 2000);
      continue;
    }
    lock.unlock();
    detect();
  }
  return;
}
int main(int argc, char *argv[])
{
  MLOGGER_MODULE_INIT("logs", "perception", "apriltag_localization");

  rclcpp::init(argc, argv);
  AprilTagLocalization apriltag_localization_node("apriltag_node");

  apriltag_localization_node.run();
  rclcpp::shutdown();
  return 0;
}