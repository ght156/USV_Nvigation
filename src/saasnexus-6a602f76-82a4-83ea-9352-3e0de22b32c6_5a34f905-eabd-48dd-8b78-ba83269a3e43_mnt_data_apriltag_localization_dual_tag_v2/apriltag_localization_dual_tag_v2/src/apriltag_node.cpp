#include "apriltag_node.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
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
  MLOGGER_INFO("=========================AprilTagLocalization Initing=====================================");
  MLOGGER_INFO("WORKSPACE_DIR:[{}]", WORKSPACE_DIR.c_str());
  if (!initConfig())
  {
    throw std::runtime_error("initConfig() Failed!!");
  }
  if (!initModel())
  {
    throw std::runtime_error("initModel() Failed!!");
  }
  if (!intiNode())
  {
    throw std::runtime_error("intiNode() Failed!!");
  }
  MLOGGER_INFO("AprilTagLocalization initialized. enhanced_mode={}", use_baseline_yaw_);
}

AprilTagLocalization::~AprilTagLocalization()
{
  running_.store(false);
  cond_var_.notify_all();
  if (main_loop_.joinable())
  {
    main_loop_.join();
  }
  if (detections_ != nullptr)
  {
    apriltag_detections_destroy(detections_);
    detections_ = nullptr;
  }
  if (td_ptr_ != nullptr)
  {
    apriltag_detector_destroy(td_ptr_);
    td_ptr_ = nullptr;
  }
  if (tf_ptr_ != nullptr && destory_apriltag_family_t_)
  {
    destory_apriltag_family_t_(tf_ptr_);
    tf_ptr_ = nullptr;
  }
}

bool AprilTagLocalization::initConfig()
{
  camera_info_topic_ = private_node_ptr_->get_parameter("camera_info_topic").as_string();
  image_topic_ = private_node_ptr_->get_parameter("image_topic").as_string();
  detection_result_topic_ = private_node_ptr_->get_parameter("detection_result_topic").as_string();
  tag_size_ = private_node_ptr_->get_parameter("tag_size").as_double();
  current_apriltag_family_name_ =
      private_node_ptr_->get_parameter("apriltag_family_name").as_string();

  if (camera_info_topic_.empty() || image_topic_.empty() || detection_result_topic_.empty() ||
      current_apriltag_family_name_.empty() || tag_size_ <= 0.0)
  {
    MLOGGER_ERROR("Invalid required AprilTag parameters.");
    return false;
  }

  const auto get_bool = [this](const std::string &name, bool fallback) {
    return private_node_ptr_->has_parameter(name)
               ? private_node_ptr_->get_parameter(name).as_bool()
               : fallback;
  };
  const auto get_int = [this](const std::string &name, int fallback) {
    return private_node_ptr_->has_parameter(name)
               ? static_cast<int>(private_node_ptr_->get_parameter(name).as_int())
               : fallback;
  };
  const auto get_double = [this](const std::string &name, double fallback) {
    return private_node_ptr_->has_parameter(name)
               ? private_node_ptr_->get_parameter(name).as_double()
               : fallback;
  };
  const auto get_string = [this](const std::string &name, const std::string &fallback) {
    return private_node_ptr_->has_parameter(name)
               ? private_node_ptr_->get_parameter(name).as_string()
               : fallback;
  };

  use_baseline_yaw_ = get_bool("use_baseline_yaw", false);
  hold_baseline_yaw_on_single_ = get_bool("hold_baseline_yaw_on_single", true);
  distance_weighted_position_ = get_bool("distance_weighted_position", true);
  baseline_tag_id_a_ = get_int("baseline_tag_id_a", 0);
  baseline_tag_id_b_ = get_int("baseline_tag_id_b", 43);
  max_hamming_ = get_int("max_hamming", 2);
  max_pose_error_ = get_double("max_pose_error", 1.0e-3);
  dual_tag_consistency_threshold_m_ =
      get_double("dual_tag_consistency_threshold_m", 0.30);
  dual_tag_yaw_consistency_deg_ = get_double("dual_tag_yaw_consistency_deg", 180.0);
  min_baseline_length_m_ = get_double("min_baseline_length_m", 0.05);
  baseline_yaw_offset_deg_ = get_double("baseline_yaw_offset_deg", 0.0);
  perception_result_topic_ = get_string("perception_result_topic", "/dock/perception");

  const double entry_x = get_double("dock_entry_offset_x", 0.0);
  const double entry_y = get_double("dock_entry_offset_y", 0.0);
  const double entry_z = get_double("dock_entry_offset_z", 0.0);
  const double entry_roll = degreesToRadians(get_double("dock_entry_offset_roll", 0.0));
  const double entry_pitch = degreesToRadians(get_double("dock_entry_offset_pitch", 0.0));
  const double entry_yaw = degreesToRadians(get_double("dock_entry_offset_yaw", 0.0));
  dock_center_to_entry_ =
      rpyToTransform(entry_x, entry_y, entry_z, entry_roll, entry_pitch, entry_yaw);

  const std::vector<int64_t> tag_ids =
      private_node_ptr_->get_parameter("tag_ids").as_integer_array();
  for (const int64_t id64 : tag_ids)
  {
    const int id = static_cast<int>(id64);
    const std::string prefix = "tag_" + std::to_string(id) + ".dock_offset_";
    const std::array<std::string, 6> names = {
        prefix + "x", prefix + "y", prefix + "z",
        prefix + "roll", prefix + "pitch", prefix + "yaw"};
    bool complete = true;
    for (const auto &name : names)
    {
      complete = complete && private_node_ptr_->has_parameter(name);
    }
    if (!complete)
    {
      MLOGGER_WARN("Tag {} offset parameters incomplete; tag ignored.", id);
      continue;
    }

    const double x = private_node_ptr_->get_parameter(names[0]).as_double();
    const double y = private_node_ptr_->get_parameter(names[1]).as_double();
    const double z = private_node_ptr_->get_parameter(names[2]).as_double();
    const double roll = degreesToRadians(private_node_ptr_->get_parameter(names[3]).as_double());
    const double pitch = degreesToRadians(private_node_ptr_->get_parameter(names[4]).as_double());
    const double yaw = degreesToRadians(private_node_ptr_->get_parameter(names[5]).as_double());
    tag_ids_.push_back(id);
    dock_pose_ext_map_[id] = rpyToTransform(x, y, z, roll, pitch, yaw);
    MLOGGER_INFO("Loaded T_tag{}_dock_center: xyz=({},{},{}), rpy_deg=({},{},{})",
                 id, x, y, z, roll / DEG_TO_RAD, pitch / DEG_TO_RAD, yaw / DEG_TO_RAD);
  }

  if (dock_pose_ext_map_.empty())
  {
    MLOGGER_ERROR("No valid tag offset configuration loaded.");
    return false;
  }
  if (use_baseline_yaw_ &&
      (dock_pose_ext_map_.count(baseline_tag_id_a_) == 0 ||
       dock_pose_ext_map_.count(baseline_tag_id_b_) == 0))
  {
    MLOGGER_ERROR("Enhanced mode requires configured baseline tags {} and {}.",
                  baseline_tag_id_a_, baseline_tag_id_b_);
    return false;
  }

  result_pose_stamp_.header.frame_id = get_string("frame_id", "camera_link");
  return true;
}

bool AprilTagLocalization::initModel()
{
  const char *famname = current_apriltag_family_name_.c_str();
  if (!strcmp(famname, "tag36h11"))
  {
    tf_ptr_ = tag36h11_create();
    destory_apriltag_family_t_ = tag36h11_destroy;
  } else if (!strcmp(famname, "tag25h9"))
  {
    tf_ptr_ = tag25h9_create();
    destory_apriltag_family_t_ = tag25h9_destroy;
  } else if (!strcmp(famname, "tag16h5"))
  {
    tf_ptr_ = tag16h5_create();
    destory_apriltag_family_t_ = tag16h5_destroy;
  } else if (!strcmp(famname, "tagCircle21h7"))
  {
    tf_ptr_ = tagCircle21h7_create();
    destory_apriltag_family_t_ = tagCircle21h7_destroy;
  } else if (!strcmp(famname, "tagCircle49h12"))
  {
    tf_ptr_ = tagCircle49h12_create();
    destory_apriltag_family_t_ = tagCircle49h12_destroy;
  } else if (!strcmp(famname, "tagStandard41h12"))
  {
    tf_ptr_ = tagStandard41h12_create();
    destory_apriltag_family_t_ = tagStandard41h12_destroy;
  } else if (!strcmp(famname, "tagStandard52h13"))
  {
    tf_ptr_ = tagStandard52h13_create();
    destory_apriltag_family_t_ = tagStandard52h13_destroy;
  } else if (!strcmp(famname, "tagCustom48h12"))
  {
    tf_ptr_ = tagCustom48h12_create();
    destory_apriltag_family_t_ = tagCustom48h12_destroy;
  } else
  {
    MLOGGER_ERROR("Unrecognized tag family: {}", current_apriltag_family_name_);
    return false;
  }

  td_ptr_ = apriltag_detector_create();
  apriltag_detector_add_family(td_ptr_, tf_ptr_);
  if (errno == ENOMEM)
  {
    MLOGGER_ERROR("Insufficient memory while adding AprilTag family.");
    return false;
  }
  td_ptr_->nthreads = 2;
  return tf_ptr_ != nullptr && td_ptr_ != nullptr;
}

bool AprilTagLocalization::intiNode()
{
  camera_info_sub_ = private_node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&AprilTagLocalization::cameraInfoCallback, this, std::placeholders::_1));
  image_sub_ = private_node_ptr_->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&AprilTagLocalization::imageCallback, this, std::placeholders::_1));
  output_result_array_pose_publisher_ =
      private_node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(
          detection_result_topic_, 10);
  perception_result_publisher_ =
      private_node_ptr_->create_publisher<std_msgs::msg::Float64MultiArray>(
          perception_result_topic_, 10);
  main_loop_ = std::thread(&AprilTagLocalization::main_loop, this);
  return true;
}

void AprilTagLocalization::run()
{
  rclcpp::spin(private_node_ptr_);
}

void AprilTagLocalization::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg == nullptr || is_camera_info_received_.load())
  {
    return;
  }
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  current_camera_intrinsics_.fx = msg->k[0];
  current_camera_intrinsics_.fy = msg->k[4];
  current_camera_intrinsics_.cx = msg->k[2];
  current_camera_intrinsics_.cy = msg->k[5];
  current_camera_intrinsics_.distortion_model = msg->distortion_model;
  current_camera_intrinsics_.distortion_coefficients.assign(msg->d.begin(), msg->d.end());
  is_camera_info_received_.store(true);
  MLOGGER_INFO("Camera intrinsics: fx={}, fy={}, cx={}, cy={}",
               current_camera_intrinsics_.fx, current_camera_intrinsics_.fy,
               current_camera_intrinsics_.cx, current_camera_intrinsics_.cy);
}

void AprilTagLocalization::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg == nullptr || !is_camera_info_received_.load())
  {
    return;
  }
  try
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    cv_bridge_shared_ = cv_bridge::toCvShare(msg, "bgr8");
    is_image_received_.store(true);
  } catch (const cv_bridge::Exception &e)
  {
    MLOGGER_ERROR("cv_bridge conversion failed: {}", e.what());
    return;
  }
  cond_var_.notify_one();
}

void AprilTagLocalization::drawResult(cv::Mat &out_image, apriltag_detection_t *det)
{
  const std::vector<cv::Scalar> colors = {
      cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
      cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 0)};
  for (int i = 0; i < 4; ++i)
  {
    cv::circle(out_image, cv::Point(det->p[i][0], det->p[i][1]), 5, colors[i], -1);
  }
  cv::putText(out_image, std::to_string(det->id), cv::Point(det->c[0], det->c[1]),
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 153, 0), 2);
}

const AprilTagLocalization::TagObservation *AprilTagLocalization::findObservation(
    const std::vector<TagObservation> &observations, int id) const
{
  for (const auto &observation : observations)
  {
    if (observation.id == id)
    {
      return &observation;
    }
  }
  return nullptr;
}

tf2::Transform AprilTagLocalization::makeSingleTagResult(
    const TagObservation &observation, int &yaw_source) const
{
  tf2::Transform result = observation.dock_center_camera_link;
  if (hold_baseline_yaw_on_single_ && has_baseline_yaw_)
  {
    double roll, pitch, ignored_yaw;
    tf2::Matrix3x3(result.getRotation()).getRPY(roll, pitch, ignored_yaw);
    tf2::Quaternion q;
    q.setRPY(roll, pitch, last_baseline_yaw_);
    result.setRotation(q);
    yaw_source = 2;
  } else
  {
    yaw_source = 0;
  }
  return result;
}

tf2::Transform AprilTagLocalization::makeEntryPose(const tf2::Transform &dock_center) const
{
  return dock_center * dock_center_to_entry_;
}

AprilTagLocalization::DockPerception AprilTagLocalization::fuseEnhanced(
    const std::vector<TagObservation> &observations)
{
  DockPerception perception;
  perception.tag_count = static_cast<int>(observations.size());
  if (observations.empty())
  {
    return perception;
  }

  const auto nearest_it = std::min_element(
      observations.begin(), observations.end(),
      [](const TagObservation &a, const TagObservation &b) { return a.distance < b.distance; });
  const TagObservation &nearest = *nearest_it;
  const TagObservation *tag_a = findObservation(observations, baseline_tag_id_a_);
  const TagObservation *tag_b = findObservation(observations, baseline_tag_id_b_);

  if (tag_a != nullptr && tag_b != nullptr)
  {
    const tf2::Vector3 center_a = tag_a->dock_center_camera_link.getOrigin();
    const tf2::Vector3 center_b = tag_b->dock_center_camera_link.getOrigin();
    perception.center_diff = (center_a - center_b).length();

    double ra, pa, ya, rb, pb, yb;
    tf2::Matrix3x3(tag_a->dock_center_camera_link.getRotation()).getRPY(ra, pa, ya);
    tf2::Matrix3x3(tag_b->dock_center_camera_link.getRotation()).getRPY(rb, pb, yb);
    const double yaw_diff_deg =
        std::abs(normalizeAngle(ya - yb)) / DEG_TO_RAD;

    // T_tag_dock is configured. Its inverse is T_dock_tag, which is the correct
    // representation for the known baseline in dock coordinates.
    const tf2::Vector3 tag_a_in_dock =
        dock_pose_ext_map_.at(baseline_tag_id_a_).inverse().getOrigin();
    const tf2::Vector3 tag_b_in_dock =
        dock_pose_ext_map_.at(baseline_tag_id_b_).inverse().getOrigin();
    const tf2::Vector3 baseline_dock = tag_b_in_dock - tag_a_in_dock;
    const tf2::Vector3 baseline_camera =
        tag_b->tag_pose_camera_link.getOrigin() - tag_a->tag_pose_camera_link.getOrigin();

    const double baseline_dock_xy = std::hypot(baseline_dock.x(), baseline_dock.y());
    const double baseline_camera_xy =
        std::hypot(baseline_camera.x(), baseline_camera.y());
    const bool baseline_valid = baseline_dock_xy >= min_baseline_length_m_ &&
                                baseline_camera_xy >= min_baseline_length_m_;
    perception.dual_consistent =
        perception.center_diff <= dual_tag_consistency_threshold_m_ &&
        yaw_diff_deg <= dual_tag_yaw_consistency_deg_ && baseline_valid;

    if (perception.dual_consistent)
    {
      tf2::Vector3 fused_position;
      if (distance_weighted_position_)
      {
        const double wa = 1.0 / std::max(tag_a->distance, 1.0e-6);
        const double wb = 1.0 / std::max(tag_b->distance, 1.0e-6);
        fused_position = (center_a * wa + center_b * wb) / (wa + wb);
      } else
      {
        fused_position = (center_a + center_b) * 0.5;
      }

      const double yaw_camera = std::atan2(baseline_camera.y(), baseline_camera.x());
      const double yaw_dock = std::atan2(baseline_dock.y(), baseline_dock.x());
      const double fused_yaw = normalizeAngle(
          yaw_camera - yaw_dock + baseline_yaw_offset_deg_ * DEG_TO_RAD);

      // Preserve the average roll/pitch behavior while replacing only yaw.
      const tf2::Transform averaged = averageTransforms(
          {tag_a->dock_center_camera_link, tag_b->dock_center_camera_link});
      double roll, pitch, ignored_yaw;
      tf2::Matrix3x3(averaged.getRotation()).getRPY(roll, pitch, ignored_yaw);
      tf2::Quaternion q;
      q.setRPY(roll, pitch, fused_yaw);
      perception.center = tf2::Transform(q, fused_position);
      perception.yaw_source = 1;
      perception.valid = true;
      last_baseline_yaw_ = fused_yaw;
      has_baseline_yaw_ = true;
    } else
    {
      perception.center = makeSingleTagResult(nearest, perception.yaw_source);
      perception.valid = true;
      MLOGGER_WARN("Dual-tag rejected: center_diff={}m, yaw_diff={}deg, baseline_valid={}",
                   perception.center_diff, yaw_diff_deg, baseline_valid);
    }
  } else
  {
    perception.center = makeSingleTagResult(nearest, perception.yaw_source);
    perception.valid = true;
  }

  perception.entry = makeEntryPose(perception.center);
  return perception;
}

void AprilTagLocalization::publishLegacyPose(const tf2::Transform &pose)
{
  result_array_pose_msg_.data.resize(6);
  transformToXYZRPY(pose,
                    result_array_pose_msg_.data[0], result_array_pose_msg_.data[1],
                    result_array_pose_msg_.data[2], result_array_pose_msg_.data[3],
                    result_array_pose_msg_.data[4], result_array_pose_msg_.data[5]);
  output_result_array_pose_publisher_->publish(result_array_pose_msg_);
}

void AprilTagLocalization::publishPerception(const DockPerception &perception)
{
  std_msgs::msg::Float64MultiArray msg;
  if (!perception.valid)
  {
    perception_result_publisher_->publish(msg);
    return;
  }
  msg.data.resize(17);
  transformToXYZRPY(perception.center,
                    msg.data[0], msg.data[1], msg.data[2],
                    msg.data[3], msg.data[4], msg.data[5]);
  transformToXYZRPY(perception.entry,
                    msg.data[6], msg.data[7], msg.data[8],
                    msg.data[9], msg.data[10], msg.data[11]);
  msg.data[12] = static_cast<double>(perception.tag_count);
  msg.data[13] = static_cast<double>(perception.yaw_source);
  msg.data[14] = perception.dual_consistent ? 1.0 : 0.0;
  msg.data[15] = perception.valid ? 1.0 : 0.0;
  msg.data[16] = perception.center_diff;
  perception_result_publisher_->publish(msg);
}

bool AprilTagLocalization::detect()
{
  cv::Mat image_mat;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!cv_bridge_shared_)
    {
      is_image_received_.store(false);
      return false;
    }
    image_mat = cv_bridge_shared_->image.clone();
    is_image_received_.store(false);
  }

  cv::Mat image_gray;
  cv::cvtColor(image_mat, image_gray, cv::COLOR_BGR2GRAY);
  image_u8_t img = {image_gray.cols, image_gray.rows, image_gray.cols, image_gray.data};
  detections_ = apriltag_detector_detect(td_ptr_, &img);

  result_array_pose_msg_.data.clear();
  const int result_size = zarray_size(detections_);
  std::vector<tf2::Transform> original_results;
  std::vector<TagObservation> observations;

  for (int i = 0; i < result_size; ++i)
  {
    apriltag_detection_t *det = nullptr;
    zarray_get(detections_, i, &det);
    if (det == nullptr || det->hamming > max_hamming_)
    {
      continue;
    }
    const auto offset_it = dock_pose_ext_map_.find(det->id);
    if (offset_it == dock_pose_ext_map_.end())
    {
      MLOGGER_EVERY_N_WARN(10, "TAG ID {} is not configured; skipping.", det->id);
      continue;
    }

    apriltag_detection_info_t info{};
    info.det = det;
    info.tagsize = tag_size_;
    info.fx = current_camera_intrinsics_.fx;
    info.fy = current_camera_intrinsics_.fy;
    info.cx = current_camera_intrinsics_.cx;
    info.cy = current_camera_intrinsics_.cy;

    apriltag_pose_t pose{};
    const double err = estimate_tag_pose(&info, &pose);
    if (!std::isfinite(err) || err > max_pose_error_ || pose.R == nullptr || pose.t == nullptr)
    {
      if (pose.R != nullptr) matd_destroy(pose.R);
      if (pose.t != nullptr) matd_destroy(pose.t);
      continue;
    }

    const tf2::Transform tag_pose_raw = apriltagPoseToTf2(pose);
    matd_destroy(pose.R);
    matd_destroy(pose.t);

    const tf2::Transform tag_pose_camera_link =
        camera2camera_link * tag_pose_raw * camera_tag2ros_;
    const tf2::Transform dock_center = tag_pose_camera_link * offset_it->second;

    original_results.push_back(dock_center);
    observations.push_back(TagObservation{
        det->id, tag_pose_camera_link.getOrigin().length(), err,
        tag_pose_camera_link, dock_center});
  }

  if (use_baseline_yaw_)
  {
    const DockPerception perception = fuseEnhanced(observations);
    if (perception.valid)
    {
      publishLegacyPose(perception.center);
      publishPerception(perception);
    } else
    {
      output_result_array_pose_publisher_->publish(result_array_pose_msg_);
      publishPerception(perception);
    }
  } else
  {
    // Exact legacy behavior: average all configured valid dock-center transforms.
    if (!original_results.empty())
    {
      publishLegacyPose(averageTransforms(original_results));
    } else
    {
      output_result_array_pose_publisher_->publish(result_array_pose_msg_);
    }
    // New diagnostic topic remains empty while enhanced mode is disabled.
    perception_result_publisher_->publish(std_msgs::msg::Float64MultiArray());
  }

  apriltag_detections_destroy(detections_);
  detections_ = nullptr;
  return !observations.empty();
}

void AprilTagLocalization::main_loop()
{
  while (running_.load() && rclcpp::ok())
  {
    std::unique_lock<std::mutex> lock(image_mutex_);
    cond_var_.wait_for(lock, std::chrono::milliseconds(2000), [this] {
      return is_image_received_.load() || !running_.load() || !rclcpp::ok();
    });
    if (!running_.load() || !rclcpp::ok())
    {
      break;
    }
    if (!is_image_received_.load())
    {
      MLOGGER_WARN("New image not received in 2000ms.");
      continue;
    }
    lock.unlock();
    detect();
  }
}

int main(int argc, char *argv[])
{
  MLOGGER_MODULE_INIT("logs", "perception", "apriltag_localization");
  rclcpp::init(argc, argv);
  {
    AprilTagLocalization node("apriltag_node");
    node.run();
  }
  rclcpp::shutdown();
  return 0;
}
