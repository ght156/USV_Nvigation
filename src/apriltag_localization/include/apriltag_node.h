#pragma once
#include "mlogger/mlogger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <xr_msgs/msg/rgbd.hpp>
// #include <xr_msgs/msg/box_pos_simple.hpp>
// #include <xr_msgs/msg/local_pose.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/sync_policies/approximate_time.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>

#ifdef __linux__
#include <unistd.h>
#endif

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/zarray.h"

#include "apriltag/apriltag_pose.h"

#define HAMM_HIST_MAX 10

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <vector>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <m_common/msg/detection_obj_list.hpp>
#include <tf2/LinearMath/Vector3.h>                // tf2::Vector3
#include <tf2/LinearMath/Quaternion.h>             // tf2::Quaternion
#include <tf2/LinearMath/Transform.h>              // tf2::Transform
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::fromMsg, tf2::toMsg
#include <tf2_ros/transform_broadcaster.h>
namespace perception {

struct CameraIntrinsics {
  float              fx;
  float              fy;
  float              cx;
  float              cy;
  std::vector<float> distortion_coefficients;
  std::string        distortion_model;
};
constexpr double DEG_TO_RAD = M_PI / 180.0; // 或使用 3.14159265358979323846 / 180.0

inline double degreesToRadians(double degrees)
{
  return degrees * DEG_TO_RAD;
}

/**
 * Convert an apriltag_pose_t to Eigen::Affine3d using Eigen::Map for fast data access.
 *
 * @param pose AprilTag pose (R: 3x3 row‑major, t: 3x1)
 * @return Eigen affine transformation
 */
inline Eigen::Affine3d poseToEigenFast(const apriltag_pose_t &pose)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  // Map the rotation matrix as a row‑major 3x3 matrix (no data copy yet)
  if (pose.R != nullptr && pose.R->nrows == 3 && pose.R->ncols == 3)
  {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_map(pose.R->data);
    transform.linear() = R_map; // Eigen handles the row‑major → column‑major conversion
  }

  // Map the translation vector as a 3x1 column vector (layout is the same)
  if (pose.t != nullptr && pose.t->nrows == 3 && pose.t->ncols == 1)
  {
    Eigen::Map<const Eigen::Vector3d> t_map(pose.t->data);
    transform.translation() = t_map;
  }

  return transform;
}

inline Eigen::Matrix3d rotation_matrix_x(double theta)
{
  double          cos_t = std::cos(theta);
  double          sin_t = std::sin(theta);
  Eigen::Matrix3d R;
  R << 1, 0, 0, 0, cos_t, -sin_t, 0, sin_t, cos_t;
  return R;
}
inline Eigen::Matrix3d rotation_matrix_z(double theta)
{
  double          cos_t = std::cos(theta);
  double          sin_t = std::sin(theta);
  Eigen::Matrix3d R;
  R << cos_t, 0, 0, 0, cos_t, -sin_t, 0, sin_t, cos_t;
  return R;
}

inline Eigen::Matrix3d rotation_matrix_y(double theta)
{
  double          cos_t = std::cos(theta);
  double          sin_t = std::sin(theta);
  Eigen::Matrix3d R;
  R << cos_t, 0, sin_t, 0, 1, 0, -sin_t, 0, cos_t;
  return R;
}
// 将角度规范化到 [-π, π]
inline double normalizeAngle(double angle)
{
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle > M_PI)
    angle -= 2.0 * M_PI;
  else if (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

inline tf2::Matrix3x3 rotation_tfmatrix_x(double theta)
{
  double cos_t = std::cos(theta);
  double sin_t = std::sin(theta);
  // 绕 X 轴旋转矩阵：
  // [1, 0,    0;
  //  0, cos, -sin;
  //  0, sin,  cos]
  return tf2::Matrix3x3(1, 0, 0, 0, cos_t, -sin_t, 0, sin_t, cos_t);
}

inline tf2::Matrix3x3 rotation_tfmatrix_y(double theta)
{
  double cos_t = std::cos(theta);
  double sin_t = std::sin(theta);
  // 绕 Y 轴旋转矩阵：
  // [cos,  0, sin;
  //  0,    1, 0;
  // -sin,  0, cos]
  return tf2::Matrix3x3(cos_t, 0, sin_t, 0, 1, 0, -sin_t, 0, cos_t);
}

inline tf2::Matrix3x3 rotation_tfmatrix_z(double theta)
{
  double cos_t = std::cos(theta);
  double sin_t = std::sin(theta);
  // 绕 Z 轴旋转矩阵（修正后的正确形式）：
  // [cos, -sin, 0;
  //  sin,  cos, 0;
  //  0,    0,   1]
  return tf2::Matrix3x3(cos_t, -sin_t, 0, sin_t, cos_t, 0, 0, 0, 1);
}

// 将ZYX欧拉角转换为最接近零的等价表示
// inline void normalizeEulerZYX(double &yaw, double &pitch, double &roll)
// {
//   // 规范化到 [-π, π]
//   yaw   = normalizeAngle(yaw);
//   pitch = normalizeAngle(pitch);
//   roll  = normalizeAngle(roll);

//   // 等价表示： (yaw + π, π - pitch, roll + π)
//   double yaw_alt   = normalizeAngle(yaw + M_PI);
//   double pitch_alt = normalizeAngle(M_PI - pitch);
//   double roll_alt  = normalizeAngle(roll + M_PI);

//   // 计算原始和替代表示的绝对值之和
//   double orig_norm = std::abs(yaw) + std::abs(pitch) + std::abs(roll);
//   double alt_norm  = std::abs(yaw_alt) + std::abs(pitch_alt) + std::abs(roll_alt);

//   // 选择范数更小的表示
//   // if (alt_norm < orig_norm)
//   // {
//   //   yaw   = yaw_alt;
//   //   pitch = pitch_alt;
//   //   roll  = roll_alt;
//   // }
// }
class RealAprilTagConverter {
public:
  static void apriltagToROS(const apriltag_pose_t &tag_pose,
                            double                &ros_x,
                            double                &ros_y,
                            double                &ros_z,
                            double                &ros_roll,
                            double                &ros_pitch,
                            double                &ros_yaw)
  {
    // 定义从物理坐标系到ROS坐标系的旋转矩阵
    Eigen::Matrix3d R_phys2ros;
    R_phys2ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    // 将apriltag的旋转矩阵映射为Eigen矩阵（行优先）
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_tag_to_phys(tag_pose.R->data);
    Eigen::Map<Eigen::Matrix<double, 3, 1>>                  t_tag_in_phys(tag_pose.t->data);

    Eigen::Matrix3d R2 = rotation_matrix_y(-M_PI_2) * rotation_matrix_x(M_PI_2);

    // 计算新的旋转和平移
    Eigen::Matrix3d R_tag_to_ros = R_phys2ros * R_tag_to_phys * R2;
    Eigen::Vector3d t_tag_in_ros = R_phys2ros * t_tag_in_phys;
    ros_x                        = t_tag_in_ros.x();
    ros_y                        = t_tag_in_ros.y();
    ros_z                        = t_tag_in_ros.z();
    // Eigen::Quaterniond quat(R_tag_to_ros);
    // quat.normalized();
    // quat.w();
    // quat.x();
    // quat.y();
    // quat.z();
    // Eigen::Vector3d rpy_in_ros = R_tag_to_ros.eulerAngles(2, 1, 0);
    // normalizeEulerZYX(rpy_in_ros(2),rpy_in_ros(1), rpy_in_ros(0));
    // ros_roll                   = rpy_in_ros(2);
    // ros_pitch                  = rpy_in_ros(1);
    // ros_yaw                    = rpy_in_ros(0);

    // std::cout << "ypr_in_ros: " << rpy_in_ros << std::endl;
    matrixToRPY(R_tag_to_ros.matrix(), ros_roll, ros_pitch, ros_yaw);
    return;
  }

private:
  static void matrixToRPY(const Eigen::Matrix3d &R, double &roll, double &pitch, double &yaw)
  {
    if (R(2, 0) < -0.999)
    {
      pitch = -M_PI / 2;
      roll  = 0;
      yaw   = atan2(R(0, 1), R(0, 2));
    } else if (R(2, 0) > 0.999)
    {
      pitch = M_PI / 2;
      roll  = 0;
      yaw   = atan2(-R(0, 1), -R(0, 2));
    } else
    {
      pitch = asin(-R(2, 0));
      roll  = atan2(R(2, 1), R(2, 2));
      yaw   = atan2(R(1, 0), R(0, 0));
    }
  }
};
class AprilTagLocalization {
public:
  explicit AprilTagLocalization(const std::string &node_name);
  ~AprilTagLocalization();
  void run();

private:
  bool initConfig();
  bool initModel();
  bool intiNode();
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool detect();
  void drawResult(cv::Mat &out_image, apriltag_detection_t *det);
  void publishDetectionObjListAndTf(
      const m_common::msg::DetectionObjList                     &detection_obj_list,
      const std::vector<geometry_msgs::msg::TransformStamped> &tag_transforms);
  void main_loop();

private:
  rclcpp::Node::SharedPtr private_node_ptr_;
  rclcpp::Logger          logger_;
  const std::string       WORKSPACE_DIR;
  std::thread             main_loop_;
  std::mutex              camera_info_mutex_, image_mutex_;

  std::string camera_info_topic_, image_topic_, detection_objects_topic_;
  std::string frame_id_{"camera_link"};
  std::string dock_frame_id_{"dock_frame"};
  double      tag_size_ = 0.5; // AprilTag的实际尺寸，单位为米
  // sensor_msgs::msg::CameraInfo current_camera_info_;
  std::string      current_apriltag_family_name_;
  CameraIntrinsics current_camera_intrinsics_;
  // std::shared_ptr<sensor_msgs::msg::Image>                      current_image_;
  cv_bridge::CvImageConstPtr                                    cv_bridge_shared_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
  std::atomic<bool>                                             is_camera_info_received_ = false;
  std::atomic<bool>                                             is_image_received_       = false;
  apriltag_family_t                                            *tf_ptr_                  = nullptr;
  apriltag_detector_t                                          *td_ptr_                  = nullptr;
  zarray_t                                                     *detections_              = nullptr;

  std::function<void(apriltag_family_t *)> destory_apriltag_family_t_;
  std::condition_variable                  cond_var_;

private:
  std::vector<int>                        tag_ids_;
  std::unordered_map<int, tf2::Transform> dock_pose_ext_map_;

  const tf2::Transform camera2camera_link =
      tf2::Transform(tf2::Matrix3x3(0, 0, 1, -1, 0, 0, 0, -1, 0), tf2::Vector3(0, 0, 0));

  const tf2::Transform camera_tag2ros_ = tf2::Transform(
      rotation_tfmatrix_y(-M_PI_2) * rotation_tfmatrix_x(M_PI_2), tf2::Vector3(0, 0, 0));

  rclcpp::Publisher<m_common::msg::DetectionObjList>::SharedPtr output_detection_objects_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;
};

} // namespace perception
