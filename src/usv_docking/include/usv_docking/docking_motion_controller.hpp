#ifndef USV_DOCKING__DOCKING_MOTION_CONTROLLER_HPP_
#define USV_DOCKING__DOCKING_MOTION_CONTROLLER_HPP_

#include "usv_docking/motion_control_core.hpp"
#include "usv_docking/yaw_rate_estimator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <string>

namespace usv_docking
{

class DockingMotionController : public rclcpp::Node
{
public:
  explicit DockingMotionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockingMotionController();

private:
  void mode_callback(const std_msgs::msg::String::SharedPtr message);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void source_callback(const std_msgs::msg::String::SharedPtr message);
  void safety_callback(const std_msgs::msg::Bool::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  MotionParams load_params() const;
  bool fresh(const std::optional<rclcpp::Time> & stamp, double timeout);
  void control_loop();

  MotionControlCore core_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string mode_{MODE_HOLD};
  std::optional<Pose2D> pose_;
  std::string pose_source_{SRC_INVALID};
  std::optional<rclcpp::Time> pose_msg_time_;
  std::optional<rclcpp::Time> source_msg_time_;
  std::optional<rclcpp::Time> mode_msg_time_;
  std::optional<rclcpp::Time> safety_msg_time_;
  bool safety_stop_{false};
  YawRateEstimator yaw_rate_estimator_;
};

}  // namespace usv_docking

#endif  // USV_DOCKING__DOCKING_MOTION_CONTROLLER_HPP_
