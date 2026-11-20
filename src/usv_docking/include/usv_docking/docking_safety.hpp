#ifndef USV_DOCKING__DOCKING_SAFETY_HPP_
#define USV_DOCKING__DOCKING_SAFETY_HPP_

#include "usv_docking/safety_core.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <limits>

namespace usv_docking
{

class DockingSafety : public rclcpp::Node
{
public:
  explicit DockingSafety(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void check();
  SafetyParams params() const;

  SafetyCore core_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr abort_request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr measurement_age_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<Pose2D> pose_;
  bool tag_visible_{false};
  std::string pose_source_{SRC_INVALID};
  double measurement_age_{std::numeric_limits<double>::infinity()};
  std::string state_{DockState::IDLE};
  rclcpp::Time last_pose_msg_time_;
  rclcpp::Time last_state_msg_time_;
};

}  // namespace usv_docking

#endif  // USV_DOCKING__DOCKING_SAFETY_HPP_
