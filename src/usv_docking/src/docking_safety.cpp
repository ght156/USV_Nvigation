#include "usv_docking/docking_safety.hpp"

#include "usv_docking/docking_logging.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <cmath>

namespace usv_docking
{

DockingSafety::DockingSafety(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_safety", options)
{
  MLOGGER_NODE_REGISTER(docking_logging::log_root(), "docking", this->get_name());
  declare_parameter("dock_pose_topic", "/docking/dock_pose");
  declare_parameter("tag_visible_topic", "/docking/tag_visible");
  declare_parameter("pose_source_topic", "/docking/pose_source");
  declare_parameter("measurement_age_topic", "/docking/measurement_age");
  declare_parameter("state_topic", "/docking/state");
  declare_parameter("safety_stop_topic", "/docking/safety_stop");
  declare_parameter("abort_request_topic", "/docking/abort_request");
  declare_parameter("check_rate", 10.0);
  declare_parameter("max_lateral_error", 0.50);
  declare_parameter("max_yaw_error_deg", 15.0);
  declare_parameter("odom_prediction_timeout", 10.0);
  declare_parameter("align_odom_prediction_timeout", 7.0);
  declare_parameter("abort_exit_odom_timeout", 60.0);
  declare_parameter("exit_pose_invalid_warn_sec", 5.0);
  declare_parameter("pose_topic_timeout_sec", 1.0);
  declare_parameter("state_topic_timeout_sec", 1.0);
  declare_parameter("max_docking_duration_sec", 150.0);
  declare_parameter("violation_cycles", 5);

  const auto qos = rclcpp::QoS(10);
  safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
    get_parameter("safety_stop_topic").as_string(), qos);
  abort_request_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("abort_request_topic").as_string(), qos);
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("dock_pose_topic").as_string(), qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      pose_ = Pose2D{
        message->pose.position.x, message->pose.position.y,
        yaw_from_quat(message->pose.orientation.z, message->pose.orientation.w)};
      last_pose_msg_time_ = get_clock()->now();
    });
  tag_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("tag_visible_topic").as_string(), qos,
    [this](const std_msgs::msg::Bool::SharedPtr message) {tag_visible_ = message->data;});
  source_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("pose_source_topic").as_string(), qos,
    [this](const std_msgs::msg::String::SharedPtr message) {pose_source_ = message->data;});
  measurement_age_sub_ = create_subscription<std_msgs::msg::Float32>(
    get_parameter("measurement_age_topic").as_string(), qos,
    [this](const std_msgs::msg::Float32::SharedPtr message) {measurement_age_ = message->data;});
  state_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("state_topic").as_string(), qos,
    [this](const std_msgs::msg::String::SharedPtr message) {
      state_ = message->data;
      last_state_msg_time_ = get_clock()->now();
    });
  last_pose_msg_time_ = get_clock()->now();
  last_state_msg_time_ = get_clock()->now();
  const double rate = get_parameter("check_rate").as_double();
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
    std::bind(&DockingSafety::check, this));
  MLOGGER_INFO("docking_safety 已启动（6 项检查）");
}

SafetyParams DockingSafety::params() const
{
  SafetyParams p;
  p.max_lateral_error = get_parameter("max_lateral_error").as_double();
  p.max_yaw_error_deg = get_parameter("max_yaw_error_deg").as_double();
  p.odom_prediction_timeout = get_parameter("odom_prediction_timeout").as_double();
  p.align_odom_prediction_timeout = get_parameter("align_odom_prediction_timeout").as_double();
  p.abort_exit_odom_timeout = get_parameter("abort_exit_odom_timeout").as_double();
  p.exit_pose_invalid_warn_sec = get_parameter("exit_pose_invalid_warn_sec").as_double();
  p.max_docking_duration_sec = get_parameter("max_docking_duration_sec").as_double();
  p.violation_cycles = get_parameter("violation_cycles").as_int();
  return p;
}

void DockingSafety::check()
{
  const auto now = get_clock()->now();
  SafetyInput input;
  input.pose = pose_;
  input.pose_source = pose_source_;
  input.measurement_age = measurement_age_;
  input.state = state_;
  input.pose_silent = (now - last_pose_msg_time_).seconds() >
    get_parameter("pose_topic_timeout_sec").as_double();
  input.fsm_silent = (now - last_state_msg_time_).seconds() >
    get_parameter("state_topic_timeout_sec").as_double();
  const auto output = core_.check(input, now.seconds(), params());
  if (input.fsm_silent && state_ != DockState::IDLE) {
    MLOGGER_THROTTLE_WARN(2000, "FSM 话题断流 -> safety_stop");
  } else if (input.pose_silent && state_ != DockState::IDLE) {
    MLOGGER_THROTTLE_WARN(2000, "位姿话题断流 -> safety_stop + abort");
  }
  std_msgs::msg::Bool stop_message;
  stop_message.data = output.stop;
  safety_stop_pub_->publish(stop_message);
  std_msgs::msg::String abort_message;
  abort_message.data = output.abort_reason;
  abort_request_pub_->publish(abort_message);
}

}  // namespace usv_docking

RCLCPP_COMPONENTS_REGISTER_NODE(usv_docking::DockingSafety)
