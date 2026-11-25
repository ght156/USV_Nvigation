#include "usv_docking/docking_motion_controller.hpp"

#include "usv_docking/docking_logging.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <cmath>

namespace usv_docking
{

DockingMotionController::DockingMotionController(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_motion_controller", options)
{
  MLOGGER_NODE_REGISTER(docking_logging::log_root(), "docking", this->get_name());
  declare_parameter("target_mode_topic", "/docking/target_mode");
  declare_parameter("dock_pose_topic", "/docking/dock_pose");
  declare_parameter("pose_source_topic", "/docking/pose_source");
  declare_parameter("safety_stop_topic", "/docking/safety_stop");
  declare_parameter("test_only", true);
  declare_parameter("cmd_vel_topic", "/cmd_vel_nav");
  declare_parameter("cmd_vel_test_topic", "/docking/cmd_vel_test");
  declare_parameter("control_rate", 20.0);
  declare_parameter("pose_msg_timeout_sec", 1.0);
  declare_parameter("mode_msg_timeout_sec", 1.0);
  declare_parameter("safety_msg_timeout_sec", 1.0);
  declare_parameter("blind_exit_enabled", false);
  declare_parameter("hold_zero_publish_sec", 1.0);
  declare_parameter("staging_x", -2.0);
  declare_parameter("final_target_x", 0.0);
  declare_parameter("max_reverse_speed", 0.8);
  declare_parameter("max_yaw_rate", 0.12);
  declare_parameter("max_linear_accel", 0.30);
  declare_parameter("max_angular_accel", 0.50);
  declare_parameter("yaw_rate_topic", "/mavros/gps_input/local");
  declare_parameter("yaw_rate_lpf_alpha", 0.35);
  declare_parameter("yaw_rate_timeout_sec", 0.30);
  declare_parameter("kd_yaw", 0.7);
  declare_parameter("approach_reexit_margin", 0.35);
  declare_parameter("approach_axis_y_far", 0.8);
  declare_parameter("approach_axis_far_x", -4.5);
  declare_parameter("approach_offaxis_y", 0.8);
  declare_parameter("approach_axis_y_near", 0.55);
  declare_parameter("approach_axis_pullout", 2.0);
  declare_parameter("approach_axis_crab_deg", 25.0);
  declare_parameter("approach_axis_max_out_x", -8.0);
  declare_parameter("approach_cone_enabled", true);
  declare_parameter("cone_apex_x", -1.0);
  declare_parameter("cone_apex_half_width", 0.25);
  declare_parameter("cone_half_angle_deg", 12.0);
  declare_parameter("approach_offaxis_speed", 0.15);
  declare_parameter("approach_offaxis_x", -4.0);
  declare_parameter("approach_degenerate_dist", 0.6);
  declare_parameter("approach_degenerate_speed", 0.18);
  declare_parameter("approach_y_near", 0.15);
  declare_parameter("search_angular_speed", 0.15);
  declare_parameter("backin_max_search_angle_deg", 8.0);
  declare_parameter("approach_speed", 0.60);
  declare_parameter("approach_turn_deg", 25.0);
  declare_parameter("approach_crab_deg", 8.0);
  declare_parameter("approach_slow_y", 1.0);
  declare_parameter("approach_slow_dist", 1.50);
  declare_parameter("approach_min_speed", 0.25);
  declare_parameter("back_in_speed_normal", 0.25);
  declare_parameter("back_in_speed_slow", 0.15);
  declare_parameter("back_in_gate1_y", 0.20);
  declare_parameter("back_in_gate1_yaw_deg", 5.0);
  declare_parameter("back_in_gate2_y", 0.35);
  declare_parameter("back_in_gate2_yaw_deg", 15.0);
  declare_parameter("final_dock_speed", 0.15);
  declare_parameter("final_push_speed", 0.4);
  declare_parameter("docked_x_tol", 0.25);
  declare_parameter("docked_y_tol", 0.10);
  declare_parameter("abort_exit_speed", 0.30);
  declare_parameter("undock_speed", 0.45);
  declare_parameter("exit_turn_deg", 30.0);
  declare_parameter("ky_approach", 0.40);
  declare_parameter("kyaw_approach", 0.45);
  declare_parameter("kyaw_align", 0.45);
  declare_parameter("align_arc_speed", 0.15);
  declare_parameter("align_arc_x_limit", -1.5);
  declare_parameter("ky_back", 0.80);
  declare_parameter("kyaw_back", 0.80);
  declare_parameter("kx_final", 0.25);
  declare_parameter("back_in_max_yaw_rate", 0.12);

  YawRateParams yaw_rate_params;
  yaw_rate_params.ema_alpha = get_parameter("yaw_rate_lpf_alpha").as_double();
  yaw_rate_params.timeout_sec = get_parameter("yaw_rate_timeout_sec").as_double();
  yaw_rate_estimator_ = YawRateEstimator(yaw_rate_params);

  const bool test_only = get_parameter("test_only").as_bool();
  const std::string output_topic = test_only ?
    get_parameter("cmd_vel_test_topic").as_string() : get_parameter("cmd_vel_topic").as_string();
  const auto qos = rclcpp::QoS(10);
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, qos);
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("target_mode_topic").as_string(), qos,
    std::bind(&DockingMotionController::mode_callback, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("dock_pose_topic").as_string(), qos,
    std::bind(&DockingMotionController::pose_callback, this, std::placeholders::_1));
  source_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("pose_source_topic").as_string(), qos,
    std::bind(&DockingMotionController::source_callback, this, std::placeholders::_1));
  safety_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("safety_stop_topic").as_string(), qos,
    std::bind(&DockingMotionController::safety_callback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("yaw_rate_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&DockingMotionController::odom_callback, this, std::placeholders::_1));
  const double rate = get_parameter("control_rate").as_double();
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
    std::bind(&DockingMotionController::control_loop, this));
  MLOGGER_INFO(
    "docking_motion_controller 已启动，输出 -> {}{}", output_topic,
    test_only ? "（test_only）" : "（真实 cmd_vel!）");
}

DockingMotionController::~DockingMotionController()
{
  if (!cmd_pub_) return;
  geometry_msgs::msg::Twist zero;
  for (int i = 0; i < 3; ++i) cmd_pub_->publish(zero);
}

void DockingMotionController::mode_callback(const std_msgs::msg::String::SharedPtr message)
{
  mode_ = message->data;
  mode_msg_time_ = get_clock()->now();
}

void DockingMotionController::pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const double x = message->pose.position.x;
  const double y = message->pose.position.y;
  const double yaw = yaw_from_quat(
    message->pose.orientation.z, message->pose.orientation.w);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    MLOGGER_THROTTLE_WARN(2000, "丢弃非有限位姿帧（NaN/inf）");
    return;
  }
  pose_ = Pose2D{x, y, yaw};
  pose_msg_time_ = get_clock()->now();
}

void DockingMotionController::source_callback(const std_msgs::msg::String::SharedPtr message)
{
  pose_source_ = message->data;
  source_msg_time_ = get_clock()->now();
}

void DockingMotionController::safety_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  safety_stop_ = message->data;
  safety_msg_time_ = get_clock()->now();
}

void DockingMotionController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const auto & q = message->pose.pose.orientation;
  const double yaw = yaw_from_quat(q.z, q.w);
  yaw_rate_estimator_.update(yaw, get_clock()->now().seconds());
}

bool DockingMotionController::fresh(
  const std::optional<rclcpp::Time> & stamp, double timeout)
{
  return stamp && (get_clock()->now() - *stamp).seconds() <= timeout;
}

MotionParams DockingMotionController::load_params() const
{
  MotionParams p;
  auto d = [this](const char * name) {return get_parameter(name).as_double();};
  auto b = [this](const char * name) {return get_parameter(name).as_bool();};
  p.control_rate = d("control_rate");
  p.hold_zero_publish_sec = d("hold_zero_publish_sec");
  p.blind_exit_enabled = b("blind_exit_enabled");
  p.staging_x = d("staging_x");
  p.final_target_x = d("final_target_x");
  p.max_reverse_speed = d("max_reverse_speed");
  p.max_yaw_rate = d("max_yaw_rate");
  p.max_linear_accel = d("max_linear_accel");
  p.max_angular_accel = d("max_angular_accel");
  p.kd_yaw = d("kd_yaw");
  p.approach_reexit_margin = d("approach_reexit_margin");
  p.approach_axis_y_far = d("approach_axis_y_far");
  p.approach_axis_far_x = d("approach_axis_far_x");
  p.approach_axis_y_near = d("approach_axis_y_near");
  p.approach_axis_pullout = d("approach_axis_pullout");
  p.approach_axis_crab_deg = d("approach_axis_crab_deg");
  p.approach_axis_max_out_x = d("approach_axis_max_out_x");
  p.approach_cone_enabled = b("approach_cone_enabled");
  p.cone_apex_x = d("cone_apex_x");
  p.cone_apex_half_width = d("cone_apex_half_width");
  p.cone_half_angle_deg = d("cone_half_angle_deg");
  p.approach_offaxis_y = d("approach_offaxis_y");
  p.approach_offaxis_speed = d("approach_offaxis_speed");
  p.approach_offaxis_x = d("approach_offaxis_x");
  p.approach_degenerate_dist = d("approach_degenerate_dist");
  p.approach_degenerate_speed = d("approach_degenerate_speed");
  p.approach_y_near = d("approach_y_near");
  p.search_angular_speed = d("search_angular_speed");
  p.backin_max_search_angle_deg = d("backin_max_search_angle_deg");
  p.approach_speed = d("approach_speed");
  p.approach_turn_deg = d("approach_turn_deg");
  p.approach_crab_deg = d("approach_crab_deg");
  p.approach_slow_y = d("approach_slow_y");
  p.approach_slow_dist = d("approach_slow_dist");
  p.approach_min_speed = d("approach_min_speed");
  p.back_in_speed_normal = d("back_in_speed_normal");
  p.back_in_speed_slow = d("back_in_speed_slow");
  p.back_in_gate1_y = d("back_in_gate1_y");
  p.back_in_gate1_yaw_deg = d("back_in_gate1_yaw_deg");
  p.back_in_gate2_y = d("back_in_gate2_y");
  p.back_in_gate2_yaw_deg = d("back_in_gate2_yaw_deg");
  p.final_dock_speed = d("final_dock_speed");
  p.final_push_speed = d("final_push_speed");
  p.docked_x_tol = d("docked_x_tol");
  p.docked_y_tol = d("docked_y_tol");
  p.abort_exit_speed = d("abort_exit_speed");
  p.undock_speed = d("undock_speed");
  p.exit_turn_deg = d("exit_turn_deg");
  p.ky_approach = d("ky_approach");
  p.kyaw_approach = d("kyaw_approach");
  p.kyaw_align = d("kyaw_align");
  p.align_arc_speed = d("align_arc_speed");
  p.align_arc_x_limit = d("align_arc_x_limit");
  p.ky_back = d("ky_back");
  p.kyaw_back = d("kyaw_back");
  p.kx_final = d("kx_final");
  p.back_in_max_yaw_rate = d("back_in_max_yaw_rate");
  return p;
}

void DockingMotionController::control_loop()
{
  MotionInput input;
  input.mode = mode_;
  input.pose = pose_;
  input.pose_source = pose_source_;
  input.pose_fresh = fresh(pose_msg_time_, get_parameter("pose_msg_timeout_sec").as_double()) &&
    fresh(source_msg_time_, get_parameter("pose_msg_timeout_sec").as_double());
  input.mode_fresh = fresh(mode_msg_time_, get_parameter("mode_msg_timeout_sec").as_double());
  input.safety_fresh = fresh(
    safety_msg_time_, get_parameter("safety_msg_timeout_sec").as_double());
  input.safety_stop = safety_stop_;
  input.yaw_rate = yaw_rate_estimator_.value(get_clock()->now().seconds());
  const auto output = core_.step(input, load_params());
  if (!is_hold_mode(mode_) && output.linear == 0.0 && output.angular == 0.0 &&
    (safety_stop_ || !input.safety_fresh || !input.mode_fresh))
  {
    MLOGGER_THROTTLE_WARN(2000, "must-stop：安全信号或输入断流，指令立即清零（绕过斜坡）");
  }
  if (!output.publish) return;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = output.linear;
  twist.angular.z = output.angular;
  cmd_pub_->publish(twist);
}

}  // namespace usv_docking

RCLCPP_COMPONENTS_REGISTER_NODE(usv_docking::DockingMotionController)
