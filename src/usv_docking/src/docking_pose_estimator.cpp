#include "usv_docking/docking_pose_estimator.hpp"

#include "usv_docking/docking_logging.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <chrono>
#include <cmath>

namespace usv_docking
{
namespace
{

Pose2D transform_to_2d(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & t = transform.transform;
  return {
    t.translation.x, t.translation.y,
    quat_to_planar_yaw(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)};
}

}  // namespace

DockingPoseEstimator::DockingPoseEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_pose_estimator", options)
{
  MLOGGER_NODE_REGISTER(docking_logging::log_root(), "docking", this->get_name());
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_link");
  declare_parameter("dock_frame", "dock_frame");
  declare_parameter("dock_est_frame", "dock_est");
  declare_parameter("tf_timeout_sec", 0.0);
  declare_parameter("odom_tf_timeout_sec", 3.0);
  declare_parameter("broadcast_tf", true);
  declare_parameter("dock_pose_topic", "/docking/dock_pose");
  declare_parameter("tag_visible_topic", "/docking/tag_visible");
  declare_parameter("pose_source_topic", "/docking/pose_source");
  declare_parameter("anchor_status_topic", "/docking/anchor_status");
  declare_parameter("measurement_age_topic", "/docking/measurement_age");
  declare_parameter("reset_anchor_topic", "/docking/reset_anchor");
  declare_parameter("publish_rate", 20.0);
  declare_parameter("ema_alpha_position", 0.25);
  declare_parameter("ema_alpha_yaw", 0.15);
  declare_parameter("max_position_jump", 0.60);
  declare_parameter("max_yaw_jump_deg", 20.0);
  declare_parameter("reloc_max_yaw_jump_deg", 10.0);
  declare_parameter("reloc_guard_escape_sec", 8.0);
  declare_parameter("reloc_max_dist_m", 4.0);
  declare_parameter("reloc_require_dual_tag", true);
  declare_parameter("jump_reject_unlock_frames", 12);
  declare_parameter("seed_frames", 8);
  declare_parameter("seed_max_spread_m", 0.40);
  declare_parameter("seed_max_yaw_spread_deg", 15.0);
  declare_parameter("tag_timeout", 0.60);
  declare_parameter("anchor_prediction_timeout_sec", 70.0);

  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  dock_frame_ = get_parameter("dock_frame").as_string();
  dock_est_frame_ = get_parameter("dock_est_frame").as_string();
  tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
  odom_tf_timeout_sec_ = get_parameter("odom_tf_timeout_sec").as_double();
  broadcast_tf_ = get_parameter("broadcast_tf").as_bool();
  anchor_params_.ema_alpha_position = get_parameter("ema_alpha_position").as_double();
  anchor_params_.ema_alpha_yaw = get_parameter("ema_alpha_yaw").as_double();
  anchor_params_.max_position_jump = get_parameter("max_position_jump").as_double();
  anchor_params_.max_yaw_jump_deg = get_parameter("max_yaw_jump_deg").as_double();
  anchor_params_.reloc_max_yaw_jump_deg = get_parameter("reloc_max_yaw_jump_deg").as_double();
  anchor_params_.reloc_guard_escape_sec = get_parameter("reloc_guard_escape_sec").as_double();
  anchor_params_.reloc_max_dist_m = get_parameter("reloc_max_dist_m").as_double();
  anchor_params_.reloc_require_dual_tag = get_parameter("reloc_require_dual_tag").as_bool();
  anchor_params_.jump_reject_unlock_frames = get_parameter("jump_reject_unlock_frames").as_int();
  anchor_params_.seed_frames = get_parameter("seed_frames").as_int();
  anchor_params_.seed_max_spread_m = get_parameter("seed_max_spread_m").as_double();
  anchor_params_.seed_max_yaw_spread_deg = get_parameter("seed_max_yaw_spread_deg").as_double();
  anchor_filter_ = AnchorFilter(anchor_params_);
  tag_timeout_sec_ = get_parameter("tag_timeout").as_double();
  prediction_timeout_sec_ = get_parameter("anchor_prediction_timeout_sec").as_double();

  const auto qos = rclcpp::QoS(10);
  dock_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    get_parameter("dock_pose_topic").as_string(), qos);
  tag_visible_pub_ = create_publisher<std_msgs::msg::Bool>(
    get_parameter("tag_visible_topic").as_string(), qos);
  pose_source_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("pose_source_topic").as_string(), qos);
  anchor_status_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("anchor_status_topic").as_string(), qos);
  measurement_age_pub_ = create_publisher<std_msgs::msg::Float32>(
    get_parameter("measurement_age_topic").as_string(), qos);
  reset_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("reset_anchor_topic").as_string(), qos,
    std::bind(&DockingPoseEstimator::reset_anchor, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  period_ = 1.0 / get_parameter("publish_rate").as_double();
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(period_),
    std::bind(&DockingPoseEstimator::update, this));
  MLOGGER_INFO(
    "docking_pose_estimator 已启动：{}/{}/{} -> {}", odom_frame_, base_frame_,
    dock_frame_, dock_est_frame_);
}

void DockingPoseEstimator::reset_anchor(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data) return;
  anchor_filter_.reset();
  last_vision_stamp_.reset();
  last_output_.reset();
  previous_source_ = SRC_INVALID;
  MLOGGER_INFO("锚点重置：清空 odom->dock_est 锁存与滤波状态");
}

std::pair<std::optional<Pose2D>, double> DockingPoseEstimator::lookup_2d(
  const std::string & target, const std::string & source, const rclcpp::Time & now)
{
  try {
    const auto transform = tf_timeout_sec_ > 0.0 ?
      tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec_)) :
      tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    const auto stamp = rclcpp::Time(transform.header.stamp, now.get_clock_type());
    return {transform_to_2d(transform), (now - stamp).seconds()};
  } catch (const tf2::TransformException &) {
    return {std::nullopt, std::numeric_limits<double>::infinity()};
  }
}

int DockingPoseEstimator::visible_tag_count(const rclcpp::Time & now)
{
  int count = 0;
  for (const char * frame : {"apriltag_0", "apriltag_1"}) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, frame, tf2::TimePointZero);
      const auto stamp = rclcpp::Time(transform.header.stamp, now.get_clock_type());
      if ((now - stamp).seconds() <= tag_timeout_sec_) ++count;
    } catch (const tf2::TransformException &) {
    }
  }
  return count;
}

void DockingPoseEstimator::update()
{
  const auto cycle_start = std::chrono::steady_clock::now();
  const auto now = get_clock()->now();
  auto [odom_pose, odom_age] = lookup_2d(odom_frame_, base_frame_, now);
  const bool odom_ok = odom_pose && odom_age <= odom_tf_timeout_sec_;
  if (!odom_ok) {
    MLOGGER_THROTTLE_WARN(2000, "odom TF 不可用（age={:.2f}s），输出 INVALID", odom_age);
  }
  auto [dock_pose, dock_age] = lookup_2d(base_frame_, dock_frame_, now);
  const bool vision_ok = dock_pose && dock_age <= tag_timeout_sec_;

  if (vision_ok && odom_ok) {
    const auto raw = compose_2d(*odom_pose, *dock_pose);
    const auto old_status = anchor_filter_.status();
    anchor_filter_.update(
      raw, std::hypot(dock_pose->x, dock_pose->y), now.seconds(),
      visible_tag_count(now));
    const auto new_status = anchor_filter_.status();
    if (old_status != new_status) {
      MLOGGER_INFO("anchor_status: {} -> {}", old_status, new_status);
    }
    last_vision_stamp_ = now - rclcpp::Duration::from_seconds(dock_age);
  }

  const double measurement_age = last_vision_stamp_ ?
    (now - *last_vision_stamp_).seconds() : std::numeric_limits<double>::infinity();
  std::string source = SRC_INVALID;
  if (odom_ok && anchor_filter_.anchor()) {
    if (measurement_age <= tag_timeout_sec_) {
      source = SRC_VISION;
    } else if (measurement_age <= prediction_timeout_sec_) {
      source = SRC_PREDICTION;
    }
  }
  if (source != previous_source_) {
    MLOGGER_INFO(
      "pose_source: {} -> {}（age={:.2f}s）", previous_source_, source, measurement_age);
    previous_source_ = source;
  }
  if (odom_ok && anchor_filter_.anchor()) {
    last_output_ = compose_2d(inverse_2d(*anchor_filter_.anchor()), *odom_pose);
  }
  publish(now, measurement_age, vision_ok, source);

  if (broadcast_tf_ && anchor_filter_.anchor()) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = dock_est_frame_;
    transform.transform.translation.x = anchor_filter_.anchor()->x;
    transform.transform.translation.y = anchor_filter_.anchor()->y;
    const auto q = yaw_to_quat(anchor_filter_.anchor()->yaw);
    transform.transform.rotation.x = q[0];
    transform.transform.rotation.y = q[1];
    transform.transform.rotation.z = q[2];
    transform.transform.rotation.w = q[3];
    tf_broadcaster_->sendTransform(transform);
  }

  const double cost = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - cycle_start).count();
  if (cost > 2.0 * period_) {
    ++overrun_warned_;
    MLOGGER_THROTTLE_WARN(
      5000, "本周期耗时 {:.0f}ms 超过定时器周期 {:.0f}ms ×2，TF 回调可能挨饿（已 {} 次）",
      cost * 1e3, period_ * 1e3, overrun_warned_);
  }
}

void DockingPoseEstimator::publish(
  const rclcpp::Time & now, double age, bool visible, const std::string & source)
{
  std_msgs::msg::Bool visible_message;
  visible_message.data = visible;
  tag_visible_pub_->publish(visible_message);
  std_msgs::msg::String source_message;
  source_message.data = source;
  pose_source_pub_->publish(source_message);
  std_msgs::msg::String anchor_message;
  anchor_message.data = anchor_filter_.status();
  anchor_status_pub_->publish(anchor_message);
  std_msgs::msg::Float32 age_message;
  age_message.data = static_cast<float>(age);
  measurement_age_pub_->publish(age_message);
  geometry_msgs::msg::PoseStamped pose_message;
  pose_message.header.stamp = now;
  pose_message.header.frame_id = dock_est_frame_;
  if (last_output_) {
    pose_message.pose.position.x = last_output_->x;
    pose_message.pose.position.y = last_output_->y;
    const auto q = yaw_to_quat(last_output_->yaw);
    pose_message.pose.orientation.x = q[0];
    pose_message.pose.orientation.y = q[1];
    pose_message.pose.orientation.z = q[2];
    pose_message.pose.orientation.w = q[3];
  }
  dock_pose_pub_->publish(pose_message);
}

}  // namespace usv_docking

RCLCPP_COMPONENTS_REGISTER_NODE(usv_docking::DockingPoseEstimator)
