#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <m_common/msg/nav_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "workspace_nav/costmap_reinflater.hpp"
#include "workspace_nav/navigation_logging.hpp"
#include "workspace_nav/nav_status_logic.hpp"

namespace workspace_nav
{
using namespace std::chrono_literals;

class NavStatusAggregator : public rclcpp::Node
{
public:
  NavStatusAggregator()
  : Node("nav_status_aggregator")
  {
    declare_parameters();
    load_parameters();
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::QoS status_qos(1);
    status_qos.reliable().transient_local();
    nav_status_pub_ = create_publisher<m_common::msg::NavStatus>(
      get_parameter("nav_status_topic").as_string(), status_qos);
    task_event_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("task_event_topic").as_string(), rclcpp::QoS(50).reliable());

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    detail_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("status_detail_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::detail_callback, this, std::placeholders::_1), options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::odom_callback, this, std::placeholders::_1), options);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::cmd_callback, this, std::placeholders::_1), options);
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("gps_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&NavStatusAggregator::gps_callback, this, std::placeholders::_1), options);
    fw_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("follow_waypoints_status_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::waypoint_status_callback, this, std::placeholders::_1), options);
    planner_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("planner_status_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::planner_status_callback, this, std::placeholders::_1), options);
    follow_path_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("follow_path_status_topic").as_string(), 10,
      std::bind(&NavStatusAggregator::follow_path_callback, this, std::placeholders::_1), options);
    rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::SystemDefaultsQoS(),
      std::bind(&NavStatusAggregator::rosout_callback, this, std::placeholders::_1), options);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    stuck_tracker_ = std::make_unique<nav_status::StuckTracker>(
      stuck_radius_, stuck_timeout_, cmd_timeout_);
    if (auto_recover_enabled_) {
      reinflater_ = std::make_unique<CostmapReinflater>(
        this, get_parameter("local_costmap_node").as_string());
    }
    startup_stamp_ = wall_seconds();
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_)),
      std::bind(&NavStatusAggregator::publish_nav_status, this), callback_group_);
    tf_timer_ = create_wall_timer(1s, std::bind(&NavStatusAggregator::check_tf, this), callback_group_);
    if (sync_stuck_from_nav2_) {
      nav2_param_client_ = create_client<rcl_interfaces::srv::GetParameters>(
        get_parameter("controller_server_node").as_string() + "/get_parameters");
      const double period = std::max(2.0, get_parameter("nav2_param_poll_period_sec").as_double());
      nav2_poll_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
        std::bind(&NavStatusAggregator::poll_nav2_params, this), callback_group_);
    }
    NAV_LOG_INFO( "nav_status_aggregator started rate=%.1fHz vehicle=%s",
      publish_rate_, vehicle_id_.c_str());
  }

private:
  static double wall_seconds()
  {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  void declare_parameters()
  {
    declare_parameter("status_detail_topic", "/mission_bridge/status_detail");
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("cmd_vel_topic", "/cmd_vel_nav");
    declare_parameter("cmd_vel_timeout_sec", 2.0);
    declare_parameter("cmd_linear_deadband", 0.05);
    declare_parameter("cmd_angular_deadband", 0.03);
    declare_parameter("gps_topic", "/gps/fixed_cov");
    declare_parameter("follow_waypoints_status_topic", "/follow_waypoints/_action/status");
    declare_parameter("planner_status_topic", "/compute_path_to_pose/_action/status");
    declare_parameter("nav_status_topic", "/nav_status");
    declare_parameter("task_event_topic", "/task_event");
    declare_parameter("publish_rate", 2.0);
    declare_parameter("vehicle_id", "usv_001");
    declare_parameter("odom_timeout", 2.0);
    declare_parameter("min_odom_hz", 8.0);
    declare_parameter("gps_timeout", 5.0);
    declare_parameter("mission_bridge_timeout", 5.0);
    declare_parameter("cov_lost_threshold", 10.0);
    declare_parameter("cov_degraded_threshold", 1.0);
    declare_parameter("stuck_progress_timeout", 12.0);
    declare_parameter("stuck_movement_radius_m", 0.3);
    declare_parameter("sync_stuck_from_nav2", true);
    declare_parameter("controller_server_node", "/controller_server");
    declare_parameter("auto_recover_costmap_current", true);
    declare_parameter("auto_recover_silence_sec", 5.0);
    declare_parameter("auto_recover_confirm_sec", 10.0);
    declare_parameter("local_costmap_node", "/local_costmap/local_costmap");
    declare_parameter("follow_path_status_topic", "/follow_path/_action/status");
    declare_parameter("nav2_param_poll_period_sec", 10.0);
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_link");
  }

  void load_parameters()
  {
    vehicle_id_ = get_parameter("vehicle_id").as_string();
    publish_rate_ = std::clamp(get_parameter("publish_rate").as_double(), 0.5, 10.0);
    odom_timeout_ = std::max(0.5, get_parameter("odom_timeout").as_double());
    min_odom_hz_ = std::max(0.1, get_parameter("min_odom_hz").as_double());
    gps_timeout_ = std::max(0.5, get_parameter("gps_timeout").as_double());
    mb_timeout_ = std::max(1.0, get_parameter("mission_bridge_timeout").as_double());
    cov_lost_ = get_parameter("cov_lost_threshold").as_double();
    cov_degraded_ = get_parameter("cov_degraded_threshold").as_double();
    stuck_timeout_ = std::max(1.0, get_parameter("stuck_progress_timeout").as_double());
    stuck_radius_ = std::max(0.01, get_parameter("stuck_movement_radius_m").as_double());
    sync_stuck_from_nav2_ = get_parameter("sync_stuck_from_nav2").as_bool();
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    cmd_timeout_ = std::max(0.1, get_parameter("cmd_vel_timeout_sec").as_double());
    cmd_linear_deadband_ = std::max(0.0, get_parameter("cmd_linear_deadband").as_double());
    cmd_angular_deadband_ = std::max(0.0, get_parameter("cmd_angular_deadband").as_double());
    auto_recover_enabled_ = get_parameter("auto_recover_costmap_current").as_bool();
    auto_recover_silence_sec_ = std::max(1.0, get_parameter("auto_recover_silence_sec").as_double());
    auto_recover_confirm_sec_ = std::max(2.0, get_parameter("auto_recover_confirm_sec").as_double());
  }

  void detail_callback(const std_msgs::msg::String::SharedPtr msg);
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void waypoint_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void planner_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void follow_path_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void rosout_callback(const rcl_interfaces::msg::Log::SharedPtr msg);
  void update_loc_health();
  void update_stuck_detection();
  void update_auto_recovery();
  void reset_stuck_tracking();
  void detect_mission_transition(
    const std::string & previous, const std::string & current,
    const std::string & transition_reason);
  void check_tf();
  void poll_nav2_params();
  void request_nav2_params(const std::vector<std::string> & names);
  void publish_nav_status();
  void fire_event(const std::string & type, const nlohmann::json & detail);
  void fire_alarm(const std::string & code, const std::string & level, const std::string & message);
  void clear_alarm(const std::string & code);
  std::string derive_nav_phase() const;
  std::string derive_planner_status() const;
  std::string derive_controller_status() const;
  static std::string suggested_action(const std::string & code);

  mutable std::recursive_mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::string vehicle_id_;
  std::string global_frame_;
  std::string robot_frame_;
  double publish_rate_{2.0};
  double odom_timeout_{2.0};
  double gps_timeout_{5.0};
  double mb_timeout_{5.0};
  double min_odom_hz_{8.0};
  double cov_lost_{10.0};
  double cov_degraded_{1.0};
  double stuck_timeout_{12.0};
  double stuck_radius_{0.3};
  double cmd_timeout_{2.0};
  double cmd_linear_deadband_{0.05};
  double cmd_angular_deadband_{0.03};
  bool sync_stuck_from_nav2_{true};

  std::string mission_state_{"IDLE"};
  std::string task_id_;
  std::string command_id_;
  int waypoint_total_{0};
  int waypoint_completed_{0};
  int waypoint_current_index_{0};
  double mission_elapsed_sec_{0.0};
  std::string last_error_;
  int nav2_error_code_{0};
  std::string nav2_error_msg_;
  double mb_last_stamp_{0.0};

  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::NavSatFix::SharedPtr gps_;
  double odom_last_stamp_{0.0};
  double gps_last_stamp_{0.0};
  nav_status::LocHealth loc_health_{nav_status::LocHealth::Good};
  bool tf_ok_{false};
  double position_cov_max_{0.0};
  double orientation_cov_max_{0.0};
  double odom_hz_{0.0};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double linear_v_{0.0};
  double angular_w_{0.0};
  std::deque<double> odom_stamps_;

  std::unique_ptr<nav_status::StuckTracker> stuck_tracker_;
  double last_cmd_stamp_{0.0};
  bool cmd_linear_active_{false};
  bool cmd_angular_active_{false};
  bool waypoint_goal_active_{false};
  bool planner_failed_{false};
  bool follow_path_executing_{false};
  double follow_path_since_{0.0};

  bool auto_recover_enabled_{true};
  double auto_recover_silence_sec_{5.0};
  double auto_recover_confirm_sec_{10.0};
  bool auto_recover_used_{false};
  bool auto_recover_done_{false};
  double auto_recover_at_{0.0};
  std::unique_ptr<CostmapReinflater> reinflater_;
  std::set<std::string> active_alarms_;
  double startup_stamp_{0.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<m_common::msg::NavStatus>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_event_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detail_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr fw_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr follow_path_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr nav2_param_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr nav2_poll_timer_;
};

}  // namespace workspace_nav

namespace workspace_nav
{

void NavStatusAggregator::detail_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto data = nlohmann::json::parse(msg->data, nullptr, false);
  if (!data.is_object()) {
    NAV_LOG_WARN( "invalid JSON in status_detail");
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  mb_last_stamp_ = wall_seconds();
  const std::string previous = mission_state_;
  mission_state_ = data.value("state", "IDLE");
  task_id_ = data.value("task_id", "");
  command_id_ = data.value("command_id", "");
  waypoint_total_ = data.value("waypoint_total", 0);
  waypoint_completed_ = data.value("waypoint_completed", 0);
  waypoint_current_index_ = data.value("waypoint_current_index", 0);
  mission_elapsed_sec_ = data.value("elapsed_sec", 0.0);
  if (data.contains("error_code") && data["error_code"].is_string() &&
    !data["error_code"].get<std::string>().empty())
  {
    last_error_ = data["error_code"].get<std::string>();
  } else if (mission_state_ == "COMPLETED") {
    last_error_.clear();
  }
  const std::string nav2_msg = data.value("nav2_error_msg", "");
  if (!nav2_msg.empty()) {
    nav2_error_code_ = data.value("nav2_error_code", 0);
    nav2_error_msg_ = nav2_msg;
  } else if (mission_state_ == "COMPLETED" || mission_state_ == "IDLE") {
    nav2_error_code_ = 0;
    nav2_error_msg_.clear();
  }
  if (previous != mission_state_) {
    detect_mission_transition(previous, mission_state_, data.value("transition_reason", ""));
  }
}

void NavStatusAggregator::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  last_cmd_stamp_ = wall_seconds();
  cmd_linear_active_ = std::abs(msg->linear.x) > cmd_linear_deadband_;
  cmd_angular_active_ = std::abs(msg->angular.z) > cmd_angular_deadband_;
}

void NavStatusAggregator::follow_path_callback(
  const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const bool executing = std::any_of(
    msg->status_list.begin(), msg->status_list.end(), [](const auto & status) {
      return status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
    });
  if (executing && !follow_path_executing_) follow_path_since_ = wall_seconds();
  if (!executing) follow_path_since_ = 0.0;
  follow_path_executing_ = executing;
}

void NavStatusAggregator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const double now = wall_seconds();
  odom_last_stamp_ = now;
  odom_ = msg;
  odom_stamps_.push_back(now);
  if (odom_stamps_.size() > 50) odom_stamps_.pop_front();
  if (odom_stamps_.size() >= 10) {
    const double window = odom_stamps_.back() - odom_stamps_.front();
    if (window > 1.0) odom_hz_ = (odom_stamps_.size() - 1) / window;
  }
  linear_v_ = msg->twist.twist.linear.x;
  angular_w_ = msg->twist.twist.angular.z;
  position_cov_max_ = std::max({msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14]});
  orientation_cov_max_ = std::max(
    {msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]});
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  const auto & orientation = msg->pose.pose.orientation;
  const double odom_yaw = std::atan2(
    2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
  double transform_yaw = 0.0;
  const std::string source = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
  if (source != global_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(global_frame_, source, tf2::TimePointZero);
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      transform_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double c = std::cos(transform_yaw);
      const double s = std::sin(transform_yaw);
      const double transformed_x = t.x + c * x - s * y;
      y = t.y + s * x + c * y;
      x = transformed_x;
    } catch (const std::exception & error) {
      tf_ok_ = false;
      NAV_LOG_WARN_THROTTLE( 2000, "odom TF unavailable: %s", error.what());
      update_loc_health();
      return;
    }
  }
  tf_ok_ = true;
  pose_x_ = x;
  pose_y_ = y;
  pose_yaw_ = std::atan2(std::sin(odom_yaw + transform_yaw), std::cos(odom_yaw + transform_yaw));
  update_loc_health();
  update_stuck_detection();
}

void NavStatusAggregator::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gps_last_stamp_ = wall_seconds();
  gps_ = msg;
  update_loc_health();
}

void NavStatusAggregator::waypoint_status_callback(
  const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (msg->status_list.empty()) return;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const int status = msg->status_list.back().status;
  if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
  {
    waypoint_goal_active_ = true;
  } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
    status == action_msgs::msg::GoalStatus::STATUS_CANCELED ||
    status == action_msgs::msg::GoalStatus::STATUS_ABORTED)
  {
    waypoint_goal_active_ = false;
  }
}

void NavStatusAggregator::planner_status_callback(
  const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (msg->status_list.empty()) return;
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const int status = msg->status_list.back().status;
  if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
    planner_failed_ = true;
    fire_alarm(
      "PLAN_FAILED", "WARN",
      "Planner failed to find valid path — target may be in obstacle or outside map");
  } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
    planner_failed_ = false;
    clear_alarm("PLAN_FAILED");
  }
}

void NavStatusAggregator::rosout_callback(const rcl_interfaces::msg::Log::SharedPtr)
{
}

void NavStatusAggregator::update_loc_health()
{
  const auto previous = loc_health_;
  const double now = wall_seconds();
  loc_health_ = nav_status::evaluate_localization(
    tf_ok_, now - odom_last_stamp_ > odom_timeout_, now - gps_last_stamp_ > gps_timeout_,
    position_cov_max_, cov_lost_, cov_degraded_, gps_ ? gps_->status.status : -1,
    static_cast<bool>(gps_), odom_hz_, min_odom_hz_);
  if (previous == loc_health_) return;
  if (loc_health_ == nav_status::LocHealth::Lost) {
    last_error_ = "LOC_LOST";
    fire_alarm("LOC_LOST", "ERROR", "localization lost");
  } else if (loc_health_ == nav_status::LocHealth::Degraded) {
    last_error_ = "LOC_DEGRADED";
    fire_alarm("LOC_DEGRADED", "WARN", "localization degraded");
  } else {
    if (previous == nav_status::LocHealth::Lost) clear_alarm("LOC_LOST");
    clear_alarm("LOC_DEGRADED");
    if (last_error_ == "LOC_LOST" || last_error_ == "LOC_DEGRADED") last_error_.clear();
  }
}

void NavStatusAggregator::update_auto_recovery()
{
  if (!auto_recover_enabled_ || !reinflater_) return;
  if (mission_state_ != "RUNNING") {
    auto_recover_used_ = false;
    auto_recover_done_ = false;
    auto_recover_at_ = 0.0;
    return;
  }
  if (!follow_path_executing_ || follow_path_since_ <= 0.0) return;
  const double now = wall_seconds();
  const double silent = now - std::max(follow_path_since_, last_cmd_stamp_);
  if (auto_recover_used_) {
    if (auto_recover_done_) return;
    if (last_cmd_stamp_ > auto_recover_at_) {
      auto_recover_done_ = true;
      clear_alarm("CTRL_OUTPUT_SILENT");
    } else if (now - auto_recover_at_ > auto_recover_confirm_sec_) {
      auto_recover_done_ = true;
    }
    return;
  }
  if (silent < auto_recover_silence_sec_ || reinflater_->busy()) return;
  auto_recover_used_ = true;
  auto_recover_at_ = now;
  if (reinflater_->trigger("controller output silent")) {
    fire_alarm("CTRL_OUTPUT_SILENT", "WARN", "controller output silent; recovery attempted");
  }
}

void NavStatusAggregator::update_stuck_detection()
{
  update_auto_recovery();
  const auto event = stuck_tracker_->update(
    mission_state_ == "RUNNING", {pose_x_, pose_y_}, wall_seconds(),
    last_cmd_stamp_, cmd_linear_active_);
  if (event == nav_status::StuckEvent::Progress) {
    clear_alarm("CTRL_STUCK");
    if (last_error_ == "CTRL_STUCK") last_error_.clear();
  } else if (event == nav_status::StuckEvent::StuckNoProgress ||
    event == nav_status::StuckEvent::StuckCommandStale)
  {
    last_error_ = "CTRL_STUCK";
    fire_alarm(
      "CTRL_STUCK", "ERROR",
      event == nav_status::StuckEvent::StuckCommandStale ?
      "controller command stream stale" : "controller made no progress");
  }
}

void NavStatusAggregator::reset_stuck_tracking()
{
  stuck_tracker_->reset();
  last_cmd_stamp_ = 0.0;
  cmd_linear_active_ = false;
  cmd_angular_active_ = false;
  auto_recover_used_ = false;
  auto_recover_done_ = false;
  auto_recover_at_ = 0.0;
  follow_path_since_ = 0.0;
}

void NavStatusAggregator::detect_mission_transition(
  const std::string & previous, const std::string & current,
  const std::string & transition_reason)
{
  if (previous == "IDLE" && current == "RUNNING") {
    reset_stuck_tracking();
    clear_alarm("CTRL_STUCK");
    last_error_.clear();
    fire_event("TASK_STARTED", {{"task_id", task_id_}, {"total_waypoints", waypoint_total_}});
  } else if (current == "COMPLETED") {
    last_error_.clear();
    fire_event("TASK_COMPLETED", {{"task_id", task_id_}, {"elapsed_sec", mission_elapsed_sec_}});
  } else if (current == "FAILED") {
    const std::string error = last_error_.empty() ? "MISSION_FAILED" : last_error_;
    nlohmann::json detail = {
      {"task_id", task_id_}, {"error_code", error},
      {"failed_waypoint_index", waypoint_completed_},
      {"reason", "Mission failed with error: " + error}};
    if (!nav2_error_msg_.empty()) {
      detail["nav2_error_code"] = nav2_error_code_;
      detail["nav2_error_msg"] = nav2_error_msg_;
    }
    fire_event("TASK_FAILED", detail);
  } else if (previous == "RUNNING" && current == "IDLE") {
    if (transition_reason != "replace" && transition_reason != "complete") {
      fire_event("TASK_CANCELLED", {{"task_id", task_id_}, {"source", "cancel"}});
    }
  } else if (previous == "PAUSED" && current == "IDLE" && transition_reason == "cancel") {
    fire_event("TASK_CANCELLED", {{"task_id", task_id_}, {"source", "cancel"}});
  } else if (current == "PAUSED") {
    fire_event("TASK_PAUSED", {{"task_id", task_id_}, {"waypoint_index", waypoint_current_index_}});
  } else if (previous == "PAUSED" && current == "RUNNING") {
    reset_stuck_tracking();
    clear_alarm("CTRL_STUCK");
    if (last_error_ == "CTRL_STUCK") last_error_.clear();
    fire_event("TASK_RESUMED", {{"task_id", task_id_}, {"waypoint_index", waypoint_current_index_}});
  }
}

void NavStatusAggregator::check_tf()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  try {
    tf_ok_ = tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 50ms);
  } catch (const std::exception &) {
    tf_ok_ = false;
  }
}

void NavStatusAggregator::poll_nav2_params()
{
  if (!nav2_param_client_ || !nav2_param_client_->service_is_ready()) return;
  request_nav2_params({
    "progress_checker_plugin", "progress_checker.required_movement_radius",
    "progress_checker.movement_time_allowance"});
}

void NavStatusAggregator::request_nav2_params(const std::vector<std::string> & names)
{
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;
  nav2_param_client_->async_send_request(
    request, [this, names](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
      try {
        const auto response = future.get();
        std::string plugin = "progress_checker";
        std::optional<double> radius;
        std::optional<double> allowance;
        for (std::size_t i = 0; i < names.size() && i < response->values.size(); ++i) {
          const auto & value = response->values[i];
          if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) continue;
          if (names[i] == "progress_checker_plugin" &&
            value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING &&
            !value.string_value.empty())
          {
            plugin = value.string_value;
          } else if (names[i].find("required_movement_radius") != std::string::npos) {
            radius = value.double_value;
          } else if (names[i].find("movement_time_allowance") != std::string::npos) {
            allowance = value.double_value;
          }
        }
        if (plugin != "progress_checker" && (!radius || !allowance)) {
          request_nav2_params({
            plugin + ".required_movement_radius", plugin + ".movement_time_allowance"});
          return;
        }
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (radius && *radius > 0.0) stuck_radius_ = *radius;
        if (allowance && *allowance >= 1.0) stuck_timeout_ = *allowance;
        stuck_tracker_->set_limits(stuck_radius_, stuck_timeout_);
      } catch (const std::exception &) {
      }
    });
}

}  // namespace workspace_nav

namespace workspace_nav
{

void NavStatusAggregator::fire_event(const std::string & type, const nlohmann::json & detail)
{
  const double now = wall_seconds();
  const auto seconds = static_cast<int64_t>(now);
  nlohmann::json payload = {
    {"schema_version", 1},
    {"stamp", {{"sec", seconds}, {"nanosec", static_cast<int64_t>((now - seconds) * 1e9)}}},
    {"vehicle_id", vehicle_id_}, {"task_id", task_id_}, {"command_id", command_id_},
    {"event", type}, {"detail", detail}};
  task_event_pub_->publish(std_msgs::msg::String().set__data(payload.dump()));
  NAV_LOG_INFO( "/task_event %s task=%s", type.c_str(), task_id_.c_str());
}

void NavStatusAggregator::fire_alarm(
  const std::string & code, const std::string & level, const std::string & message)
{
  if (active_alarms_.count(code)) return;
  active_alarms_.insert(code);
  fire_event("ALARM_RAISED", {
    {"alarm_code", code}, {"level", level}, {"message", message},
    {"suggested_action", suggested_action(code)}});
}

void NavStatusAggregator::clear_alarm(const std::string & code)
{
  if (!active_alarms_.count(code)) return;
  active_alarms_.erase(code);
  fire_event("ALARM_CLEARED", {{"alarm_code", code}});
}

std::string NavStatusAggregator::suggested_action(const std::string & code)
{
  static const std::map<std::string, std::string> suggestions{
    {"LOC_LOST", "检查 GPS/EKF 状态，确认传感器数据正常"},
    {"LOC_DEGRADED", "检查 GPS 信号强度，确认定位数据质量"},
    {"CTRL_STUCK", "检查推进器状态，确认无障碍物卡死"},
    {"CTRL_OUTPUT_SILENT", "已自动触发代价图重膨胀；未恢复请检查 controller_server 与 local_costmap"},
    {"PLAN_FAILED", "检查目标航点坐标，确认不在障碍层内"},
    {"MISSION_FAILED", "检查导航系统状态，确认任务参数正确"}};
  const auto found = suggestions.find(code);
  return found == suggestions.end() ? "" : found->second;
}

std::string NavStatusAggregator::derive_nav_phase() const
{
  if (mission_state_ == "IDLE") return "IDLE";
  if (mission_state_ == "PAUSED") return "PAUSED";
  if (stuck_tracker_->stuck()) return "STUCK";
  if (mission_state_ == "RUNNING" || waypoint_goal_active_) return "TRACKING";
  return "IDLE";
}

std::string NavStatusAggregator::derive_planner_status() const
{
  return planner_failed_ || last_error_ == "PLAN_FAILED" || last_error_ == "MISSION_FAILED" ?
         "FAILED" : "OK";
}

std::string NavStatusAggregator::derive_controller_status() const
{
  return stuck_tracker_->stuck() || last_error_ == "CTRL_STUCK" ? "STUCK" : "OK";
}

void NavStatusAggregator::publish_nav_status()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const double now = wall_seconds();
  const auto seconds = static_cast<int64_t>(now);
  const std::string nav_phase = derive_nav_phase();
  const std::string planner_status = derive_planner_status();
  const std::string controller_status = derive_controller_status();
  m_common::msg::NavStatus msg;
  msg.header.stamp.sec = static_cast<int32_t>(seconds);
  msg.header.stamp.nanosec = static_cast<uint32_t>((now - seconds) * 1e9);
  msg.header.frame_id = global_frame_;
  msg.vehicle_id = vehicle_id_;
  msg.task_state = mission_state_;
  msg.task_id = task_id_;
  msg.command_id = command_id_;
  msg.nav_phase = nav_phase;
  msg.current_waypoint = waypoint_completed_;
  msg.total_waypoints = waypoint_total_;
  msg.progress_percent = waypoint_total_ > 0 ?
    std::round(1000.0 * waypoint_completed_ / waypoint_total_) / 10.0 : 0.0;
  msg.elapsed_sec = std::round(mission_elapsed_sec_ * 10.0) / 10.0;
  msg.distance_to_goal_m = 0.0;
  msg.eta_sec = -1.0;
  msg.last_error = last_error_;
  msg.planner_status = planner_status;
  msg.planner_last_plan_time_ms = 0.0;
  msg.planner_last_error = planner_failed_ ? "PLAN_FAILED" :
    ((last_error_ == "PLAN_FAILED" || last_error_ == "MISSION_FAILED") ? last_error_ : "");
  msg.controller_status = controller_status;
  msg.controller_tracking_error_m = 0.0;
  msg.controller_last_error = last_error_ == "CTRL_STUCK" ? last_error_ : "";
  msg.localization_overall = nav_status::to_string(loc_health_);
  msg.localization_position_cov_max = std::round(position_cov_max_ * 10000.0) / 10000.0;
  msg.localization_orientation_cov_max = std::round(orientation_cov_max_ * 10000.0) / 10000.0;
  msg.localization_gps_fix = gps_ ? gps_->status.status : 0;
  msg.localization_tf_ok = tf_ok_;
  msg.localization_odom_hz = std::round(odom_hz_ * 10.0) / 10.0;
  msg.pose_x = std::round(pose_x_ * 10000.0) / 10000.0;
  msg.pose_y = std::round(pose_y_ * 10000.0) / 10000.0;
  msg.pose_yaw = std::round(pose_yaw_ * 10000.0) / 10000.0;
  msg.pose_v = std::round(linear_v_ * 10000.0) / 10000.0;
  msg.pose_w = std::round(angular_w_ * 10000.0) / 10000.0;
  msg.flags_manual_override = false;
  msg.flags_emergency_stop = false;
  msg.flags_recovery_active = nav_phase == "RECOVERY";
  msg.alerts_odom_stale = now - odom_last_stamp_ > odom_timeout_;
  msg.alerts_gps_stale = now - gps_last_stamp_ > gps_timeout_;
  msg.alerts_mission_bridge_alive = now - mb_last_stamp_ < mb_timeout_;
  msg.alerts_planner_error = planner_status == "FAILED";
  msg.alerts_controller_error = controller_status == "FAILED" || controller_status == "STUCK";
  msg.alerts_emergency_active = false;
  msg.alerts_mission_paused = mission_state_ == "PAUSED";
  nav_status_pub_->publish(msg);
}

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("nav_status_aggregator");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<workspace_nav::NavStatusAggregator>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
