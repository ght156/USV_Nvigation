#include "usv_docking/docking_fsm.hpp"

#include "usv_docking/docking_logging.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace usv_docking
{
namespace
{

bool is_corridor(const std::string & state)
{
  return state == DockState::BACK_IN || state == DockState::FINAL_DOCK;
}

bool is_outside(const std::string & state)
{
  return state == DockState::ACQUIRE_TAG || state == DockState::APPROACH_ENTRY ||
    state == DockState::ALIGN_ENTRY || state == DockState::REACQUIRE_TAG ||
    state == DockState::WAIT_DOCK_OPEN;
}

bool is_backlight_driver_state(const std::string & state)
{
  return state == DockState::WAIT_DOCK_OPEN || state == DockState::WAIT_DOCK_RELEASE ||
    is_corridor(state);
}

bool starts_with(const std::string & value, const std::string & prefix)
{
  return value.rfind(prefix, 0) == 0;
}

double monotonic_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double rounded(double value, double scale)
{
  return std::round(value * scale) / scale;
}

}  // namespace

DockingFsm::DockingFsm(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_fsm", options)
{
  MLOGGER_NODE_REGISTER(docking_logging::log_root(), "docking", this->get_name());
  declare_parameters();
  const auto qos = rclcpp::QoS(10);
  state_pub_ = create_publisher<std_msgs::msg::String>(get_parameter("state_topic").as_string(), qos);
  target_mode_pub_ = create_publisher<std_msgs::msg::String>(
    get_parameter("target_mode_topic").as_string(), qos);
  reset_anchor_pub_ = create_publisher<std_msgs::msg::Bool>(
    get_parameter("reset_anchor_topic").as_string(), qos);
  status_pub_ = create_publisher<std_msgs::msg::String>(get_parameter("status_topic").as_string(), qos);
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    get_parameter("dock_pose_topic").as_string(), qos,
    std::bind(&DockingFsm::pose_callback, this, std::placeholders::_1));
  tag_visible_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("tag_visible_topic").as_string(), qos,
    [this](const std_msgs::msg::Bool::SharedPtr message) {tag_visible_ = message->data;});
  pose_source_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("pose_source_topic").as_string(), qos,
    [this](const std_msgs::msg::String::SharedPtr message) {pose_source_ = message->data;});
  measurement_age_sub_ = create_subscription<std_msgs::msg::Float32>(
    get_parameter("measurement_age_topic").as_string(), qos,
    [this](const std_msgs::msg::Float32::SharedPtr message) {measurement_age_ = message->data;});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_speed_topic").as_string(), rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&DockingFsm::odom_callback, this, std::placeholders::_1));
  abort_request_sub_ = create_subscription<std_msgs::msg::String>(
    get_parameter("abort_request_topic").as_string(), qos,
    std::bind(&DockingFsm::abort_request_callback, this, std::placeholders::_1));
  start_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("start_topic").as_string(), qos,
    std::bind(&DockingFsm::start_callback, this, std::placeholders::_1));
  cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
    get_parameter("cancel_topic").as_string(), qos,
    std::bind(&DockingFsm::cancel_callback, this, std::placeholders::_1));
  undock_sub_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("undock_topic").as_string(), qos,
    std::bind(&DockingFsm::undock_callback, this, std::placeholders::_1));
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / d("state_rate")),
    std::bind(&DockingFsm::tick, this));
  MLOGGER_INFO("docking_fsm 已启动，状态 IDLE，等待 /dock/start");
}

void DockingFsm::declare_parameters()
{
  declare_parameter("dock_pose_topic", "/docking/dock_pose");
  declare_parameter("tag_visible_topic", "/docking/tag_visible");
  declare_parameter("pose_source_topic", "/docking/pose_source");
  declare_parameter("measurement_age_topic", "/docking/measurement_age");
  declare_parameter("abort_request_topic", "/docking/abort_request");
  declare_parameter("start_topic", "/dock/start");
  declare_parameter("cancel_topic", "/dock/cancel");
  declare_parameter("undock_topic", "/dock/undock");
  declare_parameter("state_topic", "/docking/state");
  declare_parameter("target_mode_topic", "/docking/target_mode");
  declare_parameter("reset_anchor_topic", "/docking/reset_anchor");
  declare_parameter("status_topic", "/dock/status");
  declare_parameter("state_rate", 10.0);
  declare_parameter("status_publish_rate", 1.0);
  declare_parameter("reset_anchor_on_start", true);
  declare_parameter("tag_acquire_frames", 5);
  declare_parameter("acquire_timeout_sec", 60.0);
  declare_parameter("acquire_miss_tolerance", 3);
  declare_parameter("staging_x", -2.0);
  declare_parameter("approach_x_tol", 0.4);
  declare_parameter("approach_y_tol", 0.15);
  declare_parameter("approach_yaw_tol_deg", 25.0);
  declare_parameter("approach_stuck_sec", 25.0);
  declare_parameter("approach_stop_speed", 0.15);
  declare_parameter("approach_stop_hold_sec", 0.5);
  declare_parameter("odom_speed_topic", "/mavros/gps_input/local");
  declare_parameter("approach_tag_loss_grace_sec", 3.0);
  declare_parameter("approach_timeout_sec", 120.0);
  declare_parameter("align_y_tol", 0.15);
  declare_parameter("align_yaw_tol_deg", 7.0);
  declare_parameter("align_hold_sec", 2.0);
  declare_parameter("align_y_abort", 0.45);
  declare_parameter("align_timeout_sec", 45.0);
  declare_parameter("align_y_stuck_sec", 6.0);
  declare_parameter("entry_window_min_x", -3.5);
  declare_parameter("entry_window_max_x", -1.0);
  declare_parameter("final_target_x", 0.0);
  declare_parameter("final_dock_entry_dist", 0.8);
  declare_parameter("final_dock_entry_y_tol", 0.15);
  declare_parameter("back_in_timeout_sec", 90.0);
  declare_parameter("back_in_tag_loss_hold_sec", 0.5);
  declare_parameter("back_in_tag_loss_search_sec", 2.0);
  declare_parameter("back_in_gate2_y", 0.35);
  declare_parameter("back_in_gate2_yaw_deg", 15.0);
  declare_parameter("violation_cycles", 10);
  declare_parameter("docked_x_tol", 0.25);
  declare_parameter("docked_y_tol", 0.10);
  declare_parameter("docked_yaw_tol_deg", 3.0);
  declare_parameter("docked_hold_sec", 1.0);
  declare_parameter("final_dock_timeout_sec", 60.0);
  declare_parameter("final_dock_tag_loss_timeout_sec", 2.0);
  declare_parameter("final_push_time_sec", 15.0);
  declare_parameter("align_tag_loss_grace_sec", 5.0);
  declare_parameter("reacquire_frames", 5);
  declare_parameter("reacquire_miss_tolerance", 3);
  declare_parameter("reacquire_timeout_sec", 30.0);
  declare_parameter("reacquire_search_yaw_deg", 30.0);
  declare_parameter("reacquire_drive_max_sec", 10.0);
  declare_parameter("reacquire_drive_max_dist_m", 8.0);
  declare_parameter("exit_complete_x", -4.0);
  declare_parameter("abort_exit_timeout_sec", 45.0);
  declare_parameter("blind_exit_enabled", false);
  declare_parameter("exit_blind_timeout_sec", 15.0);
  declare_parameter("undock_unconfirmed_as_success", false);
  declare_parameter("undock_vision_timeout_sec", 10.0);
  declare_parameter("undock_complete_x", -3.0);
  declare_parameter("undock_timeout_sec", 60.0);
  declare_parameter("undock_settle_sec", 1.0);
  declare_parameter("max_docking_duration_sec", 150.0);
  declare_parameter("dock_claw_enabled", true);
  declare_parameter("dock_claw_open_on_start", false);
  declare_parameter("dock_claw_release_on_undock", true);
  declare_parameter("control_do_action", "control_do");
  declare_parameter("wait_io_action", "wait_io");
  declare_parameter("dock_open_timeout_sec", 15.0);
  declare_parameter("dock_release_timeout_sec", 15.0);
  declare_parameter("wait_io_max_queries", 0);
  declare_parameter("wait_io_retry_max", 3);
  declare_parameter("backin_stuck_timeout_sec", 25.0);
  declare_parameter("backin_stuck_progress_m", 0.01);
  declare_parameter("backlight_enabled", true);
  declare_parameter("backlight_mask", 2);
  declare_parameter("backlight_keepalive_sec", 20.0);
  declare_parameter("set_do_service", "set_do");
}

void DockingFsm::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  const double x = message->pose.position.x;
  const double y = message->pose.position.y;
  const double yaw = yaw_from_quat(message->pose.orientation.z, message->pose.orientation.w);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    MLOGGER_THROTTLE_WARN(2000, "丢弃非有限 dock_pose 帧（NaN/inf）");
    return;
  }
  dock_pose_ = Pose2D{x, y, yaw};
}

void DockingFsm::odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  const double vx = message->twist.twist.linear.x;
  const double vy = message->twist.twist.linear.y;
  if (!std::isfinite(vx) || !std::isfinite(vy)) {
    MLOGGER_THROTTLE_WARN(2000, "丢弃非有限 Odometry 速度帧（NaN/inf）");
    return;
  }
  odom_speed_ = std::hypot(vx, vy);
  odom_speed_stamp_ = get_clock()->now();
}

bool DockingFsm::odom_speed_ok()
{
  return odom_speed_ && odom_speed_stamp_ &&
    (get_clock()->now() - *odom_speed_stamp_).seconds() <= 1.0;
}

void DockingFsm::abort_request_callback(const std_msgs::msg::String::SharedPtr message)
{
  if (message->data.empty() || state_ == DockState::IDLE || state_ == DockState::FAILED) return;
  const std::string & reason = message->data;
  if (starts_with(reason, "EXIT_POSE_LOST")) {
    MLOGGER_THROTTLE_WARN(5000, "安全提示: {}（当前 {}，继续按超时兜底）", reason, state_);
    return;
  }
  MLOGGER_WARN("安全退出请求: {}（当前 {}）", reason, state_);
  if (starts_with(reason, "ODOM") || starts_with(reason, "TOPIC_TIMEOUT")) {
    fail("SAFETY:" + reason);
  } else if (state_ == DockState::UNDOCK_EXIT || state_ == DockState::UNDOCK_SETTLE ||
    state_ == DockState::WAIT_DOCK_RELEASE)
  {
    fail("SAFETY:" + reason);
  } else if (is_corridor(state_)) {
    abort_corridor("SAFETY:" + reason);
  } else if (is_outside(state_)) {
    abort_and_report("SAFETY:" + reason);
  }
}

void DockingFsm::start_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data || (state_ != DockState::IDLE && state_ != DockState::FAILED)) return;
  MLOGGER_INFO("收到 /dock/start：{} -> ...", state_);
  reset_flags();
  mission_start_time_ = get_clock()->now();
  if (b("reset_anchor_on_start")) {
    std_msgs::msg::Bool reset;
    reset.data = true;
    reset_anchor_pub_->publish(reset);
  }
  backlight_session_begin("归港开始");
  enter(claw_open_on_start() ? DockState::WAIT_DOCK_OPEN : DockState::ACQUIRE_TAG);
}

void DockingFsm::cancel_callback(const std_msgs::msg::Empty::SharedPtr)
{
  if (state_ == DockState::IDLE) return;
  MLOGGER_INFO("收到 /dock/cancel：{} -> IDLE", state_);
  backlight_session_end("收到 /dock/cancel");
  enter(DockState::IDLE);
}

void DockingFsm::undock_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data || (state_ != DockState::IDLE && state_ != DockState::DOCKED)) return;
  MLOGGER_INFO("收到 /dock/undock：{} -> ...", state_);
  reset_flags();
  mission_start_time_ = get_clock()->now();
  backlight_session_active_ = false;
  backlight_last_send_time_.reset();
  enter(claw_release_on_undock() ? DockState::WAIT_DOCK_RELEASE : DockState::UNDOCK_EXIT);
}

void DockingFsm::enter(const std::string & state)
{
  MLOGGER_INFO("状态: {} -> {}", state_, state);
  state_ = state;
  state_enter_time_ = get_clock()->now();
  reset_state_timers();
  manage_io_goals_on_enter(state);
  update_flags_on_enter(state);
  backlight_on_enter(state);
}

void DockingFsm::reset_state_timers()
{
  acquire_frames_ = 0;
  acquire_miss_ = 0;
  align_hold_ = 0.0;
  align_tag_loss_ = 0.0;
  align_y_stuck_ = 0.0;
  final_push_start_.reset();
  final_push_elapsed_ = 0.0;
  violation_count_ = 0;
  reacquire_frames_ = 0;
  reacquire_miss_ = 0;
  staging_hold_ = 0.0;
  approach_tag_loss_ = 0.0;
  approach_stuck_ = 0.0;
  stuck_best_x_.reset();
  stuck_elapsed_ = 0.0;
  undock_vision_lost_elapsed_ = 0.0;
  exit_pose_lost_elapsed_ = 0.0;
}

void DockingFsm::manage_io_goals_on_enter(const std::string & state)
{
  if (!is_corridor(state)) {
    cancel_wait_io();
  } else {
    wait_io_retries_ = 0;
  }
  if (state != DockState::WAIT_DOCK_OPEN && state != DockState::WAIT_DOCK_RELEASE) {
    cancel_control_do();
  } else {
    control_do_result_.reset();
  }
}

void DockingFsm::update_flags_on_enter(const std::string & state)
{
  if (state == DockState::DOCKED) success_ = true;
  if (state == DockState::IDLE || state == DockState::FAILED || state == DockState::DOCKED) {
    reacquire_drive_elapsed_ = 0.0;
  }
  if (state == DockState::IDLE) {
    success_ = false;
    undock_success_ = false;
  }
}

void DockingFsm::reset_flags()
{
  success_ = false;
  undock_success_ = false;
  needs_reapproach_ = false;
  needs_manual_takeover_ = false;
  abort_reason_.reset();
  claw_grabbed_ = false;
  claw_partial_mask_ = 0;
  reacquire_drive_elapsed_ = 0.0;
}

void DockingFsm::fail(const std::string & reason)
{
  MLOGGER_ERROR("FAILED: {}", reason);
  abort_reason_ = reason;
  needs_manual_takeover_ = true;
  enter(DockState::FAILED);
}

void DockingFsm::abort_and_report(const std::string & reason)
{
  MLOGGER_WARN("任务中止（{}）: {}", state_, reason);
  abort_reason_ = reason;
  needs_reapproach_ = true;
  enter(DockState::IDLE);
}

void DockingFsm::abort_corridor(const std::string & reason)
{
  if (claw_partial_mask_) {
    MLOGGER_ERROR(
      "坞内中止（{}）: {}；夹爪已抓住（DI 掩码 0x{:04X}），禁止驶出，就地停车报失败",
      state_, reason, claw_partial_mask_);
    fail(reason);
    return;
  }
  MLOGGER_WARN("坞内中止（{}）: {} -> ABORT_EXIT", state_, reason);
  abort_reason_ = reason;
  enter(DockState::ABORT_EXIT);
}

double DockingFsm::state_elapsed()
{
  return state_enter_time_ ? (get_clock()->now() - *state_enter_time_).seconds() : 0.0;
}

double DockingFsm::mission_elapsed()
{
  return mission_start_time_ ? (get_clock()->now() - *mission_start_time_).seconds() : 0.0;
}

std::tuple<DockingFsm::OptDouble, DockingFsm::OptDouble, DockingFsm::OptDouble>
DockingFsm::errors() const
{
  if (!dock_pose_) return {std::nullopt, std::nullopt, std::nullopt};
  return {dock_pose_->x, dock_pose_->y, wrap_angle(dock_pose_->yaw - PI)};
}

bool DockingFsm::ensure_io_clients()
{
  if (control_do_client_) return true;
  control_do_client_ = rclcpp_action::create_client<ControlDO>(
    this, get_parameter("control_do_action").as_string());
  wait_io_client_ = rclcpp_action::create_client<WaitIO>(
    this, get_parameter("wait_io_action").as_string());
  return true;
}

bool DockingFsm::ensure_set_do_client()
{
  if (!set_do_client_) {
    set_do_client_ = create_client<usv_rs485_driver::srv::SetDO>(
      get_parameter("set_do_service").as_string());
  }
  return true;
}

void DockingFsm::send_control_do()
{
  if (control_do_goal_active_) return;
  ensure_io_clients();
  if (!control_do_client_->action_server_is_ready()) {
    MLOGGER_THROTTLE_WARN(5000, "等待 control_do action server 就绪...");
    return;
  }
  ControlDO::Goal goal;
  goal.start = true;
  control_do_goal_active_ = true;
  ControlDOClient::SendGoalOptions options;
  options.goal_response_callback = std::bind(
    &DockingFsm::control_do_goal_response, this, std::placeholders::_1);
  control_do_client_->async_send_goal(goal, options);
  MLOGGER_INFO("已发送 ControlDO（请求船坞夹爪动作）");
}

void DockingFsm::control_do_goal_response(ControlDOGoalHandle::SharedPtr handle)
{
  if (!handle) {
    control_do_goal_active_ = false;
    MLOGGER_THROTTLE_WARN(2000, "ControlDO goal 被拒（驱动侧有在途 goal？），下周期重试");
    return;
  }
  control_do_goal_handle_ = handle;
  control_do_client_->async_get_result(
    handle, std::bind(&DockingFsm::control_do_result, this, std::placeholders::_1));
}

void DockingFsm::control_do_result(const ControlDOGoalHandle::WrappedResult & result)
{
  control_do_goal_active_ = false;
  control_do_goal_handle_.reset();
  if (!result.result) {
    control_do_result_ = std::make_pair(false, "RESULT_ERROR:no response");
    return;
  }
  control_do_result_ = std::make_pair(result.result->success, result.result->message);
}

void DockingFsm::cancel_control_do()
{
  if (control_do_goal_handle_ && control_do_client_) {
    control_do_client_->async_cancel_goal(control_do_goal_handle_);
  }
  control_do_goal_handle_.reset();
  control_do_goal_active_ = false;
  control_do_result_.reset();
}

void DockingFsm::send_set_do(int mask, const std::string & reason)
{
  if (!backlight_on()) return;
  ensure_set_do_client();
  if (!set_do_client_->service_is_ready()) {
    MLOGGER_THROTTLE_WARN(5000, "等待 set_do 服务就绪...");
    return;
  }
  auto request = std::make_shared<usv_rs485_driver::srv::SetDO::Request>();
  request->digital_output = mask & 0xFFFF;
  set_do_client_->async_send_request(
    request,
    [mask, reason](rclcpp::Client<usv_rs485_driver::srv::SetDO>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (!response->success) {
          MLOGGER_WARN("set_do 0x{:04X}（{}）驱动返回失败：{}", mask, reason, response->message);
        }
      } catch (const std::exception & exception) {
        MLOGGER_WARN("set_do 0x{:04X}（{}）调用失败：{}", mask, reason, exception.what());
      }
    });
  MLOGGER_INFO("船坞背光: 写 DO 0x{:04X}（{}）", request->digital_output, reason);
}

void DockingFsm::drive_undock_do(const std::string & reason)
{
  const bool light = backlight_on() && backlight_mask() != 0;
  int mask = 0;
  if (claw_release_on_undock()) {
    mask = light ? 0x0003 : 0x0001;
  } else {
    if (!light) return;
    mask = 0x0002;
  }
  ensure_io_clients();
  if (!control_do_client_->action_server_is_ready()) {
    MLOGGER_THROTTLE_WARN(5000, "等待 control_do action server 就绪，暂不发“出港 DO”...");
    return;
  }
  ControlDO::Goal goal;
  goal.start = true;
  goal.mask = mask & 0xFFFF;
  ControlDOClient::SendGoalOptions options;
  options.goal_response_callback = [mask, reason](ControlDOGoalHandle::SharedPtr handle) {
    if (!handle) {
      MLOGGER_WARN(
        "出港 DO ControlDO 0x{:04X} 被拒（{}；驱动可能有在途 goal）", mask, reason);
    }
  };
  control_do_client_->async_send_goal(goal, options);
  MLOGGER_INFO("出港 DO：ControlDO mask=0x{:04X}（{}）", goal.mask, reason);
}

void DockingFsm::backlight_session_begin(const std::string & reason)
{
  if (!backlight_on()) return;
  backlight_session_active_ = true;
  send_set_do(backlight_mask(), reason);
  backlight_last_send_time_ = get_clock()->now();
}

void DockingFsm::backlight_session_end(const std::string & reason)
{
  if (!backlight_on() || !backlight_session_active_) return;
  backlight_session_active_ = false;
  send_set_do(0, reason);
  backlight_last_send_time_.reset();
}

void DockingFsm::backlight_on_enter(const std::string & state)
{
  if (state == DockState::DOCKED) {
    backlight_session_end("归港成功");
  } else if (state == DockState::UNDOCK_EXIT) {
    drive_undock_do("出港");
  } else if (state == DockState::UNDOCK_SETTLE) {
    send_set_do(0, "出港完成");
  }
}

void DockingFsm::backlight_tick()
{
  if (!backlight_on() || !backlight_session_active_ || is_backlight_driver_state(state_)) return;
  const double period = std::max(1.0, d("backlight_keepalive_sec"));
  const auto now = get_clock()->now();
  if (!backlight_last_send_time_ || (now - *backlight_last_send_time_).seconds() >= period) {
    send_set_do(backlight_mask(), "找 Tag 阶段保活");
    backlight_last_send_time_ = now;
  }
}

void DockingFsm::send_wait_io()
{
  if (wait_io_active_ || claw_grabbed_) return;
  ensure_io_clients();
  if (!wait_io_client_->action_server_is_ready()) {
    MLOGGER_THROTTLE_WARN(5000, "等待 wait_io action server 就绪...");
    return;
  }
  WaitIO::Goal goal;
  goal.max_queries = i("wait_io_max_queries");
  wait_io_active_ = true;
  WaitIOClient::SendGoalOptions options;
  options.feedback_callback = std::bind(
    &DockingFsm::wait_io_feedback, this, std::placeholders::_1, std::placeholders::_2);
  options.goal_response_callback = std::bind(
    &DockingFsm::wait_io_goal_response, this, std::placeholders::_1);
  wait_io_client_->async_send_goal(goal, options);
  MLOGGER_INFO("已发送 WaitIO（等待夹爪抓住/停泊到位检测）");
}

void DockingFsm::wait_io_feedback(
  WaitIOGoalHandle::SharedPtr, const std::shared_ptr<const WaitIO::Feedback> feedback)
{
  if (!wait_io_active_) return;
  const int mask = feedback->digital_input & 0x0003;
  const int new_bits = mask & ~claw_partial_mask_;
  if (new_bits) {
    claw_partial_mask_ |= new_bits;
    MLOGGER_WARN(
      "WaitIO feedback：夹爪 DI 置位 0x{:04X}（累计 0x{:04X}）——爪子已扣住船体，此后禁止驶出重试",
      new_bits, claw_partial_mask_);
  }
}

void DockingFsm::wait_io_goal_response(WaitIOGoalHandle::SharedPtr handle)
{
  if (!handle) {
    wait_io_active_ = false;
    wait_io_need_retry_ = true;
    return;
  }
  wait_io_goal_handle_ = handle;
  wait_io_client_->async_get_result(
    handle, std::bind(&DockingFsm::wait_io_result, this, std::placeholders::_1));
}

void DockingFsm::wait_io_result(const WaitIOGoalHandle::WrappedResult & result)
{
  wait_io_active_ = false;
  wait_io_goal_handle_.reset();
  if (!is_corridor(state_)) return;
  if (!result.result) {
    wait_io_need_retry_ = true;
    return;
  }
  if (result.result->success) {
    claw_grabbed_ = true;
    MLOGGER_INFO("WaitIO：夹爪已抓住（{}）", result.result->message);
  } else {
    MLOGGER_WARN("WaitIO 未检测到停泊到位：{}", result.result->message);
    wait_io_need_retry_ = true;
  }
}

void DockingFsm::cancel_wait_io()
{
  if (wait_io_goal_handle_ && wait_io_client_) {
    wait_io_client_->async_cancel_goal(wait_io_goal_handle_);
  }
  wait_io_goal_handle_.reset();
  wait_io_active_ = false;
  wait_io_need_retry_ = false;
}

void DockingFsm::manage_wait_io()
{
  if (wait_io_active_ || claw_grabbed_) return;
  if (wait_io_need_retry_) {
    wait_io_need_retry_ = false;
    ++wait_io_retries_;
    const int retry_max = i("wait_io_retry_max");
    if (wait_io_retries_ > retry_max) {
      abort_corridor("WAIT_IO_FAILED");
      return;
    }
    MLOGGER_WARN("重发 WaitIO（{}/{}）", wait_io_retries_, retry_max);
  }
  send_wait_io();
}

bool DockingFsm::claw_stuck_check(double dt, const OptDouble & x)
{
  if (!x || pose_source_ == SRC_INVALID) return false;
  const double progress = d("backin_stuck_progress_m");
  if (!stuck_best_x_ || *x > *stuck_best_x_ + progress) {
    stuck_best_x_ = *x;
    stuck_elapsed_ = 0.0;
    return false;
  }
  stuck_elapsed_ += dt;
  return stuck_elapsed_ > d("backin_stuck_timeout_sec");
}

void DockingFsm::tick()
{
  const auto now = get_clock()->now();
  const double dt = last_tick_time_ ? (now - *last_tick_time_).seconds() : 0.0;
  last_tick_time_ = now;
  transitions(dt);
  backlight_tick();
  const auto mode = compute_mode();
  publish(mode);
}

bool DockingFsm::global_timeout_hit()
{
  const double limit = d("max_docking_duration_sec");
  return limit > 0.0 && mission_elapsed() > limit;
}

void DockingFsm::transitions(double dt)
{
  if ((is_corridor(state_) || is_outside(state_)) && global_timeout_hit()) {
    if (is_corridor(state_)) {
      abort_corridor("GLOBAL_TIMEOUT");
    } else {
      abort_and_report("GLOBAL_TIMEOUT");
    }
    return;
  }
  if (state_ == DockState::ACQUIRE_TAG) {
    trans_acquire_tag(dt);
  } else if (state_ == DockState::APPROACH_ENTRY) {
    trans_approach_entry(dt);
  } else if (state_ == DockState::ALIGN_ENTRY) {
    trans_align_entry(dt);
  } else if (state_ == DockState::BACK_IN) {
    trans_back_in(dt);
  } else if (state_ == DockState::FINAL_DOCK) {
    trans_final_dock(dt);
  } else if (state_ == DockState::WAIT_DOCK_OPEN) {
    trans_wait_dock_io("打开", DockState::ACQUIRE_TAG, "DOCK_OPEN", false, "dock_open_timeout_sec");
  } else if (state_ == DockState::WAIT_DOCK_RELEASE) {
    trans_wait_dock_io("松开", DockState::UNDOCK_EXIT, "DOCK_RELEASE", true, "dock_release_timeout_sec");
  } else if (state_ == DockState::REACQUIRE_TAG) {
    trans_reacquire_tag(dt);
  } else if (state_ == DockState::ABORT_EXIT) {
    trans_abort_exit(dt);
  } else if (state_ == DockState::UNDOCK_EXIT) {
    trans_undock_exit(dt);
  } else if (state_ == DockState::UNDOCK_SETTLE) {
    trans_undock_settle();
  }
}

bool DockingFsm::staging_reached(double dt)
{
  const auto [x, e_y, e_yaw] = errors();
  const bool conditions = x && std::abs(*x - d("staging_x")) <= d("approach_x_tol") &&
    std::abs(*e_y) <= d("approach_y_tol") &&
    std::abs(*e_yaw) <= d("approach_yaw_tol_deg") * PI / 180.0 &&
    odom_speed_ok() && *odom_speed_ <= d("approach_stop_speed");
  if (!conditions) {
    staging_hold_ = 0.0;
    return false;
  }
  staging_hold_ += dt;
  return staging_hold_ >= d("approach_stop_hold_sec");
}

bool DockingFsm::align_gate_ok() const
{
  const auto [x, e_y, e_yaw] = errors();
  return x && std::abs(*e_y) < d("align_y_tol") &&
    std::abs(*e_yaw) < d("align_yaw_tol_deg") * PI / 180.0 &&
    *x >= d("entry_window_min_x") && *x <= d("entry_window_max_x");
}

bool DockingFsm::corridor_violation() const
{
  const auto [x, e_y, e_yaw] = errors();
  return x && (std::abs(*e_y) > d("back_in_gate2_y") ||
    std::abs(*e_yaw) > d("back_in_gate2_yaw_deg") * PI / 180.0);
}

bool DockingFsm::claw_wait_or_docked(
  double dt, const OptDouble & x, const std::string & stuck_reason)
{
  if (!claw_on()) return false;
  if (claw_grabbed_) {
    MLOGGER_INFO("夹爪已抓住（WaitIO）-> DOCKED");
    enter(DockState::DOCKED);
    return true;
  }
  manage_wait_io();
  if (!is_corridor(state_)) return true;
  if (claw_stuck_check(dt, x)) {
    abort_corridor(stuck_reason);
    return true;
  }
  return false;
}

bool DockingFsm::corridor_violation_abort(const OptDouble & x)
{
  if (x && corridor_violation()) {
    ++violation_count_;
    if (violation_count_ >= i("violation_cycles")) {
      abort_corridor("CORRIDOR_VIOLATION");
      return true;
    }
  } else {
    violation_count_ = 0;
  }
  return false;
}

void DockingFsm::trans_acquire_tag(double)
{
  if (pose_source_ == SRC_VISION && tag_visible_) {
    ++acquire_frames_;
    acquire_miss_ = 0;
  } else {
    ++acquire_miss_;
    if (acquire_miss_ > i("acquire_miss_tolerance")) acquire_frames_ = 0;
  }
  if (acquire_frames_ >= i("tag_acquire_frames")) {
    enter(DockState::APPROACH_ENTRY);
  } else if (state_elapsed() > d("acquire_timeout_sec")) {
    abort_and_report("ACQUIRE_TIMEOUT");
  }
}

void DockingFsm::trans_approach_entry(double dt)
{
  bool reacquire = false;
  if (pose_source_ == SRC_INVALID) {
    reacquire = true;
  } else if (pose_source_ != SRC_VISION) {
    approach_tag_loss_ += dt;
    if (approach_tag_loss_ > d("approach_tag_loss_grace_sec")) reacquire = true;
  } else {
    approach_tag_loss_ = 0.0;
  }
  const auto [x, e_y, e_yaw] = errors();
  if (x && std::abs(*e_yaw) > d("approach_yaw_tol_deg") * PI / 180.0) {
    approach_stuck_ += dt;
  } else {
    approach_stuck_ = 0.0;
  }
  if (approach_stuck_ > d("approach_stuck_sec")) {
    abort_and_report("APPROACH_STUCK");
    return;
  }
  if (reacquire) {
    enter(DockState::REACQUIRE_TAG);
  } else if (x && staging_reached(dt)) {
    enter(DockState::ALIGN_ENTRY);
  } else if (state_elapsed() > d("approach_timeout_sec")) {
    abort_and_report("APPROACH_TIMEOUT");
  }
}

void DockingFsm::trans_align_entry(double dt)
{
  if (pose_source_ == SRC_VISION) {
    align_tag_loss_ = 0.0;
  } else if (pose_source_ != SRC_INVALID) {
    align_tag_loss_ += dt;
  }
  const auto [x, e_y, e_yaw] = errors();
  if (pose_source_ == SRC_INVALID) {
    enter(DockState::REACQUIRE_TAG);
  } else if (pose_source_ != SRC_VISION) {
    if (align_tag_loss_ > d("align_tag_loss_grace_sec")) enter(DockState::REACQUIRE_TAG);
  } else if (!x) {
  } else if (std::abs(*e_y) > d("align_y_abort")) {
    MLOGGER_WARN("ALIGN 中 |e_y|={:.2f} 超限，退回 APPROACH_ENTRY", std::abs(*e_y));
    enter(DockState::APPROACH_ENTRY);
  } else if (align_gate_ok()) {
    align_hold_ += dt;
    if (align_hold_ >= d("align_hold_sec")) enter(DockState::BACK_IN);
  } else {
    align_hold_ = 0.0;
    const double yaw_tolerance = d("align_yaw_tol_deg") * PI / 180.0;
    if (std::abs(*e_yaw) <= yaw_tolerance && std::abs(*e_y) > d("align_y_tol")) {
      align_y_stuck_ += dt;
      if (align_y_stuck_ > d("align_y_stuck_sec")) {
        MLOGGER_WARN(
          "ALIGN 艏向已准但 |e_y|={:.2f} 滞留 {:.1f}s，回 APPROACH 修 y",
          std::abs(*e_y), align_y_stuck_);
        enter(DockState::APPROACH_ENTRY);
      }
    } else {
      align_y_stuck_ = 0.0;
    }
    if (state_elapsed() > d("align_timeout_sec")) abort_and_report("ALIGN_TIMEOUT");
  }
}

void DockingFsm::trans_back_in(double dt)
{
  const auto [x, e_y, e_yaw] = errors();
  if (claw_wait_or_docked(dt, x, "BACK_IN_STUCK")) return;
  if (pose_source_ != SRC_VISION && measurement_age_ > d("back_in_tag_loss_search_sec")) {
    abort_corridor("TAG_LOST_IN_BACK_IN");
    return;
  }
  if (corridor_violation_abort(x)) return;
  if (x && std::abs(*x - d("final_target_x")) < d("final_dock_entry_dist") &&
    std::abs(*e_y) <= d("final_dock_entry_y_tol"))
  {
    enter(DockState::FINAL_DOCK);
    return;
  }
  if (state_elapsed() > d("back_in_timeout_sec")) abort_corridor("BACK_IN_TIMEOUT");
}

void DockingFsm::begin_final_push_if_ready(const OptDouble & x, const OptDouble & e_y)
{
  if (final_push_start_ || !x) return;
  if (!(std::abs(*x - d("final_target_x")) < d("docked_x_tol") &&
    std::abs(*e_y) < d("docked_y_tol")))
  {
    return;
  }
  final_push_start_ = get_clock()->now();
  stuck_best_x_.reset();
  stuck_elapsed_ = 0.0;
  MLOGGER_INFO("FINAL_DOCK 到达死位置，进入压紧段（低速继续倒船，等待夹爪抓住；重置倒船停滞计时）");
}

void DockingFsm::trans_final_dock(double dt)
{
  const auto [x, e_y, e_yaw] = errors();
  begin_final_push_if_ready(x, e_y);
  if (claw_wait_or_docked(dt, x, "FINAL_DOCK_STUCK")) return;
  if (pose_source_ != SRC_VISION && measurement_age_ > d("final_dock_tag_loss_timeout_sec")) {
    abort_corridor("TAG_LOST_IN_FINAL_DOCK");
    return;
  }
  if (corridor_violation_abort(x)) return;
  if (final_push_start_) {
    final_push_elapsed_ = (get_clock()->now() - *final_push_start_).seconds();
    if (final_push_elapsed_ > d("final_push_time_sec")) {
      abort_corridor("FINAL_PUSH_TIMEOUT");
      return;
    }
  }
  if (state_elapsed() > d("final_dock_timeout_sec")) abort_corridor("FINAL_DOCK_TIMEOUT");
}

void DockingFsm::trans_wait_dock_io(
  const std::string & action_desc, const std::string & success_state,
  const std::string & fail_prefix, bool release, const char * timeout_param)
{
  if (control_do_result_) {
    const auto [ok, message] = *control_do_result_;
    if (ok) {
      MLOGGER_INFO("船坞夹爪已{}：{}", action_desc, message);
      enter(success_state);
    } else if (release) {
      fail(fail_prefix + "_FAILED:" + message);
    } else {
      abort_and_report(fail_prefix + "_FAILED:" + message);
    }
  } else if (state_elapsed() > d(timeout_param)) {
    if (release) {
      fail(fail_prefix + "_TIMEOUT");
    } else {
      abort_and_report(fail_prefix + "_TIMEOUT");
    }
  } else {
    send_control_do();
  }
}

void DockingFsm::trans_reacquire_tag(double dt)
{
  if (pose_source_ == SRC_PREDICTION) {
    const double budget = d("reacquire_drive_max_sec");
    const bool was_within = reacquire_drive_elapsed_ <= budget;
    reacquire_drive_elapsed_ += dt;
    if (was_within && reacquire_drive_elapsed_ > budget) {
      MLOGGER_WARN("REACQUIRE 推算驾驶累计超 reacquire_drive_max_sec（{:.0f}s）预算，退回 HOLD 等 Tag 恢复", budget);
    }
  }
  if (pose_source_ == SRC_VISION && tag_visible_) {
    ++reacquire_frames_;
    reacquire_miss_ = 0;
  } else {
    ++reacquire_miss_;
    if (reacquire_miss_ > i("reacquire_miss_tolerance")) reacquire_frames_ = 0;
  }
  if (reacquire_frames_ >= i("reacquire_frames")) {
    route_after_reacquire();
  } else if (state_elapsed() > d("reacquire_timeout_sec")) {
    abort_and_report("REACQUIRE_TIMEOUT");
  }
}

void DockingFsm::trans_abort_exit(double dt)
{
  const auto [x, e_y, e_yaw] = errors();
  const bool pose_ok = x && pose_source_ != SRC_INVALID;
  if (!b("blind_exit_enabled")) {
    const bool vision_ok = pose_source_ == SRC_VISION && x;
    if (!vision_ok) {
      exit_pose_lost_elapsed_ += dt;
      if (exit_pose_lost_elapsed_ > d("exit_blind_timeout_sec")) {
        MLOGGER_WARN("ABORT_EXIT 无视觉超限：不盲开，驶出未确认，回 IDLE 上报 ABORT_EXIT_UNCONFIRMED");
        abort_reason_ = "ABORT_EXIT_UNCONFIRMED";
        needs_manual_takeover_ = true;
        enter(DockState::IDLE);
      }
      return;
    }
    exit_pose_lost_elapsed_ = 0.0;
    if (*x <= d("exit_complete_x")) {
      needs_reapproach_ = true;
      enter(DockState::IDLE);
      return;
    }
    if (state_elapsed() > d("abort_exit_timeout_sec")) {
      MLOGGER_WARN("ABORT_EXIT 超时（视觉有效），回 IDLE 上报 needs_reapproach");
      needs_reapproach_ = true;
      enter(DockState::IDLE);
    }
    return;
  }

  if (pose_ok && *x <= d("exit_complete_x")) {
    needs_reapproach_ = true;
    enter(DockState::IDLE);
    return;
  }
  const double elapsed = state_elapsed();
  if (!pose_ok && elapsed > d("exit_blind_timeout_sec")) {
    MLOGGER_WARN("ABORT_EXIT 盲开超限：驶出未确认，回 IDLE 上报 ABORT_EXIT_UNCONFIRMED");
    abort_reason_ = "ABORT_EXIT_UNCONFIRMED";
    needs_manual_takeover_ = true;
    enter(DockState::IDLE);
  } else if (elapsed > d("abort_exit_timeout_sec")) {
    if (!pose_ok) {
      MLOGGER_WARN("ABORT_EXIT 超时且位姿 INVALID：驶出未确认，回 IDLE 上报 ABORT_EXIT_UNCONFIRMED");
      abort_reason_ = "ABORT_EXIT_UNCONFIRMED";
      needs_manual_takeover_ = true;
    } else {
      MLOGGER_WARN("ABORT_EXIT 超时（位姿有效），回 IDLE 上报 needs_reapproach");
      needs_reapproach_ = true;
    }
    enter(DockState::IDLE);
  }
}

void DockingFsm::trans_undock_exit(double dt)
{
  const auto [x, e_y, e_yaw] = errors();
  if (!b("blind_exit_enabled")) {
    const bool vision_ok = pose_source_ == SRC_VISION && x;
    if (!vision_ok) {
      undock_vision_lost_elapsed_ += dt;
      if (undock_vision_lost_elapsed_ > d("undock_vision_timeout_sec")) fail("UNDOCK_NO_VISION");
      return;
    }
    undock_vision_lost_elapsed_ = 0.0;
    if (*x <= d("undock_complete_x")) {
      undock_success_ = true;
      enter(DockState::UNDOCK_SETTLE);
      return;
    }
    if (state_elapsed() > d("undock_timeout_sec")) fail("UNDOCK_TIMEOUT");
    return;
  }

  const bool pose_ok = x && pose_source_ != SRC_INVALID;
  if (pose_ok && *x <= d("undock_complete_x")) {
    undock_success_ = true;
    enter(DockState::UNDOCK_SETTLE);
    return;
  }
  const double elapsed = state_elapsed();
  if (!pose_ok && elapsed > d("exit_blind_timeout_sec")) {
    if (b("undock_unconfirmed_as_success")) {
      MLOGGER_WARN("UNDOCK_EXIT_UNCONFIRMED：位姿仍无效，按人工确认驶出成功（visual_unconfirmed）");
      undock_success_ = true;
      enter(DockState::UNDOCK_SETTLE);
      return;
    }
    fail("UNDOCK_EXIT_UNCONFIRMED");
    return;
  }
  if (elapsed > d("undock_timeout_sec")) fail("UNDOCK_TIMEOUT");
}

void DockingFsm::trans_undock_settle()
{
  if (state_elapsed() > d("undock_settle_sec")) enter(DockState::IDLE);
}

void DockingFsm::route_after_reacquire()
{
  const auto [x, e_y, e_yaw] = errors();
  if (x && *x >= d("entry_window_min_x") && *x <= d("entry_window_max_x") &&
    std::abs(*e_y) <= d("align_y_tol") &&
    std::abs(*e_yaw) <= d("approach_yaw_tol_deg") * PI / 180.0 &&
    odom_speed_ok() && *odom_speed_ <= d("approach_stop_speed"))
  {
    enter(DockState::ALIGN_ENTRY);
  } else {
    enter(DockState::APPROACH_ENTRY);
  }
}

std::string DockingFsm::compute_mode() const
{
  if (state_ == DockState::ACQUIRE_TAG) return tag_visible_ ? MODE_HOLD : MODE_SEARCH;
  if (state_ == DockState::APPROACH_ENTRY) return MODE_APPROACH;
  if (state_ == DockState::ALIGN_ENTRY) return MODE_ALIGN;
  if (state_ == DockState::BACK_IN) {
    if (pose_source_ != SRC_VISION) {
      return measurement_age_ <= d("back_in_tag_loss_hold_sec") ? MODE_HOLD : MODE_SEARCH_LIMITED;
    }
    return MODE_BACK_IN;
  }
  if (state_ == DockState::FINAL_DOCK) {
    return pose_source_ == SRC_VISION ? MODE_FINAL_DOCK : MODE_HOLD;
  }
  if (state_ == DockState::REACQUIRE_TAG) {
    if (pose_source_ == SRC_PREDICTION) {
      const bool near = dock_pose_ && std::abs(dock_pose_->x) <= d("reacquire_drive_max_dist_m");
      return near && reacquire_drive_elapsed_ <= d("reacquire_drive_max_sec") ?
        MODE_APPROACH : MODE_HOLD;
    }
    return pose_source_ == SRC_VISION ? MODE_HOLD : MODE_SEARCH;
  }
  if (state_ == DockState::ABORT_EXIT) return MODE_EXIT_FORWARD;
  if (state_ == DockState::UNDOCK_EXIT) return MODE_UNDOCK_FORWARD;
  if (state_ == DockState::DOCKED) return MODE_DOCKED_HOLD;
  return MODE_HOLD;
}

void DockingFsm::publish(const std::string & mode)
{
  std_msgs::msg::String state_message;
  state_message.data = state_;
  state_pub_->publish(state_message);
  std_msgs::msg::String mode_message;
  mode_message.data = mode;
  target_mode_pub_->publish(mode_message);

  const std::string status_key = state_ + "|" + mode + "|" +
    std::to_string(success_) + "|" + std::to_string(undock_success_) + "|" +
    (abort_reason_ ? *abort_reason_ : "") + "|" + std::to_string(needs_reapproach_) +
    "|" + std::to_string(needs_manual_takeover_) + "|" + pose_source_;
  const double now = monotonic_seconds();
  if (status_key == status_pub_key_ && now - status_pub_mono_ < 1.0 / d("status_publish_rate")) {
    return;
  }
  status_pub_key_ = status_key;
  status_pub_mono_ = now;
  const auto [x, e_y, e_yaw] = errors();
  const bool failed = state_ == DockState::FAILED;
  nlohmann::json status = {
    {"state", failed ? "DOCK_ABORT" : state_},
    {"v2_state", state_},
    {"target_mode", mode},
    {"success", success_},
    {"undock_success", undock_success_},
    {"abort_reason", abort_reason_ ? nlohmann::json(*abort_reason_) : nlohmann::json(nullptr)},
    {"needs_reapproach", needs_reapproach_ || failed},
    {"needs_manual_takeover", needs_manual_takeover_},
    {"pose_valid", pose_source_ != SRC_INVALID},
    {"pose_source", pose_source_},
    {"tag_visible", tag_visible_},
    {"tag_age_sec", nullptr},
    {"mission_elapsed_sec", rounded(mission_elapsed(), 10.0)},
    {"dock_claw_enabled", claw_on()},
    {"dock_claw_open_on_start", claw_open_on_start()},
    {"dock_claw_release_on_undock", claw_release_on_undock()},
    {"backlight_enabled", backlight_on()},
    {"backlight_session_active", backlight_session_active_},
    {"claw_grabbed", claw_grabbed_}
  };
  if (std::isfinite(measurement_age_)) status["tag_age_sec"] = rounded(measurement_age_, 100.0);
  if (x) {
    status["dock_x"] = rounded(*x, 1000.0);
    status["dock_y"] = rounded(*e_y, 1000.0);
    status["e_yaw_deg"] = rounded(*e_yaw * 180.0 / PI, 100.0);
  }
  std_msgs::msg::String status_message;
  status_message.data = status.dump();
  status_pub_->publish(status_message);
}

}  // namespace usv_docking

RCLCPP_COMPONENTS_REGISTER_NODE(usv_docking::DockingFsm)
