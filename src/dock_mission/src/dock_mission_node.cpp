#include "dock_mission/dock_mission_node.hpp"

#include "dock_mission/bay_loader.hpp"
#include "dock_mission/gnss_staging.hpp"
#include "dock_mission/task_event.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <m_common/msg/mission_waypoint.hpp>
#include <m_common/srv/cancel_mission.hpp>
#include <m_common/srv/send_waypoints.hpp>
#include <mlogger/mlogger.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <thread>
#include <tuple>

namespace dock_mission
{
namespace
{

std::string log_root()
{
  const char * home = std::getenv("USV_HOME");
  return home && *home ? std::string(home) + "/logs" : "logs";
}

double monotonic_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string json_string(const nlohmann::json & value)
{
  if (value.is_null()) return "";
  if (value.is_string()) return value.get<std::string>();
  if (value.is_boolean()) return value.get<bool>() ? "True" : "False";
  return value.dump();
}

}  // namespace

class MissionBridgeClient
{
public:
  using DoneCallback = std::function<void(bool, const std::string &)>;
  explicit MissionBridgeClient(rclcpp::Node * node)
  : node_(node)
  {
    send_client_ = node_->create_client<m_common::srv::SendWaypoints>(
      node_->get_parameter("send_waypoints_service").as_string());
    cancel_client_ = node_->create_client<m_common::srv::CancelMission>(
      node_->get_parameter("cancel_mission_service").as_string());
  }
  bool wait_ready(double timeout) {return send_client_->wait_for_service(std::chrono::duration<double>(timeout));}
  void send_staging_async(
    double latitude, double longitude, double yaw, const std::string & mission_id,
    const std::string & command_id, DoneCallback callback)
  {
    if (!send_client_->service_is_ready()) {
      callback(false, "send_waypoints service not ready");
      return;
    }
    auto request = std::make_shared<m_common::srv::SendWaypoints::Request>();
    m_common::msg::MissionWaypoint waypoint;
    waypoint.latitude = latitude;
    waypoint.longitude = longitude;
    waypoint.yaw = yaw;
    waypoint.seq = 0;
    request->waypoints = {waypoint};
    request->mission_id = mission_id;
    request->command_id = command_id;
    send_client_->async_send_request(
      request,
      [callback](rclcpp::Client<m_common::srv::SendWaypoints>::SharedFuture future) {
        try {
          const auto response = future.get();
          callback(response->success, response->message);
        } catch (const std::exception & exception) {
          callback(false, std::string("send_waypoints exception: ") + exception.what());
        }
      });
  }
  void cancel_async(DoneCallback callback = {})
  {
    if (!cancel_client_->service_is_ready()) {
      if (callback) callback(false, "cancel_mission service not ready");
      return;
    }
    cancel_client_->async_send_request(
      std::make_shared<m_common::srv::CancelMission::Request>(),
      [callback](rclcpp::Client<m_common::srv::CancelMission>::SharedFuture future) {
        if (!callback) return;
        try {
          const auto response = future.get();
          callback(response->success, response->message);
        } catch (const std::exception & exception) {
          callback(false, std::string("cancel_mission exception: ") + exception.what());
        }
      });
  }
private:
  rclcpp::Node * node_;
  rclcpp::Client<m_common::srv::SendWaypoints>::SharedPtr send_client_;
  rclcpp::Client<m_common::srv::CancelMission>::SharedPtr cancel_client_;
};

class Nav2GoalCheckerSwitch
{
public:
  struct Profile
  {
    std::string name;
    double xy;
    double yaw;
  };
  explicit Nav2GoalCheckerSwitch(rclcpp::Node * node)
  : node_(node),
    controller_(node->get_parameter("nav2_controller_node").as_string()),
    use_selector_(node->get_parameter("use_goal_checker_selector").as_bool()),
    cruise_{
      node->get_parameter("cruise_goal_checker_id").as_string(),
      node->get_parameter("cruise_xy_goal_tolerance").as_double(),
      node->get_parameter("cruise_yaw_goal_tolerance").as_double()},
    docking_{
      node->get_parameter("docking_goal_checker_id").as_string(),
      node->get_parameter("docking_xy_goal_tolerance").as_double(),
      node->get_parameter("docking_yaw_goal_tolerance").as_double()}
  {
    std::string controller = controller_;
    while (!controller.empty() && controller.front() == '/') controller.erase(controller.begin());
    param_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
      "/" + controller + "/set_parameters");
    selector_pub_ = node_->create_publisher<std_msgs::msg::String>(
      node_->get_parameter("goal_checker_selector_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
  }
  const std::string & active() const {return active_;}
  void apply_docking()
  {
    if (active_ == "docking") return;
    publish_selector(docking_.name);
    set_param_tolerances(docking_);
    set_param_tolerances(Profile{cruise_.name, docking_.xy, docking_.yaw});
    active_ = "docking";
    MLOGGER_INFO(
      "Nav2 goal profile DOCKING (selector={}, xy={}, yaw={})",
      docking_.name, docking_.xy, docking_.yaw);
  }
  void apply_cruise()
  {
    if (active_ == "cruise") return;
    publish_selector(cruise_.name);
    set_param_tolerances(cruise_);
    active_ = "cruise";
    MLOGGER_INFO("Nav2 goal profile CRUISE (xy={}, yaw={})", cruise_.xy, cruise_.yaw);
  }
private:
  void publish_selector(const std::string & checker)
  {
    if (!use_selector_) return;
    std_msgs::msg::String message;
    message.data = checker;
    selector_pub_->publish(message);
    MLOGGER_INFO("goal_checker_selector → '{}'", checker);
  }
  void set_param_tolerances(const Profile & profile)
  {
    if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
      MLOGGER_WARN("{} param service unavailable; goal checker tolerances not updated", controller_);
      return;
    }
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters = {
      rclcpp::Parameter(profile.name + ".xy_goal_tolerance", profile.xy).to_parameter_msg(),
      rclcpp::Parameter(profile.name + ".yaw_goal_tolerance", profile.yaw).to_parameter_msg()};
    param_client_->async_send_request(
      request, [](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
        try {
          for (const auto & result : future.get()->results) {
            if (!result.successful) MLOGGER_WARN("param set failed: {}", result.reason);
          }
        } catch (const std::exception & exception) {
          MLOGGER_WARN("param set error: {}", exception.what());
        }
      });
  }
  rclcpp::Node * node_;
  std::string controller_;
  bool use_selector_;
  Profile cruise_;
  Profile docking_;
  std::string active_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selector_pub_;
};

DockMissionNode::DockMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("dock_mission_node", options)
{
  MLOGGER_NODE_REGISTER(log_root(), "docking", this->get_name());
  declare_parameters();
  load_parameters();
  settle_monitor_ = SettleMonitor(settle_params_);
  goal_switch_ = std::make_unique<Nav2GoalCheckerSwitch>(this);
  mission_ = std::make_unique<MissionBridgeClient>(this);
  authority_pub_ = create_publisher<std_msgs::msg::String>("/dock/speed_authority", 10);
  legacy_status_pub_ = create_publisher<std_msgs::msg::String>("/dock/mission_status", 10);
  task_status_pub_ = create_publisher<std_msgs::msg::String>(dock_task_status_topic_, 10);
  if (!dock_task_event_log_topic_.empty()) {
    task_event_log_pub_ = create_publisher<std_msgs::msg::String>(dock_task_event_log_topic_, 10);
  }
  dock_start_pub_ = create_publisher<std_msgs::msg::Bool>("/dock/start", 10);
  dock_undock_pub_ = create_publisher<std_msgs::msg::Bool>("/dock/undock", 10);
  dock_cancel_pub_ = create_publisher<std_msgs::msg::Empty>("/dock/cancel", 10);
  task_event_sub_ = create_subscription<std_msgs::msg::String>(
    "/task_event", 10, std::bind(&DockMissionNode::task_event_callback, this, std::placeholders::_1));
  dock_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/dock/status", 10, std::bind(&DockMissionNode::dock_status_callback, this, std::placeholders::_1));
  home_sub_ = create_subscription<std_msgs::msg::Bool>(
    s("dock_home_topic"), 10, std::bind(&DockMissionNode::home_callback, this, std::placeholders::_1));
  gcs_command_sub_ = create_subscription<std_msgs::msg::String>(
    gcs_dock_command_topic_, 10,
    std::bind(&DockMissionNode::gcs_command_callback, this, std::placeholders::_1));
  start_service_ = create_service<std_srvs::srv::Trigger>(
    "/dock/mission/start",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto [ok, message] = begin_dock_mission();
      response->success = ok;
      response->message = message;
    });
  cancel_service_ = create_service<std_srvs::srv::Trigger>(
    "/dock/mission/cancel",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const auto [ok, message] = cancel_active_session();
      response->success = ok;
      response->message = message;
    });
  event_client_ = create_client<m_common::srv::DockTaskEvent>(dock_task_event_service_);
  if (!s("apriltag_channel_service").empty()) {
    apriltag_channel_client_ = create_client<m_common::srv::SetChannelEnabled>(
      s("apriltag_channel_service"));
  }
  if (!s("set_do_service").empty()) {
    set_do_client_ = create_client<usv_rs485_driver::srv::SetDO>(s("set_do_service"));
  }
  if (!s("control_do_action").empty()) {
    control_do_client_ = rclcpp_action::create_client<ControlDO>(this, s("control_do_action"));
  }
  odom_speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    s("odom_speed_topic"), 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      const double vx = message->twist.twist.linear.x;
      const double vy = message->twist.twist.linear.y;
      const double wz = message->twist.twist.angular.z;
      if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
        MLOGGER_THROTTLE_WARN(2000, "丢弃非有限 Odometry 速度帧（NaN/inf）");
        return;
      }
      odom_speed_ = std::hypot(vx, vy);
      odom_yaw_rate_ = wz;
      odom_speed_stamp_ = get_clock()->now();
      if (state_ == MissionState::SETTLE && b("settle_require_still")) {
        settle_monitor_.observe(*odom_speed_, *odom_yaw_rate_, odom_speed_stamp_->seconds());
      }
    });
  dock_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    s("dock_pose_topic"), 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      dock_pose_xy_ = std::make_pair(message->pose.position.x, message->pose.position.y);
      dock_pose_stamp_ = get_clock()->now();
    });
  dock_pose_source_sub_ = create_subscription<std_msgs::msg::String>(
    s("dock_pose_source_topic"), 10,
    [this](const std_msgs::msg::String::SharedPtr message) {dock_pose_source_ = message->data;});
  dock_anchor_status_sub_ = create_subscription<std_msgs::msg::String>(
    s("dock_anchor_status_topic"), 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      dock_anchor_status_ = message->data;
      dock_anchor_status_stamp_ = get_clock()->now();
    });
  dock_measurement_age_sub_ = create_subscription<std_msgs::msg::Float32>(
    s("dock_measurement_age_topic"), 10,
    [this](const std_msgs::msg::Float32::SharedPtr message) {
      if (std::isnan(message->data)) return;
      dock_measurement_age_ = message->data;
      dock_measurement_age_stamp_ = get_clock()->now();
    });
  if (!s("in_dock_service").empty()) {
    in_dock_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    in_dock_service_ = create_service<m_common::srv::QueryInDock>(
      s("in_dock_service"),
      std::bind(&DockMissionNode::in_dock_service, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, in_dock_callback_group_);
  }
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(0.2),
    std::bind(&DockMissionNode::tick, this));
  MLOGGER_INFO(
    "dock_mission ready home_topic={} task_status={}", s("dock_home_topic"),
    dock_task_status_topic_);
}

DockMissionNode::~DockMissionNode() = default;

void DockMissionNode::declare_parameters()
{
  declare_parameter("bay_id", "bay2");
  declare_parameter("dock_database_path", "");
  declare_parameter("map_yaml_path", "");
  declare_parameter("use_gnss_staging", true);
  declare_parameter("accept_gcs_staging_override", false);
  declare_parameter("staging_retry_max", 5);
  declare_parameter("settle_sec", 2.5);
  declare_parameter("complete_settle_sec", 2.0);
  declare_parameter("settle_require_still", true);
  declare_parameter("odom_speed_topic", "/mavros/gps_input/local");
  declare_parameter("settle_max_speed", 0.15);
  declare_parameter("settle_max_yaw_rate", 0.1);
  declare_parameter("settle_still_hold_sec", 0.5);
  declare_parameter("settle_max_frame_gap_sec", 1.0);
  declare_parameter("settle_min_still_frames", 3);
  declare_parameter("settle_timeout_sec", 15.0);
  declare_parameter("dock_home_topic", "/dock/home");
  declare_parameter("gcs_dock_command_topic", "/gcs_dock/command");
  declare_parameter("dock_task_status_topic", "/dock_task/status");
  declare_parameter("status_publish_rate", 1.0);
  declare_parameter("dock_task_event_service", "/dock_task/event");
  declare_parameter("dock_task_event_log_topic", "/dock_task/event_log");
  declare_parameter("event_ack_timeout", 3.0);
  declare_parameter("apriltag_channel_service", "/apriltag_node/set_channel_enabled");
  declare_parameter("apriltag_channel", "all");
  declare_parameter("in_dock_service", "/dock/in_dock");
  declare_parameter("dock_pose_topic", "/docking/dock_pose");
  declare_parameter("dock_pose_source_topic", "/docking/pose_source");
  declare_parameter("dock_anchor_status_topic", "/docking/anchor_status");
  declare_parameter("dock_measurement_age_topic", "/docking/measurement_age");
  declare_parameter("in_dock_x_min", -1.5);
  declare_parameter("in_dock_x_max", 1.0);
  declare_parameter("in_dock_y_max", 0.6);
  declare_parameter("in_dock_pose_max_age_sec", 1.5);
  declare_parameter("in_dock_prediction_max_age_sec", 3.0);
  declare_parameter("in_dock_vision_timeout_sec", 6.0);
  declare_parameter("in_dock_restore_channel", true);
  declare_parameter("set_do_service", "set_do");
  declare_parameter("control_do_action", "control_do");
  declare_parameter("in_dock_backlight_mask", 2);
  declare_parameter("send_waypoints_service", "/mission_bridge/send_waypoints");
  declare_parameter("cancel_mission_service", "/mission_bridge/cancel_mission");
  declare_parameter("nav2_controller_node", "/controller_server");
  declare_parameter("goal_checker_selector_topic", "goal_checker_selector");
  declare_parameter("use_goal_checker_selector", true);
  declare_parameter("cruise_goal_checker_id", "general_goal_checker");
  declare_parameter("docking_goal_checker_id", "docking_goal_checker");
  declare_parameter("cruise_xy_goal_tolerance", 2.0);
  declare_parameter("cruise_yaw_goal_tolerance", 1.0);
  declare_parameter("docking_xy_goal_tolerance", 1.0);
  declare_parameter("docking_yaw_goal_tolerance", 0.4);
  declare_parameter("dock_mission_id", "dock_staging");
}

void DockMissionNode::load_parameters()
{
  bay_id_ = s("bay_id");
  dock_db_path_ = s("dock_database_path");
  const std::string map_path = s("map_yaml_path");
  if (!map_path.empty()) {
    map_yaml_ = map_path;
  } else {
    try {
      map_yaml_ = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("workspace_nav")) / "config" / "map.yaml";
    } catch (const std::exception &) {
      map_yaml_.clear();
    }
  }
  use_gnss_ = b("use_gnss_staging");
  accept_gcs_staging_override_ = b("accept_gcs_staging_override");
  mission_id_ = s("dock_mission_id");
  gcs_dock_command_topic_ = s("gcs_dock_command_topic");
  dock_task_status_topic_ = s("dock_task_status_topic");
  dock_task_event_service_ = s("dock_task_event_service");
  dock_task_event_log_topic_ = s("dock_task_event_log_topic");
  event_ack_timeout_ = d("event_ack_timeout");
  in_dock_limits_.x_min = d("in_dock_x_min");
  in_dock_limits_.x_max = d("in_dock_x_max");
  in_dock_limits_.y_max = d("in_dock_y_max");
  in_dock_limits_.pose_max_age_sec = d("in_dock_pose_max_age_sec");
  in_dock_limits_.prediction_max_age_sec = d("in_dock_prediction_max_age_sec");
  settle_params_.settle_sec = d("settle_sec");
  settle_params_.max_speed = d("settle_max_speed");
  settle_params_.max_yaw_rate = d("settle_max_yaw_rate");
  settle_params_.still_hold_sec = d("settle_still_hold_sec");
  settle_params_.max_frame_gap_sec = d("settle_max_frame_gap_sec");
  settle_params_.min_still_frames = i("settle_min_still_frames");
}

double DockMissionNode::elapsed_sec() const
{
  return session_start_mono_ ? std::max(0.0, monotonic_seconds() - *session_start_mono_) : 0.0;
}

void DockMissionNode::set_authority(SpeedAuthority authority)
{
  std_msgs::msg::String message;
  message.data = to_string(authority);
  authority_pub_->publish(message);
}

void DockMissionNode::emit_event(const std::string & event, const nlohmann::json & detail)
{
  const auto payload = build_task_event(event, mission_id_, command_id_, detail);
  last_event_ = event;
  if (task_event_log_pub_) {
    std_msgs::msg::String message;
    message.data = dumps_json(payload);
    task_event_log_pub_->publish(message);
  }
  report_event(payload);
}

void DockMissionNode::report_event(const nlohmann::json & payload)
{
  auto request = std::make_shared<m_common::srv::DockTaskEvent::Request>();
  request->event = json_string(payload.value("event", nlohmann::json()));
  request->mission_id = json_string(payload.value("mission_id", nlohmann::json()));
  request->command_id = json_string(payload.value("command_id", nlohmann::json()));
  request->detail = dumps_json(payload.value("detail", nlohmann::json::object()));
  request->stamp_sec = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const long serial = ++next_event_serial_;
  event_pending_serials_.insert(serial);
  ++event_pending_;
  event_pending_mono_ = monotonic_seconds();
  event_client_->async_send_request(
    request,
    [this, serial](rclcpp::Client<m_common::srv::DockTaskEvent>::SharedFuture future) {
      if (event_pending_serials_.erase(serial) == 0) return;
      event_pending_ = std::max(0, event_pending_ - 1);
      try {
        const auto response = future.get();
        if (response->success) {
          record_event_ack(true, !response->message.empty() ? response->message : response->ack_id);
        } else {
          record_event_ack(false, response->message);
        }
      } catch (const std::exception & exception) {
        record_event_ack(false, std::string("event ack exception: ") + exception.what());
      }
    });
}

void DockMissionNode::record_event_ack(bool ok, const std::string & note)
{
  event_ack_ok_ = ok;
  if (!ok) ++event_ack_failures_;
  MLOGGER_INFO("event ack {} {}: {}", ok ? "OK" : "FAILED", last_event_, note);
}

void DockMissionNode::publish_status_if_due()
{
  const std::string key = to_string(state_) + "|" + (mode_ ? *mode_ : "") + "|" +
    std::to_string(dock_active_) + "|" + std::to_string(staging_retry_) + "|" +
    std::to_string(needs_manual_takeover_) + "|" + std::to_string(needs_reapproach_) +
    "|" + last_fail_reason_;
  const double now = monotonic_seconds();
  if (key == status_pub_key_ && now - status_pub_mono_ < 1.0 / d("status_publish_rate")) return;
  status_pub_key_ = key;
  status_pub_mono_ = now;
  publish_task_status();
  publish_legacy_status();
}

void DockMissionNode::publish_task_status()
{
  TaskStatus status;
  status.state = state_;
  status.mode = mode_;
  status.dock_active = dock_active_;
  status.retry_count = staging_retry_;
  status.retry_max = retry_max();
  status.needs_manual_takeover = needs_manual_takeover_;
  status.needs_reapproach = needs_reapproach_;
  status.mission_id = mission_id_;
  status.command_id = command_id_;
  status.elapsed_sec = elapsed_sec();
  status.error_message = last_fail_reason_.empty() ?
    std::nullopt : std::optional<std::string>(last_fail_reason_);
  status.camera_ready = camera_ready_;
  status.event_ack_ok = event_ack_ok_;
  status.event_ack_failures = event_ack_failures_;
  status.odom_speed = odom_speed_;
  std_msgs::msg::String message;
  message.data = dumps_json(build_task_status(status));
  task_status_pub_->publish(message);
}

void DockMissionNode::publish_legacy_status()
{
  nlohmann::json payload = {
    {"state", to_string(state_)},
    {"staging_retry", staging_retry_},
    {"staging_retry_max", retry_max()},
    {"nav2_profile", goal_switch_->active().empty() ?
      nlohmann::json(nullptr) : nlohmann::json(goal_switch_->active())},
    {"use_gnss_staging", use_gnss_}
  };
  std_msgs::msg::String message;
  message.data = payload.dump();
  legacy_status_pub_->publish(message);
}

void DockMissionNode::transition(MissionState state, const std::string & note)
{
  if (state == state_) return;
  MLOGGER_INFO("mission {} → {} {}", to_string(state_), to_string(state), note);
  if (state == MissionState::SETTLE) {
    settle_elapsed_ = 0.0;
    settle_monitor_.reset();
    set_apriltag_channels(true, "到达预泊点");
  }
  if (state == MissionState::UNDOCK_HANDOFF) set_apriltag_channels(true, "出泊交接");
  if (state == MissionState::DOCK_HANDOFF) set_apriltag_channels(true, "交接精靠泊");
  if (state == MissionState::COMPLETE_SETTLE) complete_settle_elapsed_ = 0.0;
  state_ = state;
}

void DockMissionNode::set_apriltag_channels(bool enabled, const std::string & reason)
{
  if (!apriltag_channel_client_) return;
  if (!apriltag_channel_client_->service_is_ready()) {
    MLOGGER_WARN("apriltag 通道开关服务暂不在线，仍发起异步调用（结果见后续日志）");
  }
  auto request = std::make_shared<m_common::srv::SetChannelEnabled::Request>();
  request->channel = s("apriltag_channel");
  request->enabled = enabled;
  apriltag_channel_client_->async_send_request(
    request,
    [enabled, reason](rclcpp::Client<m_common::srv::SetChannelEnabled>::SharedFuture future) {
      const std::string action = enabled ? "开启" : "关闭";
      try {
        const auto response = future.get();
        if (response->success) {
          MLOGGER_INFO("apriltag 检测通道已{}({}): {}", action, reason, response->message);
        } else {
          MLOGGER_WARN("apriltag 通道{}被拒({}): {}", action, reason, response->message);
        }
      } catch (const std::exception & exception) {
        MLOGGER_WARN("apriltag 通道{}调用失败({}): {}", action, reason, exception.what());
      }
    });
}

std::optional<double> DockMissionNode::measurement_age_usable()
{
  if (!dock_measurement_age_stamp_ ||
    (get_clock()->now() - *dock_measurement_age_stamp_).seconds() > d("in_dock_pose_max_age_sec"))
  {
    return std::nullopt;
  }
  return dock_measurement_age_;
}

bool DockMissionNode::estimator_alive()
{
  return dock_pose_stamp_ &&
    (get_clock()->now() - *dock_pose_stamp_).seconds() <= d("in_dock_pose_max_age_sec");
}

InDockSnapshot DockMissionNode::in_dock_snapshot()
{
  InDockSnapshot snapshot;
  snapshot.xy = dock_pose_xy_;
  snapshot.pose_age = dock_pose_stamp_ ?
    (get_clock()->now() - *dock_pose_stamp_).seconds() : std::numeric_limits<double>::infinity();
  snapshot.source = dock_pose_source_;
  snapshot.anchor_status = dock_anchor_status_;
  snapshot.anchor_age = dock_anchor_status_stamp_ ?
    (get_clock()->now() - *dock_anchor_status_stamp_).seconds() :
    std::numeric_limits<double>::infinity();
  snapshot.measurement_age = measurement_age_usable();
  snapshot.measurement_age_stamp_age = dock_measurement_age_stamp_ ?
    (get_clock()->now() - *dock_measurement_age_stamp_).seconds() :
    std::numeric_limits<double>::infinity();
  return snapshot;
}

bool DockMissionNode::set_dock_backlight(bool on, const std::string & reason)
{
  const int mask = i("in_dock_backlight_mask") & 0xFFFF;
  if (mask == 0) return false;
  if (on) {
    if (!control_do_client_) return false;
    if (!control_do_client_->action_server_is_ready()) {
      MLOGGER_WARN("control_do action server 不在线，无法点背光（需 usv_rs485_driver 最新版本）");
      return false;
    }
    ControlDO::Goal goal;
    goal.start = true;
    goal.mask = mask;
    ControlDOClient::SendGoalOptions options;
    options.goal_response_callback = [mask](rclcpp_action::ClientGoalHandle<ControlDO>::SharedPtr handle) {
      if (!handle) MLOGGER_WARN("ControlDO 点背光 0x{:04X} 被拒（驱动可能有在途 goal）", mask);
    };
    control_do_client_->async_send_goal(goal, options);
    MLOGGER_INFO("船坞背光点亮({}): ControlDO mask=0x{:04X}（只开灯，不驱动夹爪）", reason, mask);
    return true;
  }
  if (!set_do_client_) return false;
  if (!set_do_client_->service_is_ready()) {
    MLOGGER_WARN("set_do 服务暂不在线，仍发起异步调用（关背光，结果见驱动日志）");
  }
  auto request = std::make_shared<usv_rs485_driver::srv::SetDO::Request>();
  request->digital_output = 0;
  set_do_client_->async_send_request(request);
  MLOGGER_INFO("船坞背光关闭({}): DO 0x0000", reason);
  return true;
}

std::pair<bool, bool> DockMissionNode::await_vision_pose()
{
  if (dock_active_ || !apriltag_channel_client_) return {false, false};
  const bool lit_backlight = fsm_backlight_active_ ? false : set_dock_backlight(true, "在坞查询");
  set_apriltag_channels(true, "在坞查询");
  const double deadline = monotonic_seconds() + d("in_dock_vision_timeout_sec");
  while (monotonic_seconds() < deadline) {
    if (evaluate_in_dock(in_dock_snapshot(), in_dock_limits_).pose_valid) {
      return {true, lit_backlight};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  MLOGGER_WARN("在坞查询：等视觉锚定超时，位姿仍无效");
  return {true, lit_backlight};
}

void DockMissionNode::in_dock_service(
  const std::shared_ptr<m_common::srv::QueryInDock::Request> request,
  std::shared_ptr<m_common::srv::QueryInDock::Response> response)
{
  bool opened_channel = false;
  bool lit_backlight = false;
  if (!evaluate_in_dock(in_dock_snapshot(), in_dock_limits_).pose_valid && request->wait_vision) {
    std::tie(opened_channel, lit_backlight) = await_vision_pose();
  }
  if (!dock_active_ && b("in_dock_restore_channel")) {
    if (lit_backlight) set_dock_backlight(false, "在坞查询收尾");
    if (opened_channel) set_apriltag_channels(false, "在坞查询收尾");
  }

  auto result = evaluate_in_dock(in_dock_snapshot(), in_dock_limits_);
  response->pose_valid = result.pose_valid;
  response->pose_source = result.pose_source;
  response->dock_x = result.dock_x;
  response->dock_y = result.dock_y;
  response->in_dock = result.in_dock;
  response->message = result.message;
  if (!result.pose_valid) {
    const auto measurement_age = measurement_age_usable();
    const bool channel_ready = apriltag_channel_client_ &&
      apriltag_channel_client_->service_is_ready();
    if (request->wait_vision && opened_channel && channel_ready && estimator_alive() &&
      measurement_age && std::isinf(*measurement_age) && dock_anchor_status_ == "VALID")
    {
      response->message = "检测节点正常、已开启检测但未检测到 Tag；判定不在坞内（相机故障由上层状态机判定）";
    } else if (request->wait_vision && dock_anchor_status_ == "VALID" &&
      !(channel_ready && estimator_alive()))
    {
      response->message = "检测节点异常/未就绪（apriltag 服务或估计器不在线），按失败处理，in_dock 不可信";
    } else if (!request->wait_vision && dock_anchor_status_ == "VALID" &&
      dock_pose_source_ != "VISION" && dock_pose_source_ != "ODOM_PREDICTION")
    {
      response->message = "未尝试开启检测（wait_vision=false），in_dock 不可信；出泊/派航线前请用 wait_vision=true";
    }
  }
  MLOGGER_INFO("in_dock 查询: {}", response->message);
}

void DockMissionNode::release_docking_session(
  const std::string & reason, SpeedAuthority authority, bool end_session)
{
  if (end_session && (pending_nav_ || staging_send_inflight_ ||
    state_ == MissionState::NAV_TO_STAGING))
  {
    cancel_staging_nav(reason);
  }
  dock_cancel_pub_->publish(std_msgs::msg::Empty());
  goal_switch_->apply_cruise();
  set_authority(authority);
  pending_nav_ = false;
  staging_send_inflight_ = false;
  if (end_session) {
    staging_override_.reset();
    dock_active_ = false;
  }
  MLOGGER_INFO("dock session released: {}", reason);
}

void DockMissionNode::cancel_staging_nav(const std::string & reason)
{
  mission_->cancel_async([reason](bool ok, const std::string & message) {
    if (ok) {
      MLOGGER_INFO("cancel_mission [{}]: OK {}", reason, message);
    } else {
      MLOGGER_WARN("cancel_mission [{}]: FAILED {}", reason, message);
    }
  });
}

std::pair<bool, std::string> DockMissionNode::begin_session(
  DockCommand command, const std::string & mission_id, const std::string & command_id,
  const std::optional<std::array<double, 3>> & dock_point)
{
  if (command == DockCommand::CANCEL) return cancel_active_session();
  if (!accepting_new_task()) return {false, "busy in " + to_string(state_)};
  std::string mode;
  if (command == DockCommand::UNDOCK) {
    mode = "undock";
  } else if (command == DockCommand::ONE_CLICK_DOCK) {
    mode = "one_click";
  } else if (command == DockCommand::DOCK_ONLY) {
    mode = "dock_only";
  } else {
    return {false, "unsupported command " + std::to_string(static_cast<int>(command))};
  }
  arm_session(
    mission_id, command_id, mode,
    command == DockCommand::UNDOCK ? std::nullopt : dock_point);
  transition(MissionState::ARMED, mode + " requested");
  const nlohmann::json detail = {{"mode", mode}};
  if (mode == "undock") {
    emit_event("UNDOCK_TASK_ACCEPTED", detail);
    return {true, "undock task accepted"};
  }
  emit_event("DOCK_TASK_ACCEPTED", detail);
  return {true, "dock task accepted mode=" + mode};
}

bool DockMissionNode::accepting_new_task() const
{
  return !dock_active_ && (state_ == MissionState::IDLE || state_ == MissionState::SUCCEEDED ||
    state_ == MissionState::FAILED || state_ == MissionState::CANCELLED);
}

void DockMissionNode::arm_session(
  const std::string & mission_id, const std::string & command_id, const std::string & mode,
  const std::optional<std::array<double, 3>> & dock_point)
{
  staging_retry_ = 0;
  needs_manual_takeover_ = false;
  needs_reapproach_ = false;
  last_fail_reason_.clear();
  session_start_mono_ = monotonic_seconds();
  dock_active_ = true;
  staging_override_ = dock_point;
  auto trim = [](std::string value) {
    const auto first = value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) return std::string();
    const auto last = value.find_last_not_of(" \t\n\r");
    return value.substr(first, last - first + 1);
  };
  mission_id_ = trim(mission_id).empty() ? s("dock_mission_id") : trim(mission_id);
  command_id_ = trim(command_id);
  mode_ = mode;
}

std::pair<bool, std::string> DockMissionNode::cancel_active_session()
{
  if (!dock_active_ && state_ == MissionState::IDLE) return {true, "already idle"};
  release_docking_session("cancelled");
  transition(MissionState::CANCELLED, "cancelled");
  emit_event("DOCK_CANCELLED");
  transition(MissionState::IDLE, "cancel settle");
  set_apriltag_channels(false, "取消收尾");
  return {true, "cancelled"};
}

std::pair<bool, std::string> DockMissionNode::begin_dock_mission()
{
  return begin_session(DockCommand::ONE_CLICK_DOCK);
}

void DockMissionNode::home_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  if (!message->data) return;
  const auto [ok, note] = begin_dock_mission();
  if (!ok) MLOGGER_WARN("ignore /dock/home: {}", note);
}

void DockMissionNode::gcs_command_callback(const std_msgs::msg::String::SharedPtr message)
{
  auto parsed = parse_gcs_command(message->data);
  if (!parsed) {
    MLOGGER_WARN("ignore gcs dock command: {}", message->data.substr(0, 120));
    return;
  }
  const auto dock_point = resolve_dock_point(*parsed);
  if (accept_gcs_staging_override_ && parsed->has_dock_point() && !dock_point) {
    MLOGGER_WARN("gcs dock command ignored (invalid dock point): {}", message->data.substr(0, 120));
    return;
  }
  const auto [ok, note] = begin_session(
    parsed->command, parsed->mission_id, parsed->command_id, dock_point);
  if (!ok) MLOGGER_WARN("gcs dock command rejected: {}", note);
}

std::optional<std::array<double, 3>> DockMissionNode::resolve_dock_point(
  const GcsCommand & command) const
{
  if (!accept_gcs_staging_override_ || !command.has_dock_point()) return std::nullopt;
  const double lat = *command.dock_lat;
  const double lon = *command.dock_lon;
  if (!is_valid_dock_point(lat, lon)) return std::nullopt;
  return std::array<double, 3>{lat, lon, command.dock_yaw.value_or(DOCK_YAW_UNSPECIFIED)};
}

std::array<double, 3> DockMissionNode::staging_waypoint() const
{
  if (staging_override_) {
    MLOGGER_INFO(
      "using GCS pre-dock point ({:.7f}, {:.7f}) yaw_deg={:.3f}",
      (*staging_override_)[0], (*staging_override_)[1], (*staging_override_)[2]);
    return *staging_override_;
  }
  const auto bay = load_bay(
    bay_id_, dock_db_path_.empty() ? std::nullopt :
    std::optional<std::filesystem::path>(dock_db_path_));
  return make_staging_waypoint(
    bay, use_gnss_, use_gnss_ ? std::nullopt :
    std::optional<std::filesystem::path>(map_yaml_));
}

void DockMissionNode::start_nav_to_staging()
{
  if (staging_send_inflight_) return;
  goal_switch_->apply_docking();
  set_authority(SpeedAuthority::NAVIGATION);
  if (!mission_->wait_ready(3.0)) {
    on_staging_failed("send_waypoints service unavailable");
    return;
  }
  std::array<double, 3> waypoint;
  try {
    waypoint = staging_waypoint();
  } catch (const std::exception & exception) {
    on_staging_failed(std::string("staging waypoint: ") + exception.what());
    return;
  }
  ++staging_attempt_;
  const int attempt = staging_attempt_;
  staging_send_inflight_ = true;
  staging_send_mono_ = monotonic_seconds();
  mission_->send_staging_async(
    waypoint[0], waypoint[1], waypoint[2], mission_id_,
    "retry" + std::to_string(staging_retry_),
    [this, attempt](bool ok, const std::string & message) {on_staging_sent(ok, message, attempt);});
  MLOGGER_INFO(
    "Nav2 staging goal sent gnss=({:.7f}, {:.7f}) yaw_deg={:.2f} mission_id='{}' attempt={}",
    waypoint[0], waypoint[1], waypoint[2], mission_id_, attempt);
}

void DockMissionNode::on_staging_sent(bool ok, const std::string & message, int attempt)
{
  if (attempt != staging_attempt_ || !staging_send_inflight_) {
    if (ok && state_ != MissionState::NAV_TO_STAGING) cancel_staging_nav("late staging response");
    return;
  }
  staging_send_inflight_ = false;
  if (state_ != MissionState::NAV_TO_STAGING) return;
  if (!ok) {
    on_staging_failed("send_waypoints failed: " + message);
    return;
  }
  pending_nav_ = true;
  emit_event("DOCK_STAGING_STARTED");
}

void DockMissionNode::task_event_callback(const std_msgs::msg::String::SharedPtr message)
{
  if (state_ != MissionState::NAV_TO_STAGING || !pending_nav_) return;
  const auto parsed = parse_task_event(message->data);
  if (!parsed) return;
  const auto & [event, detail] = *parsed;
  const std::string mid = detail.contains("task_id") && !detail["task_id"].is_null() ?
    json_string(detail["task_id"]) : json_string(detail.value("mission_id", nlohmann::json()));
  if (!mid.empty() && mid != mission_id_) return;
  const std::string cid = json_string(detail.value("command_id", nlohmann::json()));
  if (!cid.empty() && cid != "retry" + std::to_string(staging_retry_)) {
    MLOGGER_WARN("忽略残留任务事件 {}（command_id='{}'，当前 retry{}）", event, cid, staging_retry_);
    return;
  }
  if (event == "TASK_COMPLETED") {
    pending_nav_ = false;
    transition(MissionState::SETTLE, "nav staging completed");
  } else if (event == "TASK_FAILED") {
    pending_nav_ = false;
    const std::string reason = detail.contains("reason") && !detail["reason"].is_null() ?
      json_string(detail["reason"]) : "nav task failed";
    on_staging_failed(reason);
  }
}

void DockMissionNode::dock_status_callback(const std_msgs::msg::String::SharedPtr message)
{
  nlohmann::json status;
  try {
    status = nlohmann::json::parse(message->data);
  } catch (const nlohmann::json::exception &) {
    return;
  }
  if (!status.is_object()) return;
  fsm_backlight_active_ = status.value("backlight_session_active", false);
  if (state_ != MissionState::MONITOR_DOCK && state_ != MissionState::MONITOR_UNDOCK) return;
  if (state_ == MissionState::MONITOR_UNDOCK) {
    if (status.value("undock_success", false)) {
      goal_switch_->apply_cruise();
      set_authority(SpeedAuthority::FAILED);
      transition(MissionState::COMPLETE_SETTLE, "undock distance reached");
    } else if (status.value("state", "") == "DOCK_ABORT") {
      const auto value = status.value("abort_reason", nlohmann::json());
      finish_failed(value.is_null() ? "undock failed" : json_string(value));
    }
    return;
  }
  if (status.value("success", false)) {
    goal_switch_->apply_cruise();
    set_authority(SpeedAuthority::FAILED);
    transition(MissionState::COMPLETE_SETTLE, "dock pose success");
  } else if (status.value("needs_manual_takeover", false)) {
    needs_manual_takeover_ = true;
    const auto value = status.value("abort_reason", nlohmann::json());
    finish_failed(value.is_null() ? "manual takeover" : json_string(value));
  } else if (status.value("needs_reapproach", false)) {
    needs_reapproach_ = true;
    const auto value = status.value("abort_reason", nlohmann::json("needs_reapproach"));
    on_staging_failed(value.is_null() ? "needs_reapproach" : json_string(value));
  }
}

void DockMissionNode::finish_failed(const std::string & reason)
{
  last_fail_reason_ = reason;
  release_docking_session(reason, SpeedAuthority::FAILED);
  if (needs_manual_takeover_ || staging_retry_ >= retry_max()) {
    needs_manual_takeover_ = true;
    emit_event("MANUAL_TAKEOVER_REQUESTED", {{"reason", reason}, {"retry_count", staging_retry_}});
  }
  transition(MissionState::FAILED, reason);
  emit_event(
    "DOCK_FAILED",
    {{"reason", reason}, {"retry_count", staging_retry_},
      {"needs_manual_takeover", needs_manual_takeover_},
      {"needs_reapproach", needs_reapproach_}});
  set_apriltag_channels(false, "失败收尾");
}

void DockMissionNode::on_staging_failed(const std::string & reason)
{
  ++staging_retry_;
  release_docking_session("staging retry: " + reason, SpeedAuthority::FAILED, false);
  if (staging_retry_ >= retry_max()) {
    finish_failed(reason);
    return;
  }
  MLOGGER_WARN("staging retry {}/{}: {}", staging_retry_, retry_max(), reason);
  transition(mode_ && *mode_ == "dock_only" ? MissionState::DOCK_HANDOFF : MissionState::NAV_TO_STAGING, reason);
}

void DockMissionNode::finish_succeeded(bool undock)
{
  const std::string event = undock ? "UNDOCK_SUCCEEDED" : "DOCK_SUCCEEDED";
  const std::string kind = undock ? "undock" : "dock";
  release_docking_session(kind + " succeeded");
  transition(MissionState::SUCCEEDED, kind + " complete");
  set_apriltag_channels(false, kind + " 成功收尾");
  emit_event(event, {{"elapsed_sec", elapsed_sec()}, {"mode", mode_ ? nlohmann::json(*mode_) : nlohmann::json(nullptr)}});
  mode_.reset();
  transition(MissionState::IDLE, "session closed");
}

void DockMissionNode::tick()
{
  constexpr double dt = 0.2;
  if (event_pending_ && monotonic_seconds() - event_pending_mono_ > event_ack_timeout_) {
    event_pending_serials_.clear();
    event_pending_ = 0;
    record_event_ack(false, "event ack timeout");
  }
  if (state_ == MissionState::ARMED) {
    if (mode_ && *mode_ == "undock") {
      transition(MissionState::UNDOCK_HANDOFF, "undock skip nav");
    } else if (mode_ && *mode_ == "dock_only") {
      transition(MissionState::DOCK_HANDOFF, "dock_only skip nav");
    } else {
      transition(MissionState::NAV_TO_STAGING, "start nav staging");
      start_nav_to_staging();
    }
  } else if (state_ == MissionState::NAV_TO_STAGING) {
    if (staging_send_inflight_ && monotonic_seconds() - staging_send_mono_ > 12.0) {
      staging_send_inflight_ = false;
      on_staging_failed("send_waypoints response timeout");
    } else if (!pending_nav_ && !staging_send_inflight_) {
      start_nav_to_staging();
    }
  } else if (state_ == MissionState::SETTLE || state_ == MissionState::COMPLETE_SETTLE) {
    tick_settle(dt);
  } else if (state_ == MissionState::DOCK_HANDOFF || state_ == MissionState::UNDOCK_HANDOFF) {
    tick_handoff(state_ == MissionState::UNDOCK_HANDOFF);
  }
  publish_status_if_due();
}

bool DockMissionNode::odom_speed_fresh()
{
  return odom_speed_stamp_ && (get_clock()->now() - *odom_speed_stamp_).seconds() <= 1.0;
}

void DockMissionNode::tick_settle(double dt)
{
  if (state_ == MissionState::SETTLE) {
    set_authority(SpeedAuthority::STAGING_VERIFY);
    settle_elapsed_ += dt;
    if (!b("settle_require_still")) {
      if (settle_elapsed_ >= d("settle_sec")) {
        transition(MissionState::DOCK_HANDOFF, "settle done (timer only)");
      }
      return;
    }
    settle_params_.settle_sec = d("settle_sec");
    settle_params_.max_speed = d("settle_max_speed");
    settle_params_.max_yaw_rate = d("settle_max_yaw_rate");
    settle_params_.still_hold_sec = d("settle_still_hold_sec");
    settle_params_.max_frame_gap_sec = d("settle_max_frame_gap_sec");
    settle_params_.min_still_frames = i("settle_min_still_frames");
    settle_monitor_.set_params(settle_params_);
    if (settle_monitor_.ready(get_clock()->now().seconds(), settle_elapsed_)) {
      transition(
        MissionState::DOCK_HANDOFF,
        "settle done (speed=" + std::to_string(odom_speed_.value_or(0.0)) + " m/s)");
      return;
    }
    if (settle_elapsed_ > d("settle_timeout_sec")) {
      const std::string note = odom_speed_ && odom_speed_fresh() ?
        "speed=" + std::to_string(*odom_speed_) + " m/s" : "odom speed source missing/stale";
      on_staging_failed("SETTLE_TIMEOUT: " + note);
    }
    return;
  }
  complete_settle_elapsed_ += dt;
  if (complete_settle_elapsed_ >= d("complete_settle_sec")) {
    finish_succeeded(mode_ && *mode_ == "undock");
  }
}

void DockMissionNode::tick_handoff(bool undock)
{
  set_authority(SpeedAuthority::DOCKING);
  std_msgs::msg::Bool message;
  message.data = true;
  if (undock) {
    dock_undock_pub_->publish(message);
    emit_event("UNDOCK_HANDOFF");
    transition(MissionState::MONITOR_UNDOCK, "handoff usv_docking undock");
  } else {
    dock_start_pub_->publish(message);
    emit_event("DOCK_HANDOFF");
    transition(MissionState::MONITOR_DOCK, "handoff usv_docking");
  }
}

void DockMissionNode::shutdown()
{
  release_docking_session("shutdown");
}

}  // namespace dock_mission
