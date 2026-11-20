#include "workspace_nav/mission_bridge_node.hpp"
#include "workspace_nav/navigation_logging.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace workspace_nav::mission_bridge
{
using namespace std::chrono_literals;

namespace
{

std::string trim(std::string value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }).base();
  return first < last ? std::string(first, last) : "";
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

std::string normalize_color(const std::string & raw)
{
  const std::string value = lower(trim(raw));
  if (value == "red" || value == "green" || value == "black") return value;
  if (value == "#ff0000") return "red";
  if (value == "#00ff00") return "green";
  if (value == "#000000") return "black";
  return "";
}

std::filesystem::path default_waypoint_path()
{
  if (const char * env = std::getenv("WAYPOINT_FILE_PATH")) {
    if (*env) return std::filesystem::absolute(env);
  }
  try {
    const auto candidate = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("workspace_nav")) / "json" / "waypoints.json";
    if (std::filesystem::exists(candidate)) return candidate;
  } catch (const std::exception &) {
  }
  return std::filesystem::current_path() / "src" / "USV_NAV" / "workspace_nav" / "json" /
         "waypoints.json";
}

}  // namespace

MissionBridgeNode::MissionBridgeNode()
: Node("mission_bridge"), ctrl_stuck_watchdog_(30.0)
{
  declare_parameters();
  resolve_paths();
  load_map();
  tolerance_ = resolve_waypoint_tolerance();
  setup_interfaces();
}

MissionBridgeNode::~MissionBridgeNode()
{
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    navigate_done_ = true;
  }
  navigate_condition_.notify_all();
  for (auto & thread : navigate_threads_) {
    if (thread.joinable()) thread.join();
  }
}

void MissionBridgeNode::declare_parameters()
{
  declare_parameter("waypoint_topic", "/waypoint");
  declare_parameter("color_topic", "/color_code");
  declare_parameter("follow_waypoints_action", "follow_waypoints");
  declare_parameter("compute_path_through_poses_action", "compute_path_through_poses");
  declare_parameter("mission_plan_timeout_sec", 15.0);
  declare_parameter("global_frame", "map");
  declare_parameter("robot_frame", "base_link");
  declare_parameter("map_yaml_path", "");
  declare_parameter("waypoints_json_path", "");
  declare_parameter("target_buoy_json_path", "");
  declare_parameter("datum_source", "map_yaml");
  declare_parameter("map_datum_ref_key", "ref_gnss_10");
  declare_parameter("projection", "enu");
  declare_parameter("odom_topic", "/odometry/filtered");
  declare_parameter("localization_timeout_sec", 3.0);
  declare_parameter("localization_startup_grace_sec", 5.0);
  declare_parameter("ctrl_stuck_timeout_sec", 30.0);
  declare_parameter("nav_status_topic", "/nav_status");
  declare_parameter("waypoint_tolerance_m", 0.0);
  declare_parameter("nav2_params_path", "");
  declare_parameter("tf_check_period_sec", 1.0);
  declare_parameter("debug_mode", false);
  declare_parameter("allow_replace_running_mission", false);
  declare_parameter("allow_repeat_identical_route", false);
  declare_parameter("target_buoy_force_rewrite", false);
  declare_parameter("waypoint_commit_delay_sec", 0.45);
  declare_parameter("mission_cancel_topic", "");
  declare_parameter("suppress_passive_waypoints_after_cancel", true);
  declare_parameter("target_buoy_min_write_period_sec", 0.0);
  declare_parameter("nav_zones_topic", "/nav_zones/current");
  declare_parameter("clear_costmap_on_mission_start", true);
  declare_parameter("reinflate_after_costmap_clear", true);
  declare_parameter("costmap_clear_period_sec", 0.0);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  datum_source_ = trim(get_parameter("datum_source").as_string());
  if (datum_source_.empty()) datum_source_ = "map_yaml";
  ref_key_ = trim(get_parameter("map_datum_ref_key").as_string());
  if (ref_key_.empty()) ref_key_ = "ref_gnss_10";
  projection_ = lower(trim(get_parameter("projection").as_string()));
  if (projection_ != "enu" && projection_ != "utm") projection_ = "enu";
  localization_timeout_sec_ = std::max(0.0, get_parameter("localization_timeout_sec").as_double());
  localization_startup_grace_sec_ = std::max(
    0.0, get_parameter("localization_startup_grace_sec").as_double());
  ctrl_stuck_timeout_sec_ = std::max(0.0, get_parameter("ctrl_stuck_timeout_sec").as_double());
  ctrl_stuck_watchdog_ = CtrlStuckWatchdog(ctrl_stuck_timeout_sec_);
  waypoint_commit_delay_sec_ = std::max(0.05, get_parameter("waypoint_commit_delay_sec").as_double());
  target_buoy_min_period_sec_ = std::max(
    0.0, get_parameter("target_buoy_min_write_period_sec").as_double());
  costmap_clear_period_sec_ = std::max(0.0, get_parameter("costmap_clear_period_sec").as_double());
  debug_ = get_parameter("debug_mode").as_bool();
  allow_replace_running_ = get_parameter("allow_replace_running_mission").as_bool();
  allow_repeat_identical_ = get_parameter("allow_repeat_identical_route").as_bool();
  target_force_rewrite_ = get_parameter("target_buoy_force_rewrite").as_bool();
  suppress_passive_after_cancel_ = get_parameter(
    "suppress_passive_waypoints_after_cancel").as_bool();
  clear_costmap_on_start_ = get_parameter("clear_costmap_on_mission_start").as_bool();
  reinflate_after_clear_ = get_parameter("reinflate_after_costmap_clear").as_bool();
  mission_cancel_topic_ = trim(get_parameter("mission_cancel_topic").as_string());
}

void MissionBridgeNode::resolve_paths()
{
  const std::string waypoint_param = trim(get_parameter("waypoints_json_path").as_string());
  waypoints_path_ = waypoint_param.empty() ? default_waypoint_path() :
    std::filesystem::absolute(waypoint_param);

  const std::string target_param = trim(get_parameter("target_buoy_json_path").as_string());
  if (const char * env = std::getenv("TARGET_JSON_PATH"); env && *env) {
    target_path_ = std::filesystem::absolute(env);
  } else if (!target_param.empty()) {
    target_path_ = std::filesystem::absolute(target_param);
  } else {
    target_path_ = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("workspace_nav")) / "json" / "target_buoy.json";
  }

  const std::string map_param = trim(get_parameter("map_yaml_path").as_string());
  map_path_ = map_param.empty() ? std::filesystem::path(
    ament_index_cpp::get_package_share_directory("workspace_nav")) / "config" / "map.yaml" :
    std::filesystem::absolute(map_param);
}

void MissionBridgeNode::load_map()
{
  if (datum_source_ != "map_yaml") {
    throw std::runtime_error("mission_bridge only supports datum_source=map_yaml");
  }
  try {
    const YAML::Node config = YAML::LoadFile(map_path_.string());
    YAML::Node reference;
    if (!ref_key_.empty() && config[ref_key_]) reference = config[ref_key_];
    if (!reference && config["ref_gnss_10"]) reference = config["ref_gnss_10"];
    if (!reference && config["ref_gnss"]) reference = config["ref_gnss"];
    if (!reference || reference.size() < 2) {
      throw std::runtime_error("map.yaml has no valid ref_gnss_10/ref_gnss datum");
    }
    datum_lon_ = reference[0].as<double>();
    datum_lat_ = reference[1].as<double>();
    if (config["origin"] && config["origin"].IsSequence()) {
      const auto origin = config["origin"];
      if (origin.size() > 0) map_ox_ = origin[0].as<double>();
      if (origin.size() > 1) map_oy_ = origin[1].as<double>();
      if (origin.size() > 2) map_origin_yaw_ = origin[2].as<double>();
    }
    const auto utm = lat_lon_to_utm(datum_lat_, datum_lon_);
    datum_easting_ = utm.easting;
    datum_northing_ = utm.northing;
    map_yaml_resolved_ = map_path_.string();
  } catch (const std::exception & error) {
    throw std::runtime_error("failed to load map " + map_path_.string() + ": " + error.what());
  }
  NAV_LOG_INFO( "loaded map yaml: %s", map_yaml_resolved_.c_str());
  NAV_LOG_INFO("datum latitude: %.10f, datum longitude: %.10f", datum_lat_, datum_lon_);
  NAV_LOG_INFO(
    "origin (ox oy yaw_rad): %.4f %.4f %.6f", map_ox_, map_oy_, map_origin_yaw_);
  NAV_LOG_INFO( "Writing waypoints to: %s", waypoints_path_.c_str());
  NAV_LOG_INFO( "Writing target buoy to: %s", target_path_.c_str());
}

double MissionBridgeNode::monotonic_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double MissionBridgeNode::wall_seconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::optional<double> MissionBridgeNode::read_nav2_tolerance(
  const std::filesystem::path & path) const
{
  try {
    const YAML::Node config = YAML::LoadFile(path.string());
    const YAML::Node params = config["controller_server"]["ros__parameters"];
    if (!params || !params.IsMap()) return std::nullopt;
    std::vector<std::string> names;
    const auto plugins = params["goal_checker_plugins"];
    if (plugins.IsSequence()) {
      for (const auto & name : plugins) names.push_back(name.as<std::string>());
    } else if (plugins.IsScalar()) {
      names.push_back(plugins.as<std::string>());
    }
    names.push_back("general_goal_checker");
    for (const auto & name : names) {
      if (params[name] && params[name]["xy_goal_tolerance"]) {
        return params[name]["xy_goal_tolerance"].as<double>();
      }
    }
    for (const auto & item : params) {
      if (item.second.IsMap() && item.second["xy_goal_tolerance"]) {
        return item.second["xy_goal_tolerance"].as<double>();
      }
    }
  } catch (const std::exception &) {
  }
  return std::nullopt;
}

double MissionBridgeNode::resolve_waypoint_tolerance() const
{
  const double explicit_value = get_parameter("waypoint_tolerance_m").as_double();
  if (explicit_value > 0.0) {
    NAV_LOG_INFO( "waypoint_tolerance_m: %.2f m (explicit)", explicit_value);
    return explicit_value;
  }
  std::filesystem::path path;
  const std::string candidate = trim(get_parameter("nav2_params_path").as_string());
  if (!candidate.empty()) {
    path = std::filesystem::absolute(candidate);
  } else {
    path = map_path_.parent_path() / "nav2_params_chengfeng.yaml";
    if (!std::filesystem::exists(path)) {
      const auto alternate = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("workspace_nav")) / "config" /
        "nav2_params_chengfeng.yaml";
      if (std::filesystem::exists(alternate)) path = alternate;
    }
  }
  const auto tolerance = read_nav2_tolerance(path);
  if (!tolerance || *tolerance <= 0.0) {
    NAV_LOG_WARN( "failed to read xy_goal_tolerance from %s; using 1.5 m", path.c_str());
    return 1.5;
  }
  NAV_LOG_INFO( "waypoint_tolerance_m: %.2f m (from %s)", *tolerance, path.c_str());
  return *tolerance;
}

void MissionBridgeNode::setup_interfaces()
{
  NAV_LOG_INFO( "setting up mission_bridge interfaces");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  waypoint_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  path_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  waypoint_client_ = rclcpp_action::create_client<FollowWaypoints>(
    this, get_parameter("follow_waypoints_action").as_string(), waypoint_group_);
  path_client_ = rclcpp_action::create_client<ComputePath>(
    this, get_parameter("compute_path_through_poses_action").as_string(), path_group_);
  clear_local_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap");
  clear_global_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap");
  if (reinflate_after_clear_) {
    reinflate_get_client_ = create_client<rcl_interfaces::srv::GetParameters>(
      "/local_costmap/local_costmap/get_parameters");
    reinflate_set_client_ = create_client<rcl_interfaces::srv::SetParameters>(
      "/local_costmap/local_costmap/set_parameters");
  }

  waypoint_subscription_ = create_subscription<std_msgs::msg::String>(
    get_parameter("waypoint_topic").as_string(), 10,
    std::bind(&MissionBridgeNode::waypoint_callback, this, std::placeholders::_1));
  color_subscription_ = create_subscription<std_msgs::msg::String>(
    get_parameter("color_topic").as_string(), 10,
    std::bind(&MissionBridgeNode::color_callback, this, std::placeholders::_1));
  rclcpp::QoS zone_qos(1);
  zone_qos.reliable().transient_local();
  zones_subscription_ = create_subscription<std_msgs::msg::String>(
    get_parameter("nav_zones_topic").as_string(), zone_qos,
    std::bind(&MissionBridgeNode::nav_zones_callback, this, std::placeholders::_1));
  if (!mission_cancel_topic_.empty()) {
    cancel_subscription_ = create_subscription<std_msgs::msg::Empty>(
      mission_cancel_topic_, 10,
      std::bind(&MissionBridgeNode::mission_cancel_callback, this, std::placeholders::_1));
  }

  watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions watchdog_options;
  watchdog_options.callback_group = watchdog_group_;
  nav_status_subscription_ = create_subscription<m_common::msg::NavStatus>(
    get_parameter("nav_status_topic").as_string(), 10,
    std::bind(&MissionBridgeNode::nav_status_callback, this, std::placeholders::_1),
    watchdog_options);
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MissionBridgeNode::tick_ctrl_stuck_watchdog, this), watchdog_group_);

  state_publisher_ = create_publisher<std_msgs::msg::String>("/mission_bridge/state", 10);
  status_detail_publisher_ = create_publisher<std_msgs::msg::String>(
    "/mission_bridge/status_detail", 10);
  status_detail_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), [this]() {publish_status_detail();});

  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  send_waypoints_service_ = create_service<m_common::srv::SendWaypoints>(
    "mission_bridge/send_waypoints",
    std::bind(
      &MissionBridgeNode::send_waypoints_callback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, service_group_);
  plan_mission_service_ = create_service<m_common::srv::PlanMission>(
    "mission_bridge/plan_mission",
    std::bind(
      &MissionBridgeNode::plan_mission_callback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, service_group_);
  set_pause_service_ = create_service<m_common::srv::SetPause>(
    "mission_bridge/set_pause",
    std::bind(
      &MissionBridgeNode::set_pause_callback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, service_group_);
  cancel_mission_service_ = create_service<m_common::srv::CancelMission>(
    "mission_bridge/cancel_mission",
    std::bind(
      &MissionBridgeNode::cancel_mission_callback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, service_group_);

  NAV_LOG_INFO( "mission_bridge services ready");
  navigate_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rcl_action_server_options_t action_options = rcl_action_server_get_default_options();
  navigate_server_ = rclcpp_action::create_server<NavigateTask>(
    this, "/mission_bridge/navigate",
    std::bind(
      &MissionBridgeNode::navigate_goal_callback, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&MissionBridgeNode::navigate_cancel_callback, this, std::placeholders::_1),
    std::bind(&MissionBridgeNode::navigate_accepted_callback, this, std::placeholders::_1),
    action_options, navigate_group_);

  NAV_LOG_INFO( "NavigateTask action server ready");
  publish_status_detail();
  double period = std::max(0.1, get_parameter("tf_check_period_sec").as_double());
  ready_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
    std::bind(&MissionBridgeNode::tick_ready, this));
}

void MissionBridgeNode::publish_status_detail(
  const std::optional<std::string> & state_override,
  const std::optional<std::string> & error_code,
  int nav2_error_code, const std::string & nav2_error_msg,
  const std::string & transition_reason)
{
  nlohmann::json payload;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    payload["state"] = state_override.value_or(to_string(state_));
    payload["task_id"] = current_mission_id_;
    payload["command_id"] = current_command_id_;
    payload["waypoint_total"] = nav_xy_.size();
    payload["waypoint_completed"] = current_index_;
    payload["waypoint_current_index"] = current_index_;
    payload["elapsed_sec"] = mission_start_wall_ > 0.0 ?
      std::round((wall_seconds() - mission_start_wall_) * 10.0) / 10.0 : 0.0;
  }
  payload["error_code"] = error_code.has_value() ? nlohmann::json(*error_code) : nlohmann::json();
  if (nav2_error_code != 0 || !nav2_error_msg.empty()) {
    payload["nav2_error_code"] = nav2_error_code;
    payload["nav2_error_msg"] = nav2_error_msg;
  }
  if (!transition_reason.empty()) payload["transition_reason"] = transition_reason;
  status_detail_publisher_->publish(std_msgs::msg::String().set__data(payload.dump()));
}

void MissionBridgeNode::transition_state(MissionState state, const std::string & reason)
{
  bool changed = false;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    changed = state != state_;
    if (changed) {
      state_ = state;
      ctrl_stuck_watchdog_.reset();
    }
  }
  if (changed) {
    state_publisher_->publish(std_msgs::msg::String().set__data(to_string(state)));
    NAV_LOG_INFO( "STATE -> %s", to_string(state).c_str());
    publish_status_detail(std::nullopt, std::nullopt, 0, "", reason);
  }
}

bool MissionBridgeNode::tf_ok()
{
  try {
    return tf_buffer_->canTransform(global_frame_, robot_frame_, tf2::TimePointZero, 50ms);
  } catch (const std::exception &) {
    return false;
  }
}

bool MissionBridgeNode::odom_fresh() const
{
  if (!odom_subscription_ || localization_timeout_sec_ <= 0.0) return true;
  const double now = monotonic_seconds();
  const double last = odom_last_mono_.load();
  if (last <= 0.0) {
    return odom_sub_started_mono_ <= 0.0 ||
           now - odom_sub_started_mono_ <= localization_startup_grace_sec_;
  }
  return now - last <= localization_timeout_sec_;
}

void MissionBridgeNode::tick_ready()
{
  const bool transform_ok = tf_ok();
  const bool action_ok = waypoint_client_->wait_for_action_server(200ms);
  MissionState state;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state = state_;
  }
  const bool odom_ok = (state == MissionState::Running || state == MissionState::WaitingSystem) ?
    odom_fresh() : true;
  if (state == MissionState::WaitingSystem) {
    NAV_LOG_INFO_THROTTLE(
      2000, "readiness: tf=%d action=%d odom=%d", transform_ok, action_ok, odom_ok);
    if (transform_ok && action_ok && odom_ok) {
      transition_state(MissionState::Idle);
      NAV_LOG_INFO( "TF and FollowWaypoints are ready");
    }
    return;
  }
  if (!transform_ok || !action_ok) {
    on_system_not_ready();
  } else if (!odom_ok) {
    on_system_not_ready("LOCALIZATION_TIMEOUT");
  }
}

void MissionBridgeNode::on_system_not_ready(const std::string & reason)
{
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ == MissionState::WaitingSystem) return;
    if (state_ == MissionState::Running || state_ == MissionState::Paused) ++mission_token_;
  }
  cancel_active_nav2_goal();
  const bool localization = reason == "LOCALIZATION_TIMEOUT";
  navigate_terminate(
    NavigateTask::Result::RESULT_LOCALIZATION_LOST,
    localization ? "LOC_LOST" : "SYSTEM_NOT_READY",
    localization ? "Localization odom timeout" : "TF or FollowWaypoints unavailable during mission",
    "abort");
  transition_state(MissionState::WaitingSystem);
}

void MissionBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const std::string source_frame = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  if (source_frame != global_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(global_frame_, source_frame, tf2::TimePointZero);
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      const double transformed_x = t.x + c * x - s * y;
      y = t.y + s * x + c * y;
      x = transformed_x;
    } catch (const std::exception & error) {
      have_odom_ = false;
      NAV_LOG_WARN_THROTTLE( 2000, "odom TF unavailable: %s", error.what());
      return;
    }
  }
  odom_last_mono_ = monotonic_seconds();
  have_odom_ = true;
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_xy_ = {x, y};
}

std::pair<double, double> MissionBridgeNode::robot_xy() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return current_pose_xy_;
}

void MissionBridgeNode::nav_status_callback(const m_common::msg::NavStatus::SharedPtr msg)
{
  if (ctrl_stuck_timeout_sec_ <= 0.0) return;
  const bool stuck = msg->task_state == "RUNNING" &&
    (msg->last_error == "CTRL_STUCK" || msg->controller_status == "STUCK");
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  ctrl_stuck_watchdog_.update(state_, stuck, monotonic_seconds());
}

void MissionBridgeNode::tick_ctrl_stuck_watchdog()
{
  if (ctrl_stuck_timeout_sec_ <= 0.0) return;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ != MissionState::Running || !ctrl_stuck_watchdog_.should_fail(monotonic_seconds())) {
      return;
    }
    ++mission_token_;
    ctrl_stuck_watchdog_.reset();
  }
  cancel_active_nav2_goal();
  on_nav_fatal(
    "CTRL_STUCK", 0, "CTRL_STUCK persisted beyond configured timeout",
    NavigateTask::Result::RESULT_CONTROLLER_FAILED);
}

}  // namespace workspace_nav::mission_bridge

namespace workspace_nav::mission_bridge
{
using namespace std::chrono_literals;

nlohmann::json MissionBridgeNode::make_waypoints_document(
  const std::vector<LatLon> & waypoints) const
{
  nlohmann::json document = {
    {"waypoints", nlohmann::json::array()},
    {"datum", {{"latitude", datum_lat_}, {"longitude", datum_lon_}}},
    {"datum_source_meta", {
      {"datum_source", datum_source_}, {"map_yaml", map_yaml_resolved_},
      {"map_datum_ref_key", ref_key_}, {"projection", projection_}}},
    {"map_frame_meta", {
      {"frame_id", "map"},
      {"description", "x,y are Nav2 map coordinates in metres with map.yaml origin applied"},
      {"origin_x", map_ox_}, {"origin_y", map_oy_}, {"origin_yaw_rad", map_origin_yaw_},
      {"applied_origin_transform", true}}}
  };
  for (const auto & waypoint : waypoints) {
    Point point;
    if (projection_ == "utm") {
      const auto converted = lat_lon_to_utm(waypoint.latitude, waypoint.longitude);
      point = enu_delta_to_map_xy(
        converted.easting - datum_easting_, converted.northing - datum_northing_,
        map_ox_, map_oy_, map_origin_yaw_);
    } else {
      const auto delta = geodetic_delta_enu_m(
        datum_lat_, datum_lon_, waypoint.latitude, waypoint.longitude);
      point = enu_delta_to_map_xy(delta.x, delta.y, map_ox_, map_oy_, map_origin_yaw_);
    }
    document["waypoints"].push_back({
      {"latitude", waypoint.latitude}, {"longitude", waypoint.longitude},
      {"x", std::round(point.x * 10000.0) / 10000.0},
      {"y", std::round(point.y * 10000.0) / 10000.0}});
  }
  return document;
}

void MissionBridgeNode::atomic_write_json(
  const std::filesystem::path & path, const nlohmann::json & document) const
{
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.parent_path() /
    ("json_" + std::to_string(getpid()) + "_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".tmp");
  const std::string content = document.dump(2, ' ', true);
  const int descriptor = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (descriptor < 0) throw std::runtime_error("cannot create temporary JSON file");
  try {
    std::size_t written = 0;
    while (written < content.size()) {
      const ssize_t count = ::write(descriptor, content.data() + written, content.size() - written);
      if (count <= 0) throw std::runtime_error("failed writing JSON file");
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(descriptor) != 0) throw std::runtime_error("failed syncing JSON file");
    if (::close(descriptor) != 0) throw std::runtime_error("failed closing JSON file");
    if (::rename(temporary.c_str(), path.c_str()) != 0) {
      throw std::runtime_error("failed replacing JSON file");
    }
  } catch (...) {
    ::close(descriptor);
    ::unlink(temporary.c_str());
    throw;
  }
}

bool MissionBridgeNode::verify_waypoints_file(const std::filesystem::path & path) const
{
  try {
    std::ifstream stream(path);
    const auto document = nlohmann::json::parse(stream);
    return document.is_object() && document.contains("waypoints") &&
           document["waypoints"].is_array() && document.contains("datum") &&
           document["datum"].is_object();
  } catch (const std::exception &) {
    return false;
  }
}

void MissionBridgeNode::clear_waypoint_file()
{
  try {
    atomic_write_json(waypoints_path_, {{"waypoints", nlohmann::json::array()}});
    NAV_LOG_INFO( "Waypoint file cleared: %s", waypoints_path_.c_str());
  } catch (const std::exception & error) {
    NAV_LOG_ERROR( "Failed clearing waypoint file: %s", error.what());
  }
}

geometry_msgs::msg::PoseStamped MissionBridgeNode::create_pose(
  double x, double y, double yaw_degrees)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  const double yaw = yaw_degrees * M_PI / 180.0;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

Point MissionBridgeNode::to_map(const LatLon & waypoint) const
{
  const auto delta = geodetic_delta_enu_m(
    datum_lat_, datum_lon_, waypoint.latitude, waypoint.longitude);
  return enu_delta_to_map_xy(delta.x, delta.y, map_ox_, map_oy_, map_origin_yaw_);
}

void MissionBridgeNode::color_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string semantic = normalize_color(msg->data);
  if (semantic.empty()) {
    NAV_LOG_WARN( "Unknown color payload: '%s', skipped", msg->data.c_str());
    return;
  }
  if (semantic == last_written_target_semantic_ && !target_force_rewrite_) return;
  const double now = wall_seconds();
  if (target_buoy_min_period_sec_ > 0.0 && last_target_write_wall_ > 0.0 &&
    now - last_target_write_wall_ < target_buoy_min_period_sec_)
  {
    return;
  }
  const std::time_t time = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&time, &utc);
  char timestamp[32];
  std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc);
  try {
    atomic_write_json(target_path_, {
      {"color", semantic},
      {"target", {{"color", semantic}, {"timestamp", timestamp}}}});
    last_written_target_semantic_ = semantic;
    last_target_write_wall_ = wall_seconds();
    NAV_LOG_INFO( "Updated target_buoy.json (%s)", semantic.c_str());
  } catch (const std::exception & error) {
    NAV_LOG_ERROR( "Failed writing target_buoy.json: %s", error.what());
  }
}

void MissionBridgeNode::nav_zones_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto document = nlohmann::json::parse(msg->data, nullptr, false);
  if (!document.is_object()) {
    NAV_LOG_WARN( "/nav_zones/current JSON is invalid; ignored");
    return;
  }
  std::vector<Fence> fences;
  if (document.contains("fences") && document["fences"].is_array()) {
    for (const auto & item : document["fences"]) {
      try {
        const std::string type = item.value("type", "");
        if (type != "inclusion" && type != "exclusion") continue;
        Fence fence;
        fence.fence_id = item.value("fence_id", "");
        fence.type = type;
        fence.shape = item.value("shape", "");
        if (fence.shape == "polygon") {
          for (const auto & point : item.value("points", nlohmann::json::array())) {
            const LatLon geographic{point.at(1).get<double>(), point.at(0).get<double>()};
            fence.points.push_back(to_map(geographic));
          }
          if (fence.points.size() < 3) continue;
        } else if (fence.shape == "circle") {
          const auto & center = item.at("center");
          fence.center = to_map({center.at(1).get<double>(), center.at(0).get<double>()});
          fence.radius_m = item.value("radius_m", 0.0);
          if (fence.radius_m <= 0.0) continue;
        } else {
          continue;
        }
        fences.push_back(std::move(fence));
      } catch (const std::exception &) {
      }
    }
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    zone_fences_ = std::move(fences);
  }
}

std::optional<std::string> MissionBridgeNode::nav_zones_violation(
  const std::vector<LatLon> & waypoints) const
{
  std::vector<Fence> fences;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    fences = zone_fences_;
  }
  if (fences.empty()) return std::nullopt;
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto violation = fence_violation_index(to_map(waypoints[i]), fences);
    if (!violation) continue;
    std::ostringstream message;
    message << "Waypoint #" << i << " (" << waypoints[i].latitude << ", " <<
      waypoints[i].longitude << ") violates fence " << fences[*violation].fence_id;
    return message.str();
  }
  return std::nullopt;
}

void MissionBridgeNode::waypoint_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto parsed = parse_waypoint_message(msg->data);
  if (!parsed || parsed->waypoints.empty()) {
    NAV_LOG_WARN( "Invalid or empty waypoint message; skipped");
    return;
  }
  if (suppress_passive_after_cancel_ && suppress_passive_waypoints_ && !parsed->explicit_replan) {
    const double now = wall_seconds();
    if (now - last_passive_waypoint_wall_ > 4.0) {
      last_passive_waypoint_wall_ = now;
      NAV_LOG_INFO( "passive /waypoint ignored after Cancel Nav");
    }
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    debounced_route_ = DebouncedRoute{
      parsed->waypoints, waypoint_mission_hash(parsed->waypoints, parsed->mission_id),
      parsed->explicit_replan, parsed->mission_id, parsed->command_id};
  }
  reschedule_waypoint_commit();
}

void MissionBridgeNode::reschedule_waypoint_commit()
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (waypoint_commit_timer_) waypoint_commit_timer_->cancel();
  waypoint_commit_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(waypoint_commit_delay_sec_)),
    [this]() {
      {
        std::lock_guard<std::mutex> timer_lock(timer_mutex_);
        if (waypoint_commit_timer_) waypoint_commit_timer_->cancel();
        waypoint_commit_timer_.reset();
      }
      std::optional<DebouncedRoute> route;
      {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        route = std::move(debounced_route_);
        debounced_route_.reset();
      }
      if (route) {
        consume_waypoint_command(
          route->waypoints, route->hash, route->explicit_replan,
          route->mission_id, route->command_id);
      }
    });
}

void MissionBridgeNode::consume_waypoint_command(
  const std::vector<LatLon> & waypoints, const std::string & hash,
  bool explicit_replan, const std::string & mission_id, const std::string & command_id)
{
  if (explicit_replan) {
    suppress_passive_waypoints_ = false;
  } else if (suppress_passive_after_cancel_ && suppress_passive_waypoints_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (delayed_mission_timer_) delayed_mission_timer_->cancel();
    delayed_mission_timer_.reset();
    delayed_mission_.reset();
  }

  bool preempt = false;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ == MissionState::WaitingSystem) {
      NAV_LOG_WARN( "System not ready; waypoint ignored");
      return;
    }
    if (state_ == MissionState::Running) {
      if (hash == running_mission_hash_) {
        if (explicit_replan) preempt = true;
        else return;
      } else if (!allow_replace_running_ && !explicit_replan) {
        NAV_LOG_WARN( "mission running, new mission rejected");
        return;
      } else {
        preempt = true;
      }
    } else if (state_ == MissionState::Paused) {
      if (!explicit_replan) {
        NAV_LOG_WARN( "PAUSED: waypoint ignored without explicit_replan");
        return;
      }
      paused_nav_xy_.clear();
      paused_index_ = 0;
      preempt = true;
    } else if (state_ != MissionState::Idle && state_ != MissionState::Completed &&
      state_ != MissionState::Failed)
    {
      return;
    } else if (!explicit_replan && hash == last_completed_mission_hash_ && !allow_repeat_identical_) {
      return;
    }
    current_mission_id_ = mission_id;
    current_command_id_ = command_id;
  }

  if (!preempt) {
    execute_mission_atomic(waypoints, hash);
    return;
  }
  preempt_running_mission(true);
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    delayed_mission_ = DelayedMission{waypoints, hash};
    delayed_mission_timer_ = create_wall_timer(220ms, [this, hash]() {
        std::optional<DelayedMission> mission;
        {
          std::lock_guard<std::mutex> timer_lock(timer_mutex_);
          if (delayed_mission_timer_) delayed_mission_timer_->cancel();
          delayed_mission_timer_.reset();
          if (delayed_mission_ && delayed_mission_->hash == hash) {
            mission = std::move(delayed_mission_);
            delayed_mission_.reset();
          }
        }
        if (mission) execute_mission_atomic(mission->waypoints, mission->hash);
      });
  }
}

void MissionBridgeNode::preempt_running_mission(bool for_replace)
{
  if (active_goal_handle_ || navigating_) ++mission_token_;
  cancel_active_nav2_goal();
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    running_mission_hash_.clear();
  }
  transition_state(MissionState::Idle, for_replace ? "replace" : "cancel");
  navigate_terminate(
    NavigateTask::Result::RESULT_CANCELED, "PREEMPTED",
    for_replace ? "mission preempted by legacy channel (replace)" :
    "mission preempted by legacy channel (cancel)", "abort");
}

void MissionBridgeNode::mission_cancel_callback(const std_msgs::msg::Empty::SharedPtr)
{
  const auto result = apply_mission_cancel();
  if (!result.first) NAV_LOG_WARN( "%s", result.second.c_str());
}

std::pair<bool, std::string> MissionBridgeNode::apply_mission_cancel()
{
  MissionState state;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state = state_;
  }
  if (state == MissionState::WaitingSystem) return {false, "System not ready (WAITING_SYSTEM)"};
  if (suppress_passive_after_cancel_) suppress_passive_waypoints_ = true;
  {
    std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
    debounced_route_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (waypoint_commit_timer_) waypoint_commit_timer_->cancel();
    waypoint_commit_timer_.reset();
    if (delayed_mission_timer_) delayed_mission_timer_->cancel();
    delayed_mission_timer_.reset();
    delayed_mission_.reset();
  }
  clear_waypoint_file();

  if (state == MissionState::Paused) {
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      paused_nav_xy_.clear();
      paused_index_ = 0;
      nav_xy_.clear();
      current_index_ = 0;
      running_mission_hash_.clear();
    }
    transition_state(MissionState::Idle, "cancel");
    navigate_terminate(
      NavigateTask::Result::RESULT_CANCELED, "CANCELED", "paused mission cancelled", "abort");
    return {true, "Paused mission cancelled, state IDLE"};
  }
  if (state == MissionState::Running) {
    preempt_running_mission(false);
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    nav_xy_.clear();
    current_index_ = 0;
    return {true, "Running mission cancelled, state IDLE"};
  }
  if (state == MissionState::Idle) return {true, "Already idle (idempotent)"};
  transition_state(MissionState::Idle, "cancel");
  return {true, "Mission cleared, state IDLE"};
}

void MissionBridgeNode::execute_mission_atomic(
  const std::vector<LatLon> & waypoints, const std::string & hash)
{
  try {
    const auto document = make_waypoints_document(waypoints);
    atomic_write_json(waypoints_path_, document);
    if (!verify_waypoints_file(waypoints_path_)) {
      NAV_LOG_ERROR( "waypoints.json verification failed; mission aborted");
      return;
    }
    std::vector<NavWaypoint> converted;
    for (const auto & entry : document["waypoints"]) {
      converted.push_back({entry["x"].get<double>(), entry["y"].get<double>(), kYawUnspecified});
    }
    ++mission_token_;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      if (state_ != MissionState::Idle && state_ != MissionState::Completed &&
        state_ != MissionState::Failed) return;
      nav_xy_ = std::move(converted);
      current_index_ = 0;
      navigating_ = false;
      running_mission_hash_ = hash;
    }
    transition_state(MissionState::Running);
    mission_start_wall_ = wall_seconds();
    publish_status_detail();
    if (clear_costmap_on_start_) {
      clear_local_costmap("new mission");
      clear_global_costmap("new mission");
    }
    setup_costmap_clear_timer();
    start_nav_stack();
  } catch (const std::exception & error) {
    NAV_LOG_ERROR( "Failed to execute mission: %s", error.what());
    transition_state(MissionState::Failed);
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      running_mission_hash_.clear();
    }
    state_to_idle_relaxed();
  }
}

std::string MissionBridgeNode::write_and_start_mission(
  const std::vector<LatLon> & waypoints, const std::vector<double> & yaws,
  const std::string & mission_id, const std::string & command_id)
{
  auto document = make_waypoints_document(waypoints);
  std::vector<NavWaypoint> converted;
  for (std::size_t i = 0; i < document["waypoints"].size(); ++i) {
    document["waypoints"][i]["yaw"] = yaws[i];
    converted.push_back({
      document["waypoints"][i]["x"].get<double>(),
      document["waypoints"][i]["y"].get<double>(), yaws[i]});
  }
  atomic_write_json(waypoints_path_, document);
  const std::string hash = waypoint_mission_hash(waypoints, mission_id);
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    current_mission_id_ = mission_id;
    current_command_id_ = command_id;
    nav_xy_ = std::move(converted);
    current_index_ = 0;
    navigating_ = false;
    running_mission_hash_ = hash;
  }
  suppress_passive_waypoints_ = false;
  ++mission_token_;
  transition_state(MissionState::Running);
  mission_start_wall_ = wall_seconds();
  if (clear_costmap_on_start_) {
    clear_local_costmap("new mission");
    clear_global_costmap("new mission");
  }
  setup_costmap_clear_timer();
  start_nav_stack();
  return hash;
}

void MissionBridgeNode::start_nav_stack()
{
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (send_timer_) send_timer_->cancel();
    send_timer_.reset();
  }
  have_odom_ = false;
  odom_last_mono_ = 0.0;
  odom_sub_started_mono_ = monotonic_seconds();
  if (!odom_subscription_) {
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&MissionBridgeNode::odom_callback, this, std::placeholders::_1));
  }
  reset_send_timer(2.0);
}

void MissionBridgeNode::reset_send_timer(double seconds)
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (send_timer_) send_timer_->cancel();
  send_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds)),
    std::bind(&MissionBridgeNode::send_next_waypoint, this));
}

void MissionBridgeNode::send_next_waypoint()
{
  if (!have_odom_) {
    NAV_LOG_WARN( "Waiting for odometry before sending waypoint");
    reset_send_timer(0.5);
    return;
  }
  NavWaypoint waypoint;
  std::size_t index;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ != MissionState::Running || navigating_ || current_index_ >= nav_xy_.size()) return;
    waypoint = nav_xy_[current_index_];
    index = current_index_;
  }
  const auto robot = robot_xy();
  const double distance = std::hypot(waypoint.x - robot.first, waypoint.y - robot.second);
  if (should_skip_waypoint(distance, tolerance_, waypoint.yaw)) {
    NAV_LOG_INFO( "Waypoint %zu within tolerance; skipping", index + 1);
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      ++current_index_;
    }
    reset_send_timer(0.5);
    finalize_if_done();
    return;
  }
  if (!yaw_is_specified(waypoint.yaw)) {
    double previous_x = robot.first;
    double previous_y = robot.second;
    if (index > 0) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      previous_x = nav_xy_[index - 1].x;
      previous_y = nav_xy_[index - 1].y;
    }
    waypoint.yaw = std::atan2(waypoint.y - previous_y, waypoint.x - previous_x) * 180.0 / M_PI;
  }
  FollowWaypoints::Goal goal;
  goal.poses = {create_pose(waypoint.x, waypoint.y, waypoint.yaw)};
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    navigating_ = true;
  }
  NAV_LOG_INFO( "Sending waypoint %zu x=%.2f y=%.2f yaw=%.3f",
    index + 1, waypoint.x, waypoint.y, waypoint.yaw);
  if (!waypoint_client_->wait_for_action_server(5s)) {
    NAV_LOG_ERROR( "FollowWaypoints server not available");
    on_nav_fatal();
    return;
  }
  const uint64_t token = mission_token_.load();
  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
  options.goal_response_callback = [this, token](FollowGoalHandle::SharedPtr handle) {
      goal_response(handle, token);
    };
  options.result_callback = [this, token](const FollowGoalHandle::WrappedResult & result) {
      goal_result(result, token);
    };
  options.feedback_callback =
    [this](FollowGoalHandle::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback>) {
      navigate_feedback_seen_ = true;
    };
  waypoint_client_->async_send_goal(goal, options);
}

void MissionBridgeNode::goal_response(FollowGoalHandle::SharedPtr goal_handle, uint64_t token)
{
  if (token != mission_token_) {
    if (goal_handle) waypoint_client_->async_cancel_goal(goal_handle);
    return;
  }
  if (!goal_handle) {
    NAV_LOG_ERROR( "FollowWaypoints goal rejected");
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      navigating_ = false;
      active_goal_handle_.reset();
    }
    on_nav_fatal();
    return;
  }
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  active_goal_handle_ = goal_handle;
}

void MissionBridgeNode::goal_result(const FollowGoalHandle::WrappedResult & result, uint64_t token)
{
  if (token != mission_token_) {
    NAV_LOG_INFO( "Discarding stale FollowWaypoints result");
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    navigating_ = false;
    active_goal_handle_.reset();
  }
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    on_nav_fatal("MISSION_FAILED");
    return;
  }
  if (!result.result->missed_waypoints.empty()) {
    on_nav_fatal("MISSION_FAILED", 0, "waypoint missed", NavigateTask::Result::RESULT_PLANNING_FAILED);
    return;
  }
  bool done = false;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    NAV_LOG_INFO( "Waypoint %zu reached successfully", current_index_ + 1);
    ++current_index_;
    done = current_index_ >= nav_xy_.size();
  }
  publish_status_detail();
  if (done) finish_all_waypoints_success();
  else reset_send_timer(1.0);
}

void MissionBridgeNode::cancel_active_nav2_goal()
{
  FollowGoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    handle = active_goal_handle_;
    active_goal_handle_.reset();
    navigating_ = false;
  }
  if (handle) waypoint_client_->async_cancel_goal(handle);
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (send_timer_) send_timer_->cancel();
  send_timer_.reset();
}

void MissionBridgeNode::finalize_if_done()
{
  std::lock_guard<std::recursive_mutex> lock(state_mutex_);
  if (current_index_ >= nav_xy_.size()) finish_all_waypoints_success();
}

void MissionBridgeNode::finish_all_waypoints_success()
{
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (send_timer_) send_timer_->cancel();
    send_timer_.reset();
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    active_goal_handle_.reset();
    last_completed_mission_hash_ = running_mission_hash_;
    running_mission_hash_.clear();
  }
  clear_waypoint_file();
  transition_state(MissionState::Completed);
  NAV_LOG_INFO( "All waypoints completed");
  navigate_terminate(NavigateTask::Result::RESULT_SUCCESS, "", "all waypoints reached", "succeed");
  state_to_idle_relaxed();
}

void MissionBridgeNode::on_nav_fatal(
  const std::string & error_code, int nav2_error_code,
  const std::string & nav2_error_msg, std::optional<int> nt_result)
{
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    running_mission_hash_.clear();
    navigating_ = false;
    active_goal_handle_.reset();
  }
  transition_state(MissionState::Failed);
  {
    std::lock_guard<std::mutex> lock(timer_mutex_);
    if (send_timer_) send_timer_->cancel();
    send_timer_.reset();
  }
  publish_status_detail(std::nullopt, error_code, nav2_error_code, nav2_error_msg);
  navigate_terminate(
    nt_result.value_or(map_nav2_failure(nav2_error_code, nav2_error_msg)), error_code,
    nav2_error_msg.empty() ? "FollowWaypoints mission failed" : nav2_error_msg, "abort");
  state_to_idle_relaxed();
}

void MissionBridgeNode::state_to_idle_relaxed()
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (idle_transition_timer_) idle_transition_timer_->cancel();
  idle_transition_timer_ = create_wall_timer(50ms, [this]() {
      {
        std::lock_guard<std::mutex> timer_lock(timer_mutex_);
        if (idle_transition_timer_) idle_transition_timer_->cancel();
        idle_transition_timer_.reset();
      }
      MissionState state;
      {
        std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
        state = state_;
      }
      if (state == MissionState::Completed || state == MissionState::Failed) {
        transition_state(MissionState::Idle, "complete");
      }
      have_odom_ = false;
      odom_last_mono_ = 0.0;
    });
}

void MissionBridgeNode::clear_local_costmap(const std::string & why)
{
  if (!clear_local_client_->service_is_ready()) {
    NAV_LOG_WARN( "local costmap clear service unavailable (%s)", why.c_str());
    return;
  }
  clear_local_client_->async_send_request(
    std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>(),
    [this, why](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture) {
      NAV_LOG_INFO( "local costmap cleared (%s)", why.c_str());
      if (reinflate_after_clear_) trigger_reinflate(why);
    });
}

void MissionBridgeNode::clear_global_costmap(const std::string & why)
{
  if (!clear_global_client_->service_is_ready()) {
    NAV_LOG_WARN( "global costmap clear service unavailable (%s)", why.c_str());
    return;
  }
  clear_global_client_->async_send_request(
    std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>(),
    [this, why](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture) {
      NAV_LOG_INFO( "global costmap cleared (%s)", why.c_str());
    });
}

void MissionBridgeNode::setup_costmap_clear_timer()
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (costmap_clear_timer_) costmap_clear_timer_->cancel();
  costmap_clear_timer_.reset();
  if (costmap_clear_period_sec_ <= 0.0) return;
  costmap_clear_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(costmap_clear_period_sec_)),
    [this]() {
      std::lock_guard<std::recursive_mutex> state_lock(state_mutex_);
      if (state_ == MissionState::Running) clear_local_costmap("periodic");
    });
}

}  // namespace workspace_nav::mission_bridge

namespace workspace_nav::mission_bridge
{
using namespace std::chrono_literals;

void MissionBridgeNode::send_waypoints_callback(
  const std::shared_ptr<m_common::srv::SendWaypoints::Request> request,
  std::shared_ptr<m_common::srv::SendWaypoints::Response> response)
{
  if (request->waypoints.empty()) {
    response->success = false;
    response->message = "Waypoints list is empty";
    return;
  }
  MissionState state;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state = state_;
  }
  if (state == MissionState::WaitingSystem) {
    response->success = false;
    response->message = "System not ready (WAITING_SYSTEM)";
    return;
  }
  std::vector<LatLon> waypoints;
  std::vector<double> yaws;
  for (std::size_t i = 0; i < request->waypoints.size(); ++i) {
    const auto & input = request->waypoints[i];
    const LatLon waypoint{input.latitude, input.longitude};
    const std::string error = validate_waypoint(waypoint, i);
    if (!error.empty()) {
      response->success = false;
      response->message = error;
      NAV_LOG_WARN( "%s", error.c_str());
      return;
    }
    waypoints.push_back(waypoint);
    yaws.push_back(input.yaw);
  }
  if (const auto violation = nav_zones_violation(waypoints)) {
    response->success = false;
    response->message = *violation;
    NAV_LOG_WARN( "%s", violation->c_str());
    return;
  }
  if (state == MissionState::Running || state == MissionState::Paused) {
    preempt_running_mission(true);
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    paused_nav_xy_.clear();
    paused_index_ = 0;
  }
  try {
    const auto hash = write_and_start_mission(
      waypoints, yaws, trim(request->mission_id), trim(request->command_id));
    response->success = true;
    response->message = "Mission started: " + std::to_string(waypoints.size()) +
      " waypoints, hash=" + hash.substr(0, 12);
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("Failed to convert/write waypoints: ") + error.what();
    NAV_LOG_ERROR( "%s", response->message.c_str());
  }
}

void MissionBridgeNode::plan_mission_callback(
  const std::shared_ptr<m_common::srv::PlanMission::Request> request,
  std::shared_ptr<m_common::srv::PlanMission::Response> response)
{
  auto fail = [this, response](const std::string & message) {
      response->success = false;
      response->message = message;
      NAV_LOG_WARN( "[service] plan_mission: %s", message.c_str());
    };
  if (request->waypoints.empty()) {
    fail("Waypoints list is empty");
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ != MissionState::Idle && state_ != MissionState::Completed &&
      state_ != MissionState::Failed)
    {
      fail("Cannot plan mission in state " + to_string(state_));
      return;
    }
  }
  std::vector<LatLon> waypoints;
  std::vector<double> yaws;
  for (std::size_t i = 0; i < request->waypoints.size(); ++i) {
    const LatLon waypoint{request->waypoints[i].latitude, request->waypoints[i].longitude};
    const std::string error = validate_waypoint(waypoint, i);
    if (!error.empty()) {
      fail(error);
      return;
    }
    waypoints.push_back(waypoint);
    yaws.push_back(request->waypoints[i].yaw);
  }

  ComputePath::Goal goal;
  goal.use_start = false;
  try {
    const auto document = make_waypoints_document(waypoints);
    for (std::size_t i = 0; i < document["waypoints"].size(); ++i) {
      goal.goals.push_back(create_pose(
        document["waypoints"][i]["x"].get<double>(),
        document["waypoints"][i]["y"].get<double>(), yaw_is_specified(yaws[i]) ? yaws[i] : 0.0));
    }
  } catch (const std::exception & error) {
    fail(std::string("Failed to convert waypoints: ") + error.what());
    return;
  }
  const double timeout_sec = std::max(0.1, get_parameter("mission_plan_timeout_sec").as_double());
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));
  if (!path_client_->wait_for_action_server(timeout)) {
    fail("ComputePathThroughPoses server not available");
    return;
  }

  using WrappedResult = PathGoalHandle::WrappedResult;
  auto completion = std::make_shared<std::promise<WrappedResult>>();
  auto result_future = completion->get_future();
  auto goal_mutex = std::make_shared<std::mutex>();
  auto accepted_goal = std::make_shared<PathGoalHandle::SharedPtr>();
  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  options.goal_response_callback =
    [this, completion, goal_mutex, accepted_goal](PathGoalHandle::SharedPtr handle) mutable {
      NAV_LOG_INFO( "ComputePathThroughPoses goal response received");
      if (!handle) {
        completion->set_exception(std::make_exception_ptr(
          std::runtime_error("ComputePathThroughPoses goal rejected")));
        return;
      }
      std::lock_guard<std::mutex> lock(*goal_mutex);
      *accepted_goal = handle;
    };
  options.result_callback = [this, completion](const WrappedResult & result) mutable {
      NAV_LOG_INFO( "ComputePathThroughPoses result received");
      completion->set_value(result);
    };
  try {
    path_client_->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    fail(std::string("Failed to send ComputePathThroughPoses goal: ") + error.what());
    return;
  }
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    PathGoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(*goal_mutex);
      handle = *accepted_goal;
    }
    if (handle) path_client_->async_cancel_goal(handle);
    fail("ComputePathThroughPoses timed out after " + std::to_string(timeout_sec) + "s");
    return;
  }
  try {
    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      fail("ComputePathThroughPoses failed with action status " +
        std::to_string(static_cast<int>(result.code)));
      return;
    }
    if (result.result->path.poses.empty()) {
      fail("ComputePathThroughPoses returned an empty path");
      return;
    }
    response->success = true;
    response->path = result.result->path;
    response->message = "Planned " + std::to_string(goal.goals.size()) + " waypoints, " +
      std::to_string(response->path.poses.size()) + " path poses";
    NAV_LOG_INFO( "[service] plan_mission: %s", response->message.c_str());
  } catch (const std::exception & error) {
    fail(error.what());
  }
}

void MissionBridgeNode::set_pause_callback(
  const std::shared_ptr<m_common::srv::SetPause::Request> request,
  std::shared_ptr<m_common::srv::SetPause::Response> response)
{
  MissionState state;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state = state_;
  }
  if (state == MissionState::WaitingSystem) {
    response->success = false;
    response->message = "System not ready (WAITING_SYSTEM)";
    return;
  }
  if (request->pause) {
    if (state == MissionState::Running) {
      pause_mission();
      response->success = true;
      response->message = "Mission paused; progress saved";
    } else if (state == MissionState::Paused) {
      response->success = true;
      response->message = "Already paused (idempotent)";
    } else {
      response->success = false;
      response->message = state == MissionState::Idle ? "No active mission to pause" :
        "Cannot pause in state " + to_string(state);
    }
  } else {
    if (state == MissionState::Paused) {
      resume_mission();
      response->success = true;
      response->message = "Mission resumed from saved progress";
    } else if (state == MissionState::Running) {
      response->success = true;
      response->message = "Already running (idempotent)";
    } else {
      response->success = false;
      response->message = "Cannot resume in state " + to_string(state);
    }
  }
}

void MissionBridgeNode::cancel_mission_callback(
  const std::shared_ptr<m_common::srv::CancelMission::Request>,
  std::shared_ptr<m_common::srv::CancelMission::Response> response)
{
  const auto result = apply_mission_cancel();
  response->success = result.first;
  response->message = result.second;
}

void MissionBridgeNode::pause_mission()
{
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    paused_nav_xy_ = nav_xy_;
    paused_index_ = current_index_;
  }
  ++mission_token_;
  cancel_active_nav2_goal();
  transition_state(MissionState::Paused);
}

void MissionBridgeNode::resume_mission()
{
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    nav_xy_ = paused_nav_xy_;
    current_index_ = paused_index_;
    paused_nav_xy_.clear();
    paused_index_ = 0;
  }
  transition_state(MissionState::Running);
  start_nav_stack();
}

void MissionBridgeNode::trigger_reinflate(const std::string & why)
{
  {
    std::lock_guard<std::mutex> lock(reinflate_mutex_);
    if (reinflate_busy_ && monotonic_seconds() > reinflate_deadline_) reinflate_busy_ = false;
    if (reinflate_busy_) return;
    if (!reinflate_get_client_->service_is_ready() || !reinflate_set_client_->service_is_ready()) {
      NAV_LOG_WARN( "costmap parameter services unavailable; skip reinflation");
      return;
    }
    reinflate_busy_ = true;
    reinflate_deadline_ = monotonic_seconds() + 10.0;
    reinflate_why_ = why;
  }
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"inflation_layer.inflation_radius"};
  reinflate_get_client_->async_send_request(
    request,
    [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (response->values.empty() ||
          response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        {
          throw std::runtime_error("inflation_radius is not a double");
        }
        {
          std::lock_guard<std::mutex> lock(reinflate_mutex_);
          reinflate_original_ = response->values[0].double_value;
        }
        reinflate_set_value(response->values[0].double_value + 0.01, 0);
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(reinflate_mutex_);
        reinflate_busy_ = false;
        NAV_LOG_ERROR( "failed reading inflation_radius: %s", error.what());
      }
    });
}

void MissionBridgeNode::reinflate_set_value(double value, int stage)
{
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = "inflation_layer.inflation_radius";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  parameter.value.double_value = value;
  request->parameters = {parameter};
  reinflate_set_client_->async_send_request(
    request,
    [this, stage](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
      bool success = false;
      std::string reason;
      try {
        const auto response = future.get();
        success = !response->results.empty() && response->results[0].successful;
        if (!response->results.empty()) reason = response->results[0].reason;
      } catch (const std::exception & error) {
        reason = error.what();
      }
      if (!success) {
        std::lock_guard<std::mutex> lock(reinflate_mutex_);
        reinflate_busy_ = false;
        NAV_LOG_ERROR( "failed updating inflation_radius: %s", reason.c_str());
        return;
      }
      if (stage == 0) {
        double original;
        {
          std::lock_guard<std::mutex> lock(reinflate_mutex_);
          original = reinflate_original_;
        }
        reinflate_set_value(original, 1);
      } else {
        std::lock_guard<std::mutex> lock(reinflate_mutex_);
        reinflate_busy_ = false;
        NAV_LOG_INFO( "local costmap reinflation completed");
      }
    });
}

rclcpp_action::GoalResponse MissionBridgeNode::navigate_goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateTask::Goal> goal)
{
  if (goal->waypoints.empty()) {
    NAV_LOG_WARN( "[navigate] reject: waypoints empty");
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::vector<LatLon> waypoints;
  for (std::size_t i = 0; i < goal->waypoints.size(); ++i) {
    const LatLon waypoint{goal->waypoints[i].latitude, goal->waypoints[i].longitude};
    if (!validate_waypoint(waypoint, i).empty()) {
      NAV_LOG_WARN( "[navigate] reject: invalid waypoint #%zu", i);
      return rclcpp_action::GoalResponse::REJECT;
    }
    waypoints.push_back(waypoint);
  }
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    if (state_ == MissionState::WaitingSystem || state_ == MissionState::Running ||
      state_ == MissionState::Paused)
    {
      NAV_LOG_WARN( "[navigate] reject: state=%s", to_string(state_).c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    if (navigate_goal_handle_) return rclcpp_action::GoalResponse::REJECT;
  }
  if (nav_zones_violation(waypoints)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionBridgeNode::navigate_cancel_callback(
  const std::shared_ptr<NavigateGoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionBridgeNode::navigate_accepted_callback(
  const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(navigate_mutex_);
  navigate_threads_.emplace_back(&MissionBridgeNode::navigate_execute, this, goal_handle);
}

void MissionBridgeNode::navigate_execute(const std::shared_ptr<NavigateGoalHandle> goal_handle)
{
  const auto & input = goal_handle->get_goal()->waypoints;
  std::vector<LatLon> waypoints;
  std::vector<double> yaws;
  std::vector<int> seqs;
  for (const auto & waypoint : input) {
    waypoints.push_back({waypoint.latitude, waypoint.longitude});
    yaws.push_back(waypoint.yaw);
    seqs.push_back(waypoint.seq);
  }
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    if (navigate_goal_handle_) {
      auto result = std::make_shared<NavigateTask::Result>();
      result->result = NavigateTask::Result::RESULT_BUSY;
      result->error_code = "BUSY";
      result->message = "another NavigateTask goal active";
      result->final_current_seq = -1;
      result->final_reached_seq = -1;
      goal_handle->abort(result);
      return;
    }
    navigate_goal_handle_ = goal_handle;
    navigate_seqs_ = seqs;
    navigate_result_info_.reset();
    navigate_phase_ = NavigateTask::Feedback::PHASE_VALIDATING;
    navigate_feedback_seen_ = false;
    navigate_done_ = false;
  }
  start_navigate_feedback_timer();

  if (goal_handle->is_canceling()) {
    navigate_terminate(
      NavigateTask::Result::RESULT_CANCELED, "CANCELED", "canceled before start", "canceled");
  } else {
    MissionState state;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      state = state_;
    }
    if (state == MissionState::Idle || state == MissionState::Completed ||
      state == MissionState::Failed)
    {
      try {
        write_and_start_mission(waypoints, yaws, "", "");
        std::lock_guard<std::mutex> lock(navigate_mutex_);
        navigate_phase_ = NavigateTask::Feedback::PHASE_PLANNING;
      } catch (const std::exception & error) {
        navigate_terminate(
          NavigateTask::Result::RESULT_INTERNAL_ERROR, "INTERNAL_ERROR",
          std::string("failed to start mission: ") + error.what(), "abort");
      }
    } else {
      const int code = state == MissionState::WaitingSystem ?
        NavigateTask::Result::RESULT_NOT_READY : NavigateTask::Result::RESULT_BUSY;
      navigate_terminate(
        code, "STATE_" + to_string(state), "cannot start mission in state " + to_string(state),
        "abort");
    }
  }

  while (true) {
    {
      std::unique_lock<std::mutex> lock(navigate_mutex_);
      if (navigate_condition_.wait_for(lock, 100ms, [this]() {return navigate_done_;})) break;
    }
    if (goal_handle->is_canceling()) {
      navigate_terminate(
        NavigateTask::Result::RESULT_CANCELED, "CANCELED", "canceled by decision", "canceled");
      apply_mission_cancel();
    }
  }

  std::optional<NavigateResultInfo> info;
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    info = navigate_result_info_;
    navigate_goal_handle_.reset();
    navigate_seqs_.clear();
  }
  stop_navigate_feedback_timer();
  auto result = std::make_shared<NavigateTask::Result>();
  if (!info) {
    result->result = NavigateTask::Result::RESULT_INTERNAL_ERROR;
    result->error_code = "INTERNAL_ERROR";
    result->message = "mission terminated without result info";
    result->final_current_seq = -1;
    result->final_reached_seq = -1;
    if (goal_handle->is_active()) goal_handle->abort(result);
    return;
  }
  result->result = info->result;
  result->error_code = info->error_code;
  result->message = info->message;
  result->final_current_seq = info->final_current_seq;
  result->final_reached_seq = info->final_reached_seq;
  if (info->how == "succeed") goal_handle->succeed(result);
  else if (info->how == "canceled") goal_handle->canceled(result);
  else goal_handle->abort(result);
  NAV_LOG_INFO( "[navigate] goal finished: result=%d", result->result);
}

void MissionBridgeNode::navigate_terminate(
  int result_code, const std::string & error_code, const std::string & message,
  const std::string & how)
{
  std::size_t index;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    index = current_index_;
  }
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    if (!navigate_goal_handle_ || navigate_result_info_) return;
    const int current = index < navigate_seqs_.size() ? navigate_seqs_[index] :
      (navigate_seqs_.empty() ? -1 : navigate_seqs_.back());
    const int reached = index >= 1 && index <= navigate_seqs_.size() ?
      navigate_seqs_[index - 1] : -1;
    navigate_result_info_ = NavigateResultInfo{
      result_code, error_code, message, how, current, reached};
    navigate_done_ = true;
  }
  navigate_condition_.notify_all();
}

void MissionBridgeNode::start_navigate_feedback_timer()
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (navigate_feedback_timer_) navigate_feedback_timer_->cancel();
  navigate_feedback_timer_ = create_wall_timer(
    500ms, std::bind(&MissionBridgeNode::publish_navigate_feedback, this), navigate_group_);
}

void MissionBridgeNode::stop_navigate_feedback_timer()
{
  std::lock_guard<std::mutex> lock(timer_mutex_);
  if (navigate_feedback_timer_) navigate_feedback_timer_->cancel();
  navigate_feedback_timer_.reset();
}

void MissionBridgeNode::publish_navigate_feedback()
{
  std::shared_ptr<NavigateGoalHandle> handle;
  std::vector<int> seqs;
  uint8_t phase;
  {
    std::lock_guard<std::mutex> lock(navigate_mutex_);
    handle = navigate_goal_handle_;
    seqs = navigate_seqs_;
    phase = navigate_phase_;
  }
  if (!handle) return;
  MissionState state;
  std::size_t index;
  std::vector<NavWaypoint> waypoints;
  {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state = state_;
    index = current_index_;
    waypoints = nav_xy_;
  }
  NavigateTask::Feedback feedback;
  if (state == MissionState::Paused) {
    feedback.phase = NavigateTask::Feedback::PHASE_PAUSED;
    feedback.message = "paused";
  } else if (phase == NavigateTask::Feedback::PHASE_VALIDATING) {
    feedback.phase = NavigateTask::Feedback::PHASE_VALIDATING;
    feedback.message = "validating";
  } else if (!navigate_feedback_seen_) {
    feedback.phase = NavigateTask::Feedback::PHASE_PLANNING;
    feedback.message = "planning";
  } else {
    feedback.phase = NavigateTask::Feedback::PHASE_TRACKING;
    feedback.message = "tracking";
  }
  feedback.current_seq = index < seqs.size() ? seqs[index] : (seqs.empty() ? -1 : seqs.back());
  feedback.reached_seq = index >= 1 && index <= seqs.size() ? seqs[index - 1] : -1;
  if (index < waypoints.size()) {
    const auto robot = robot_xy();
    feedback.distance_remaining_m = std::hypot(
      waypoints[index].x - robot.first, waypoints[index].y - robot.second);
  }
  handle->publish_feedback(std::make_shared<NavigateTask::Feedback>(feedback));
}

}  // namespace workspace_nav::mission_bridge

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("mission_bridge");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<workspace_nav::mission_bridge::MissionBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
