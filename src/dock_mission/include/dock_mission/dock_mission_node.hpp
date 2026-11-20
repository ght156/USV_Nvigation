#ifndef DOCK_MISSION__DOCK_MISSION_NODE_HPP_
#define DOCK_MISSION__DOCK_MISSION_NODE_HPP_

#include "dock_mission/dock_task_api.hpp"
#include "dock_mission/mission_logic.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <m_common/srv/dock_task_event.hpp>
#include <m_common/srv/query_in_dock.hpp>
#include <m_common/srv/set_channel_enabled.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <usv_rs485_driver/action/control_do.hpp>
#include <usv_rs485_driver/srv/set_do.hpp>

#include <array>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

namespace dock_mission
{

class Nav2GoalCheckerSwitch;
class MissionBridgeClient;

class DockMissionNode : public rclcpp::Node
{
public:
  explicit DockMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockMissionNode();
  void shutdown();

private:
  using ControlDO = usv_rs485_driver::action::ControlDO;
  using ControlDOClient = rclcpp_action::Client<ControlDO>;
  double d(const char * name) const {return get_parameter(name).as_double();}
  int i(const char * name) const {return get_parameter(name).as_int();}
  bool b(const char * name) const {return get_parameter(name).as_bool();}
  std::string s(const char * name) const {return get_parameter(name).as_string();}
  void declare_parameters();
  void load_parameters();
  int retry_max() const {return i("staging_retry_max");}
  double elapsed_sec() const;
  void set_authority(SpeedAuthority authority);
  void emit_event(const std::string & event, const nlohmann::json & detail = nlohmann::json());
  void report_event(const nlohmann::json & payload);
  void record_event_ack(bool ok, const std::string & note);
  void publish_status_if_due();
  void publish_task_status();
  void publish_legacy_status();
  void transition(MissionState state, const std::string & note = "");
  void set_apriltag_channels(bool enabled, const std::string & reason);
  std::optional<double> measurement_age_usable();
  bool estimator_alive();
  InDockSnapshot in_dock_snapshot();
  bool set_dock_backlight(bool on, const std::string & reason);
  std::pair<bool, bool> await_vision_pose();
  void in_dock_service(
    const std::shared_ptr<m_common::srv::QueryInDock::Request> request,
    std::shared_ptr<m_common::srv::QueryInDock::Response> response);
  void release_docking_session(
    const std::string & reason, SpeedAuthority authority = SpeedAuthority::RELEASE,
    bool end_session = true);
  void cancel_staging_nav(const std::string & reason);
  std::pair<bool, std::string> begin_session(
    DockCommand command, const std::string & mission_id = "",
    const std::string & command_id = "",
    const std::optional<std::array<double, 3>> & dock_point = std::nullopt);
  bool accepting_new_task() const;
  void arm_session(
    const std::string & mission_id, const std::string & command_id,
    const std::string & mode,
    const std::optional<std::array<double, 3>> & dock_point = std::nullopt);
  std::pair<bool, std::string> cancel_active_session();
  std::pair<bool, std::string> begin_dock_mission();
  void home_callback(const std_msgs::msg::Bool::SharedPtr message);
  void gcs_command_callback(const std_msgs::msg::String::SharedPtr message);
  std::optional<std::array<double, 3>> resolve_dock_point(const GcsCommand & command) const;
  std::array<double, 3> staging_waypoint() const;
  void start_nav_to_staging();
  void on_staging_sent(bool ok, const std::string & message, int attempt);
  void task_event_callback(const std_msgs::msg::String::SharedPtr message);
  void dock_status_callback(const std_msgs::msg::String::SharedPtr message);
  void finish_failed(const std::string & reason);
  void on_staging_failed(const std::string & reason);
  void finish_succeeded(bool undock = false);
  void tick();
  bool odom_speed_fresh();
  void tick_settle(double dt);
  void tick_handoff(bool undock);

  MissionState state_{MissionState::IDLE};
  int staging_retry_{0};
  double settle_elapsed_{0.0};
  double complete_settle_elapsed_{0.0};
  bool pending_nav_{false};
  bool staging_send_inflight_{false};
  double staging_send_mono_{0.0};
  int staging_attempt_{0};
  std::string mission_id_{"dock_staging"};
  std::string command_id_;
  std::optional<std::array<double, 3>> staging_override_;
  std::optional<std::string> mode_;
  bool dock_active_{false};
  bool needs_manual_takeover_{false};
  bool needs_reapproach_{false};
  std::string last_fail_reason_;
  std::optional<double> session_start_mono_;
  bool camera_ready_{true};
  std::optional<double> odom_speed_;
  std::optional<double> odom_yaw_rate_;
  std::optional<rclcpp::Time> odom_speed_stamp_;
  std::optional<std::pair<double, double>> dock_pose_xy_;
  std::optional<rclcpp::Time> dock_pose_stamp_;
  std::string dock_pose_source_;
  std::string dock_anchor_status_;
  std::optional<rclcpp::Time> dock_anchor_status_stamp_;
  std::optional<double> dock_measurement_age_;
  std::optional<rclcpp::Time> dock_measurement_age_stamp_;
  bool fsm_backlight_active_{false};
  SettleParams settle_params_;
  SettleMonitor settle_monitor_;
  bool event_ack_ok_{true};
  int event_ack_failures_{0};
  int event_pending_{0};
  double event_pending_mono_{0.0};
  std::set<long> event_pending_serials_;
  long next_event_serial_{0};
  std::string last_event_;
  double status_pub_mono_{0.0};
  std::string status_pub_key_;
  std::string bay_id_;
  std::string dock_db_path_;
  std::filesystem::path map_yaml_;
  bool use_gnss_{true};
  bool accept_gcs_staging_override_{false};
  std::string gcs_dock_command_topic_;
  std::string dock_task_status_topic_;
  std::string dock_task_event_service_;
  std::string dock_task_event_log_topic_;
  double event_ack_timeout_{3.0};
  InDockLimits in_dock_limits_;

  std::unique_ptr<Nav2GoalCheckerSwitch> goal_switch_;
  std::unique_ptr<MissionBridgeClient> mission_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr authority_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_event_log_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dock_undock_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr dock_cancel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_event_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gcs_command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_speed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_pose_source_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_anchor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dock_measurement_age_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<m_common::srv::QueryInDock>::SharedPtr in_dock_service_;
  rclcpp::CallbackGroup::SharedPtr in_dock_callback_group_;
  rclcpp::Client<m_common::srv::DockTaskEvent>::SharedPtr event_client_;
  rclcpp::Client<m_common::srv::SetChannelEnabled>::SharedPtr apriltag_channel_client_;
  rclcpp::Client<usv_rs485_driver::srv::SetDO>::SharedPtr set_do_client_;
  ControlDOClient::SharedPtr control_do_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dock_mission

#endif  // DOCK_MISSION__DOCK_MISSION_NODE_HPP_
