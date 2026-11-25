#ifndef WORKSPACE_NAV__MISSION_BRIDGE_NODE_HPP_
#define WORKSPACE_NAV__MISSION_BRIDGE_NODE_HPP_

#include <action_msgs/msg/goal_status.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <m_common/action/navigate_task.hpp>
#include <m_common/msg/nav_status.hpp>
#include <m_common/srv/cancel_mission.hpp>
#include <m_common/srv/plan_mission.hpp>
#include <m_common/srv/send_waypoints.hpp>
#include <m_common/srv/set_pause.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "workspace_nav/mission_bridge_core.hpp"

namespace workspace_nav::mission_bridge
{

class MissionBridgeNode : public rclcpp::Node
{
public:
  MissionBridgeNode();
  ~MissionBridgeNode() override;

private:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using ComputePath = nav2_msgs::action::ComputePathThroughPoses;
  using PathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using NavigateTask = m_common::action::NavigateTask;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateTask>;

  struct NavWaypoint
  {
    double x;
    double y;
    double yaw;
  };
  struct DebouncedRoute
  {
    std::vector<LatLon> waypoints;
    std::string hash;
    bool explicit_replan;
    std::string mission_id;
    std::string command_id;
  };
  struct DelayedMission
  {
    std::vector<LatLon> waypoints;
    std::string hash;
  };
  struct NavigateResultInfo
  {
    int result;
    std::string error_code;
    std::string message;
    std::string how;
    int final_current_seq;
    int final_reached_seq;
  };

  void declare_parameters();
  void load_map();
  void resolve_paths();
  void setup_interfaces();
  double resolve_waypoint_tolerance() const;
  std::optional<double> read_nav2_tolerance(const std::filesystem::path & path) const;
  nlohmann::json make_waypoints_document(const std::vector<LatLon> & waypoints) const;
  void atomic_write_json(
    const std::filesystem::path & path, const nlohmann::json & document) const;
  bool verify_waypoints_file(const std::filesystem::path & path) const;
  void clear_waypoint_file();
  geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw_degrees);

  void publish_status_detail(
    const std::optional<std::string> & state_override = std::nullopt,
    const std::optional<std::string> & error_code = std::nullopt,
    int nav2_error_code = 0, const std::string & nav2_error_msg = "",
    const std::string & transition_reason = "");
  void transition_state(MissionState state, const std::string & reason = "");
  void tick_ready();
  bool tf_ok();
  bool odom_fresh() const;
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void global_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  std::pair<double, double> robot_xy() const;
  PlannerFailure classify_missed_waypoint() const;
  void nav_status_callback(const m_common::msg::NavStatus::SharedPtr msg);
  void tick_ctrl_stuck_watchdog();
  void on_system_not_ready(const std::string & reason = "SYSTEM_NOT_READY");

  void waypoint_callback(const std_msgs::msg::String::SharedPtr msg);
  void color_callback(const std_msgs::msg::String::SharedPtr msg);
  void nav_zones_callback(const std_msgs::msg::String::SharedPtr msg);
  void mission_cancel_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void reschedule_waypoint_commit();
  void consume_waypoint_command(
    const std::vector<LatLon> & waypoints, const std::string & hash,
    bool explicit_replan, const std::string & mission_id, const std::string & command_id);
  void preempt_running_mission(bool for_replace);
  std::pair<bool, std::string> apply_mission_cancel();
  void execute_mission_atomic(const std::vector<LatLon> & waypoints, const std::string & hash);
  std::string write_and_start_mission(
    const std::vector<LatLon> & waypoints, const std::vector<double> & yaws,
    const std::string & mission_id, const std::string & command_id);
  std::optional<std::string> nav_zones_violation(const std::vector<LatLon> & waypoints) const;
  Point to_map(const LatLon & waypoint) const;

  void start_nav_stack();
  void send_next_waypoint();
  void reset_send_timer(double seconds);
  void goal_response(FollowGoalHandle::SharedPtr goal_handle, uint64_t token);
  void goal_result(const FollowGoalHandle::WrappedResult & result, uint64_t token);
  void cancel_active_nav2_goal();
  void finalize_if_done();
  void finish_all_waypoints_success();
  void on_nav_fatal(
    const std::string & error_code = "MISSION_FAILED", int nav2_error_code = 0,
    const std::string & nav2_error_msg = "", std::optional<int> nt_result = std::nullopt);
  void state_to_idle_relaxed();

  void clear_local_costmap(const std::string & why);
  void clear_global_costmap(const std::string & why);
  void setup_costmap_clear_timer();
  void trigger_reinflate(const std::string & why);
  void reinflate_set_value(double value, int stage);

  void send_waypoints_callback(
    const std::shared_ptr<m_common::srv::SendWaypoints::Request> request,
    std::shared_ptr<m_common::srv::SendWaypoints::Response> response);
  void plan_mission_callback(
    const std::shared_ptr<m_common::srv::PlanMission::Request> request,
    std::shared_ptr<m_common::srv::PlanMission::Response> response);
  void set_pause_callback(
    const std::shared_ptr<m_common::srv::SetPause::Request> request,
    std::shared_ptr<m_common::srv::SetPause::Response> response);
  void cancel_mission_callback(
    const std::shared_ptr<m_common::srv::CancelMission::Request> request,
    std::shared_ptr<m_common::srv::CancelMission::Response> response);
  void pause_mission();
  void resume_mission();

  rclcpp_action::GoalResponse navigate_goal_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateTask::Goal> goal);
  rclcpp_action::CancelResponse navigate_cancel_callback(
    const std::shared_ptr<NavigateGoalHandle> goal_handle);
  void navigate_accepted_callback(const std::shared_ptr<NavigateGoalHandle> goal_handle);
  void navigate_execute(const std::shared_ptr<NavigateGoalHandle> goal_handle);
  void navigate_terminate(
    int result_code, const std::string & error_code, const std::string & message,
    const std::string & how);
  void publish_navigate_feedback();
  void start_navigate_feedback_timer();
  void stop_navigate_feedback_timer();

  static double monotonic_seconds();
  static double wall_seconds();

  std::filesystem::path map_path_;
  std::filesystem::path waypoints_path_;
  std::filesystem::path target_path_;
  std::string map_yaml_resolved_;
  std::string datum_source_{"map_yaml"};
  std::string ref_key_{"ref_gnss_10"};
  std::string projection_{"enu"};
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double datum_easting_{0.0};
  double datum_northing_{0.0};
  double map_ox_{0.0};
  double map_oy_{0.0};
  double map_origin_yaw_{0.0};
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  std::string odom_topic_;
  double tolerance_{1.5};
  double localization_timeout_sec_{3.0};
  double localization_startup_grace_sec_{5.0};
  double ctrl_stuck_timeout_sec_{30.0};
  double waypoint_commit_delay_sec_{0.45};
  double target_buoy_min_period_sec_{0.0};
  double costmap_clear_period_sec_{0.0};
  bool debug_{false};
  bool allow_replace_running_{false};
  bool allow_repeat_identical_{false};
  bool target_force_rewrite_{false};
  bool suppress_passive_after_cancel_{true};
  bool clear_costmap_on_start_{true};
  bool reinflate_after_clear_{true};
  std::string mission_cancel_topic_;

  mutable std::recursive_mutex state_mutex_;
  MissionState state_{MissionState::WaitingSystem};
  std::string running_mission_hash_;
  std::string last_completed_mission_hash_;
  std::vector<NavWaypoint> nav_xy_;
  std::vector<NavWaypoint> paused_nav_xy_;
  std::size_t current_index_{0};
  std::size_t paused_index_{0};
  bool navigating_{false};
  std::string current_mission_id_;
  std::string current_command_id_;
  double mission_start_wall_{0.0};
  std::atomic<uint64_t> mission_token_{0};
  FollowGoalHandle::SharedPtr active_goal_handle_;
  std::optional<DebouncedRoute> debounced_route_;
  std::optional<DelayedMission> delayed_mission_;
  bool suppress_passive_waypoints_{false};
  double last_passive_waypoint_wall_{0.0};
  double last_duplicate_wall_{0.0};
  double last_completed_duplicate_wall_{0.0};
  std::vector<Fence> zone_fences_;

  mutable std::mutex pose_mutex_;
  std::pair<double, double> current_pose_xy_{0.0, 0.0};
  std::atomic<bool> have_odom_{false};
  std::atomic<double> odom_last_mono_{0.0};
  double odom_sub_started_mono_{0.0};

  std::mutex timer_mutex_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::TimerBase::SharedPtr status_detail_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr idle_transition_timer_;
  rclcpp::TimerBase::SharedPtr waypoint_commit_timer_;
  rclcpp::TimerBase::SharedPtr delayed_mission_timer_;
  rclcpp::TimerBase::SharedPtr costmap_clear_timer_;
  rclcpp::TimerBase::SharedPtr navigate_feedback_timer_;

  CtrlStuckWatchdog ctrl_stuck_watchdog_;
  bool vessel_no_progress_pending_{false};
  double vessel_no_progress_last_mono_{0.0};
  nav_msgs::msg::OccupancyGrid::SharedPtr global_costmap_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr path_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr color_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zones_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_subscription_;
  rclcpp::Subscription<m_common::msg::NavStatus>::SharedPtr nav_status_subscription_;
  rclcpp::Service<m_common::srv::SendWaypoints>::SharedPtr send_waypoints_service_;
  rclcpp::Service<m_common::srv::PlanMission>::SharedPtr plan_mission_service_;
  rclcpp::Service<m_common::srv::SetPause>::SharedPtr set_pause_service_;
  rclcpp::Service<m_common::srv::CancelMission>::SharedPtr cancel_mission_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_detail_publisher_;
  rclcpp_action::Server<NavigateTask>::SharedPtr navigate_server_;
  rclcpp::CallbackGroup::SharedPtr waypoint_group_;
  rclcpp::CallbackGroup::SharedPtr path_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_group_;
  rclcpp::CallbackGroup::SharedPtr navigate_group_;

  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr reinflate_get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr reinflate_set_client_;
  std::mutex reinflate_mutex_;
  bool reinflate_busy_{false};
  double reinflate_deadline_{0.0};
  double reinflate_original_{0.0};
  std::string reinflate_why_;

  mutable std::mutex navigate_mutex_;
  std::condition_variable navigate_condition_;
  std::shared_ptr<NavigateGoalHandle> navigate_goal_handle_;
  std::optional<NavigateResultInfo> navigate_result_info_;
  std::vector<int> navigate_seqs_;
  uint8_t navigate_phase_{NavigateTask::Feedback::PHASE_VALIDATING};
  std::atomic<bool> navigate_feedback_seen_{false};
  bool navigate_done_{false};
  std::vector<std::thread> navigate_threads_;

  double last_target_write_wall_{0.0};
  std::string last_written_target_semantic_;
};

}  // namespace workspace_nav::mission_bridge

#endif  // WORKSPACE_NAV__MISSION_BRIDGE_NODE_HPP_
