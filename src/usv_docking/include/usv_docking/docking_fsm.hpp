#ifndef USV_DOCKING__DOCKING_FSM_HPP_
#define USV_DOCKING__DOCKING_FSM_HPP_

#include "usv_docking/docking_common.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <usv_rs485_driver/action/control_do.hpp>
#include <usv_rs485_driver/action/wait_io.hpp>
#include <usv_rs485_driver/srv/set_do.hpp>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

namespace usv_docking
{

class DockingFsm : public rclcpp::Node
{
public:
  explicit DockingFsm(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using OptDouble = std::optional<double>;
  using ControlDO = usv_rs485_driver::action::ControlDO;
  using WaitIO = usv_rs485_driver::action::WaitIO;
  using ControlDOClient = rclcpp_action::Client<ControlDO>;
  using WaitIOClient = rclcpp_action::Client<WaitIO>;
  using ControlDOGoalHandle = rclcpp_action::ClientGoalHandle<ControlDO>;
  using WaitIOGoalHandle = rclcpp_action::ClientGoalHandle<WaitIO>;

  double d(const char * name) const {return get_parameter(name).as_double();}
  int i(const char * name) const {return get_parameter(name).as_int();}
  bool b(const char * name) const {return get_parameter(name).as_bool();}
  void declare_parameters();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message);
  void abort_request_callback(const std_msgs::msg::String::SharedPtr message);
  void start_callback(const std_msgs::msg::Bool::SharedPtr message);
  void cancel_callback(const std_msgs::msg::Empty::SharedPtr);
  void undock_callback(const std_msgs::msg::Bool::SharedPtr message);
  void enter(const std::string & state);
  void reset_state_timers();
  void manage_io_goals_on_enter(const std::string & state);
  void update_flags_on_enter(const std::string & state);
  void reset_flags();
  void fail(const std::string & reason);
  void abort_and_report(const std::string & reason);
  void abort_corridor(const std::string & reason);
  double state_elapsed();
  double mission_elapsed();
  std::tuple<OptDouble, OptDouble, OptDouble> errors() const;
  bool odom_speed_ok();
  bool claw_on() const {return b("dock_claw_enabled");}
  bool claw_open_on_start() const {return claw_on() && b("dock_claw_open_on_start");}
  bool claw_release_on_undock() const {return claw_on() && b("dock_claw_release_on_undock");}
  bool backlight_on() const {return b("backlight_enabled");}
  int backlight_mask() const {return i("backlight_mask") & 0xFFFF;}
  bool ensure_io_clients();
  bool ensure_set_do_client();
  void send_control_do();
  void control_do_goal_response(ControlDOGoalHandle::SharedPtr handle);
  void control_do_result(const ControlDOGoalHandle::WrappedResult & result);
  void cancel_control_do();
  void send_set_do(int mask, const std::string & reason);
  void drive_undock_do(const std::string & reason);
  void backlight_session_begin(const std::string & reason);
  void backlight_session_end(const std::string & reason);
  void backlight_on_enter(const std::string & state);
  void backlight_tick();
  void send_wait_io();
  void wait_io_feedback(
    WaitIOGoalHandle::SharedPtr, const std::shared_ptr<const WaitIO::Feedback> feedback);
  void wait_io_goal_response(WaitIOGoalHandle::SharedPtr handle);
  void wait_io_result(const WaitIOGoalHandle::WrappedResult & result);
  void cancel_wait_io();
  void manage_wait_io();
  bool claw_stuck_check(double dt, const OptDouble & x);
  void tick();
  bool global_timeout_hit();
  void transitions(double dt);
  bool staging_reached(double dt);
  bool align_gate_ok() const;
  bool corridor_violation() const;
  bool claw_wait_or_docked(double dt, const OptDouble & x, const std::string & stuck_reason);
  bool corridor_violation_abort(const OptDouble & x);
  void trans_acquire_tag(double dt);
  void trans_approach_entry(double dt);
  void trans_align_entry(double dt);
  void trans_back_in(double dt);
  void begin_final_push_if_ready(const OptDouble & x, const OptDouble & e_y);
  void trans_final_dock(double dt);
  void trans_wait_dock_io(
    const std::string & action_desc, const std::string & success_state,
    const std::string & fail_prefix, bool release, const char * timeout_param);
  void trans_reacquire_tag(double dt);
  void trans_abort_exit(double dt);
  void trans_undock_exit(double dt);
  void trans_undock_settle();
  void route_after_reacquire();
  std::string compute_mode() const;
  void publish(const std::string & mode);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_anchor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tag_visible_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr measurement_age_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr abort_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr undock_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string state_{DockState::IDLE};
  std::optional<rclcpp::Time> state_enter_time_;
  std::optional<rclcpp::Time> mission_start_time_;
  std::optional<rclcpp::Time> last_tick_time_;
  std::optional<Pose2D> dock_pose_;
  bool tag_visible_{false};
  std::string pose_source_{SRC_INVALID};
  double measurement_age_{std::numeric_limits<double>::infinity()};
  OptDouble odom_speed_;
  std::optional<rclcpp::Time> odom_speed_stamp_;
  int acquire_frames_{0};
  int acquire_miss_{0};
  double align_hold_{0.0};
  double align_tag_loss_{0.0};
  double align_y_stuck_{0.0};
  std::optional<rclcpp::Time> final_push_start_;
  double final_push_elapsed_{0.0};
  int violation_count_{0};
  int reacquire_frames_{0};
  int reacquire_miss_{0};
  double reacquire_drive_elapsed_{0.0};
  double staging_hold_{0.0};
  double approach_tag_loss_{0.0};
  double approach_stuck_{0.0};
  bool success_{false};
  bool undock_success_{false};
  bool needs_reapproach_{false};
  bool needs_manual_takeover_{false};
  std::optional<std::string> abort_reason_;
  ControlDOClient::SharedPtr control_do_client_;
  WaitIOClient::SharedPtr wait_io_client_;
  rclcpp::Client<usv_rs485_driver::srv::SetDO>::SharedPtr set_do_client_;
  std::optional<rclcpp::Time> backlight_last_send_time_;
  bool backlight_session_active_{false};
  ControlDOGoalHandle::SharedPtr control_do_goal_handle_;
  bool control_do_goal_active_{false};
  std::optional<std::pair<bool, std::string>> control_do_result_;
  WaitIOGoalHandle::SharedPtr wait_io_goal_handle_;
  bool wait_io_active_{false};
  bool wait_io_need_retry_{false};
  int wait_io_retries_{0};
  bool claw_grabbed_{false};
  int claw_partial_mask_{0};
  OptDouble stuck_best_x_;
  double stuck_elapsed_{0.0};
  double undock_vision_lost_elapsed_{0.0};
  double exit_pose_lost_elapsed_{0.0};
  std::string status_pub_key_;
  double status_pub_mono_{0.0};
};

}  // namespace usv_docking

#endif  // USV_DOCKING__DOCKING_FSM_HPP_
