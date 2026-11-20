#ifndef USV_DOCKING__MOTION_CONTROL_CORE_HPP_
#define USV_DOCKING__MOTION_CONTROL_CORE_HPP_

#include "usv_docking/docking_common.hpp"

#include <optional>
#include <string>
#include <utility>

namespace usv_docking
{

struct MotionParams
{
  double control_rate{20.0};
  double hold_zero_publish_sec{1.0};
  bool blind_exit_enabled{false};
  double staging_x{-2.0};
  double final_target_x{0.0};
  double max_reverse_speed{0.8};
  double max_yaw_rate{0.12};
  double max_linear_accel{0.30};
  double max_angular_accel{0.50};
  double kd_yaw{0.7};
  double approach_reexit_margin{0.35};
  double approach_axis_y_far{0.8};
  double approach_axis_far_x{-4.5};
  double approach_axis_y_near{0.55};
  double approach_axis_pullout{2.0};
  double approach_axis_crab_deg{25.0};
  double approach_axis_max_out_x{-8.0};
  bool approach_cone_enabled{true};
  double cone_apex_x{-1.0};
  double cone_apex_half_width{0.25};
  double cone_half_angle_deg{12.0};
  double approach_offaxis_y{0.8};
  double approach_offaxis_speed{0.15};
  double approach_offaxis_x{-4.0};
  double approach_degenerate_dist{0.6};
  double approach_degenerate_speed{0.18};
  double approach_y_near{0.15};
  double search_angular_speed{0.15};
  double backin_max_search_angle_deg{8.0};
  double approach_speed{0.60};
  double approach_turn_deg{25.0};
  double approach_crab_deg{8.0};
  double approach_slow_y{1.0};
  double approach_slow_dist{1.50};
  double approach_min_speed{0.25};
  double back_in_speed_normal{0.25};
  double back_in_speed_slow{0.15};
  double back_in_gate1_y{0.20};
  double back_in_gate1_yaw_deg{5.0};
  double back_in_gate2_y{0.35};
  double back_in_gate2_yaw_deg{15.0};
  double final_dock_speed{0.15};
  double final_push_speed{0.4};
  double docked_x_tol{0.25};
  double docked_y_tol{0.10};
  double abort_exit_speed{0.30};
  double undock_speed{0.45};
  double exit_turn_deg{30.0};
  double ky_approach{0.40};
  double kyaw_approach{0.45};
  double kyaw_align{0.45};
  double align_arc_speed{0.15};
  double align_arc_x_limit{-1.5};
  double ky_back{0.80};
  double kyaw_back{0.80};
  double kx_final{0.25};
  double back_in_max_yaw_rate{0.12};
};

struct MotionInput
{
  std::string mode{MODE_HOLD};
  std::optional<Pose2D> pose;
  std::string pose_source{SRC_INVALID};
  bool pose_fresh{false};
  bool mode_fresh{false};
  bool safety_fresh{false};
  bool safety_stop{false};
  std::optional<double> yaw_rate;
};

struct MotionOutput
{
  double linear{0.0};
  double angular{0.0};
  bool publish{true};
};

class MotionControlCore
{
public:
  std::pair<double, double> compute_approach(
    double x, double e_y, double yaw_b, const MotionParams & params);
  std::pair<double, double> compute_align(
    double x, double e_yaw, const MotionParams & params);
  std::pair<double, double> compute_back_in(
    double x, double e_y, double e_yaw, double speed_cap, double yaw_cap, bool push,
    const MotionParams & params);
  std::pair<double, double> compute_exit_forward(
    double e_y, double e_yaw, double speed, const MotionParams & params);
  MotionOutput step(const MotionInput & input, const MotionParams & params);
  double axis_target_x() const {return axis_target_x_;}
  double commanded_linear() const {return v_cmd_;}
  double commanded_angular() const {return w_cmd_;}

private:
  bool pose_valid(const MotionInput & input) const;
  double yaw_damp(double value, const MotionParams & params) const;
  std::pair<double, double> compute_search(
    const MotionInput & input, bool limited, const MotionParams & params);
  bool axis_mode_{false};
  double axis_target_x_{0.0};
  double v_cmd_{0.0};
  double w_cmd_{0.0};
  std::string last_mode_{MODE_HOLD};
  std::optional<double> limited_entry_yaw_;
  int hold_settle_ticks_{0};
  std::optional<double> yaw_rate_;
};

bool is_motion_mode(const std::string & mode);
bool is_hold_mode(const std::string & mode);
bool is_exit_mode(const std::string & mode);

}  // namespace usv_docking

#endif  // USV_DOCKING__MOTION_CONTROL_CORE_HPP_
