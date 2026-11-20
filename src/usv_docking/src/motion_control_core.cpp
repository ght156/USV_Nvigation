#include "usv_docking/motion_control_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

namespace usv_docking
{

bool is_motion_mode(const std::string & mode)
{
  static const std::array<const char *, 6> modes{
    MODE_APPROACH, MODE_ALIGN, MODE_BACK_IN, MODE_FINAL_DOCK,
    MODE_EXIT_FORWARD, MODE_UNDOCK_FORWARD};
  return std::any_of(modes.begin(), modes.end(), [&mode](const char * value) {
    return mode == value;
  });
}

bool is_hold_mode(const std::string & mode)
{
  return mode == MODE_HOLD || mode == MODE_DOCKED_HOLD;
}

bool is_exit_mode(const std::string & mode)
{
  return mode == MODE_EXIT_FORWARD || mode == MODE_UNDOCK_FORWARD;
}

bool MotionControlCore::pose_valid(const MotionInput & input) const
{
  return input.pose_source != SRC_INVALID && input.pose && input.pose_fresh;
}

double MotionControlCore::yaw_damp(double value, const MotionParams & params) const
{
  return yaw_rate_ ? value - params.kd_yaw * *yaw_rate_ : value;
}

std::pair<double, double> MotionControlCore::compute_approach(
  double x, double e_y, double yaw_b, const MotionParams & params)
{
  const double e_yaw = wrap_angle(yaw_b - PI);
  const double cone_width = params.approach_cone_enabled ?
    params.cone_apex_half_width +
    std::tan(params.cone_half_angle_deg * PI / 180.0) *
    std::max(0.0, params.cone_apex_x - x) : 0.0;
  if (axis_mode_ && (std::abs(e_y) < params.approach_axis_y_near ||
    std::abs(e_y) <= cone_width))
  {
    axis_mode_ = false;
  } else if (!axis_mode_ && std::abs(e_y) > params.approach_axis_y_far &&
    std::abs(e_y) > cone_width * 1.15)
  {
    axis_mode_ = true;
  }

  double staging = params.staging_x;
  double crab_deg = params.approach_crab_deg;
  if (axis_mode_) {
    staging = std::max(
      std::min(x, params.approach_axis_far_x) - params.approach_axis_pullout,
      params.approach_axis_max_out_x);
    crab_deg = params.approach_axis_crab_deg;
  }
  axis_target_x_ = staging;
  const double dx = staging - x;
  const double dy = -e_y;
  const double turn_threshold = params.approach_turn_deg * PI / 180.0;
  const double cap = params.approach_speed;
  const double distance = std::hypot(dx, dy);
  const double scale = cap > 0.0 ?
    std::max(
    params.approach_min_speed / cap,
    std::min({1.0, std::abs(e_y) / params.approach_slow_y,
      distance / params.approach_slow_dist})) : 1.0;
  double velocity_cap = cap * scale;
  if (std::abs(e_y) > params.approach_offaxis_y && x > params.approach_offaxis_x) {
    velocity_cap = std::min(velocity_cap, params.approach_offaxis_speed);
  }

  if (distance < params.approach_degenerate_dist) {
    const double w = yaw_damp(
      -params.kyaw_approach * (e_yaw + params.ky_approach * e_y), params);
    const double v = std::abs(e_y) > params.approach_y_near && dx > -0.1 ?
      -params.approach_degenerate_speed : 0.0;
    return {v, clamp_value(w, params.max_yaw_rate)};
  }

  if (dx < -params.approach_reexit_margin) {
    const double crab = crab_deg * PI / 180.0;
    const double desired_yaw = clamp_value(params.ky_approach * e_y, crab);
    const double w = yaw_damp(params.kyaw_approach * (desired_yaw - e_yaw), params);
    if (std::abs(e_yaw) > turn_threshold) {
      return {0.0, clamp_value(w, params.max_yaw_rate)};
    }
    return {std::min(velocity_cap, std::abs(dx)), clamp_value(w, params.max_yaw_rate)};
  }

  const double stern_bearing = wrap_angle(std::atan2(dy, dx) - (yaw_b + PI));
  const double w = clamp_value(
    yaw_damp(params.kyaw_approach * stern_bearing, params), params.max_yaw_rate);
  if (std::abs(stern_bearing) > turn_threshold) return {0.0, w};
  return {-std::min(velocity_cap, distance), w};
}

std::pair<double, double> MotionControlCore::compute_align(
  double x, double e_yaw, const MotionParams & params)
{
  const double w = clamp_value(
    yaw_damp(-params.kyaw_align * e_yaw, params), params.max_yaw_rate);
  const double v = params.align_arc_speed > 0.0 && x < params.align_arc_x_limit ?
    -params.align_arc_speed : 0.0;
  return {v, w};
}

std::pair<double, double> MotionControlCore::compute_back_in(
  double x, double e_y, double e_yaw, double speed_cap, double yaw_cap, bool push,
  const MotionParams & params)
{
  double v = 0.0;
  const double gate2_yaw = params.back_in_gate2_yaw_deg * PI / 180.0;
  if (std::abs(e_y) > params.back_in_gate2_y || std::abs(e_yaw) > gate2_yaw) {
    v = 0.0;
  } else {
    const double x_tol = params.docked_x_tol;
    const double target = params.final_target_x;
    const double distance = target - x;
    if (std::abs(distance) < x_tol && std::abs(e_y) < params.docked_y_tol) {
      const double w = yaw_damp(-params.kyaw_back * e_yaw, params);
      if (push) {
        const double entry_speed = std::min(speed_cap, params.kx_final * x_tol);
        const double ratio = x_tol > 0.0 ?
          1.0 - std::min(std::abs(distance) / x_tol, 1.0) : 1.0;
        const double push_speed = entry_speed + (params.final_push_speed - entry_speed) * ratio;
        return {-push_speed, clamp_value(w, yaw_cap)};
      }
      return {0.0, clamp_value(w, yaw_cap)};
    }
    if (std::abs(distance) < x_tol) {
      const double remaining = target + x_tol - x;
      v = -std::min(speed_cap, std::max(0.0, params.kx_final * remaining));
    } else {
      v = -std::min(speed_cap, std::max(0.0, params.kx_final * distance));
    }
  }
  const double w = yaw_damp(
    -params.kyaw_back * (e_yaw + params.ky_back * e_y), params);
  return {v, clamp_value(w, yaw_cap)};
}

std::pair<double, double> MotionControlCore::compute_exit_forward(
  double e_y, double e_yaw, double speed, const MotionParams & params)
{
  const double w = yaw_damp(
    params.kyaw_approach * (params.ky_approach * e_y - e_yaw), params);
  if (std::abs(e_yaw) > params.exit_turn_deg * PI / 180.0) {
    return {0.0, clamp_value(w, params.max_yaw_rate)};
  }
  return {speed, clamp_value(w, params.max_yaw_rate)};
}

std::pair<double, double> MotionControlCore::compute_search(
  const MotionInput & input, bool limited, const MotionParams & params)
{
  if (pose_valid(input)) {
    const double yaw_b = input.pose->yaw;
    const double stern_bearing = wrap_angle(
      std::atan2(-input.pose->y, -input.pose->x) - (yaw_b + PI));
    double w = clamp_value(params.kyaw_approach * stern_bearing, params.search_angular_speed);
    if (limited && limited_entry_yaw_) {
      const double deviation = wrap_angle(yaw_b - *limited_entry_yaw_);
      const double max_deviation = params.backin_max_search_angle_deg * PI / 180.0;
      if ((deviation > max_deviation && w > 0.0) ||
        (deviation < -max_deviation && w < 0.0))
      {
        w = 0.0;
      }
    }
    return {0.0, w};
  }
  return {0.0, limited ? 0.0 : params.search_angular_speed};
}

MotionOutput MotionControlCore::step(const MotionInput & input, const MotionParams & params)
{
  yaw_rate_ = input.yaw_rate;
  const double dt = 1.0 / params.control_rate;
  if (input.mode != last_mode_) {
    limited_entry_yaw_ = input.mode == MODE_SEARCH_LIMITED && input.pose ?
      std::optional<double>(input.pose->yaw) : std::nullopt;
    if (is_hold_mode(input.mode)) {
      v_cmd_ = 0.0;
      w_cmd_ = 0.0;
      hold_settle_ticks_ = std::max(
        1, static_cast<int>(std::ceil(params.hold_zero_publish_sec / dt)));
    }
    last_mode_ = input.mode;
  }

  const bool hold = is_hold_mode(input.mode);
  const double x = input.pose ? input.pose->x : 0.0;
  const double e_y = input.pose ? input.pose->y : 0.0;
  const double yaw_b = input.pose ? input.pose->yaw : 0.0;
  const double e_yaw = input.pose ? wrap_angle(yaw_b - PI) : 0.0;
  double v_target = 0.0;
  double w_target = 0.0;
  const bool open_loop_exit = params.blind_exit_enabled && is_exit_mode(input.mode);
  const bool exit_needs_vision = is_exit_mode(input.mode) && !params.blind_exit_enabled;

  if (hold) {
  } else if (open_loop_exit && !pose_valid(input)) {
    v_target = input.mode == MODE_EXIT_FORWARD ? params.abort_exit_speed : params.undock_speed;
  } else if (exit_needs_vision && input.pose_source != SRC_VISION) {
  } else if (is_motion_mode(input.mode) && !pose_valid(input)) {
  } else if (input.mode == MODE_APPROACH) {
    std::tie(v_target, w_target) = compute_approach(x, e_y, yaw_b, params);
  } else if (input.mode == MODE_ALIGN) {
    std::tie(v_target, w_target) = compute_align(x, e_yaw, params);
  } else if (input.mode == MODE_BACK_IN) {
    const bool normal = std::abs(e_y) < params.back_in_gate1_y &&
      std::abs(e_yaw) < params.back_in_gate1_yaw_deg * PI / 180.0;
    std::tie(v_target, w_target) = compute_back_in(
      x, e_y, e_yaw, normal ? params.back_in_speed_normal : params.back_in_speed_slow,
      params.back_in_max_yaw_rate, false, params);
  } else if (input.mode == MODE_FINAL_DOCK) {
    std::tie(v_target, w_target) = compute_back_in(
      x, e_y, e_yaw, params.final_dock_speed, params.back_in_max_yaw_rate, true, params);
  } else if (input.mode == MODE_EXIT_FORWARD) {
    std::tie(v_target, w_target) = compute_exit_forward(e_y, e_yaw, params.abort_exit_speed, params);
  } else if (input.mode == MODE_UNDOCK_FORWARD) {
    std::tie(v_target, w_target) = compute_exit_forward(e_y, e_yaw, params.undock_speed, params);
  } else if (input.mode == MODE_SEARCH) {
    std::tie(v_target, w_target) = compute_search(input, false, params);
  } else if (input.mode == MODE_SEARCH_LIMITED) {
    std::tie(v_target, w_target) = compute_search(input, true, params);
  }

  v_target = clamp_value(v_target, params.max_reverse_speed);
  w_target = clamp_value(w_target, params.max_yaw_rate);

  bool must_stop = false;
  if (!hold) {
    if (input.safety_stop) {
      must_stop = true;
    } else if (!input.safety_fresh || !input.mode_fresh) {
      must_stop = true;
    } else if (exit_needs_vision && input.pose_source != SRC_VISION) {
      must_stop = true;
    } else if (is_motion_mode(input.mode) && !open_loop_exit && !pose_valid(input)) {
      must_stop = true;
    }
  }

  if (input.safety_stop) {
    v_cmd_ = 0.0;
    w_cmd_ = 0.0;
    if (hold) hold_settle_ticks_ = std::max(hold_settle_ticks_, 3);
  } else if (must_stop) {
    v_cmd_ = 0.0;
    w_cmd_ = 0.0;
  } else {
    v_cmd_ += clamp_value(v_target - v_cmd_, params.max_linear_accel * dt);
    w_cmd_ += clamp_value(w_target - w_cmd_, params.max_angular_accel * dt);
  }

  MotionOutput output{v_cmd_, w_cmd_, true};
  if (hold) {
    const bool stopped = std::abs(v_cmd_) < 1e-3 && std::abs(w_cmd_) < 1e-3;
    if (stopped && hold_settle_ticks_ <= 0) {
      output.publish = false;
      return output;
    }
    if (hold_settle_ticks_ > 0) --hold_settle_ticks_;
  }
  return output;
}

}  // namespace usv_docking
