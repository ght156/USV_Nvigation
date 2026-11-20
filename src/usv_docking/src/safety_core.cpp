#include "usv_docking/safety_core.hpp"

#include <algorithm>
#include <array>

namespace usv_docking
{
namespace
{

template<std::size_t N>
bool contains(const std::array<const char *, N> & values, const std::string & value)
{
  return std::any_of(values.begin(), values.end(), [&value](const char * item) {
    return value == item;
  });
}

const std::array<const char *, 2> kErrorStates{DockState::BACK_IN, DockState::FINAL_DOCK};
const std::array<const char *, 3> kExitStates{
  DockState::ABORT_EXIT, DockState::UNDOCK_EXIT, DockState::UNDOCK_SETTLE};
const std::array<const char *, 2> kSearchStates{
  DockState::ACQUIRE_TAG, DockState::REACQUIRE_TAG};
const std::array<const char *, 5> kQuietStates{
  DockState::IDLE, DockState::DOCKED, DockState::FAILED,
  DockState::WAIT_DOCK_OPEN, DockState::WAIT_DOCK_RELEASE};

}  // namespace

SafetyOutput SafetyCore::check(
  const SafetyInput & input, double now, const SafetyParams & params)
{
  if (input.state != last_state_) {
    if (!contains(kQuietStates, input.state) && contains(kQuietStates, last_state_)) {
      mission_start_time_ = now;
    }
    exit_invalid_since_.reset();
    last_state_ = input.state;
  }

  SafetyOutput output;
  const bool quiet = contains(kQuietStates, input.state);
  if (!quiet) {
    if (input.fsm_silent) {
      output.stop = true;
    } else if (input.pose_silent) {
      output.stop = true;
      output.abort_reason = "TOPIC_TIMEOUT:POSE";
    }
  }

  if (!output.stop && output.abort_reason.empty() && !quiet && mission_start_time_) {
    if (now - *mission_start_time_ > params.max_docking_duration_sec) {
      ++global_count_;
      if (global_count_ >= params.violation_cycles) output.abort_reason = "GLOBAL_TIMEOUT";
    } else {
      global_count_ = 0;
    }
  }

  if (!output.stop && output.abort_reason.empty() && !quiet &&
    !contains(kSearchStates, input.state))
  {
    double limit = params.odom_prediction_timeout;
    if (contains(kExitStates, input.state)) {
      limit = params.abort_exit_odom_timeout;
    } else if (input.state == DockState::ALIGN_ENTRY) {
      limit = params.align_odom_prediction_timeout;
    }
    if (input.pose_source == SRC_INVALID ||
      (input.pose_source != SRC_VISION && input.measurement_age > limit))
    {
      if (input.pose_source == SRC_INVALID && contains(kExitStates, input.state)) {
        if (!exit_invalid_since_) {
          exit_invalid_since_ = now;
        } else if (now - *exit_invalid_since_ > params.exit_pose_invalid_warn_sec) {
          output.abort_reason = "EXIT_POSE_LOST";
        }
      } else {
        ++odom_count_;
        if (odom_count_ >= params.violation_cycles) output.abort_reason = "ODOM_TIMEOUT";
      }
    } else {
      odom_count_ = 0;
      exit_invalid_since_.reset();
    }
  }

  if (output.abort_reason.empty() && contains(kErrorStates, input.state) &&
    input.pose_source != SRC_INVALID && input.pose)
  {
    const double e_y = input.pose->y;
    const double e_yaw = std::abs(wrap_angle(input.pose->yaw - PI));
    if (std::abs(e_y) > params.max_lateral_error) {
      ++lateral_count_;
      if (lateral_count_ >= params.violation_cycles) output.abort_reason = "LATERAL_ERROR";
    } else {
      lateral_count_ = 0;
    }
    if (output.abort_reason.empty()) {
      if (e_yaw > params.max_yaw_error_deg * PI / 180.0) {
        ++yaw_count_;
        if (yaw_count_ >= params.violation_cycles) output.abort_reason = "YAW_ERROR";
      } else {
        yaw_count_ = 0;
      }
    } else {
      lateral_count_ = 0;
      yaw_count_ = 0;
    }
  }
  return output;
}

}  // namespace usv_docking
