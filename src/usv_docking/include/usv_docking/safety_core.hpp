#ifndef USV_DOCKING__SAFETY_CORE_HPP_
#define USV_DOCKING__SAFETY_CORE_HPP_

#include "usv_docking/docking_common.hpp"

#include <limits>
#include <optional>
#include <string>

namespace usv_docking
{

struct SafetyParams
{
  double max_lateral_error{0.50};
  double max_yaw_error_deg{15.0};
  double odom_prediction_timeout{10.0};
  double align_odom_prediction_timeout{7.0};
  double abort_exit_odom_timeout{60.0};
  double exit_pose_invalid_warn_sec{5.0};
  double max_docking_duration_sec{150.0};
  int violation_cycles{5};
};

struct SafetyInput
{
  std::optional<Pose2D> pose;
  std::string pose_source{SRC_INVALID};
  double measurement_age{std::numeric_limits<double>::infinity()};
  std::string state{DockState::IDLE};
  bool pose_silent{false};
  bool fsm_silent{false};
};

struct SafetyOutput
{
  bool stop{false};
  std::string abort_reason;
};

class SafetyCore
{
public:
  SafetyOutput check(const SafetyInput & input, double now, const SafetyParams & params);

private:
  std::string last_state_{DockState::IDLE};
  std::optional<double> mission_start_time_;
  int lateral_count_{0};
  int yaw_count_{0};
  int odom_count_{0};
  int global_count_{0};
  std::optional<double> exit_invalid_since_;
};

}  // namespace usv_docking

#endif  // USV_DOCKING__SAFETY_CORE_HPP_
