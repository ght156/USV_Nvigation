#ifndef WORKSPACE_NAV__NAV_STATUS_LOGIC_HPP_
#define WORKSPACE_NAV__NAV_STATUS_LOGIC_HPP_

#include <optional>
#include <string>

#include "workspace_nav/mission_bridge_core.hpp"

namespace workspace_nav::nav_status
{

enum class LocHealth {Good, Degraded, Lost};
enum class StuckEvent {None, Progress, StuckNoProgress, StuckCommandStale};

class StuckTracker
{
public:
  StuckTracker(double movement_radius, double no_progress_timeout, double command_timeout);
  StuckEvent update(
    bool mission_running, const mission_bridge::Point & pose, double now,
    double last_command_stamp, bool linear_command_active);
  void reset();
  void set_limits(double movement_radius, double no_progress_timeout);
  bool stuck() const;
  bool has_progress_baseline() const;
  double last_progress_time() const;

private:
  double movement_radius_;
  double no_progress_timeout_;
  double command_timeout_;
  bool stuck_{false};
  std::optional<mission_bridge::Point> last_progress_pose_;
  double last_progress_time_{0.0};
};

std::string to_string(LocHealth health);
LocHealth evaluate_localization(
  bool tf_ok, bool odom_stale, bool gps_stale, double position_cov_max,
  double lost_cov_threshold, double degraded_cov_threshold, int gps_status,
  bool has_gps, double odom_hz, double min_odom_hz);

}  // namespace workspace_nav::nav_status

#endif  // WORKSPACE_NAV__NAV_STATUS_LOGIC_HPP_
