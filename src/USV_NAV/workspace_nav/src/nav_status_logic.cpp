#include "workspace_nav/nav_status_logic.hpp"

#include <cmath>

namespace workspace_nav::nav_status
{

StuckTracker::StuckTracker(
  double movement_radius, double no_progress_timeout, double command_timeout)
: movement_radius_(movement_radius), no_progress_timeout_(no_progress_timeout),
  command_timeout_(command_timeout) {}

StuckEvent StuckTracker::update(
  bool mission_running, const mission_bridge::Point & pose, double now,
  double last_command_stamp, bool linear_command_active)
{
  if (!mission_running) {
    reset();
    return StuckEvent::None;
  }
  if (last_command_stamp > 0.0 && now - last_command_stamp > command_timeout_) {
    stuck_ = true;
    return StuckEvent::StuckCommandStale;
  }
  if (!last_progress_pose_) {
    last_progress_pose_ = pose;
    last_progress_time_ = now;
    return StuckEvent::None;
  }
  const double distance = std::hypot(
    pose.x - last_progress_pose_->x, pose.y - last_progress_pose_->y);
  if (distance > movement_radius_ && linear_command_active) {
    last_progress_pose_ = pose;
    last_progress_time_ = now;
    stuck_ = false;
    return StuckEvent::Progress;
  }
  if (now - last_progress_time_ > no_progress_timeout_ && !stuck_) {
    stuck_ = true;
    return StuckEvent::StuckNoProgress;
  }
  return StuckEvent::None;
}

void StuckTracker::set_limits(double movement_radius, double no_progress_timeout)
{
  movement_radius_ = movement_radius;
  no_progress_timeout_ = no_progress_timeout;
}

void StuckTracker::reset()
{
  stuck_ = false;
  last_progress_pose_.reset();
  last_progress_time_ = 0.0;
}

bool StuckTracker::stuck() const {return stuck_;}
bool StuckTracker::has_progress_baseline() const {return last_progress_pose_.has_value();}
double StuckTracker::last_progress_time() const {return last_progress_time_;}

std::string to_string(LocHealth health)
{
  switch (health) {
    case LocHealth::Good: return "GOOD";
    case LocHealth::Degraded: return "DEGRADED";
    case LocHealth::Lost: return "LOST";
  }
  return "LOST";
}

LocHealth evaluate_localization(
  bool tf_ok, bool odom_stale, bool gps_stale, double position_cov_max,
  double lost_cov_threshold, double degraded_cov_threshold, int gps_status,
  bool has_gps, double odom_hz, double min_odom_hz)
{
  if (!tf_ok || odom_stale || position_cov_max > lost_cov_threshold) return LocHealth::Lost;
  if (position_cov_max > degraded_cov_threshold || (has_gps && gps_status < 0) ||
    (odom_hz > 0.0 && odom_hz < min_odom_hz) || gps_stale)
  {
    return LocHealth::Degraded;
  }
  return LocHealth::Good;
}

}  // namespace workspace_nav::nav_status
