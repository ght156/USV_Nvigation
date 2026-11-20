#include "dock_mission/mission_logic.hpp"

#include <cmath>
#include <cstdio>

namespace dock_mission
{

InDockResult evaluate_in_dock(const InDockSnapshot & snapshot, const InDockLimits & limits)
{
  InDockResult result;
  result.pose_source = snapshot.source.empty() ? "NONE" : snapshot.source;
  bool valid = snapshot.xy.has_value();
  valid = valid && snapshot.anchor_status == "VALID";
  valid = valid && snapshot.anchor_age <= limits.pose_max_age_sec;
  valid = valid && (snapshot.source == "VISION" || snapshot.source == "ODOM_PREDICTION");
  valid = valid && snapshot.pose_age <= limits.pose_max_age_sec;
  if (valid && snapshot.source == "ODOM_PREDICTION" && snapshot.measurement_age &&
    snapshot.measurement_age_stamp_age <= limits.pose_max_age_sec &&
    std::isfinite(*snapshot.measurement_age))
  {
    valid = *snapshot.measurement_age <= limits.prediction_max_age_sec;
  }
  result.pose_valid = valid;
  if (valid) {
    result.dock_x = snapshot.xy->first;
    result.dock_y = snapshot.xy->second;
    result.in_dock = result.dock_x >= limits.x_min && result.dock_x <= limits.x_max &&
      std::abs(result.dock_y) <= limits.y_max;
    char buffer[256];
    std::snprintf(
      buffer, sizeof(buffer), "dock系(%.2f,%.2f) 区域x∈[%.2f,%.2f] |y|<=%.2f → %s",
      result.dock_x, result.dock_y, limits.x_min, limits.x_max, limits.y_max,
      result.in_dock ? "在坞内" : "不在坞内");
    result.message = buffer;
  } else if (snapshot.anchor_status != "VALID") {
    result.message = "锚点状态 " +
      (snapshot.anchor_status.empty() ? std::string("UNKNOWN") : snapshot.anchor_status) +
      "，重定位未完成或状态不可用，in_dock 不可信";
  } else if (result.pose_source == "VISION" || result.pose_source == "ODOM_PREDICTION") {
    result.message = "坞系位姿过期/不可用(pose_source=" + result.pose_source +
      ")，in_dock 不可信";
  } else {
    result.message = "无有效坞系位姿(pose_source=" + result.pose_source + ")，in_dock 不可信";
  }
  return result;
}

SettleMonitor::SettleMonitor(const SettleParams & params)
: params_(params)
{
}

void SettleMonitor::reset()
{
  still_start_stamp_.reset();
  still_last_stamp_.reset();
  still_frames_ = 0;
}

void SettleMonitor::observe(double speed, double yaw_rate, double stamp)
{
  const bool still = speed <= params_.max_speed && std::abs(yaw_rate) <= params_.max_yaw_rate;
  if (!still) {
    reset();
    return;
  }
  if (!still_last_stamp_ || stamp - *still_last_stamp_ > params_.max_frame_gap_sec) {
    still_start_stamp_ = stamp;
    still_frames_ = 1;
  } else {
    ++still_frames_;
  }
  still_last_stamp_ = stamp;
}

double SettleMonitor::still_hold() const
{
  return still_start_stamp_ && still_last_stamp_ ? *still_last_stamp_ - *still_start_stamp_ : 0.0;
}

bool SettleMonitor::ready(double now, double elapsed)
{
  if (still_last_stamp_ && now - *still_last_stamp_ > params_.max_frame_gap_sec) reset();
  return still_start_stamp_ && still_hold() >= params_.still_hold_sec &&
    still_frames_ >= params_.min_still_frames && elapsed >= params_.settle_sec;
}

}  // namespace dock_mission
