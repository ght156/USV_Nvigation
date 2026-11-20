#ifndef DOCK_MISSION__MISSION_LOGIC_HPP_
#define DOCK_MISSION__MISSION_LOGIC_HPP_

#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace dock_mission
{

struct InDockLimits
{
  double x_min{-1.5};
  double x_max{1.0};
  double y_max{0.6};
  double pose_max_age_sec{1.5};
  double prediction_max_age_sec{3.0};
};

struct InDockSnapshot
{
  std::optional<std::pair<double, double>> xy;
  double pose_age{std::numeric_limits<double>::infinity()};
  std::string source;
  std::string anchor_status;
  double anchor_age{std::numeric_limits<double>::infinity()};
  std::optional<double> measurement_age;
  double measurement_age_stamp_age{std::numeric_limits<double>::infinity()};
};

struct InDockResult
{
  bool in_dock{false};
  bool pose_valid{false};
  std::string pose_source{"NONE"};
  double dock_x{0.0};
  double dock_y{0.0};
  std::string message;
};

InDockResult evaluate_in_dock(const InDockSnapshot & snapshot, const InDockLimits & limits);

struct SettleParams
{
  double settle_sec{2.5};
  double max_speed{0.15};
  double max_yaw_rate{0.1};
  double still_hold_sec{0.5};
  double max_frame_gap_sec{1.0};
  int min_still_frames{3};
};

class SettleMonitor
{
public:
  explicit SettleMonitor(const SettleParams & params = SettleParams{});
  void reset();
  void set_params(const SettleParams & params) {params_ = params;}
  void observe(double speed, double yaw_rate, double stamp);
  bool ready(
    double now, double elapsed = std::numeric_limits<double>::infinity());
  double still_hold() const;
  int still_frames() const {return still_frames_;}

private:
  SettleParams params_;
  std::optional<double> still_start_stamp_;
  std::optional<double> still_last_stamp_;
  int still_frames_{0};
};

}  // namespace dock_mission

#endif  // DOCK_MISSION__MISSION_LOGIC_HPP_
