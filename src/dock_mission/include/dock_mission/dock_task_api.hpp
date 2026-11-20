#ifndef DOCK_MISSION__DOCK_TASK_API_HPP_
#define DOCK_MISSION__DOCK_TASK_API_HPP_

#include "dock_mission/types.hpp"

#include <nlohmann/json.hpp>

#include <optional>
#include <string>

namespace dock_mission
{

inline constexpr double DOCK_YAW_UNSPECIFIED = 65536.0;
inline constexpr double DOCK_POINT_UNSPECIFIED = DOCK_YAW_UNSPECIFIED;

enum class DockCommand
{
  ONE_CLICK_DOCK = 1,
  DOCK_ONLY = 2,
  UNDOCK = 3,
  CANCEL = 4
};

struct GcsCommand
{
  DockCommand command{DockCommand::ONE_CLICK_DOCK};
  std::string mission_id;
  std::string command_id;
  std::optional<double> dock_lat;
  std::optional<double> dock_lon;
  std::optional<double> dock_yaw;
  bool has_dock_point() const {return dock_lat.has_value() && dock_lon.has_value();}
};

struct TaskStatus
{
  MissionState state{MissionState::IDLE};
  std::optional<std::string> mode;
  bool dock_active{false};
  int retry_count{0};
  int retry_max{0};
  bool needs_manual_takeover{false};
  bool needs_reapproach{false};
  std::string mission_id;
  std::string command_id;
  double elapsed_sec{0.0};
  int error_code{0};
  std::optional<std::string> error_message;
  bool camera_ready{true};
  bool event_ack_ok{true};
  int event_ack_failures{0};
  std::optional<double> odom_speed;
};

std::optional<GcsCommand> parse_gcs_command(const std::string & raw);
bool is_valid_dock_point(double latitude, double longitude);
std::string run_state_from_mission(MissionState state, bool dock_active);
std::string phase_from_mission(MissionState state, const std::optional<std::string> & mode);
nlohmann::json build_task_status(const TaskStatus & status);
nlohmann::json build_task_event(
  const std::string & event, const std::string & mission_id = "",
  const std::string & command_id = "", const nlohmann::json & detail = nlohmann::json());
std::string dumps_json(const nlohmann::json & payload);

}  // namespace dock_mission

#endif  // DOCK_MISSION__DOCK_TASK_API_HPP_
