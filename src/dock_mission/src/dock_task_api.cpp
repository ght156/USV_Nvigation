#include "dock_mission/dock_task_api.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace dock_mission
{
namespace
{

std::optional<double> optional_float(const nlohmann::json & value)
{
  if (value.is_null() || value.is_boolean()) return std::nullopt;
  try {
    double result = 0.0;
    if (value.is_number()) {
      result = value.get<double>();
    } else if (value.is_string()) {
      std::size_t used = 0;
      const auto text = value.get<std::string>();
      result = std::stod(text, &used);
      if (used != text.size()) return std::nullopt;
    } else {
      return std::nullopt;
    }
    return std::isfinite(result) ? std::optional<double>(result) : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> optional_coordinate(const nlohmann::json & value)
{
  auto result = optional_float(value);
  if (result && *result == DOCK_POINT_UNSPECIFIED) result.reset();
  return result;
}

std::string id_string(const nlohmann::json & value)
{
  if (value.is_null()) return "";
  if (value.is_string()) return value.get<std::string>();
  if (value.is_boolean()) return value.get<bool>() ? "True" : "False";
  return value.dump();
}

}  // namespace

std::optional<GcsCommand> parse_gcs_command(const std::string & raw)
{
  nlohmann::json data;
  try {
    data = nlohmann::json::parse(raw);
  } catch (const nlohmann::json::exception &) {
    return std::nullopt;
  }
  if (!data.is_object()) return std::nullopt;
  std::string action = data.value("action", "");
  action.erase(action.begin(), std::find_if(action.begin(), action.end(), [](unsigned char c) {
    return !std::isspace(c);
  }));
  action.erase(std::find_if(action.rbegin(), action.rend(), [](unsigned char c) {
    return !std::isspace(c);
  }).base(), action.end());
  std::transform(action.begin(), action.end(), action.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  GcsCommand result;
  if (action == "one_click_dock") {
    result.command = DockCommand::ONE_CLICK_DOCK;
  } else if (action == "dock_only") {
    result.command = DockCommand::DOCK_ONLY;
  } else if (action == "undock") {
    result.command = DockCommand::UNDOCK;
  } else if (action == "cancel") {
    result.command = DockCommand::CANCEL;
  } else {
    return std::nullopt;
  }
  result.mission_id = id_string(data.value("mission_id", nlohmann::json()));
  result.command_id = id_string(data.value("command_id", nlohmann::json()));
  result.dock_lat = optional_coordinate(data.value("dock_lat", nlohmann::json()));
  result.dock_lon = optional_coordinate(data.value("dock_lon", nlohmann::json()));
  result.dock_yaw = optional_float(data.value("dock_yaw", nlohmann::json()));
  return result;
}

bool is_valid_dock_point(double latitude, double longitude)
{
  if (latitude == 0.0 && longitude == 0.0) return false;
  return latitude >= -90.0 && latitude <= 90.0 && longitude >= -180.0 && longitude <= 180.0;
}

std::string run_state_from_mission(MissionState state, bool dock_active)
{
  if (state == MissionState::SUCCEEDED) return "SUCCEEDED";
  if (state == MissionState::FAILED) return "FAILED";
  if (state == MissionState::CANCELLED) return "CANCELLED";
  if (state == MissionState::IDLE && !dock_active) return "IDLE";
  return "RUNNING";
}

std::string phase_from_mission(MissionState state, const std::optional<std::string> & mode)
{
  if (state == MissionState::IDLE || state == MissionState::SUCCEEDED ||
    state == MissionState::FAILED || state == MissionState::CANCELLED)
  {
    return "IDLE";
  }
  if (mode && *mode == "undock") return "UNDOCKING";
  if (state == MissionState::ARMED || state == MissionState::NAV_TO_STAGING ||
    state == MissionState::SETTLE)
  {
    return "STAGING";
  }
  if (state == MissionState::DOCK_HANDOFF || state == MissionState::MONITOR_DOCK ||
    state == MissionState::COMPLETE_SETTLE)
  {
    return "DOCKING";
  }
  return "IDLE";
}

nlohmann::json build_task_status(const TaskStatus & status)
{
  nlohmann::json result = {
    {"schema_version", 1},
    {"run_state", run_state_from_mission(status.state, status.dock_active)},
    {"phase", phase_from_mission(status.state, status.mode)},
    {"mode", status.mode ? nlohmann::json(*status.mode) : nlohmann::json(nullptr)},
    {"dock_active", status.dock_active},
    {"retry_count", status.retry_count},
    {"retry_max", status.retry_max},
    {"needs_manual_takeover", status.needs_manual_takeover},
    {"needs_reapproach", status.needs_reapproach},
    {"camera_ready", status.camera_ready},
    {"error_code", status.error_code},
    {"error_message", status.error_message ? nlohmann::json(*status.error_message) : nlohmann::json(nullptr)},
    {"mission_id", status.mission_id.empty() ? nlohmann::json(nullptr) : nlohmann::json(status.mission_id)},
    {"command_id", status.command_id.empty() ? nlohmann::json(nullptr) : nlohmann::json(status.command_id)},
    {"elapsed_sec", std::round(status.elapsed_sec * 100.0) / 100.0},
    {"event_ack_ok", status.event_ack_ok},
    {"event_ack_failures", status.event_ack_failures},
    {"odom_speed", nullptr}
  };
  if (status.odom_speed && std::isfinite(*status.odom_speed)) {
    result["odom_speed"] = std::round(*status.odom_speed * 1000.0) / 1000.0;
  }
  return result;
}

nlohmann::json build_task_event(
  const std::string & event, const std::string & mission_id,
  const std::string & command_id, const nlohmann::json & detail)
{
  nlohmann::json result = {
    {"schema_version", 1},
    {"event", event},
    {"mission_id", mission_id.empty() ? nlohmann::json(nullptr) : nlohmann::json(mission_id)},
    {"command_id", command_id.empty() ? nlohmann::json(nullptr) : nlohmann::json(command_id)}
  };
  if (!detail.is_null() && !(detail.is_object() && detail.empty())) result["detail"] = detail;
  return result;
}

std::string dumps_json(const nlohmann::json & payload)
{
  return payload.dump();
}

}  // namespace dock_mission
