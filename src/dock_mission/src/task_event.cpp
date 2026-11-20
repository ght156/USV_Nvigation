#include "dock_mission/task_event.hpp"

namespace dock_mission
{

std::optional<std::pair<std::string, nlohmann::json>> parse_task_event(const std::string & raw)
{
  try {
    const auto data = nlohmann::json::parse(raw);
    if (!data.is_object()) return std::nullopt;
    const std::string event = data.value("event", "");
    const auto detail = data.contains("detail") && data["detail"].is_object() ?
      data["detail"] : nlohmann::json::object();
    return std::make_pair(event, detail);
  } catch (const nlohmann::json::exception &) {
    if (raw.find("TASK_COMPLETED") != std::string::npos) {
      return std::make_pair("TASK_COMPLETED", nlohmann::json::object());
    }
    if (raw.find("TASK_FAILED") != std::string::npos) {
      return std::make_pair("TASK_FAILED", nlohmann::json::object());
    }
    return std::nullopt;
  }
}

}  // namespace dock_mission
