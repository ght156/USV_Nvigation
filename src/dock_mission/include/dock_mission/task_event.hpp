#ifndef DOCK_MISSION__TASK_EVENT_HPP_
#define DOCK_MISSION__TASK_EVENT_HPP_

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <utility>

namespace dock_mission
{

std::optional<std::pair<std::string, nlohmann::json>> parse_task_event(const std::string & raw);

}  // namespace dock_mission

#endif  // DOCK_MISSION__TASK_EVENT_HPP_
