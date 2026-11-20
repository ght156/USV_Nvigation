#include "dock_mission/types.hpp"

#include <stdexcept>

namespace dock_mission
{

std::string to_string(MissionState state)
{
  switch (state) {
    case MissionState::IDLE: return "DOCK_MISSION_IDLE";
    case MissionState::ARMED: return "DOCK_ARMED";
    case MissionState::NAV_TO_STAGING: return "NAV_TO_STAGING";
    case MissionState::SETTLE: return "SETTLE";
    case MissionState::DOCK_HANDOFF: return "DOCK_HANDOFF";
    case MissionState::MONITOR_DOCK: return "MONITOR_DOCK";
    case MissionState::UNDOCK_HANDOFF: return "UNDOCK_HANDOFF";
    case MissionState::MONITOR_UNDOCK: return "MONITOR_UNDOCK";
    case MissionState::COMPLETE_SETTLE: return "COMPLETE_SETTLE";
    case MissionState::SUCCEEDED: return "DOCK_SUCCEEDED";
    case MissionState::FAILED: return "DOCK_FAILED";
    case MissionState::CANCELLED: return "DOCK_CANCELLED";
  }
  throw std::invalid_argument("unknown mission state");
}

std::string to_string(SpeedAuthority authority)
{
  switch (authority) {
    case SpeedAuthority::NAVIGATION: return "NAVIGATION";
    case SpeedAuthority::STAGING_VERIFY: return "STAGING_VERIFY";
    case SpeedAuthority::DOCKING: return "DOCKING";
    case SpeedAuthority::RELEASE: return "RELEASE";
    case SpeedAuthority::FAILED: return "FAILED";
  }
  throw std::invalid_argument("unknown speed authority");
}

}  // namespace dock_mission
