#ifndef DOCK_MISSION__TYPES_HPP_
#define DOCK_MISSION__TYPES_HPP_

#include <string>

namespace dock_mission
{

enum class MissionState
{
  IDLE, ARMED, NAV_TO_STAGING, SETTLE, DOCK_HANDOFF, MONITOR_DOCK,
  UNDOCK_HANDOFF, MONITOR_UNDOCK, COMPLETE_SETTLE, SUCCEEDED, FAILED, CANCELLED
};

enum class SpeedAuthority
{
  NAVIGATION, STAGING_VERIFY, DOCKING, RELEASE, FAILED
};

std::string to_string(MissionState state);
std::string to_string(SpeedAuthority authority);

}  // namespace dock_mission

#endif  // DOCK_MISSION__TYPES_HPP_
