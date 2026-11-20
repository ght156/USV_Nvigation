#ifndef DOCK_MISSION__GNSS_STAGING_HPP_
#define DOCK_MISSION__GNSS_STAGING_HPP_

#include "dock_mission/bay_loader.hpp"

#include <array>
#include <filesystem>
#include <optional>

namespace dock_mission
{

GnssStaging map_xy_to_gnss(double x, double y, const std::filesystem::path & map_yaml);
std::array<double, 3> make_staging_waypoint(
  const BayRecord & bay, bool use_gnss = true,
  const std::optional<std::filesystem::path> & map_yaml = std::nullopt);

}  // namespace dock_mission

#endif  // DOCK_MISSION__GNSS_STAGING_HPP_
