#ifndef DOCK_MISSION__BAY_LOADER_HPP_
#define DOCK_MISSION__BAY_LOADER_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace dock_mission
{

struct GnssStaging
{
  double latitude{0.0};
  double longitude{0.0};
  double yaw_deg{0.0};
};

struct MapStaging
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct BayRecord
{
  std::string bay_id;
  std::string name;
  GnssStaging gnss;
  MapStaging map_pose;
  double standoff_m{4.0};
};

std::filesystem::path default_database_path();
BayRecord load_bay(
  const std::string & bay_id,
  const std::optional<std::filesystem::path> & database_path = std::nullopt);

}  // namespace dock_mission

#endif  // DOCK_MISSION__BAY_LOADER_HPP_
