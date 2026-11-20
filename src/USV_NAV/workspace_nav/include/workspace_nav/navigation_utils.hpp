#ifndef WORKSPACE_NAV__NAVIGATION_UTILS_HPP_
#define WORKSPACE_NAV__NAVIGATION_UTILS_HPP_

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "workspace_nav/mission_bridge_core.hpp"

namespace workspace_nav::navigation_utils
{

struct MapAnchor
{
  double datum_lat{0.0};
  double datum_lon{0.0};
  double datum_easting{0.0};
  double datum_northing{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::string map_yaml;
  std::string ref_key{"ref_gnss_10"};
  std::string datum_source{"map_yaml"};
  std::string projection{"enu"};
};

MapAnchor load_map_anchor(
  const std::filesystem::path & map_path, const std::string & ref_key,
  const std::string & projection = "enu");
mission_bridge::Point to_map(const MapAnchor & anchor, const mission_bridge::LatLon & waypoint);
nlohmann::json make_waypoints_document(
  const MapAnchor & anchor, const std::vector<mission_bridge::LatLon> & waypoints);
void atomic_write_json(const std::filesystem::path & path, const nlohmann::json & document);
std::filesystem::path package_path(const std::string & relative);

}  // namespace workspace_nav::navigation_utils

#endif  // WORKSPACE_NAV__NAVIGATION_UTILS_HPP_
