#include "dock_mission/gnss_staging.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace dock_mission
{

GnssStaging map_xy_to_gnss(double x, double y, const std::filesystem::path & map_yaml)
{
  const YAML::Node config = YAML::LoadFile(map_yaml.string());
  YAML::Node ref = config["ref_gnss_10"];
  if (!ref) ref = config["ref_gnss"];
  if (!ref) throw std::invalid_argument("no ref_gnss in " + map_yaml.string());
  const double lon0 = ref[0].as<double>();
  const double lat0 = ref[1].as<double>();
  const YAML::Node origin = config["origin"];
  const double ox = origin && origin.size() > 0 ? origin[0].as<double>() : 0.0;
  const double oy = origin && origin.size() > 1 ? origin[1].as<double>() : 0.0;
  const double oyaw = origin && origin.size() > 2 ? origin[2].as<double>() : 0.0;
  const double dx = x - ox;
  const double dy = y - oy;
  const double c = std::cos(oyaw);
  const double s = std::sin(oyaw);
  const double east = dx * c + dy * s;
  const double north = -dx * s + dy * c;
  constexpr double earth_radius = 6378137.0;
  constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
  return {
    lat0 + rad_to_deg * north / earth_radius,
    lon0 + rad_to_deg * east / (earth_radius * std::cos(lat0 / rad_to_deg)),
    0.0};
}

std::array<double, 3> make_staging_waypoint(
  const BayRecord & bay, bool use_gnss,
  const std::optional<std::filesystem::path> & map_yaml)
{
  if (use_gnss) {
    if (bay.gnss.latitude == 0.0 && bay.gnss.longitude == 0.0) {
      throw std::invalid_argument(
        "bay " + bay.bay_id + ": gnss_staging not configured in dock_database.yaml");
    }
    return {bay.gnss.latitude, bay.gnss.longitude, bay.gnss.yaw_deg};
  }
  if (!map_yaml) {
    throw std::invalid_argument("map_yaml required when use_gnss=False (map→WGS84 反解)");
  }
  const auto gnss = map_xy_to_gnss(bay.map_pose.x, bay.map_pose.y, *map_yaml);
  return {gnss.latitude, gnss.longitude, bay.map_pose.yaw * 180.0 / 3.14159265358979323846};
}

}  // namespace dock_mission
