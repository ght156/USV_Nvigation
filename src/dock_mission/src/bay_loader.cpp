#include "dock_mission/bay_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace dock_mission
{

std::filesystem::path default_database_path()
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory("dock_mission")) /
    "config" / "dock_database.yaml";
}

BayRecord load_bay(
  const std::string & bay_id, const std::optional<std::filesystem::path> & database_path)
{
  const auto path = database_path ? *database_path : default_database_path();
  const YAML::Node root = YAML::LoadFile(path.string());
  const YAML::Node bays = root["bays"];
  if (!bays || !bays[bay_id]) {
    throw std::out_of_range("bay '" + bay_id + "' not in " + path.string());
  }
  const YAML::Node data = bays[bay_id];
  const YAML::Node gnss = data["gnss_staging"];
  const YAML::Node map = data["map_staging"];
  BayRecord result;
  result.bay_id = bay_id;
  result.name = data["name"] ? data["name"].as<std::string>() : bay_id;
  result.gnss.latitude = gnss && gnss["latitude"] ? gnss["latitude"].as<double>() : 0.0;
  result.gnss.longitude = gnss && gnss["longitude"] ? gnss["longitude"].as<double>() : 0.0;
  result.gnss.yaw_deg = gnss && gnss["yaw_deg"] ? gnss["yaw_deg"].as<double>() : 0.0;
  result.map_pose.x = map && map["x"] ? map["x"].as<double>() : 0.0;
  result.map_pose.y = map && map["y"] ? map["y"].as<double>() : 0.0;
  result.map_pose.yaw = map && map["yaw"] ? map["yaw"].as<double>() : 0.0;
  result.standoff_m = data["standoff_m"] ? data["standoff_m"].as<double>() : 4.0;
  return result;
}

}  // namespace dock_mission
