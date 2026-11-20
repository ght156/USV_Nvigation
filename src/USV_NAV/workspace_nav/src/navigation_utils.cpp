#include "workspace_nav/navigation_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace workspace_nav::navigation_utils
{

MapAnchor load_map_anchor(
  const std::filesystem::path & map_path, const std::string & ref_key,
  const std::string & projection)
{
  MapAnchor anchor;
  anchor.map_yaml = std::filesystem::absolute(map_path).string();
  anchor.ref_key = ref_key.empty() ? "ref_gnss_10" : ref_key;
  anchor.projection = projection == "utm" ? "utm" : "enu";
  const YAML::Node config = YAML::LoadFile(map_path.string());
  YAML::Node reference;
  if (!anchor.ref_key.empty() && config[anchor.ref_key]) reference = config[anchor.ref_key];
  if (!reference && config["ref_gnss_10"]) reference = config["ref_gnss_10"];
  if (!reference && config["ref_gnss"]) reference = config["ref_gnss"];
  if (!reference || reference.size() < 2) throw std::runtime_error("map datum is missing");
  anchor.datum_lon = reference[0].as<double>();
  anchor.datum_lat = reference[1].as<double>();
  if (config["origin"] && config["origin"].IsSequence()) {
    const auto origin = config["origin"];
    if (origin.size() > 0) anchor.origin_x = origin[0].as<double>();
    if (origin.size() > 1) anchor.origin_y = origin[1].as<double>();
    if (origin.size() > 2) anchor.origin_yaw = origin[2].as<double>();
  }
  const auto utm = mission_bridge::lat_lon_to_utm(anchor.datum_lat, anchor.datum_lon);
  anchor.datum_easting = utm.easting;
  anchor.datum_northing = utm.northing;
  return anchor;
}

mission_bridge::Point to_map(const MapAnchor & anchor, const mission_bridge::LatLon & waypoint)
{
  double east;
  double north;
  if (anchor.projection == "utm") {
    const auto converted = mission_bridge::lat_lon_to_utm(waypoint.latitude, waypoint.longitude);
    east = converted.easting - anchor.datum_easting;
    north = converted.northing - anchor.datum_northing;
  } else {
    const auto delta = mission_bridge::geodetic_delta_enu_m(
      anchor.datum_lat, anchor.datum_lon, waypoint.latitude, waypoint.longitude);
    east = delta.x;
    north = delta.y;
  }
  if (anchor.datum_source != "map_yaml") return {east, north};
  return mission_bridge::enu_delta_to_map_xy(
    east, north, anchor.origin_x, anchor.origin_y, anchor.origin_yaw);
}

nlohmann::json make_waypoints_document(
  const MapAnchor & anchor, const std::vector<mission_bridge::LatLon> & waypoints)
{
  nlohmann::json document = {
    {"waypoints", nlohmann::json::array()},
    {"datum", {{"latitude", anchor.datum_lat}, {"longitude", anchor.datum_lon}}},
    {"datum_source_meta", {
      {"datum_source", anchor.datum_source},
      {"map_yaml", anchor.map_yaml.empty() ? nlohmann::json() : nlohmann::json(anchor.map_yaml)},
      {"map_datum_ref_key", anchor.datum_source == "map_yaml" ?
        nlohmann::json(anchor.ref_key) : nlohmann::json()},
      {"projection", anchor.projection}}},
    {"map_frame_meta", {
      {"frame_id", "map"},
      {"description", "x,y are Nav2 map coordinates in metres"},
      {"origin_x", anchor.origin_x}, {"origin_y", anchor.origin_y},
      {"origin_yaw_rad", anchor.origin_yaw},
      {"applied_origin_transform", anchor.datum_source == "map_yaml"}}}
  };
  for (const auto & waypoint : waypoints) {
    const auto point = to_map(anchor, waypoint);
    document["waypoints"].push_back({
      {"latitude", waypoint.latitude}, {"longitude", waypoint.longitude},
      {"x", std::round(point.x * 10000.0) / 10000.0},
      {"y", std::round(point.y * 10000.0) / 10000.0}});
  }
  return document;
}

void atomic_write_json(const std::filesystem::path & path, const nlohmann::json & document)
{
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.parent_path() /
    ("json_" + std::to_string(getpid()) + "_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".tmp");
  const std::string content = document.dump(2, ' ', false);
  const int descriptor = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (descriptor < 0) throw std::runtime_error("cannot create JSON temporary file");
  try {
    std::size_t offset = 0;
    while (offset < content.size()) {
      const ssize_t written = ::write(descriptor, content.data() + offset, content.size() - offset);
      if (written <= 0) throw std::runtime_error("cannot write JSON file");
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor) != 0) throw std::runtime_error("cannot sync JSON file");
    if (::close(descriptor) != 0) throw std::runtime_error("cannot close JSON file");
    if (::rename(temporary.c_str(), path.c_str()) != 0) {
      throw std::runtime_error("cannot replace JSON file");
    }
  } catch (...) {
    ::close(descriptor);
    ::unlink(temporary.c_str());
    throw;
  }
}

std::filesystem::path package_path(const std::string & relative)
{
  return std::filesystem::path(
    ament_index_cpp::get_package_share_directory("workspace_nav")) / relative;
}

}  // namespace workspace_nav::navigation_utils
