#include "workspace_nav/mission_bridge_core.hpp"

#include <openssl/sha.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace workspace_nav::mission_bridge
{
namespace
{

std::string trim(std::string value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }).base();
  if (first >= last) {
    return "";
  }
  return std::string(first, last);
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return value;
}

bool number_value(const nlohmann::json & value, double & output)
{
  try {
    if (value.is_number()) {
      output = value.get<double>();
      return true;
    }
    if (value.is_boolean()) {
      output = value.get<bool>() ? 1.0 : 0.0;
      return true;
    }
    if (value.is_string()) {
      std::size_t consumed = 0;
      const std::string text = trim(value.get<std::string>());
      output = std::stod(text, &consumed);
      return consumed == text.size();
    }
  } catch (const std::exception &) {
  }
  return false;
}

bool truthy_explicit_flag(const nlohmann::json & value)
{
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number()) {
    return value.get<double>() != 0.0;
  }
  if (value.is_string()) {
    const std::string text = lower(trim(value.get<std::string>()));
    return text == "true" || text == "1" || text == "yes" || text == "on";
  }
  return false;
}

bool point_in_polygon(const Point & point, const std::vector<Point> & vertices)
{
  if (vertices.size() < 3) {
    return false;
  }
  bool inside = false;
  std::size_t previous = vertices.size() - 1;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    const auto & a = vertices[i];
    const auto & b = vertices[previous];
    if ((a.y > point.y) != (b.y > point.y)) {
      const double crossing = (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
      if (point.x < crossing) {
        inside = !inside;
      }
    }
    previous = i;
  }
  return inside;
}

bool point_in_fence(const Point & point, const Fence & fence)
{
  if (fence.shape == "circle" && fence.center.has_value()) {
    const double dx = point.x - fence.center->x;
    const double dy = point.y - fence.center->y;
    return dx * dx + dy * dy <= fence.radius_m * fence.radius_m;
  }
  return point_in_polygon(point, fence.points);
}

std::string python_float(double value)
{
  return nlohmann::json(value).dump();
}

}  // namespace

std::string to_string(MissionState state)
{
  switch (state) {
    case MissionState::WaitingSystem: return "WAITING_SYSTEM";
    case MissionState::Idle: return "IDLE";
    case MissionState::Running: return "RUNNING";
    case MissionState::Paused: return "PAUSED";
    case MissionState::Completed: return "COMPLETED";
    case MissionState::Failed: return "FAILED";
  }
  return "FAILED";
}

bool yaw_is_specified(double yaw)
{
  return std::isfinite(yaw) && yaw >= -360.0 && yaw <= 360.0;
}

bool should_skip_waypoint(double distance, double tolerance, double yaw)
{
  return distance <= tolerance && !yaw_is_specified(yaw);
}

std::string validate_waypoint(const LatLon & waypoint, std::size_t index)
{
  std::ostringstream prefix;
  prefix << "Waypoint #" << index;
  if (!std::isfinite(waypoint.latitude) || !std::isfinite(waypoint.longitude)) {
    prefix << " lat/lon not finite: (" << waypoint.latitude << ", " << waypoint.longitude << ")";
    return prefix.str();
  }
  if (waypoint.latitude == 0.0 && waypoint.longitude == 0.0) {
    prefix << " is (0, 0)";
    return prefix.str();
  }
  if (waypoint.latitude < -90.0 || waypoint.latitude > 90.0 ||
    waypoint.longitude < -180.0 || waypoint.longitude > 180.0)
  {
    prefix << " lat/lon out of range: (" << waypoint.latitude << ", " <<
      waypoint.longitude << ")";
    return prefix.str();
  }
  return "";
}

std::optional<ParsedWaypointMessage> parse_waypoint_message(const std::string & data)
{
  std::string text = trim(data);
  if (text.rfind("data: ", 0) == 0) {
    text = trim(text.substr(6));
  }
  const auto document = nlohmann::json::parse(text, nullptr, false);
  if (document.is_discarded()) {
    return std::nullopt;
  }

  ParsedWaypointMessage result;
  const nlohmann::json * list = nullptr;
  if (document.is_array()) {
    list = &document;
  } else if (document.is_object() && document.contains("waypoints") &&
    document["waypoints"].is_array())
  {
    list = &document["waypoints"];
    for (const char * key : {"explicit_replan", "from_start_mission", "start_mission",
        "force_replan", "restart_mission"})
    {
      if (document.contains(key) && truthy_explicit_flag(document[key])) {
        result.explicit_replan = true;
      }
    }
    if (document.contains("command") && document["command"].is_string()) {
      const std::string command = lower(trim(document["command"].get<std::string>()));
      if (command == "start" || command == "replan" || command == "restart" ||
        command == "reload")
      {
        result.explicit_replan = true;
      }
    }
    if (document.contains("mission_id") && document["mission_id"].is_string()) {
      result.mission_id = trim(document["mission_id"].get<std::string>());
    }
    if (document.contains("command_id") && document["command_id"].is_string()) {
      result.command_id = trim(document["command_id"].get<std::string>());
    }
  }
  if (list == nullptr || list->empty()) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < list->size(); ++i) {
    const auto & item = (*list)[i];
    LatLon waypoint{0.0, 0.0};
    if (item.is_object() && item.contains("latitude") && item.contains("longitude")) {
      if (!number_value(item["latitude"], waypoint.latitude) ||
        !number_value(item["longitude"], waypoint.longitude))
      {
        return std::nullopt;
      }
    } else if (item.is_array() && item.size() >= 2) {
      if (!number_value(item[0], waypoint.latitude) ||
        !number_value(item[1], waypoint.longitude))
      {
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
    if (!validate_waypoint(waypoint, i).empty()) {
      return std::nullopt;
    }
    result.waypoints.push_back(waypoint);
  }
  return result;
}

std::string waypoint_mission_hash(
  const std::vector<LatLon> & waypoints, const std::string & mission_id)
{
  std::ostringstream blob;
  blob << "[";
  bool first = true;
  for (const auto & waypoint : waypoints) {
    if (!first) {
      blob << ", ";
    }
    first = false;
    blob << "{\"latitude\": " << python_float(waypoint.latitude) <<
      ", \"longitude\": " << python_float(waypoint.longitude) << "}";
  }
  if (!mission_id.empty()) {
    if (!first) {
      blob << ", ";
    }
    blob << "{\"mission_id\": " << nlohmann::json(mission_id).dump(-1, ' ', true) << "}";
  }
  blob << "]";
  const std::string input = blob.str();
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.data()), input.size(), digest);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto byte : digest) {
    output << std::setw(2) << static_cast<int>(byte);
  }
  return output.str();
}

Point geodetic_delta_enu_m(double lat0, double lon0, double lat, double lon)
{
  constexpr double earth_radius = 6378137.0;
  constexpr double to_rad = M_PI / 180.0;
  return {
    earth_radius * std::cos(lat0 * to_rad) * (lon - lon0) * to_rad,
    earth_radius * (lat - lat0) * to_rad};
}

Point enu_delta_to_map_xy(
  double east, double north, double origin_x, double origin_y, double origin_yaw)
{
  const double c = std::cos(origin_yaw);
  const double s = std::sin(origin_yaw);
  return {origin_x + east * c - north * s, origin_y + east * s + north * c};
}

UtmCoordinate lat_lon_to_utm(double latitude, double longitude)
{
  if (latitude < -80.0 || latitude > 84.0) {
    throw std::invalid_argument("UTM projection requires latitude in [-80, 84]");
  }
  constexpr double a = 6378137.0;
  constexpr double f = 1.0 / 298.257223563;
  constexpr double k0 = 0.9996;
  constexpr double e2 = f * (2.0 - f);
  constexpr double ep2 = e2 / (1.0 - e2);
  constexpr double to_rad = M_PI / 180.0;

  int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
  if (longitude == 180.0) {
    zone = 60;
  }
  if (latitude >= 56.0 && latitude < 64.0 && longitude >= 3.0 && longitude < 12.0) {
    zone = 32;
  }
  if (latitude >= 72.0 && latitude < 84.0) {
    if (longitude >= 0.0 && longitude < 9.0) zone = 31;
    else if (longitude >= 9.0 && longitude < 21.0) zone = 33;
    else if (longitude >= 21.0 && longitude < 33.0) zone = 35;
    else if (longitude >= 33.0 && longitude < 42.0) zone = 37;
  }

  const double lat = latitude * to_rad;
  const double lon = longitude * to_rad;
  const double central = ((zone - 1) * 6 - 180 + 3) * to_rad;
  const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
  const double t = std::tan(lat) * std::tan(lat);
  const double c = ep2 * std::cos(lat) * std::cos(lat);
  const double aa = std::cos(lat) * (lon - central);
  const double m = a * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 -
      5.0 * e2 * e2 * e2 / 256.0) * lat -
    (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) *
    std::sin(2.0 * lat) +
    (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat) -
    (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat));
  const double easting = k0 * n * (aa + (1.0 - t + c) * std::pow(aa, 3) / 6.0 +
    (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * std::pow(aa, 5) / 120.0) +
    500000.0;
  double northing = k0 * (m + n * std::tan(lat) * (aa * aa / 2.0 +
    (5.0 - t + 9.0 * c + 4.0 * c * c) * std::pow(aa, 4) / 24.0 +
    (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * std::pow(aa, 6) / 720.0));
  const bool northern = latitude >= 0.0;
  if (!northern) {
    northing += 10000000.0;
  }
  return {easting, northing, zone, northern};
}

std::optional<std::size_t> fence_violation_index(
  const Point & point, const std::vector<Fence> & fences)
{
  for (std::size_t i = 0; i < fences.size(); ++i) {
    if (fences[i].type == "inclusion" && point_in_fence(point, fences[i])) {
      bool found_inclusion = true;
      (void)found_inclusion;
      for (std::size_t j = 0; j < fences.size(); ++j) {
        if (fences[j].type == "exclusion" && point_in_fence(point, fences[j])) {
          return j;
        }
      }
      return std::nullopt;
    }
  }
  for (std::size_t i = 0; i < fences.size(); ++i) {
    if (fences[i].type == "inclusion") {
      return i;
    }
  }
  for (std::size_t i = 0; i < fences.size(); ++i) {
    if (fences[i].type == "exclusion" && point_in_fence(point, fences[i])) {
      return i;
    }
  }
  return std::nullopt;
}

PlannerFailure classify_planner_failure(
  const Point & start, const Point & target, const CostmapView * costmap)
{
  if (costmap == nullptr || costmap->width == 0 || costmap->height == 0 ||
    costmap->resolution <= 0.0 || costmap->data.size() < costmap->width * costmap->height)
  {
    return PlannerFailure::NoPath;
  }
  const auto index = [costmap](const Point & point) -> std::optional<std::size_t> {
      const double grid_x = (point.x - costmap->origin_x) / costmap->resolution;
      const double grid_y = (point.y - costmap->origin_y) / costmap->resolution;
      if (grid_x < 0.0 || grid_y < 0.0 ||
        grid_x >= static_cast<double>(costmap->width) ||
        grid_y >= static_cast<double>(costmap->height))
      {
        return std::nullopt;
      }
      return static_cast<std::size_t>(grid_y) * costmap->width +
             static_cast<std::size_t>(grid_x);
    };
  const auto blocked = [costmap](std::size_t index) {
      return static_cast<uint8_t>(costmap->data[index]) >= 253;
    };
  const auto target_index = index(target);
  if (!target_index || blocked(*target_index)) return PlannerFailure::TargetInvalid;
  const auto start_index = index(start);
  if (!start_index || blocked(*start_index)) return PlannerFailure::StartBlocked;
  return PlannerFailure::NoPath;
}

int map_nav2_failure(int nav2_error_code, const std::string & nav2_error_msg)
{
  const std::string message = lower(nav2_error_msg);
  for (const char * key : {"plan", "path", "route", "missed"}) {
    if (message.find(key) != std::string::npos) return 5;
  }
  for (const char * key : {"controller", "control", "follow", "stuck", "collision"}) {
    if (message.find(key) != std::string::npos) return 6;
  }
  if (nav2_error_code >= 100 && nav2_error_code < 200) return 5;
  if (nav2_error_code >= 200 && nav2_error_code < 300) return 6;
  return 99;
}

CtrlStuckWatchdog::CtrlStuckWatchdog(double timeout_sec)
: timeout_sec_(timeout_sec) {}

void CtrlStuckWatchdog::update(MissionState state, bool stuck, double now)
{
  if (state != MissionState::Running || !stuck || timeout_sec_ <= 0.0) {
    reset();
    return;
  }
  if (!since_.has_value()) {
    since_ = now;
  }
  last_seen_ = now;
}

bool CtrlStuckWatchdog::should_fail(double now) const
{
  return timeout_sec_ > 0.0 && since_.has_value() && last_seen_ > 0.0 &&
         now - last_seen_ <= 2.0 && now - *since_ >= timeout_sec_;
}

void CtrlStuckWatchdog::reset()
{
  since_.reset();
  last_seen_ = 0.0;
}

}  // namespace workspace_nav::mission_bridge
