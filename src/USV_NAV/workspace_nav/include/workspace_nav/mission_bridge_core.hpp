#ifndef WORKSPACE_NAV__MISSION_BRIDGE_CORE_HPP_
#define WORKSPACE_NAV__MISSION_BRIDGE_CORE_HPP_

#include <optional>
#include <string>
#include <vector>

namespace workspace_nav::mission_bridge
{

inline constexpr double kYawUnspecified = 65536.0;

enum class MissionState
{
  WaitingSystem,
  Idle,
  Running,
  Paused,
  Completed,
  Failed,
};

std::string to_string(MissionState state);

struct LatLon
{
  double latitude;
  double longitude;
};

struct Point
{
  double x;
  double y;
};

struct ParsedWaypointMessage
{
  std::vector<LatLon> waypoints;
  bool explicit_replan{false};
  std::string mission_id;
  std::string command_id;
};

struct UtmCoordinate
{
  double easting;
  double northing;
  int zone;
  bool northern;
};

struct Fence
{
  std::string fence_id;
  std::string type;
  std::string shape;
  std::vector<Point> points;
  std::optional<Point> center;
  double radius_m{0.0};
};

bool yaw_is_specified(double yaw);
bool should_skip_waypoint(double distance, double tolerance, double yaw);
std::string validate_waypoint(const LatLon & waypoint, std::size_t index);
std::optional<ParsedWaypointMessage> parse_waypoint_message(const std::string & data);
std::string waypoint_mission_hash(
  const std::vector<LatLon> & waypoints, const std::string & mission_id = "");
Point geodetic_delta_enu_m(double lat0, double lon0, double lat, double lon);
Point enu_delta_to_map_xy(
  double east, double north, double origin_x, double origin_y, double origin_yaw);
UtmCoordinate lat_lon_to_utm(double latitude, double longitude);
std::optional<std::size_t> fence_violation_index(
  const Point & point, const std::vector<Fence> & fences);
int map_nav2_failure(int nav2_error_code, const std::string & nav2_error_msg);

class CtrlStuckWatchdog
{
public:
  explicit CtrlStuckWatchdog(double timeout_sec);
  void update(MissionState state, bool stuck, double now);
  bool should_fail(double now) const;
  void reset();

private:
  double timeout_sec_;
  std::optional<double> since_;
  double last_seen_{0.0};
};

}  // namespace workspace_nav::mission_bridge

#endif  // WORKSPACE_NAV__MISSION_BRIDGE_CORE_HPP_
