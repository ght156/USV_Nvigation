#include <m_common/msg/geo_fence.hpp>
#include <m_common/msg/geo_point.hpp>
#include <m_common/msg/geo_polygon.hpp>
#include <m_common/msg/nav_zones.hpp>
#include <m_common/srv/get_geo_fence.hpp>
#include <m_common/srv/get_nav_zones.hpp>
#include <m_common/srv/set_geo_fence.hpp>
#include <m_common/srv/set_nav_zones.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "workspace_nav/navigation_logging.hpp"
#include "workspace_nav/navigation_utils.hpp"
#include "workspace_nav/zone_geometry.hpp"

namespace workspace_nav
{

struct GeoFenceData
{
  std::string fence_id;
  std::string type;
  std::string shape{"polygon"};
  std::vector<mission_bridge::LatLon> points;
  std::optional<mission_bridge::LatLon> center;
  double radius_m{0.0};
};

class ZoneManagerNode : public rclcpp::Node
{
public:
  ZoneManagerNode()
  : Node("zone_manager")
  {
    declare_parameter("map_yaml_path", "");
    declare_parameter("map_datum_ref_key", "ref_gnss_10");
    declare_parameter("map_topic", "/map");
    declare_parameter("global_frame", "map");
    declare_parameter("filter_info_topic", "/keepout_filter/costmap_filter_info");
    declare_parameter("filter_mask_topic", "/keepout_filter/keepout_filter_mask");
    declare_parameter("zones_input_topic", "/nav_zones");
    declare_parameter("zones_current_topic", "/nav_zones/current");
    declare_parameter("boundary_dilation_cells", 2);
    declare_parameter("republish_period_sec", 0.0);
    declare_parameter("zones_persist_path", "");
    global_frame_ = get_parameter("global_frame").as_string();
    dilation_ = std::max(0, static_cast<int>(get_parameter("boundary_dilation_cells").as_int()));
    std::string map_path = get_parameter("map_yaml_path").as_string();
    if (map_path.empty()) map_path = navigation_utils::package_path("config/map.yaml").string();
    anchor_ = navigation_utils::load_map_anchor(
      map_path, get_parameter("map_datum_ref_key").as_string());
    std::string persist = get_parameter("zones_persist_path").as_string();
    persist_path_ = persist.empty() ? navigation_utils::package_path("json/nav_zones.json") :
      std::filesystem::absolute(persist);

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    filter_info_pub_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      get_parameter("filter_info_topic").as_string(), latched);
    mask_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      get_parameter("filter_mask_topic").as_string(), latched);
    zones_current_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("zones_current_topic").as_string(), latched);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("map_topic").as_string(), latched,
      std::bind(&ZoneManagerNode::map_callback, this, std::placeholders::_1));
    zones_input_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("zones_input_topic").as_string(), 10,
      std::bind(&ZoneManagerNode::zones_json_callback, this, std::placeholders::_1));
    set_geofence_service_ = create_service<m_common::srv::SetGeoFence>(
      "mission_bridge/set_geofence",
      std::bind(&ZoneManagerNode::set_geofence_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_geofence_service_ = create_service<m_common::srv::GetGeoFence>(
      "mission_bridge/get_geofence",
      std::bind(&ZoneManagerNode::get_geofence_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_nav_zones_service_ = create_service<m_common::srv::SetNavZones>(
      "mission_bridge/set_nav_zones",
      std::bind(&ZoneManagerNode::set_nav_zones_callback, this, std::placeholders::_1, std::placeholders::_2));
    clear_nav_zones_service_ = create_service<std_srvs::srv::Trigger>(
      "mission_bridge/clear_nav_zones",
      std::bind(&ZoneManagerNode::clear_nav_zones_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_nav_zones_service_ = create_service<m_common::srv::GetNavZones>(
      "mission_bridge/get_nav_zones",
      std::bind(&ZoneManagerNode::get_nav_zones_callback, this, std::placeholders::_1, std::placeholders::_2));
    const double period = get_parameter("republish_period_sec").as_double();
    if (period > 0.0) {
      republish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(0.2, period))),
        [this]() {publish_filter();});
    }
    load_persisted();
    NAV_LOG_INFO( "zone_manager ready; persist=%s", persist_path_.c_str());
  }

private:
  struct MapInfo
  {
    uint32_t width;
    uint32_t height;
    double resolution;
    double origin_x;
    double origin_y;
    bool operator==(const MapInfo & other) const
    {
      return width == other.width && height == other.height && resolution == other.resolution &&
             origin_x == other.origin_x && origin_y == other.origin_y;
    }
  };

  mission_bridge::Point to_map(double lon, double lat) const;
  mission_bridge::Point to_grid(const mission_bridge::Point & point) const;
  std::string validate_fence(const GeoFenceData & fence) const;
  std::string validate_fences(const std::vector<GeoFenceData> & fences) const;
  std::pair<bool, std::string> apply(
    std::optional<std::vector<GeoFenceData>> fences,
    std::optional<std::vector<std::vector<mission_bridge::LatLon>>> boundaries,
    const std::string & source);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void set_geofence_callback(
    const std::shared_ptr<m_common::srv::SetGeoFence::Request> request,
    std::shared_ptr<m_common::srv::SetGeoFence::Response> response);
  void get_geofence_callback(
    const std::shared_ptr<m_common::srv::GetGeoFence::Request>,
    std::shared_ptr<m_common::srv::GetGeoFence::Response> response);
  void set_nav_zones_callback(
    const std::shared_ptr<m_common::srv::SetNavZones::Request> request,
    std::shared_ptr<m_common::srv::SetNavZones::Response> response);
  void clear_nav_zones_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void get_nav_zones_callback(
    const std::shared_ptr<m_common::srv::GetNavZones::Request>,
    std::shared_ptr<m_common::srv::GetNavZones::Response> response);
  void zones_json_callback(const std_msgs::msg::String::SharedPtr msg);
  void rasterize_and_publish();
  void publish_filter();
  void persist();
  void load_persisted();
  void publish_zones_current();

  static std::vector<mission_bridge::LatLon> json_poly(const nlohmann::json & raw);
  static GeoFenceData json_to_fence(const nlohmann::json & raw);
  static nlohmann::json fence_to_json(const GeoFenceData & fence);
  static GeoFenceData msg_to_fence(const m_common::msg::GeoFence & msg);
  static m_common::msg::GeoFence fence_to_msg(const GeoFenceData & fence);
  static std::vector<mission_bridge::LatLon> msg_poly(const m_common::msg::GeoPolygon & polygon);
  static m_common::msg::GeoPolygon to_msg_poly(const std::vector<mission_bridge::LatLon> & points);
  m_common::msg::NavZones fences_to_nav_zones() const;
  nlohmann::json zones_to_json() const;

  navigation_utils::MapAnchor anchor_;
  std::filesystem::path persist_path_;
  std::string global_frame_;
  int dilation_{2};
  std::vector<GeoFenceData> fences_;
  std::vector<std::vector<mission_bridge::LatLon>> hard_boundaries_;
  std::optional<MapInfo> map_info_;
  std::vector<int8_t> mask_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_current_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zones_input_sub_;
  rclcpp::Service<m_common::srv::SetGeoFence>::SharedPtr set_geofence_service_;
  rclcpp::Service<m_common::srv::GetGeoFence>::SharedPtr get_geofence_service_;
  rclcpp::Service<m_common::srv::SetNavZones>::SharedPtr set_nav_zones_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_nav_zones_service_;
  rclcpp::Service<m_common::srv::GetNavZones>::SharedPtr get_nav_zones_service_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
};

}  // namespace workspace_nav

namespace workspace_nav
{

mission_bridge::Point ZoneManagerNode::to_map(double lon, double lat) const
{
  return navigation_utils::to_map(anchor_, {lat, lon});
}

mission_bridge::Point ZoneManagerNode::to_grid(const mission_bridge::Point & point) const
{
  return {(point.x - map_info_->origin_x) / map_info_->resolution,
    (point.y - map_info_->origin_y) / map_info_->resolution};
}

std::vector<mission_bridge::LatLon> ZoneManagerNode::json_poly(const nlohmann::json & raw)
{
  std::vector<mission_bridge::LatLon> points;
  if (!raw.is_array()) return points;
  for (const auto & item : raw) {
    try {
      if (item.is_object()) {
        const double lon = item.contains("longitude") ? item["longitude"].get<double>() :
          item.at("lon").get<double>();
        const double lat = item.contains("latitude") ? item["latitude"].get<double>() :
          item.at("lat").get<double>();
        points.push_back({lat, lon});
      } else if (item.is_array() && item.size() >= 2) {
        points.push_back({item[1].get<double>(), item[0].get<double>()});
      }
    } catch (const std::exception &) {
    }
  }
  return points;
}

GeoFenceData ZoneManagerNode::json_to_fence(const nlohmann::json & raw)
{
  GeoFenceData fence;
  if (!raw.is_object()) return fence;
  fence.fence_id = raw.value("fence_id", "");
  fence.type = raw.value("type", "");
  fence.shape = raw.value("shape", "polygon");
  if (fence.shape.empty()) fence.shape = "polygon";
  fence.points = json_poly(raw.value("points", nlohmann::json::array()));
  fence.radius_m = raw.value("radius_m", 0.0);
  if (raw.contains("center") && !raw["center"].is_null()) {
    const auto center = json_poly(nlohmann::json::array({raw["center"]}));
    if (!center.empty()) fence.center = center.front();
  }
  return fence;
}

nlohmann::json ZoneManagerNode::fence_to_json(const GeoFenceData & fence)
{
  nlohmann::json points = nlohmann::json::array();
  for (const auto & point : fence.points) points.push_back({point.longitude, point.latitude});
  nlohmann::json center;
  if (fence.center) center = {fence.center->longitude, fence.center->latitude};
  return {
    {"fence_id", fence.fence_id}, {"type", fence.type}, {"shape", fence.shape},
    {"points", points}, {"center", center}, {"radius_m", fence.radius_m}};
}

GeoFenceData ZoneManagerNode::msg_to_fence(const m_common::msg::GeoFence & msg)
{
  GeoFenceData fence;
  fence.fence_id = msg.fence_id;
  fence.type = msg.type;
  fence.shape = msg.shape.empty() ? "polygon" : msg.shape;
  for (const auto & point : msg.points) fence.points.push_back({point.lat, point.lon});
  if (msg.center.lon != 0.0 || msg.center.lat != 0.0) fence.center = {msg.center.lat, msg.center.lon};
  fence.radius_m = msg.radius_m;
  return fence;
}

m_common::msg::GeoFence ZoneManagerNode::fence_to_msg(const GeoFenceData & fence)
{
  m_common::msg::GeoFence msg;
  msg.fence_id = fence.fence_id;
  msg.type = fence.type;
  msg.shape = fence.shape;
  for (const auto & point : fence.points) {
    m_common::msg::GeoPoint geo;
    geo.lat = point.latitude;
    geo.lon = point.longitude;
    msg.points.push_back(geo);
  }
  if (fence.center) {
    msg.center.lat = fence.center->latitude;
    msg.center.lon = fence.center->longitude;
  }
  msg.radius_m = fence.radius_m;
  return msg;
}

std::vector<mission_bridge::LatLon> ZoneManagerNode::msg_poly(
  const m_common::msg::GeoPolygon & polygon)
{
  std::vector<mission_bridge::LatLon> points;
  const std::size_t count = std::min(polygon.longitude.size(), polygon.latitude.size());
  for (std::size_t i = 0; i < count; ++i) points.push_back({polygon.latitude[i], polygon.longitude[i]});
  return points;
}

m_common::msg::GeoPolygon ZoneManagerNode::to_msg_poly(
  const std::vector<mission_bridge::LatLon> & points)
{
  m_common::msg::GeoPolygon polygon;
  for (const auto & point : points) {
    polygon.longitude.push_back(point.longitude);
    polygon.latitude.push_back(point.latitude);
  }
  return polygon;
}

std::string ZoneManagerNode::validate_fence(const GeoFenceData & fence) const
{
  if (fence.fence_id.empty()) return "fence_id cannot be empty";
  if (fence.type != "exclusion" && fence.type != "inclusion") return "invalid fence type";
  if (fence.shape != "polygon" && fence.shape != "circle") return "invalid fence shape";
  const auto valid = [](const mission_bridge::LatLon & point) {
      return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
             point.latitude >= -90.0 && point.latitude <= 90.0 &&
             point.longitude >= -180.0 && point.longitude <= 180.0;
    };
  if (fence.shape == "polygon") {
    if (fence.points.size() < 3) return "polygon requires at least 3 points";
    std::set<std::pair<double, double>> unique;
    for (const auto & point : fence.points) {
      if (!valid(point)) return "invalid polygon coordinate";
      unique.insert({point.longitude, point.latitude});
    }
    if (unique.size() != fence.points.size()) return "polygon contains duplicate points";
  } else {
    if (!fence.center || !valid(*fence.center)) return "invalid circle center";
    if (!std::isfinite(fence.radius_m) || fence.radius_m <= 0.0) return "radius_m must be > 0";
  }
  return "";
}

std::string ZoneManagerNode::validate_fences(const std::vector<GeoFenceData> & fences) const
{
  std::set<std::string> ids;
  for (const auto & fence : fences) {
    const auto error = validate_fence(fence);
    if (!error.empty()) return error;
    if (!ids.insert(fence.fence_id).second) return "duplicate fence_id: " + fence.fence_id;
  }
  return "";
}

std::pair<bool, std::string> ZoneManagerNode::apply(
  std::optional<std::vector<GeoFenceData>> fences,
  std::optional<std::vector<std::vector<mission_bridge::LatLon>>> boundaries,
  const std::string &)
{
  if (fences) {
    const auto error = validate_fences(*fences);
    if (!error.empty()) return {false, error};
  }
  if (boundaries) {
    for (const auto & line : *boundaries) {
      if (line.size() < 2) return {false, "hard boundary requires at least 2 points"};
      for (const auto & point : line) {
        if (!std::isfinite(point.latitude) || !std::isfinite(point.longitude) ||
          std::abs(point.latitude) > 90.0 || std::abs(point.longitude) > 180.0)
        {
          return {false, "invalid hard boundary coordinate"};
        }
      }
    }
  }
  if (fences) fences_ = std::move(*fences);
  if (boundaries) hard_boundaries_ = std::move(*boundaries);
  persist();
  publish_zones_current();
  if (!map_info_) return {true, "zones accepted (mask pending /map)"};
  rasterize_and_publish();
  return {true, "applied"};
}

void ZoneManagerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  const MapInfo info{
    msg->info.width, msg->info.height, msg->info.resolution,
    msg->info.origin.position.x, msg->info.origin.position.y};
  if (map_info_ && *map_info_ == info) return;
  map_info_ = info;
  NAV_LOG_INFO( "/map ready: %ux%u res=%.3f", info.width, info.height, info.resolution);
  rasterize_and_publish();
}

void ZoneManagerNode::set_geofence_callback(
  const std::shared_ptr<m_common::srv::SetGeoFence::Request> request,
  std::shared_ptr<m_common::srv::SetGeoFence::Response> response)
{
  std::vector<GeoFenceData> fences;
  for (const auto & msg : request->fences) fences.push_back(msg_to_fence(msg));
  const auto result = apply(std::move(fences), std::nullopt, "set_geofence");
  response->success = result.first;
  response->message = result.second;
}

void ZoneManagerNode::get_geofence_callback(
  const std::shared_ptr<m_common::srv::GetGeoFence::Request>,
  std::shared_ptr<m_common::srv::GetGeoFence::Response> response)
{
  response->success = true;
  for (const auto & fence : fences_) response->fences.push_back(fence_to_msg(fence));
}

void ZoneManagerNode::set_nav_zones_callback(
  const std::shared_ptr<m_common::srv::SetNavZones::Request> request,
  std::shared_ptr<m_common::srv::SetNavZones::Response> response)
{
  std::vector<GeoFenceData> fences;
  if (!request->zones.work_area.longitude.empty()) {
    fences.push_back({"work_area", "inclusion", "polygon", msg_poly(request->zones.work_area), std::nullopt, 0.0});
  }
  for (std::size_t i = 0; i < request->zones.forbidden_zones.size(); ++i) {
    auto points = msg_poly(request->zones.forbidden_zones[i]);
    if (!points.empty()) {
      fences.push_back({"forbidden_" + std::to_string(i), "exclusion", "polygon", points, std::nullopt, 0.0});
    }
  }
  std::vector<std::vector<mission_bridge::LatLon>> boundaries;
  for (const auto & line : request->zones.hard_boundaries) boundaries.push_back(msg_poly(line));
  const auto result = apply(std::move(fences), std::move(boundaries), "set_nav_zones");
  response->success = result.first;
  response->message = result.second;
}

void ZoneManagerNode::clear_nav_zones_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  const auto result = apply(std::vector<GeoFenceData>{},
    std::vector<std::vector<mission_bridge::LatLon>>{}, "clear_nav_zones");
  response->success = result.first;
  response->message = "all nav zones cleared";
}

m_common::msg::NavZones ZoneManagerNode::fences_to_nav_zones() const
{
  m_common::msg::NavZones zones;
  for (const auto & fence : fences_) {
    auto points = fence.points;
    if (fence.shape == "circle" && fence.center) {
      points.clear();
      for (int i = 0; i < 16; ++i) {
        const double angle = 2.0 * M_PI * i / 16.0;
        const double east = fence.radius_m * std::cos(angle);
        const double north = fence.radius_m * std::sin(angle);
        points.push_back({
          fence.center->latitude + north / 6378137.0 * 180.0 / M_PI,
          fence.center->longitude + east /
          (6378137.0 * std::cos(fence.center->latitude * M_PI / 180.0)) * 180.0 / M_PI});
      }
    }
    if (fence.type == "inclusion" && zones.work_area.longitude.empty()) {
      zones.work_area = to_msg_poly(points);
    } else if (fence.type == "exclusion") {
      zones.forbidden_zones.push_back(to_msg_poly(points));
    }
  }
  for (const auto & line : hard_boundaries_) zones.hard_boundaries.push_back(to_msg_poly(line));
  return zones;
}

void ZoneManagerNode::get_nav_zones_callback(
  const std::shared_ptr<m_common::srv::GetNavZones::Request>,
  std::shared_ptr<m_common::srv::GetNavZones::Response> response)
{
  response->success = true;
  response->zones = fences_to_nav_zones();
}

void ZoneManagerNode::zones_json_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const auto data = nlohmann::json::parse(msg->data, nullptr, false);
  if (!data.is_object()) {
    NAV_LOG_WARN( "/nav_zones payload must be a JSON object");
    return;
  }
  if (data.value("clear", false)) {
    apply(std::vector<GeoFenceData>{}, std::vector<std::vector<mission_bridge::LatLon>>{}, "/nav_zones");
    return;
  }
  std::optional<std::vector<GeoFenceData>> fences;
  if (data.contains("fences")) {
    fences = std::vector<GeoFenceData>{};
    if (data["fences"].is_array()) {
      for (const auto & raw : data["fences"]) fences->push_back(json_to_fence(raw));
    }
  } else if (data.contains("work_area") || data.contains("forbidden_zones") || data.contains("forbidden")) {
    fences = std::vector<GeoFenceData>{};
    auto work = json_poly(data.value("work_area", nlohmann::json::array()));
    if (!work.empty()) fences->push_back({"work_area", "inclusion", "polygon", work, std::nullopt, 0.0});
    const auto forbidden = data.contains("forbidden_zones") ? data["forbidden_zones"] :
      data.value("forbidden", nlohmann::json::array());
    if (forbidden.is_array()) {
      for (std::size_t i = 0; i < forbidden.size(); ++i) {
        auto points = json_poly(forbidden[i]);
        if (!points.empty()) fences->push_back({
          "forbidden_" + std::to_string(i), "exclusion", "polygon", points, std::nullopt, 0.0});
      }
    }
  }
  std::optional<std::vector<std::vector<mission_bridge::LatLon>>> boundaries;
  if (data.contains("hard_boundaries") || data.contains("boundaries")) {
    boundaries = std::vector<std::vector<mission_bridge::LatLon>>{};
    const auto lines = data.contains("hard_boundaries") ? data["hard_boundaries"] : data["boundaries"];
    if (lines.is_array()) for (const auto & line : lines) boundaries->push_back(json_poly(line));
  }
  if (!fences && !boundaries) return;
  const auto result = apply(std::move(fences), std::move(boundaries), "/nav_zones");
  if (!result.first) NAV_LOG_WARN( "/nav_zones rejected: %s", result.second.c_str());
}

}  // namespace workspace_nav

namespace workspace_nav
{

void ZoneManagerNode::rasterize_and_publish()
{
  if (!map_info_) return;
  std::vector<zone_geometry::Fill> inclusion;
  std::vector<zone_geometry::Fill> exclusion;
  for (const auto & fence : fences_) {
    zone_geometry::Fill fill;
    if (fence.shape == "circle" && fence.center) {
      const auto center = to_grid(to_map(fence.center->longitude, fence.center->latitude));
      fill = zone_geometry::Fill::circle(center, fence.radius_m / map_info_->resolution);
    } else {
      std::vector<mission_bridge::Point> points;
      for (const auto & point : fence.points) points.push_back(to_grid(to_map(point.longitude, point.latitude)));
      fill = zone_geometry::Fill::polygon(std::move(points));
    }
    (fence.type == "inclusion" ? inclusion : exclusion).push_back(std::move(fill));
  }
  std::vector<std::vector<mission_bridge::Point>> boundaries;
  for (const auto & line : hard_boundaries_) {
    std::vector<mission_bridge::Point> grid;
    for (const auto & point : line) grid.push_back(to_grid(to_map(point.longitude, point.latitude)));
    boundaries.push_back(std::move(grid));
  }
  mask_ = zone_geometry::build_fence_mask(
    map_info_->width, map_info_->height, inclusion, exclusion, boundaries, dilation_);
  NAV_LOG_INFO( "zone mask updated: %ux%u", map_info_->width, map_info_->height);
  publish_filter();
}

void ZoneManagerNode::publish_filter()
{
  if (!map_info_ || mask_.empty()) return;
  const auto stamp = now();
  nav2_msgs::msg::CostmapFilterInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = global_frame_;
  info.type = 0;
  info.filter_mask_topic = get_parameter("filter_mask_topic").as_string();
  info.base = 0.0;
  info.multiplier = 1.0;
  filter_info_pub_->publish(info);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = global_frame_;
  grid.info.resolution = map_info_->resolution;
  grid.info.width = map_info_->width;
  grid.info.height = map_info_->height;
  grid.info.origin.position.x = map_info_->origin_x;
  grid.info.origin.position.y = map_info_->origin_y;
  grid.info.origin.orientation.w = 1.0;
  grid.data = mask_;
  mask_pub_->publish(grid);
}

nlohmann::json ZoneManagerNode::zones_to_json() const
{
  nlohmann::json fences = nlohmann::json::array();
  for (const auto & fence : fences_) fences.push_back(fence_to_json(fence));
  nlohmann::json boundaries = nlohmann::json::array();
  for (const auto & line : hard_boundaries_) {
    nlohmann::json points = nlohmann::json::array();
    for (const auto & point : line) points.push_back({point.longitude, point.latitude});
    boundaries.push_back(std::move(points));
  }
  return {{"fences", fences}, {"hard_boundaries", boundaries}};
}

void ZoneManagerNode::publish_zones_current()
{
  zones_current_pub_->publish(std_msgs::msg::String().set__data(zones_to_json().dump()));
}

void ZoneManagerNode::persist()
{
  try {
    navigation_utils::atomic_write_json(persist_path_, zones_to_json());
  } catch (const std::exception & error) {
    NAV_LOG_ERROR( "failed persisting zones: %s", error.what());
  }
}

void ZoneManagerNode::load_persisted()
{
  if (!std::filesystem::exists(persist_path_)) return;
  try {
    std::ifstream stream(persist_path_);
    const auto data = nlohmann::json::parse(stream);
    std::vector<GeoFenceData> fences;
    if (data.contains("fences") && data["fences"].is_array()) {
      for (const auto & raw : data["fences"]) fences.push_back(json_to_fence(raw));
    }
    const auto error = validate_fences(fences);
    if (!error.empty()) {
      NAV_LOG_ERROR( "invalid persisted zones: %s", error.c_str());
      return;
    }
    fences_ = std::move(fences);
    if (data.contains("hard_boundaries") && data["hard_boundaries"].is_array()) {
      for (const auto & line : data["hard_boundaries"]) hard_boundaries_.push_back(json_poly(line));
    }
    publish_zones_current();
  } catch (const std::exception & error) {
    NAV_LOG_ERROR( "failed loading persisted zones: %s", error.what());
  }
}

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("zone_manager");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<workspace_nav::ZoneManagerNode>());
  rclcpp::shutdown();
  return 0;
}
