#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "workspace_nav/navigation_logging.hpp"
#include "workspace_nav/navigation_utils.hpp"

namespace workspace_nav
{

class WaypointTransformNode : public rclcpp::Node
{
public:
  WaypointTransformNode()
  : Node("waypoint_transform")
  {
    declare_parameter("datum_source", "map_yaml");
    declare_parameter("map_yaml_path", "");
    declare_parameter("gps_topic", "/mavros/global_position/global");
    declare_parameter("map_datum_ref_key", "ref_gnss_10");
    declare_parameter("projection", "enu");

    anchor_.datum_source = get_parameter("datum_source").as_string();
    anchor_.projection = get_parameter("projection").as_string();
    anchor_.ref_key = get_parameter("map_datum_ref_key").as_string();
    if (const char * output = std::getenv("WAYPOINT_OUTPUT_PATH"); output && *output) {
      output_path_ = std::filesystem::absolute(output);
    } else {
      output_path_ = navigation_utils::package_path("json/waypoints.json");
    }

    if (anchor_.datum_source == "map_yaml") {
      std::string map_path = get_parameter("map_yaml_path").as_string();
      if (map_path.empty()) {
        map_path = navigation_utils::package_path("config/map_real_boat_hk.yaml").string();
      }
      anchor_ = navigation_utils::load_map_anchor(map_path, anchor_.ref_key, anchor_.projection);
      datum_ready_ = true;
      NAV_LOG_INFO( "Using datum from map_yaml: %s", map_path.c_str());
    } else if (anchor_.datum_source == "first_gps") {
      anchor_.datum_source = "first_gps";
      anchor_.map_yaml.clear();
      gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        get_parameter("gps_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&WaypointTransformNode::gps_callback, this, std::placeholders::_1));
      NAV_LOG_WARN( "datum_source=first_gps is deprecated for real USV use");
    } else {
      throw std::invalid_argument("datum_source must be map_yaml or first_gps");
    }

    waypoint_subscription_ = create_subscription<std_msgs::msg::String>(
      "/waypoint", 10, std::bind(&WaypointTransformNode::waypoint_callback, this, std::placeholders::_1));
    shutdown_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        if (done_) {
          shutdown_timer_->cancel();
          rclcpp::shutdown();
        }
      });
    NAV_LOG_INFO( "GPS-to-file node started; output=%s", output_path_.c_str());
  }

private:
  void waypoint_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (waypoint_received_) return;
    try {
      std::string text = msg->data;
      const auto first = text.find_first_not_of(" \t\r\n");
      const auto last = text.find_last_not_of(" \t\r\n");
      text = first == std::string::npos ? "" : text.substr(first, last - first + 1);
      if (text.rfind("data: ", 0) == 0) text = text.substr(6);
      const auto document = nlohmann::json::parse(text);
      if (!document.is_object() || !document.contains("waypoints") ||
        !document["waypoints"].is_array() || document["waypoints"].empty())
      {
        NAV_LOG_ERROR( "Invalid waypoint message: missing or empty waypoints list");
        return;
      }
      std::vector<mission_bridge::LatLon> parsed;
      for (std::size_t i = 0; i < document["waypoints"].size(); ++i) {
        const auto & item = document["waypoints"][i];
        if (!item.is_object() || !item.contains("latitude") || !item.contains("longitude")) {
          NAV_LOG_ERROR( "Waypoint #%zu has invalid format", i + 1);
          return;
        }
        const mission_bridge::LatLon waypoint{
          item["latitude"].get<double>(), item["longitude"].get<double>()};
        if (!std::isfinite(waypoint.latitude) || !std::isfinite(waypoint.longitude) ||
          (waypoint.latitude == 0.0 && waypoint.longitude == 0.0) ||
          waypoint.latitude < -90.0 || waypoint.latitude > 90.0 ||
          waypoint.longitude < -180.0 || waypoint.longitude > 180.0)
        {
          NAV_LOG_ERROR( "Waypoint #%zu has invalid latitude/longitude", i + 1);
          return;
        }
        parsed.push_back(waypoint);
      }
      waypoints_ = std::move(parsed);
      waypoint_received_ = true;
      NAV_LOG_INFO( "Received %zu waypoint(s)", waypoints_.size());
      try_conversion();
    } catch (const std::exception & error) {
      NAV_LOG_ERROR( "Failed to parse waypoint message: %s", error.what());
    }
  }

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (datum_ready_) return;
    try {
      anchor_.datum_lat = msg->latitude;
      anchor_.datum_lon = msg->longitude;
      const auto converted = mission_bridge::lat_lon_to_utm(anchor_.datum_lat, anchor_.datum_lon);
      anchor_.datum_easting = converted.easting;
      anchor_.datum_northing = converted.northing;
      datum_ready_ = true;
      NAV_LOG_INFO( "Datum acquired: lat=%.10f lon=%.10f", anchor_.datum_lat, anchor_.datum_lon);
      try_conversion();
    } catch (const std::exception & error) {
      NAV_LOG_ERROR( "Error processing GPS message: %s", error.what());
    }
  }

  void try_conversion()
  {
    if (!datum_ready_ || waypoints_.empty()) return;
    try {
      const auto document = navigation_utils::make_waypoints_document(anchor_, waypoints_);
      navigation_utils::atomic_write_json(output_path_, document);
      std::ifstream stream(output_path_);
      const auto verify = nlohmann::json::parse(stream);
      if (!verify.is_object() || !verify.contains("waypoints") ||
        !verify["waypoints"].is_array() || !verify.contains("datum") || !verify["datum"].is_object())
      {
        NAV_LOG_ERROR( "Verification failed for %s", output_path_.c_str());
        return;
      }
      NAV_LOG_INFO( "Waypoints written to %s", output_path_.c_str());
      done_ = true;
    } catch (const std::exception & error) {
      NAV_LOG_ERROR( "Conversion failed: %s", error.what());
    }
  }

  navigation_utils::MapAnchor anchor_;
  std::filesystem::path output_path_;
  std::vector<mission_bridge::LatLon> waypoints_;
  bool waypoint_received_{false};
  bool datum_ready_{false};
  bool done_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("waypoint_transform");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<workspace_nav::WaypointTransformNode>());
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
