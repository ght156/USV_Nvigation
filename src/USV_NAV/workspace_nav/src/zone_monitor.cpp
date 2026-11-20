#include <m_common/msg/nav_safety_event.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "workspace_nav/navigation_logging.hpp"
#include "workspace_nav/navigation_utils.hpp"
#include "workspace_nav/zone_geometry.hpp"

namespace workspace_nav
{

class ZoneMonitorNode : public rclcpp::Node
{
public:
  ZoneMonitorNode()
  : Node("zone_monitor")
  {
    declare_parameter("enabled", true);
    declare_parameter("map_yaml_path", "");
    declare_parameter("map_datum_ref_key", "ref_gnss_10");
    declare_parameter("global_frame", "map");
    declare_parameter("odom_topic", "/mavros/gps_input/local");
    declare_parameter("nav_zones_topic", "/nav_zones/current");
    declare_parameter("safety_event_topic", "/mission_bridge/safety_event");
    declare_parameter("check_period_sec", 0.5);
    declare_parameter("violation_threshold", 5);
    enabled_ = get_parameter("enabled").as_bool();
    global_frame_ = get_parameter("global_frame").as_string();
    threshold_ = std::max(1, static_cast<int>(get_parameter("violation_threshold").as_int()));
    std::string map_path = get_parameter("map_yaml_path").as_string();
    if (map_path.empty()) map_path = navigation_utils::package_path("config/map.yaml").string();
    anchor_ = navigation_utils::load_map_anchor(map_path, get_parameter("map_datum_ref_key").as_string());

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    zones_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("nav_zones_topic").as_string(), latched,
      std::bind(&ZoneMonitorNode::zones_callback, this, std::placeholders::_1));
    safety_pub_ = create_publisher<m_common::msg::NavSafetyEvent>(
      get_parameter("safety_event_topic").as_string(), 1);
    const double period = std::max(0.1, get_parameter("check_period_sec").as_double());
    check_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&ZoneMonitorNode::check, this));
    NAV_LOG_INFO( "zone_monitor ready; standby until fences arrive");
  }

private:
  mission_bridge::Point to_map(double lon, double lat) const
  {
    return navigation_utils::to_map(anchor_, {lat, lon});
  }

  void start_pose_tracking()
  {
    if (!enabled_ || odom_sub_) return;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = msg;});
    NAV_LOG_INFO( "fences received; pose monitoring enabled");
  }

  void stop_pose_tracking()
  {
    odom_.reset();
    odom_sub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
  }

  void zones_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto data = nlohmann::json::parse(msg->data, nullptr, false);
    if (!data.is_object()) return;
    std::vector<mission_bridge::Fence> fences;
    if (data.contains("fences") && data["fences"].is_array()) {
      for (const auto & raw : data["fences"]) {
        try {
          mission_bridge::Fence fence;
          fence.fence_id = raw.value("fence_id", "");
          fence.type = raw.value("type", "");
          fence.shape = raw.value("shape", "polygon");
          for (const auto & point : raw.value("points", nlohmann::json::array())) {
            fence.points.push_back(to_map(point.at(0).get<double>(), point.at(1).get<double>()));
          }
          if (raw.contains("center") && !raw["center"].is_null()) {
            const auto & center = raw["center"];
            fence.center = to_map(center.at(0).get<double>(), center.at(1).get<double>());
          }
          fence.radius_m = raw.value("radius_m", 0.0);
          fences.push_back(std::move(fence));
        } catch (const std::exception &) {
        }
      }
    }
    fences_ = std::move(fences);
    violation_streak_ = 0;
    if (event_active_) publish_clear_event();
    if (enabled_ && !fences_.empty()) start_pose_tracking();
    else stop_pose_tracking();
  }

  std::optional<mission_bridge::Point> pose_in_map()
  {
    if (!odom_) return std::nullopt;
    const std::string source = odom_->header.frame_id.empty() ? "odom" : odom_->header.frame_id;
    const double x = odom_->pose.pose.position.x;
    const double y = odom_->pose.pose.position.y;
    if (source == global_frame_) return mission_bridge::Point{x, y};
    if (!tf_buffer_) return std::nullopt;
    try {
      const auto transform = tf_buffer_->lookupTransform(global_frame_, source, tf2::TimePointZero);
      const auto & t = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return mission_bridge::Point{
        t.x + std::cos(yaw) * x - std::sin(yaw) * y,
        t.y + std::sin(yaw) * x + std::cos(yaw) * y};
    } catch (const std::exception & error) {
      NAV_LOG_WARN_THROTTLE( 2000, "zone monitor TF unavailable: %s", error.what());
      return std::nullopt;
    }
  }

  void check()
  {
    if (!enabled_ || fences_.empty()) return;
    const auto pose = pose_in_map();
    if (!pose) return;
    const auto violation = mission_bridge::fence_violation_index(*pose, fences_);
    if (!violation) {
      violation_streak_ = 0;
      if (event_active_) publish_clear_event(*pose);
      return;
    }
    const auto & fence = fences_[*violation];
    const std::string transition = fence.type == "inclusion" ? "EXIT" : "ENTER";
    ++violation_streak_;
    if (violation_streak_ < threshold_ || event_active_) return;
    alarmed_violation_ = std::make_pair(fence, transition);
    publish_safety_event(fence, transition, true, *pose);
    event_active_ = true;
  }

  void publish_clear_event(std::optional<mission_bridge::Point> pose = std::nullopt)
  {
    if (alarmed_violation_) {
      publish_safety_event(alarmed_violation_->first, alarmed_violation_->second, false, pose);
    } else {
      mission_bridge::Fence empty;
      publish_safety_event(empty, "", false, pose);
    }
    event_active_ = false;
    alarmed_violation_.reset();
  }

  void publish_safety_event(
    const mission_bridge::Fence & fence, const std::string & transition,
    bool enabled, std::optional<mission_bridge::Point> pose)
  {
    m_common::msg::NavSafetyEvent msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;
    msg.severity = 2;
    msg.event_code = "GEOFENCE_VIOLATION";
    msg.enabled = enabled;
    msg.fence_id = fence.fence_id;
    msg.fence_type = fence.type;
    msg.transition = transition;
    msg.message = enabled ? "electronic geofence violation" : "geofence violation cleared";
    if (pose) {
      msg.distance_to_boundary_m = zone_geometry::distance_to_fence(*pose, fence);
      const double c = std::cos(anchor_.origin_yaw);
      const double s = std::sin(anchor_.origin_yaw);
      const double dx = pose->x - anchor_.origin_x;
      const double dy = pose->y - anchor_.origin_y;
      const double east = dx * c + dy * s;
      const double north = -dx * s + dy * c;
      msg.latitude = anchor_.datum_lat + north / 6378137.0 * 180.0 / M_PI;
      msg.longitude = anchor_.datum_lon + east /
        (6378137.0 * std::cos(anchor_.datum_lat * M_PI / 180.0)) * 180.0 / M_PI;
    }
    safety_pub_->publish(msg);
  }

  navigation_utils::MapAnchor anchor_;
  std::string global_frame_;
  bool enabled_{true};
  int threshold_{5};
  std::vector<mission_bridge::Fence> fences_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  int violation_streak_{0};
  bool event_active_{false};
  std::optional<std::pair<mission_bridge::Fence, std::string>> alarmed_violation_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr zones_sub_;
  rclcpp::Publisher<m_common::msg::NavSafetyEvent>::SharedPtr safety_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("zone_monitor");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<workspace_nav::ZoneMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
