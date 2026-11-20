#include <action_msgs/msg/goal_status.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "workspace_nav/navigation_logging.hpp"
#include "workspace_nav/navigation_utils.hpp"

namespace workspace_nav
{
using namespace std::chrono_literals;

class WaypointWithStateNode : public rclcpp::Node
{
public:
  WaypointWithStateNode()
  : Node("waypoint_with_state")
  {
    declare_parameter("odom_topic", "/mavros/local_position/odom");
    if (const char * path = std::getenv("WAYPOINT_FILE_PATH"); path && *path) {
      waypoint_file_ = std::filesystem::absolute(path);
    } else {
      waypoint_file_ = navigation_utils::package_path("json/waypoints.json");
    }
    waypoint_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      this, "follow_waypoints");
    file_check_timer_ = create_wall_timer(1s, std::bind(&WaypointWithStateNode::check_file, this));
    NAV_LOG_INFO( "Using waypoint file: %s", waypoint_file_.c_str());
  }

private:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  struct Waypoint {double x; double y;};

  void check_file()
  {
    if (loaded_) {
      file_check_timer_->cancel();
      return;
    }
    if (!std::filesystem::exists(waypoint_file_) ||
      std::filesystem::file_size(waypoint_file_) == 0) return;
    auto points = load_waypoints();
    if (points.empty()) return;
    waypoints_ = std::move(points);
    loaded_ = true;
    file_check_timer_->cancel();
    NAV_LOG_INFO( "Loaded %zu waypoint(s); starting navigation", waypoints_.size());
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
      });
    reset_send_timer(2.0);
  }

  std::vector<Waypoint> load_waypoints() const
  {
    std::vector<Waypoint> points;
    try {
      std::ifstream stream(waypoint_file_);
      const auto document = nlohmann::json::parse(stream);
      const nlohmann::json * list = nullptr;
      if (document.is_array()) {
        list = &document;
      } else if (document.is_object()) {
        if (document.contains("waypoints") && document["waypoints"].is_array()) {
          list = &document["waypoints"];
        } else {
          for (const auto & item : document.items()) {
            if (item.value().is_array()) {
              list = &item.value();
              break;
            }
          }
        }
      }
      if (!list) return points;
      for (const auto & item : *list) {
        if (item.is_array() && item.size() >= 2) {
          points.push_back({item[0].get<double>(), item[1].get<double>()});
        } else if (item.is_object() && item.contains("x") && item.contains("y")) {
          points.push_back({item["x"].get<double>(), item["y"].get<double>()});
        }
      }
    } catch (const std::exception & error) {
      NAV_LOG_WARN( "Failed to load waypoint file: %s", error.what());
    }
    return points;
  }

  geometry_msgs::msg::PoseStamped create_pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void reset_send_timer(double seconds)
  {
    if (send_timer_) send_timer_->cancel();
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds)),
      std::bind(&WaypointWithStateNode::send_next, this));
  }

  void send_next()
  {
    if (navigating_ || current_index_ >= waypoints_.size()) return;
    double robot_x;
    double robot_y;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      robot_x = current_x_;
      robot_y = current_y_;
    }
    const auto waypoint = waypoints_[current_index_];
    if (std::hypot(waypoint.x - robot_x, waypoint.y - robot_y) <= 1.5) {
      NAV_LOG_INFO( "Waypoint %zu within tolerance; skipping", current_index_ + 1);
      ++current_index_;
      reset_send_timer(0.5);
      return;
    }
    FollowWaypoints::Goal goal;
    goal.poses = {create_pose(waypoint.x, waypoint.y)};
    navigating_ = true;
    if (!waypoint_client_->wait_for_action_server(5s)) {
      NAV_LOG_WARN( "FollowWaypoints server unavailable");
      navigating_ = false;
      reset_send_timer(2.0);
      return;
    }
    NAV_LOG_INFO( "Sending waypoint %zu/%zu", current_index_ + 1, waypoints_.size());
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          NAV_LOG_WARN( "Waypoint goal rejected");
          navigating_ = false;
          reset_send_timer(2.0);
          return;
        }
        NAV_LOG_INFO( "Waypoint goal accepted");
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        navigating_ = false;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          ++current_index_;
          NAV_LOG_INFO( "Waypoint %zu reached", current_index_);
          if (current_index_ >= waypoints_.size()) {
            clear_file();
            if (send_timer_) send_timer_->cancel();
            return;
          }
        }
        reset_send_timer(1.0);
      };
    waypoint_client_->async_send_goal(goal, options);
  }

  void clear_file()
  {
    try {
      std::filesystem::create_directories(waypoint_file_.parent_path());
      std::ofstream stream(waypoint_file_, std::ios::trunc);
      stream << "{}";
      NAV_LOG_INFO( "All waypoints completed; waypoint file cleared");
    } catch (const std::exception & error) {
      NAV_LOG_ERROR( "Failed clearing waypoint file: %s", error.what());
    }
  }

  std::filesystem::path waypoint_file_;
  std::vector<Waypoint> waypoints_;
  std::size_t current_index_{0};
  bool loaded_{false};
  bool navigating_{false};
  double current_x_{0.0};
  double current_y_{0.0};
  std::mutex pose_mutex_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr file_check_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("waypoint_with_state");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<workspace_nav::WaypointWithStateNode>());
  rclcpp::shutdown();
  return 0;
}
