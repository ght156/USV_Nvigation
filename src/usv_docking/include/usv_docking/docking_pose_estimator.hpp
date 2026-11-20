#ifndef USV_DOCKING__DOCKING_POSE_ESTIMATOR_HPP_
#define USV_DOCKING__DOCKING_POSE_ESTIMATOR_HPP_

#include "usv_docking/anchor_filter.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace usv_docking
{

class DockingPoseEstimator : public rclcpp::Node
{
public:
  explicit DockingPoseEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void reset_anchor(const std_msgs::msg::Bool::SharedPtr message);
  std::pair<std::optional<Pose2D>, double> lookup_2d(
    const std::string & target, const std::string & source, const rclcpp::Time & now);
  int visible_tag_count(const rclcpp::Time & now);
  void update();
  void publish(
    const rclcpp::Time & now, double age, bool visible, const std::string & source);

  std::string odom_frame_;
  std::string base_frame_;
  std::string dock_frame_;
  std::string dock_est_frame_;
  double tf_timeout_sec_{0.0};
  double odom_tf_timeout_sec_{3.0};
  bool broadcast_tf_{true};
  double tag_timeout_sec_{0.6};
  double prediction_timeout_sec_{70.0};
  double period_{0.05};
  AnchorParams anchor_params_;
  AnchorFilter anchor_filter_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tag_visible_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr anchor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr measurement_age_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<rclcpp::Time> last_vision_stamp_;
  std::string previous_source_{SRC_INVALID};
  std::optional<Pose2D> last_output_;
  rclcpp::TimerBase::SharedPtr timer_;
  int overrun_warned_{0};
};

}  // namespace usv_docking

#endif  // USV_DOCKING__DOCKING_POSE_ESTIMATOR_HPP_
