#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "workspace_nav/cloud_filter_core.hpp"
#include "workspace_nav/navigation_logging.hpp"

namespace workspace_nav
{
using namespace std::chrono_literals;

class LivoxCloudFilterNode : public rclcpp::Node
{
public:
  LivoxCloudFilterNode()
  : Node("livox_cloud_filter")
  {
    declare_parameter("input_topic", "/livox/lidar");
    declare_parameter("output_topic", "/livox/lidar_filtered");
    declare_parameter("mask_frame", "base_link");
    declare_parameter("mask_boxes", std::vector<double>{2.70, -1.20, -0.40, 3.70, 1.40, 0.65});
    mask_frame_ = get_parameter("mask_frame").as_string();
    boxes_ = cloud_filter::parse_mask_boxes(get_parameter("mask_boxes").as_double_array());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("output_topic").as_string(), 5);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS().keep_last(5),
      std::bind(&LivoxCloudFilterNode::cloud_callback, this, std::placeholders::_1));
    parameter_handle_ = add_on_set_parameters_callback(
      std::bind(&LivoxCloudFilterNode::set_parameters, this, std::placeholders::_1));
    NAV_LOG_INFO( "livox_cloud_filter started with %zu mask box(es)", boxes_.size());
  }

private:
  rcl_interfaces::msg::SetParametersResult set_parameters(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      if (param.get_name() != "mask_boxes") continue;
      try {
        boxes_ = cloud_filter::parse_mask_boxes(param.as_double_array());
        NAV_LOG_INFO( "mask_boxes updated to %zu box(es)", boxes_.size());
      } catch (const std::exception & error) {
        result.successful = false;
        result.reason = error.what();
      }
    }
    return result;
  }

  bool lock_transform(const std::string & cloud_frame)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(mask_frame_, cloud_frame, tf2::TimePointZero, 200ms);
      const auto & q = tf.transform.rotation;
      const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      const double x = q.x / norm;
      const double y = q.y / norm;
      const double z = q.z / norm;
      const double w = q.w / norm;
      cloud_filter::Transform3D transform;
      transform.rotation <<
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
      transform.translation = {
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
      transform_ = transform;
      NAV_LOG_INFO( "TF %s -> %s locked", cloud_frame.c_str(), mask_frame_.c_str());
      return true;
    } catch (const std::exception & error) {
      NAV_LOG_WARN_THROTTLE( 5000, "waiting for cloud TF: %s", error.what());
      return false;
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!transform_ && !lock_transform(msg->header.frame_id)) return;
    try {
      publisher_->publish(cloud_filter::filter_point_cloud(*msg, boxes_, *transform_));
    } catch (const std::exception & error) {
      NAV_LOG_ERROR_THROTTLE( 2000, "cloud filter failed: %s", error.what());
    }
  }

  std::string mask_frame_;
  std::vector<cloud_filter::MaskBox> boxes_;
  std::optional<cloud_filter::Transform3D> transform_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_handle_;
};

}  // namespace workspace_nav

int main(int argc, char ** argv)
{
  NAV_LOGGER_INIT("livox_cloud_filter");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<workspace_nav::LivoxCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
