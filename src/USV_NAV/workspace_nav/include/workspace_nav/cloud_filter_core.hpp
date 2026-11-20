#ifndef WORKSPACE_NAV__CLOUD_FILTER_CORE_HPP_
#define WORKSPACE_NAV__CLOUD_FILTER_CORE_HPP_

#include <Eigen/Core>

#include <array>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace workspace_nav::cloud_filter
{

struct MaskBox
{
  std::array<double, 6> bounds;
  bool contains(double x, double y, double z) const;
};

struct Transform3D
{
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};

  static Transform3D identity();
  Eigen::Vector3d apply(double x, double y, double z) const;
};

std::vector<MaskBox> parse_mask_boxes(const std::vector<double> & flat);
sensor_msgs::msg::PointCloud2 filter_point_cloud(
  const sensor_msgs::msg::PointCloud2 & input, const std::vector<MaskBox> & boxes,
  const Transform3D & transform);

}  // namespace workspace_nav::cloud_filter

#endif  // WORKSPACE_NAV__CLOUD_FILTER_CORE_HPP_
