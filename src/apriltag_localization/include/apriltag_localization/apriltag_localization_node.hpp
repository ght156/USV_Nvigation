// Copyright (c) 2026 jzw
// SPDX-License-Identifier: Apache-2.0
//
// rclcpp_components 入口：可独立进程运行，也可加载到 ComposableNodeContainer。

#ifndef APRILTAG_LOCALIZATION_APRILTAG_LOCALIZATION_NODE_HPP_
#define APRILTAG_LOCALIZATION_APRILTAG_LOCALIZATION_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "apriltag_localization/visibility_control.hpp"
#include "apriltag_node.h"

namespace perception
{

class APRILTAG_LOCALIZATION_COMPONENTS_PUBLIC AprilTagLocalizationNode : public rclcpp::Node
{
public:
  explicit AprilTagLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AprilTagLocalizationNode() override;

private:
  std::unique_ptr<AprilTagLocalization> impl_;
};

}  // namespace perception

#endif  // APRILTAG_LOCALIZATION_APRILTAG_LOCALIZATION_NODE_HPP_
