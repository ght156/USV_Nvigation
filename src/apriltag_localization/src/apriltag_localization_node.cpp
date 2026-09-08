// Copyright (c) 2026 jzw
// SPDX-License-Identifier: Apache-2.0

#include "apriltag_localization/apriltag_localization_node.hpp"

#include <mlogger/mlogger.hpp>

namespace perception
{

AprilTagLocalizationNode::AprilTagLocalizationNode(const rclcpp::NodeOptions & options)
: Node(
    "apriltag_node",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  MLOGGER_NODE_REGISTER("logs", "perception", this->get_name());
  impl_ = std::make_unique<AprilTagLocalization>(this);
}

AprilTagLocalizationNode::~AprilTagLocalizationNode() = default;

}  // namespace perception
