// Copyright (c) 2026 jzw
// SPDX-License-Identifier: Apache-2.0
//
// 独立进程入口；与 rclcpp_components 中 perception::AprilTagLocalizationNode 共享实现。

#include <memory>

#include <mlogger/mlogger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "apriltag_localization/apriltag_localization_node.hpp"

int main(int argc, char * argv[])
{
  MLOGGER_MODULE_INIT("logs", "perception", "apriltag_localization");

  rclcpp::init(argc, argv);
  try {
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<perception::AprilTagLocalizationNode>(options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    MLOGGER_ERROR("apriltag_localization 异常退出: {}", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
