#include "usv_docking/docking_motion_controller.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<usv_docking::DockingMotionController>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
