#ifndef WORKSPACE_NAV__COSTMAP_REINFLATER_HPP_
#define WORKSPACE_NAV__COSTMAP_REINFLATER_HPP_

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace workspace_nav
{

class CostmapReinflater
{
public:
  CostmapReinflater(
    rclcpp::Node * node, std::string costmap_node = "/local_costmap/local_costmap",
    double delta = 0.01, double timeout_sec = 10.0);
  bool trigger(const std::string & why);
  bool busy();
  std::string last_result() const;

private:
  void expire();
  void fail(const std::string & message);
  void set_value(double value, bool restore);

  rclcpp::Node * node_;
  double delta_;
  double timeout_sec_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_client_;
  mutable std::mutex mutex_;
  bool busy_{false};
  double deadline_{0.0};
  double original_{0.0};
  std::string why_;
  std::string last_result_{"idle"};
};

}  // namespace workspace_nav

#endif  // WORKSPACE_NAV__COSTMAP_REINFLATER_HPP_
