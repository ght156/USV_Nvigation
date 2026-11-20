#include "workspace_nav/costmap_reinflater.hpp"
#include "workspace_nav/navigation_logging.hpp"

#include <chrono>
#include <cmath>

namespace workspace_nav
{

CostmapReinflater::CostmapReinflater(
  rclcpp::Node * node, std::string costmap_node, double delta, double timeout_sec)
: node_(node), delta_(std::abs(delta) > 0.0 ? std::abs(delta) : 0.01),
  timeout_sec_(std::max(1.0, timeout_sec))
{
  get_client_ = node_->create_client<rcl_interfaces::srv::GetParameters>(
    costmap_node + "/get_parameters");
  set_client_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
    costmap_node + "/set_parameters");
}

void CostmapReinflater::expire()
{
  if (busy_ && std::chrono::steady_clock::now().time_since_epoch().count() / 1e9 > deadline_) {
    busy_ = false;
    last_result_ = "timeout";
  }
}

bool CostmapReinflater::busy()
{
  std::lock_guard<std::mutex> lock(mutex_);
  expire();
  return busy_;
}

std::string CostmapReinflater::last_result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_result_;
}

void CostmapReinflater::fail(const std::string & message)
{
  busy_ = false;
  last_result_ = "fail: " + message;
  NAV_LOG_ERROR( "[costmap_reinflate] %s", message.c_str());
}

bool CostmapReinflater::trigger(const std::string & why)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    expire();
    if (busy_) return false;
    if (!get_client_->service_is_ready() || !set_client_->service_is_ready()) {
      last_result_ = "service_not_ready";
      return false;
    }
    busy_ = true;
    why_ = why;
    deadline_ = std::chrono::steady_clock::now().time_since_epoch().count() / 1e9 + timeout_sec_;
    last_result_ = "running";
  }
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"inflation_layer.inflation_radius"};
  get_client_->async_send_request(
    request, [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (response->values.empty() ||
          response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        {
          throw std::runtime_error("inflation_radius is not a double");
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          original_ = response->values[0].double_value;
        }
        set_value(response->values[0].double_value + delta_, false);
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex_);
        fail(error.what());
      }
    });
  return true;
}

void CostmapReinflater::set_value(double value, bool restore)
{
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter parameter;
  parameter.name = "inflation_layer.inflation_radius";
  parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  parameter.value.double_value = value;
  request->parameters = {parameter};
  set_client_->async_send_request(
    request, [this, restore](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
      bool success = false;
      std::string reason;
      try {
        const auto response = future.get();
        success = !response->results.empty() && response->results[0].successful;
        if (!response->results.empty()) reason = response->results[0].reason;
      } catch (const std::exception & error) {
        reason = error.what();
      }
      if (!success) {
        std::lock_guard<std::mutex> lock(mutex_);
        fail(reason);
        return;
      }
      if (!restore) {
        double original;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          original = original_;
        }
        set_value(original, true);
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
        last_result_ = "ok";
        NAV_LOG_INFO( "[costmap_reinflate] reinflation completed");
      }
    });
}

}  // namespace workspace_nav
