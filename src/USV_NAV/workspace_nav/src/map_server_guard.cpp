#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "workspace_nav/navigation_logging.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;
}

int main(int argc, char ** argv)
{
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::string target = "/map_server";
  double start_delay = 20.0;
  double check_period = 3.0;
  double timeout = 90.0;
  for (std::size_t i = 1; i < arguments.size(); ++i) {
    const std::string arg = arguments[i];
    if (i + 1 >= arguments.size()) break;
    if (arg == "--node-name") target = arguments[++i];
    else if (arg == "--start-delay") start_delay = std::stod(arguments[++i]);
    else if (arg == "--check-period") check_period = std::stod(arguments[++i]);
    else if (arg == "--timeout") timeout = std::stod(arguments[++i]);
  }
  while (!target.empty() && target.back() == '/') target.pop_back();

  NAV_LOGGER_INIT("map_server_guard");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_server_guard");
  auto get_client = node->create_client<lifecycle_msgs::srv::GetState>(target + "/get_state");
  auto change_client = node->create_client<lifecycle_msgs::srv::ChangeState>(target + "/change_state");
  const auto get_state = [&]() -> std::optional<int> {
      auto future = get_client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
      if (rclcpp::spin_until_future_complete(node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        return std::nullopt;
      }
      return future.get()->current_state.id;
    };
  const auto change = [&](uint8_t transition) {
      auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
      request->transition.id = transition;
      auto future = change_client->async_send_request(request);
      return rclcpp::spin_until_future_complete(node, future, 20s) ==
             rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
    };

  std::this_thread::sleep_for(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, start_delay))));
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(std::max(1.0, timeout)));
  int previous_state = -1;
  int attempts = 0;
  int return_code = 0;
  const auto pause = [&]() {
      std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(check_period)));
    };

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    get_client->wait_for_service(1s);
    change_client->wait_for_service(1s);
    if (!get_client->service_is_ready() || !change_client->service_is_ready()) {
      pause();
      continue;
    }
    const auto state = get_state();
    if (!state) {
      pause();
      continue;
    }
    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      NAV_LOG_INFO( "%s is active; guard not needed", target.c_str());
      rclcpp::shutdown();
      return 0;
    }
    if (*state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
      *state != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      previous_state = *state;
      pause();
      continue;
    }
    if (*state != previous_state) {
      previous_state = *state;
      pause();
      continue;
    }

    NAV_LOG_ERROR( "%s is stuck in state=%d; activating", target.c_str(), *state);
    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      change(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }
    change(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ++attempts;
    pause();
    const auto updated = get_state();
    if (updated && *updated == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      NAV_LOG_INFO( "%s activated by guard", target.c_str());
      rclcpp::shutdown();
      return 0;
    }
    if (attempts >= 3) {
      return_code = 1;
      break;
    }
    previous_state = -1;
  }
  NAV_LOG_ERROR( "%s did not become active before timeout", target.c_str());
  rclcpp::shutdown();
  return return_code;
}
