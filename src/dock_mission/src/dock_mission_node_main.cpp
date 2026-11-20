#include "dock_mission/dock_mission_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

namespace
{
std::atomic<bool> stop_requested{false};
void request_stop(int) {stop_requested = true;}
}

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<dock_mission::DockMissionNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});
  while (rclcpp::ok() && !stop_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (rclcpp::ok()) node->shutdown();
  executor.cancel();
  if (executor_thread.joinable()) executor_thread.join();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
