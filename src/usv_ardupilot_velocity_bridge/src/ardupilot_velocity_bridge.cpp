#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace
{
[[maybe_unused]] double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}
}  // namespace

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : Node("ardupilot_velocity_bridge"),
    last_cmd_time_(this->now())
  {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/mavros/state");
    input_cmd_topic_ =
      this->declare_parameter<std::string>("input_cmd_topic", "/cmd_vel_nav");
    output_cmd_topic_ = this->declare_parameter<std::string>(
      "output_cmd_topic", "/mavros/setpoint_velocity/cmd_vel");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    command_timeout_sec_ = this->declare_parameter<double>("command_timeout_sec", 1.0);
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 1.5);
    max_linear_y_ = this->declare_parameter<double>("max_linear_y", 1.5);
    max_linear_z_ = this->declare_parameter<double>("max_linear_z", 0.5);
    max_angular_z_ = this->declare_parameter<double>("max_angular_z", 1.0);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      state_topic_, 10, std::bind(&OffboardController::state_cb, this, std::placeholders::_1));

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_topic_, 10, std::bind(&OffboardController::cmd_cb, this, std::placeholders::_1));

    // 用 MAVROS setpoint_velocity 的带时间戳话题 ~/cmd_vel：
    //   该回调只做一次 ENU→NED 变换，+z = 左转（正确）；
    //   而 ~/cmd_vel_unstamped 回调里对 angular.z 多取反一次，两次抵消后 +z = 右转
    //   （本 mavros fork 的坑）。因此这里发 TwistStamped，桥内不做任何转向反转。
    // MAVROS 侧订阅是 SensorDataQoS (best-effort)，发布端用同一 QoS 才能连通。
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      output_cmd_topic_, rclcpp::SensorDataQoS());

    const auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&OffboardController::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "ardupilot_velocity_bridge ready, input=%s output=%s",
      input_cmd_topic_.c_str(),
      output_cmd_topic_.c_str());
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_cmd_ = *msg;
    // 调试阶段：关闭全部限幅，原样转发。需要恢复限速时取消下面注释。
    // current_cmd_.linear.x = clamp(current_cmd_.linear.x, -max_linear_x_, max_linear_x_);
    // current_cmd_.linear.y = clamp(current_cmd_.linear.y, -max_linear_y_, max_linear_y_);
    // current_cmd_.linear.z = clamp(current_cmd_.linear.z, -max_linear_z_, max_linear_z_);
    // current_cmd_.angular.z = clamp(current_cmd_.angular.z, -max_angular_z_, max_angular_z_);
    last_cmd_time_ = this->now();

    if (!has_received_command_) {
      has_received_command_ = true;
      RCLCPP_INFO(this->get_logger(), "Received first velocity command");
    }

    stale_warned_ = false;
  }

  bool command_is_fresh() const
  {
    return has_received_command_ &&
           (this->now() - last_cmd_time_).seconds() <= command_timeout_sec_;
  }

  geometry_msgs::msg::Twist effective_command()
  {
    if (command_is_fresh()) {
      return current_cmd_;
    }

    if (has_received_command_ && !stale_warned_) {
      stale_warned_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "Velocity command timeout %.2fs exceeded, publishing zero command",
        command_timeout_sec_);
    }

    return geometry_msgs::msg::Twist();
  }

  void control_loop()
  {
    if (!current_state_.connected) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "Waiting for FCU connection");
      return;
    }

    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = this->now();
    out.twist = effective_command();
    cmd_pub_->publish(out);
  }

  std::string state_topic_;
  std::string input_cmd_topic_;
  std::string output_cmd_topic_;
  double publish_rate_hz_{10.0};
  double command_timeout_sec_{1.0};
  double max_linear_x_{1.5};
  double max_linear_y_{1.5};
  double max_linear_z_{0.5};
  double max_angular_z_{1.0};

  bool has_received_command_{false};
  bool stale_warned_{false};

  geometry_msgs::msg::Twist current_cmd_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
