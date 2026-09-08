#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace
{

double clamp_value(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double interpolate_limit(
  const std::vector<std::pair<double, double>> & lut,
  double yaw_rate_abs)
{
  if (lut.empty()) {
    return 0.0;
  }

  if (yaw_rate_abs <= lut.front().first) {
    return lut.front().second;
  }
  if (yaw_rate_abs >= lut.back().first) {
    return lut.back().second;
  }

  for (std::size_t i = 1; i < lut.size(); ++i) {
    if (yaw_rate_abs <= lut[i].first) {
      const double x0 = lut[i - 1].first;
      const double y0 = lut[i - 1].second;
      const double x1 = lut[i].first;
      const double y1 = lut[i].second;
      const double t = (yaw_rate_abs - x0) / std::max(x1 - x0, 1e-9);
      return y0 + t * (y1 - y0);
    }
  }

  return lut.back().second;
}

}  // namespace

class OffboardController : public rclcpp::Node
{
public:
  OffboardController()
  : Node("ardupilot_velocity_bridge"),
    last_cmd_time_(this->now()),
    last_output_time_(this->now())
  {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/mavros/state");
    // Legacy single-topic parameter, kept for backward compatibility.
    input_cmd_topic_ =
      this->declare_parameter<std::string>("input_cmd_topic", "/cmd_vel_nav");
    // Any number of input topics; each feeds the same command state, latest wins.
    input_cmd_topics_ = this->declare_parameter<std::vector<std::string>>(
      "input_cmd_topics", {input_cmd_topic_});
    // "reliable" or "best_effort". DDS matches a best_effort subscription with
    // BOTH reliable and best_effort publishers (requested <= offered), so
    // best_effort accepts cmd_vel sources regardless of their QoS.
    input_cmd_qos_ =
      this->declare_parameter<std::string>("input_cmd_qos", "best_effort");
    output_cmd_topic_ = this->declare_parameter<std::string>(
      "output_cmd_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped");

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    command_timeout_sec_ = this->declare_parameter<double>("command_timeout_sec", 1.0);

    // Absolute limits. These are safety rails, not the 2-D boat envelope itself.
    max_linear_x_ = this->declare_parameter<double>("max_linear_x", 1.10);
    max_linear_y_ = this->declare_parameter<double>("max_linear_y", 0.0);
    max_linear_z_ = this->declare_parameter<double>("max_linear_z", 0.0);
    max_positive_angular_z_ =
      this->declare_parameter<double>("max_positive_angular_z", 0.35);
    max_negative_angular_z_ =
      this->declare_parameter<double>("max_negative_angular_z", 0.28);

    // Boat-specific feasibility envelope.
    enable_velocity_envelope_ =
      this->declare_parameter<bool>("enable_velocity_envelope", true);
    envelope_safety_factor_ =
      this->declare_parameter<double>("envelope_safety_factor", 0.90);
    min_turning_linear_speed_ =
      this->declare_parameter<double>("min_turning_linear_speed", 0.0);

    // Slew-rate limits. Set <= 0 to disable an individual limiter.
    max_linear_accel_ = this->declare_parameter<double>("max_linear_accel", 0.40);
    max_linear_decel_ = this->declare_parameter<double>("max_linear_decel", 0.70);
    max_angular_accel_ = this->declare_parameter<double>("max_angular_accel", 0.35);

    // Conservative LUTs derived from the 100-point sea-trial dataset.
    // PWM/servo data were used as a calibration aid:
    //   common  = (SERVO1 + SERVO3) / 2  tracks longitudinal effort strongly;
    //   diff    = (SERVO1 - SERVO3) / 2  tracks yaw effort very strongly.
    // Many large-|omega| points had one propulsion channel at/near 800 or 2200 us,
    // showing actuator saturation. Therefore the LUT is intentionally inside the
    // raw Pareto edge, and a separate safety factor is applied at runtime.
    positive_envelope_ = {
      {0.00, 1.10},
      {0.05, 1.04},
      {0.10, 0.91},
      {0.15, 0.84},
      {0.20, 0.74},
      {0.25, 0.65},
      {0.30, 0.60},
      {0.35, 0.54},
    };

    negative_envelope_ = {
      {0.00, 1.03},
      {0.05, 0.92},
      {0.10, 0.88},
      {0.15, 0.75},
      {0.20, 0.67},
      {0.25, 0.59},
      {0.28, 0.47},
    };

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      state_topic_, 10, std::bind(&OffboardController::state_cb, this, std::placeholders::_1));

    rclcpp::QoS input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    if (input_cmd_qos_ == "reliable") {
      input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    } else if (input_cmd_qos_ != "best_effort") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown input_cmd_qos '%s', falling back to best_effort",
        input_cmd_qos_.c_str());
    }

    for (const auto & topic : input_cmd_topics_) {
      cmd_subs_.push_back(
        this->create_subscription<geometry_msgs::msg::Twist>(
          topic, input_qos,
          std::bind(&OffboardController::cmd_cb, this, std::placeholders::_1)));
    }

    // Use MAVROS setpoint_velocity unstamped Twist topic; no timestamp needed.
    // MAVROS handles ENU -> NED.
    // Keep angular.z sign unchanged here: +z = left turn for this setup.
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_cmd_topic_, rclcpp::SensorDataQoS());

    const auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&OffboardController::control_loop, this));

    std::ostringstream topics_oss;
    for (std::size_t i = 0; i < input_cmd_topics_.size(); ++i) {
      if (i > 0) {
        topics_oss << ", ";
      }
      topics_oss << input_cmd_topics_[i];
    }

    RCLCPP_INFO(
      this->get_logger(),
      "ardupilot_velocity_bridge ready, input=[%s] qos=%s output=%s envelope=%s safety=%.2f",
      topics_oss.str().c_str(), input_cmd_qos_.c_str(), output_cmd_topic_.c_str(),
      enable_velocity_envelope_ ? "ON" : "OFF", envelope_safety_factor_);
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_cmd_ = *msg;
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

  geometry_msgs::msg::Twist requested_command()
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

  geometry_msgs::msg::Twist apply_static_limits(geometry_msgs::msg::Twist cmd)
  {
    // This boat is operated as forward + yaw in Nav2. Lateral / vertical motion are disabled.
    cmd.linear.y = clamp_value(cmd.linear.y, -max_linear_y_, max_linear_y_);
    cmd.linear.z = clamp_value(cmd.linear.z, -max_linear_z_, max_linear_z_);

    cmd.linear.x = clamp_value(cmd.linear.x, -max_linear_x_, max_linear_x_);
    cmd.angular.z = clamp_value(
      cmd.angular.z, -max_negative_angular_z_, max_positive_angular_z_);

    if (!enable_velocity_envelope_ || cmd.linear.x <= 0.0) {
      return cmd;
    }

    const double yaw_abs = std::abs(cmd.angular.z);
    const auto & lut = cmd.angular.z >= 0.0 ? positive_envelope_ : negative_envelope_;

    double vx_limit = interpolate_limit(lut, yaw_abs);
    vx_limit *= clamp_value(envelope_safety_factor_, 0.0, 1.0);

    // Optional floor only matters while turning. Default=0 means no artificial floor.
    if (yaw_abs > 1e-3 && min_turning_linear_speed_ > 0.0) {
      vx_limit = std::max(vx_limit, min_turning_linear_speed_);
    }

    if (cmd.linear.x > vx_limit) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Envelope limiting vx %.3f -> %.3f at wz %.3f",
        cmd.linear.x, vx_limit, cmd.angular.z);
      cmd.linear.x = vx_limit;
    }

    return cmd;
  }

  geometry_msgs::msg::Twist apply_slew_limits(
    const geometry_msgs::msg::Twist & target,
    double dt)
  {
    if (!has_previous_output_ || dt <= 0.0) {
      return target;
    }

    geometry_msgs::msg::Twist out = target;

    const double dv = target.linear.x - previous_output_.linear.x;
    if (dv >= 0.0 && max_linear_accel_ > 0.0) {
      const double max_dv = max_linear_accel_ * dt;
      out.linear.x = previous_output_.linear.x + clamp_value(dv, -max_dv, max_dv);
    } else if (dv < 0.0 && max_linear_decel_ > 0.0) {
      const double max_dv = max_linear_decel_ * dt;
      out.linear.x = previous_output_.linear.x + clamp_value(dv, -max_dv, max_dv);
    }

    if (max_angular_accel_ > 0.0) {
      const double dw = target.angular.z - previous_output_.angular.z;
      const double max_dw = max_angular_accel_ * dt;
      out.angular.z = previous_output_.angular.z + clamp_value(dw, -max_dw, max_dw);
    }

    // Do not slew y/z; they are normally zero and should remain safety-clamped.
    out.linear.y = target.linear.y;
    out.linear.z = target.linear.z;
    out.angular.x = target.angular.x;
    out.angular.y = target.angular.y;

    return out;
  }

  void control_loop()
  {
    if (!current_state_.connected) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "Waiting for FCU connection");
      return;
    }

    const auto now = this->now();
    const double dt = std::max((now - last_output_time_).seconds(), 0.0);
    last_output_time_ = now;

    const bool fresh = command_is_fresh();
    geometry_msgs::msg::Twist target = apply_static_limits(requested_command());

    // Timeout / stop must win immediately. Do not slowly ramp through a stale-command stop.
    geometry_msgs::msg::Twist final_cmd;
    if (!fresh) {
      final_cmd = geometry_msgs::msg::Twist();
    } else {
      final_cmd = apply_slew_limits(target, dt);
    }

    previous_output_ = final_cmd;
    has_previous_output_ = true;

    cmd_pub_->publish(final_cmd);
  }

  std::string state_topic_;
  std::string input_cmd_topic_;
  std::vector<std::string> input_cmd_topics_;
  std::string input_cmd_qos_;
  std::string output_cmd_topic_;

  double publish_rate_hz_{20.0};
  double command_timeout_sec_{1.0};

  double max_linear_x_{1.10};
  double max_linear_y_{0.0};
  double max_linear_z_{0.0};
  double max_positive_angular_z_{0.35};
  double max_negative_angular_z_{0.28};

  bool enable_velocity_envelope_{true};
  double envelope_safety_factor_{0.90};
  double min_turning_linear_speed_{0.0};

  double max_linear_accel_{0.40};
  double max_linear_decel_{0.70};
  double max_angular_accel_{0.35};

  std::vector<std::pair<double, double>> positive_envelope_;
  std::vector<std::pair<double, double>> negative_envelope_;

  bool has_received_command_{false};
  bool stale_warned_{false};
  bool has_previous_output_{false};

  geometry_msgs::msg::Twist current_cmd_;
  geometry_msgs::msg::Twist previous_output_;
  mavros_msgs::msg::State current_state_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_output_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
