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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
    last_output_time_(this->now()),
    last_pose_time_(this->now()),
    last_imu_time_(this->now()),
    hold_engage_time_(this->now())
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

    // In-place rotation floor. On this hull a small |wz| pivot command makes
    // the boat drift backwards while turning; only a large enough yaw-rate
    // command produces a clean pivot. In-place rotation commands always carry
    // vx=0, so raise |wz| to at least min_in_place_angular_z then (never above
    // the hard angular caps). 0 disables the floor.
    min_in_place_angular_z_ =
      this->declare_parameter<double>("min_in_place_angular_z", 0.0);
    // Only commands with |wz| >= this are considered intentional pivots and
    // get boosted to the floor; smaller ones pass through untouched.
    in_place_floor_trigger_ =
      this->declare_parameter<double>("in_place_floor_trigger", 0.08);

    // In-place rotation position hold (P loop). ArduRover GUIDED
    // turn-rate-and-speed has no position loop, so the hull drifts while
    // pivoting. When the command is a pure rotation (|vx| ~ 0, |wz| large),
    // anchor the current position and add a small forward/backward correction
    // along body-x from the world-frame position error. Lateral (body-y)
    // correction is impossible on this hull; the rotating heading lets the
    // body-x correction cover the drift over time.
    pose_topic_ =
      this->declare_parameter<std::string>("pose_topic", "/mavros/local_position/odom");
    enable_in_place_hold_ =
      this->declare_parameter<bool>("enable_in_place_hold", false);
    hold_linear_deadband_ =
      this->declare_parameter<double>("hold_linear_deadband", 0.02);
    hold_angular_min_ =
      this->declare_parameter<double>("hold_angular_min", 0.05);
    hold_kp_ = this->declare_parameter<double>("hold_kp", 0.5);
    hold_max_correction_ =
      this->declare_parameter<double>("hold_max_correction", 0.3);
    hold_error_deadband_ =
      this->declare_parameter<double>("hold_error_deadband", 0.10);
    hold_max_error_ = this->declare_parameter<double>("hold_max_error", 2.0);
    pose_timeout_sec_ = this->declare_parameter<double>("pose_timeout_sec", 1.0);
    // Correction decay over a sustained rotation episode (anti limit-cycle).
    hold_decay_start_sec_ =
      this->declare_parameter<double>("hold_decay_start_sec", 5.0);
    hold_decay_full_sec_ =
      this->declare_parameter<double>("hold_decay_full_sec", 15.0);
    hold_min_correction_ =
      this->declare_parameter<double>("hold_min_correction", 0.05);

    // Yaw brake (rate damper). The hull keeps rotating 25-35 deg for ~10s
    // after the yaw-rate command returns to zero (measured 2026-10-05). When
    // the upstream command is idle (|vx|~0, |wz|~0) but the IMU still reports
    // rotation, actively counter-command wz = -kd*w_meas to kill the residual
    // spin. Never fights upstream: any real motion command disables it.
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    enable_yaw_brake_ = this->declare_parameter<bool>("enable_yaw_brake", false);
    yaw_brake_kd_ = this->declare_parameter<double>("yaw_brake_kd", 1.0);
    yaw_brake_max_ = this->declare_parameter<double>("yaw_brake_max", 0.2);
    yaw_brake_engage_rate_ = this->declare_parameter<double>("yaw_brake_engage_rate", 0.03);
    yaw_brake_release_rate_ =
      this->declare_parameter<double>("yaw_brake_release_rate", 0.02);
    yaw_brake_cmd_deadband_ =
      this->declare_parameter<double>("yaw_brake_cmd_deadband", 0.01);
    imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.5);

    // Conservative LUTs derived from the 100-point sea-trial dataset, updated
    // with the 2026-10-05 on-water bridge calibration (env_probe phase):
    // measured max forward speed is ~1.0 m/s with no PWM saturation.
    // Low-yaw-rate entries sit slightly above the measured edge so that after
    // the 0.90 safety factor the straight-line output limit is ~1.0 m/s
    // (1.11*0.9=0.999, requirement); mid/high wz entries stay conservative.
    // PWM/servo data were used as a calibration aid:
    //   common  = (SERVO1 + SERVO3) / 2  tracks longitudinal effort strongly;
    //   diff    = (SERVO1 - SERVO3) / 2  tracks yaw effort very strongly.
    // Many large-|omega| points had one propulsion channel at/near 800 or 2200 us,
    // showing actuator saturation. Therefore the LUT is intentionally inside the
    // raw Pareto edge, and a separate safety factor is applied at runtime.
    positive_envelope_ = {
      {0.00, 1.11},
      {0.05, 1.05},
      {0.10, 0.91},
      {0.15, 0.84},
      {0.20, 0.74},
      {0.25, 0.66},
      {0.30, 0.58},
      {0.35, 0.54},
    };

    negative_envelope_ = {
      {0.00, 1.11},
      {0.05, 0.95},
      {0.10, 0.88},
      {0.15, 0.78},
      {0.20, 0.68},
      {0.25, 0.60},
      {0.28, 0.50},
    };

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      state_topic_, 10, std::bind(&OffboardController::state_cb, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardController::pose_cb, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardController::imu_cb, this, std::placeholders::_1));

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
      "ardupilot_velocity_bridge ready, input=[%s] qos=%s output=%s envelope=%s safety=%.2f in_place_hold=%s yaw_brake=%s",
      topics_oss.str().c_str(), input_cmd_qos_.c_str(), output_cmd_topic_.c_str(),
      enable_velocity_envelope_ ? "ON" : "OFF", envelope_safety_factor_,
      enable_in_place_hold_ ? "ON" : "OFF",
      enable_yaw_brake_ ? "ON" : "OFF");
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    pose_yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_pose_time_ = this->now();
    has_pose_ = true;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_wz_ = msg->angular_velocity.z;
    last_imu_time_ = this->now();
    has_imu_ = true;
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

    // In-place rotation floor: boost small pivot commands up to the minimum
    // yaw rate, capped by the hard per-direction angular limits. Only kicks in
    // for commands above in_place_floor_trigger: smaller |wz| requests are
    // closed-loop fine adjustments (e.g. docking) and must pass through
    // unchanged, otherwise the floor turns them into bang-bang oscillation.
    if (min_in_place_angular_z_ > 0.0 && std::abs(cmd.linear.x) <= 1e-3) {
      const double cap = cmd.angular.z >= 0.0 ?
        max_positive_angular_z_ : max_negative_angular_z_;
      const double floor_mag = std::min(min_in_place_angular_z_, cap);
      if (std::abs(cmd.angular.z) >= in_place_floor_trigger_ &&
          std::abs(cmd.angular.z) < floor_mag) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 500,
          "In-place rotation floor: wz %.3f -> %.3f",
          cmd.angular.z, std::copysign(floor_mag, cmd.angular.z));
        cmd.angular.z = std::copysign(floor_mag, cmd.angular.z);
      }
    }

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

  void apply_in_place_hold(geometry_msgs::msg::Twist & target, bool command_fresh)
  {
    if (!enable_in_place_hold_) {
      return;
    }

    const bool pose_fresh = has_pose_ &&
      (this->now() - last_pose_time_).seconds() <= pose_timeout_sec_;

    const bool in_place = command_fresh &&
      std::abs(target.linear.x) <= hold_linear_deadband_ &&
      std::abs(target.angular.z) >= hold_angular_min_;

    if (!in_place || !pose_fresh) {
      if (hold_active_) {
        hold_active_ = false;
        RCLCPP_INFO(this->get_logger(), "In-place position hold released");
      }
      return;
    }

    if (!hold_active_) {
      hold_active_ = true;
      anchor_x_ = pose_x_;
      anchor_y_ = pose_y_;
      hold_engage_time_ = this->now();
      RCLCPP_INFO(
        this->get_logger(), "In-place position hold engaged at (%.2f, %.2f)",
        anchor_x_, anchor_y_);
      return;
    }

    double ex = anchor_x_ - pose_x_;
    double ey = anchor_y_ - pose_y_;
    const double mag = std::hypot(ex, ey);
    if (mag <= hold_error_deadband_) {
      return;
    }
    if (mag > hold_max_error_) {
      ex *= hold_max_error_ / mag;
      ey *= hold_max_error_ / mag;
    }

    // Project the world-frame (ENU) error onto body-x; only forward/backward
    // thrust is available on this hull.
    //
    // Correction magnitude decays with sustained-rotation time (2026-10-05
    // field finding): a body-x-only correction inside a continuously rotating
    // frame cannot converge — it forms a limit cycle (the boat spirals around
    // the anchor with radius correction/|wz|, measured 2 m at 0.30/0.15).
    // Short rotations (nav2 rotate-to-heading, a few seconds) keep the full
    // correction; long ones (docking search) fade to hold_min_correction so
    // the residual circle stays small (0.05/0.15 ≈ 0.33 m).
    const double episode = (this->now() - hold_engage_time_).seconds();
    double eff_max = hold_min_correction_;
    if (episode <= hold_decay_start_sec_) {
      eff_max = hold_max_correction_;
    } else if (episode < hold_decay_full_sec_) {
      const double t =
        (episode - hold_decay_start_sec_) /
        std::max(hold_decay_full_sec_ - hold_decay_start_sec_, 1e-6);
      eff_max = hold_max_correction_ +
        t * (hold_min_correction_ - hold_max_correction_);
    }

    const double ex_body = std::cos(pose_yaw_) * ex + std::sin(pose_yaw_) * ey;
    const double correction = clamp_value(hold_kp_ * ex_body, -eff_max, eff_max);
    target.linear.x += correction;

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "In-place hold: err %.2f m, body-x %.2f m, vx correction %.3f (max %.2f)",
      mag, ex_body, correction, eff_max);
  }

  void apply_yaw_brake(geometry_msgs::msg::Twist & target)
  {
    if (!enable_yaw_brake_) {
      return;
    }

    const bool imu_fresh = has_imu_ &&
      (this->now() - last_imu_time_).seconds() <= imu_timeout_sec_;

    // Only when upstream is idle: no translation and no rotation requested.
    const bool idle = std::abs(target.linear.x) <= hold_linear_deadband_ &&
      std::abs(target.angular.z) <= yaw_brake_cmd_deadband_;

    if (!idle || !imu_fresh) {
      if (brake_active_) {
        brake_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Yaw brake released");
      }
      return;
    }

    if (!brake_active_) {
      if (std::abs(imu_wz_) >= yaw_brake_engage_rate_) {
        brake_active_ = true;
        RCLCPP_INFO(
          this->get_logger(), "Yaw brake engaged at w_meas %.3f rad/s", imu_wz_);
      } else {
        return;
      }
    }

    if (std::abs(imu_wz_) < yaw_brake_release_rate_) {
      brake_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Yaw brake released (stopped)");
      return;
    }

    target.angular.z = clamp_value(
      -yaw_brake_kd_ * imu_wz_, -yaw_brake_max_, yaw_brake_max_);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "Yaw brake: w_meas %.3f -> wz %.3f", imu_wz_, target.angular.z);
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
    apply_in_place_hold(target, fresh);

    // Timeout / stop must win immediately. Do not slowly ramp through a stale-command stop.
    // The yaw brake still applies on the stale path: after the command stops it is
    // exactly when the residual spin needs killing.
    geometry_msgs::msg::Twist final_cmd;
    if (!fresh) {
      final_cmd = geometry_msgs::msg::Twist();
      apply_yaw_brake(final_cmd);
    } else {
      apply_yaw_brake(target);
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

  double min_in_place_angular_z_{0.0};
  double in_place_floor_trigger_{0.08};

  std::string imu_topic_;
  bool enable_yaw_brake_{false};
  double yaw_brake_kd_{1.0};
  double yaw_brake_max_{0.2};
  double yaw_brake_engage_rate_{0.03};
  double yaw_brake_release_rate_{0.02};
  double yaw_brake_cmd_deadband_{0.01};
  double imu_timeout_sec_{0.5};

  bool has_imu_{false};
  double imu_wz_{0.0};
  bool brake_active_{false};

  std::string pose_topic_;
  bool enable_in_place_hold_{false};
  double hold_linear_deadband_{0.02};
  double hold_angular_min_{0.05};
  double hold_kp_{0.5};
  double hold_max_correction_{0.3};
  double hold_error_deadband_{0.10};
  double hold_max_error_{2.0};
  double pose_timeout_sec_{1.0};
  double hold_decay_start_sec_{5.0};
  double hold_decay_full_sec_{15.0};
  double hold_min_correction_{0.05};

  bool has_pose_{false};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  bool hold_active_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};

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
  rclcpp::Time last_pose_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time hold_engage_time_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
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
