#!/usr/bin/env python3

"""Launch usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge.

Nav2 /cmd_vel_nav (Twist) -> ardupilot_velocity_bridge ->
/mavros/setpoint_velocity/cmd_vel_unstamped (Twist).

Input: one subscription per topic in input_cmd_topics (comma-separated list).
input_cmd_qos selects the subscription reliability:
  - best_effort (default): matches BOTH reliable and best_effort publishers,
    so any cmd_vel source works without touching the bridge.
  - reliable: strict, matches reliable publishers only.

The bridge now performs:
  1) command timeout protection,
  2) absolute velocity/yaw-rate limits,
  3) asymmetric vx-wz boat feasibility-envelope limiting,
  4) output acceleration / yaw-acceleration limiting.

The envelope was calibrated from sea-trial vx/wz data with RC/servo PWM used as an
actuator-saturation reference. ArduPilot still owns the low-level motor control;
this bridge does not directly command PWM.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    def ps(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    def pb(name: str) -> bool:
        return ps(name).lower() in ("true", "1", "yes", "on")

    def pf(name: str) -> float:
        return float(ps(name))

    def psl(name: str) -> list:
        return [t.strip() for t in ps(name).split(",") if t.strip()]

    return [
        Node(
            package="usv_ardupilot_velocity_bridge",
            executable="ardupilot_velocity_bridge",
            name="ardupilot_velocity_bridge",
            output="screen",
            parameters=[{
                "state_topic": ps("state_topic"),
                "input_cmd_topic": ps("input_cmd_topic"),
                "input_cmd_topics": psl("input_cmd_topics"),
                "input_cmd_qos": ps("input_cmd_qos"),
                "output_cmd_topic": ps("output_cmd_topic"),
                "publish_rate_hz": pf("publish_rate_hz"),
                "command_timeout_sec": pf("command_timeout_sec"),
                "max_linear_x": pf("max_linear_x"),
                "max_linear_y": pf("max_linear_y"),
                "max_linear_z": pf("max_linear_z"),
                "max_positive_angular_z": pf("max_positive_angular_z"),
                "max_negative_angular_z": pf("max_negative_angular_z"),
                "enable_velocity_envelope": pb("enable_velocity_envelope"),
                "envelope_safety_factor": pf("envelope_safety_factor"),
                "min_turning_linear_speed": pf("min_turning_linear_speed"),
                "max_linear_accel": pf("max_linear_accel"),
                "max_linear_decel": pf("max_linear_decel"),
                "max_angular_accel": pf("max_angular_accel"),
                "min_in_place_angular_z": pf("min_in_place_angular_z"),
                "in_place_floor_trigger": pf("in_place_floor_trigger"),
                "pose_topic": ps("pose_topic"),
                "enable_in_place_hold": pb("enable_in_place_hold"),
                "hold_linear_deadband": pf("hold_linear_deadband"),
                "hold_angular_min": pf("hold_angular_min"),
                "hold_kp": pf("hold_kp"),
                "hold_max_correction": pf("hold_max_correction"),
                "hold_error_deadband": pf("hold_error_deadband"),
                "hold_max_error": pf("hold_max_error"),
                "pose_timeout_sec": pf("pose_timeout_sec"),
                "hold_decay_start_sec": pf("hold_decay_start_sec"),
                "hold_decay_full_sec": pf("hold_decay_full_sec"),
                "hold_min_correction": pf("hold_min_correction"),
                "imu_topic": ps("imu_topic"),
                "enable_yaw_brake": pb("enable_yaw_brake"),
                "yaw_brake_kd": pf("yaw_brake_kd"),
                "yaw_brake_max": pf("yaw_brake_max"),
                "yaw_brake_engage_rate": pf("yaw_brake_engage_rate"),
                "yaw_brake_release_rate": pf("yaw_brake_release_rate"),
                "yaw_brake_cmd_deadband": pf("yaw_brake_cmd_deadband"),
                "imu_timeout_sec": pf("imu_timeout_sec"),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument(
            "input_cmd_topics", default_value="/cmd_vel_nav",
            description="Comma-separated list of input cmd_vel topics"),
        DeclareLaunchArgument(
            "input_cmd_qos", default_value="best_effort",
            description="reliable or best_effort; best_effort matches both"),
        DeclareLaunchArgument(
            "output_cmd_topic",
            default_value="/mavros/setpoint_velocity/cmd_vel_unstamped",
        ),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("command_timeout_sec", default_value="1.0"),

        # Hard safety rails.
        DeclareLaunchArgument("max_linear_x", default_value="1.10"),
        DeclareLaunchArgument("max_linear_y", default_value="0.0"),
        DeclareLaunchArgument("max_linear_z", default_value="0.0"),
        DeclareLaunchArgument("max_positive_angular_z", default_value="0.35"),
        DeclareLaunchArgument("max_negative_angular_z", default_value="0.28"),

        # Sea-trial velocity envelope.
        DeclareLaunchArgument("enable_velocity_envelope", default_value="true"),
        DeclareLaunchArgument("envelope_safety_factor", default_value="0.90"),
        DeclareLaunchArgument("min_turning_linear_speed", default_value="0.0"),

        # Slew-rate limits; <= 0 disables an individual limiter.
        DeclareLaunchArgument("max_linear_accel", default_value="0.40"),
        DeclareLaunchArgument("max_linear_decel", default_value="0.70"),
        DeclareLaunchArgument("max_angular_accel", default_value="0.35"),

        # In-place rotation yaw-rate floor; 0 disables.
        DeclareLaunchArgument("min_in_place_angular_z", default_value="0.0"), #原地旋转角速度下限
        # 微调豁免：|wz| 低于此值的指令视为闭环微调（如归港对角度），原样透传不抬升
        DeclareLaunchArgument("in_place_floor_trigger", default_value="0.08"),

        # In-place rotation position hold (ROS-side P loop on /mavros/local_position/odom).
        DeclareLaunchArgument("pose_topic", default_value="/mavros/local_position/odom"),
        DeclareLaunchArgument("enable_in_place_hold", default_value="false"), #纯旋转位置保持开关
        DeclareLaunchArgument("hold_linear_deadband", default_value="0.02"), #|vx|低于此值视为原地旋转
        DeclareLaunchArgument("hold_angular_min", default_value="0.05"), #|wz|高于此值才触发保持
        DeclareLaunchArgument("hold_kp", default_value="0.5"), #位置保持P增益(m/s每m误差)
        DeclareLaunchArgument("hold_max_correction", default_value="0.3"), #保持修正速度上限m/s
        DeclareLaunchArgument("hold_error_deadband", default_value="0.10"), #误差死区m
        DeclareLaunchArgument("hold_max_error", default_value="2.0"), #误差钳制m(防GPS跳变)
        DeclareLaunchArgument("pose_timeout_sec", default_value="1.0"), #位姿超时s
        # 修正随持续旋转时长衰减（防极限环画圈）
        DeclareLaunchArgument("hold_decay_start_sec", default_value="5.0"),
        DeclareLaunchArgument("hold_decay_full_sec", default_value="15.0"),
        DeclareLaunchArgument("hold_min_correction", default_value="0.05"),

        # Yaw brake（角速度阻尼刹车）：默认关，兴飞 launch 里开
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("enable_yaw_brake", default_value="false"), #旋转刹车开关
        DeclareLaunchArgument("yaw_brake_kd", default_value="1.0"), #阻尼增益 wz=-kd*w_meas
        DeclareLaunchArgument("yaw_brake_max", default_value="0.2"), #刹车角速度上限rad/s
        DeclareLaunchArgument("yaw_brake_engage_rate", default_value="0.03"), #残转超过此值触发
        DeclareLaunchArgument("yaw_brake_release_rate", default_value="0.02"), #低于此值退出
        DeclareLaunchArgument("yaw_brake_cmd_deadband", default_value="0.01"), #|wz指令|低于此值算空闲
        DeclareLaunchArgument("imu_timeout_sec", default_value="0.5"), #IMU超时s

        OpaqueFunction(function=_setup),
    ])
