#!/usr/bin/env python3

"""Launch usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge.

Nav2 /cmd_vel_nav (Twist) -> ardupilot_velocity_bridge ->
/mavros/setpoint_velocity/cmd_vel_unstamped (Twist).

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

    return [
        Node(
            package="usv_ardupilot_velocity_bridge",
            executable="ardupilot_velocity_bridge",
            name="ardupilot_velocity_bridge",
            output="screen",
            parameters=[{
                "state_topic": ps("state_topic"),
                "input_cmd_topic": ps("input_cmd_topic"),
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
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel_nav"),
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

        OpaqueFunction(function=_setup),
    ])
