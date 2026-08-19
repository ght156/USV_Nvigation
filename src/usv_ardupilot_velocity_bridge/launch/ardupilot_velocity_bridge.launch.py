#!/usr/bin/env python3

"""Launch usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge (ArduPilot MAVROS velocity bridge).

Nav2 /cmd_vel_nav (Twist) -> ardupilot_velocity_bridge ->
/mavros/setpoint_velocity/cmd_vel (TwistStamped，MAVROS 已正确做 ENU→NED，+z=左转)。
The node handles command timeout -> zero and velocity limits only.
Mode switching (GUIDED) and arming are intentionally NOT handled by this bridge.
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
                "max_angular_z": pf("max_angular_z"),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("input_cmd_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument(
            "output_cmd_topic",
            default_value="/mavros/setpoint_velocity/cmd_vel",
        ),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("command_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("max_linear_x", default_value="2.0"),
        DeclareLaunchArgument("max_linear_y", default_value="0.0"),
        DeclareLaunchArgument("max_linear_z", default_value="0.0"),
        DeclareLaunchArgument("max_angular_z", default_value="1.0"),
        OpaqueFunction(function=_setup),
    ])
