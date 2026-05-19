#!/usr/bin/env python3

"""Nav2 速度 → MAVROS PositionTarget (/mavros/setpoint_raw/local).

Nav2 bringup remap：controller → /cmd_vel_nav；velocity_smoother → /cmd_vel。
默认 cmd_vel_src=/cmd_vel_nav（绕过 smoother）；可改为 /cmd_vel 使用平滑后指令。
固件自定义映射：velocity.x=线速度，velocity.y=偏航角速度。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    def ps(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    def pb(name: str) -> bool:
        return ps(name).lower() in ('true', '1', 'yes', 'on')

    def pf(name: str) -> float:
        return float(LaunchConfiguration(name).perform(context))

    return [
        Node(
            package='workspace_ros',
            executable='nav2_cmd_vel_to_mavros',
            name='nav2_cmd_vel_to_mavros',
            parameters=[{
                'use_sim_time': pb('use_sim_time'),
                'cmd_vel_src': ps('cmd_vel_src'),
                'mavros_raw_dst': ps('mavros_raw_dst'),
                'min_linear_x': pf('min_linear_x'),
                'max_linear_x': pf('max_linear_x'),
                'min_angular_z': pf('min_angular_z'),
                'max_angular_z': pf('max_angular_z'),
                'forbid_reverse': pb('forbid_reverse'),
                'linear_deadband': pf('linear_deadband'),
                'angular_deadband': pf('angular_deadband'),
                'cmd_timeout_sec': pf('cmd_timeout_sec'),
                'publish_hz': pf('publish_hz'),
                'estop_topic': ps('estop_topic'),
                'cmd_vel_sub_qos_best_effort': pb('cmd_vel_sub_qos_best_effort'),
                'mavros_state_src': ps('mavros_state_src'),
                'require_offboard_for_motion': pb('require_offboard_for_motion'),
                'offboard_cmode_allowlist': ps('offboard_cmode_allowlist'),
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Real boat: false',
        ),
        DeclareLaunchArgument(
            'cmd_vel_src',
            default_value='/cmd_vel_nav',
            description='Default /cmd_vel_nav (controller raw). Use /cmd_vel for velocity_smoother output.',
        ),
        DeclareLaunchArgument(
            'mavros_raw_dst',
            default_value='/mavros/setpoint_raw/local',
            description='MAVROS PositionTarget publish topic',
        ),
        DeclareLaunchArgument('min_linear_x', default_value='0.0'),
        DeclareLaunchArgument('max_linear_x', default_value='1.0'),
        DeclareLaunchArgument('min_angular_z', default_value='-1.0'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument(
            'forbid_reverse',
            default_value='true',
            description='Force surge ≥0 before limits (differential boat with no astern)',
        ),
        DeclareLaunchArgument('linear_deadband', default_value='0.03'),
        DeclareLaunchArgument('angular_deadband', default_value='0.01'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.3'),
        DeclareLaunchArgument('publish_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'estop_topic',
            default_value='',
            description='Optional std_msgs/Bool; true forces zero velocity (PositionTarget)',
        ),
        DeclareLaunchArgument(
            'mavros_state_src',
            default_value='/mavros/state',
            description='Empty disables State subscription (not recommended)',
        ),
        DeclareLaunchArgument(
            'require_offboard_for_motion',
            default_value='true',
            description='If true, forward non-zero cmd_vel only when connected and mode OFFBOARD',
        ),
        DeclareLaunchArgument(
            'cmd_vel_sub_qos_best_effort',
            default_value='true',
            description='Subscribe with BEST_EFFORT QoS (Nav2 /cmd_vel_nav is typically best_effort)',
        ),
        DeclareLaunchArgument(
            'offboard_cmode_allowlist',
            default_value='',
            description='Comma list of CMODE ints (0x hex ok); empty = built-in defaults',
        ),
        OpaqueFunction(function=_setup),
    ])
