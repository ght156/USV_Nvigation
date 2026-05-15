#!/usr/bin/env python3

"""Nav2 /cmd_vel_nav (controller raw) → MAVROS velocity setpoint with limits, deadband, timeout republish.
绕过 velocity_smoother，直接订阅 controller_server 原始输出 /cmd_vel_nav，
由本节点自行做限幅/死区/超时保护，避免 smoother OPEN_LOOP 引入额外延迟。
仿真 converter.py 同样订阅 /cmd_vel_nav，两边行为一致。"""

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
                'mavros_cmd_vel_dst': ps('mavros_cmd_vel_dst'),
                'min_linear_x': pf('min_linear_x'),
                'max_linear_x': pf('max_linear_x'),
                'min_angular_z': pf('min_angular_z'),
                'max_angular_z': pf('max_angular_z'),
                'forbid_reverse': pb('forbid_reverse'),
                'min_surge_for_turn': pf('min_surge_for_turn'),
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
            description='Nav2 controller_server raw output (bypass velocity_smoother)',
        ),
        DeclareLaunchArgument(
            'mavros_cmd_vel_dst',
            default_value='/mavros/setpoint_velocity/cmd_vel_unstamped',
        ),
        DeclareLaunchArgument('min_linear_x', default_value='0.0'),
        DeclareLaunchArgument('max_linear_x', default_value='1.5'),
        DeclareLaunchArgument('min_angular_z', default_value='-1.0'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument(
            'forbid_reverse',
            default_value='true',
            description='Force surge ≥0 before limits (differential boat with no astern)',
        ),
        DeclareLaunchArgument(
            'min_surge_for_turn',
            default_value='0.00',
            description='If >0, zero angular.z when |surge| below this after deadband (no inplace turn)',
        ),
        DeclareLaunchArgument('linear_deadband', default_value='0.03'),
        DeclareLaunchArgument('angular_deadband', default_value='0.03'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.3'),
        DeclareLaunchArgument('publish_hz', default_value='20.0'),
        DeclareLaunchArgument(
            'estop_topic',
            default_value='',
            description='Optional std_msgs/Bool; true forces zero Twist',
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
            default_value='false',
            description='If true, subscribe with BEST_EFFORT (match some Nav2 cmd_vel pubs)',
        ),
        DeclareLaunchArgument(
            'offboard_cmode_allowlist',
            default_value='',
            description='Comma list of CMODE ints (0x hex ok); empty = built-in defaults',
        ),
        OpaqueFunction(function=_setup),
    ])
