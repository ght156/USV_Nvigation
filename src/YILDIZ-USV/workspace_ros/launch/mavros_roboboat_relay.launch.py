#!/usr/bin/env python3

"""Start mavros_roboboat_relay with typical MAVROS topic names."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_src = LaunchConfiguration('imu_src')
    gps_src = LaunchConfiguration('gps_src')
    scan_src = LaunchConfiguration('scan_src')
    enable_nav2_to_mavros_cmd_vel = LaunchConfiguration('enable_nav2_to_mavros_cmd_vel')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Real boat uses system clock (false)',
        ),
        DeclareLaunchArgument(
            'imu_src',
            default_value='/mavros/imu/data',
            description='MAVROS IMU subscription',
        ),
        DeclareLaunchArgument(
            'gps_src',
            default_value='/mavros/global_position/raw/fix',
            description='NavSatFix from MAVROS (raw GNSS)',
        ),
        DeclareLaunchArgument(
            'scan_src',
            default_value='',
            description='Optional LaserScan source (empty = disable)',
        ),
        DeclareLaunchArgument(
            'enable_nav2_to_mavros_cmd_vel',
            default_value='false',
            description='If true, relay /cmd_vel_nav to MAVROS setpoint_velocity',
        ),

        Node(
            package='workspace_ros',
            executable='mavros_roboboat_relay',
            name='mavros_roboboat_relay',
            parameters=[{
                'use_sim_time': use_sim_time,
                'imu_src': imu_src,
                'gps_src': gps_src,
                'scan_src': scan_src,
                'enable_nav2_to_mavros_cmd_vel': enable_nav2_to_mavros_cmd_vel,
            }],
            output='screen',
        ),
    ])
