#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for initializing the localization stack of the RoboBoat.
#  It fuses IMU, GPS, and odometry data using the robot_localization package
#  to provide a continuous state estimate for navigation and control.
#  The file also republishes sensor data with fixed covariances and sets up
#  static transforms required for consistent frame alignment across the system.
# ----------------------------------------------------------------------------------------------- #

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'workspace_ros'
    package_share = FindPackageShare(package_name)
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_path = PathJoinSubstitution([package_share, 'config', 'ekf.yaml'])
    navsat_path = PathJoinSubstitution([package_share, 'config', 'navsat.yaml'])
    static_transform_path = PathJoinSubstitution([package_share, 'config', 'static_transform.yaml'])

    imu_src = LaunchConfiguration('imu_src')
    gps_src = LaunchConfiguration('gps_src')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='false on real boat; true in Gazebo when /clock is published',
        ),
        DeclareLaunchArgument(
            'imu_src',
            default_value='/roboboat/sensors/imu/imu',
            description='IMU topic for covariance republish',
        ),
        DeclareLaunchArgument(
            'gps_src',
            default_value='/roboboat/sensors/gps/navsat',
            description='GPS NavSatFix topic for covariance republish',
        ),

        Node(
            package=package_name,
            executable='imu_covariance_repub',
            name='imu_covariance_repub',
            parameters=[{'use_sim_time': use_sim_time, 'imu_topic': imu_src}],
        ),

        Node(
            package=package_name,
            executable='gps_covariance_repub',
            name='gps_covariance_repub',
            parameters=[{'use_sim_time': use_sim_time, 'gps_topic': gps_src}],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/imu/fixed_cov'),
                ('gps/fix', '/gps/fixed_cov')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_path, {'use_sim_time': use_sim_time}]
        ),

        Node(
            package=package_name,
            executable='static_transform_publisher',
            name='static_transforms_publisher',
            parameters=[
                {'static_transform_file': static_transform_path},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
               '--x', '0.0',
               '--y', '0.0',
               '--z', '0.0',
               '--roll', '0.0',
               '--pitch', '0.0',
               '--yaw', '0.0',
               '--frame-id', 'map',
               '--child-frame-id', 'odom'
            ]
        )
    ])