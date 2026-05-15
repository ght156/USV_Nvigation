#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 实船专用 Nav2 启动：默认 nav2_params_real_mavros.yaml + config/map.yaml。
# 不改变 nav2.launch.py；仿真亦为 config/map.yaml + map/map.pgm。
# ----------------------------------------------------------------------------------------------- #

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    real_nav2_config = PathJoinSubstitution(
        [package_share, 'config', 'nav2_params_real_mavros.yaml'])
    map_default = PathJoinSubstitution([package_share, 'config', 'map.yaml'])

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    bringup_launch_path = os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_default,
        description='实船默认 map.yaml；可 map:=/绝对路径/其它.yaml 覆盖',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Real boat: false. Set true only if playing bags with /clock.',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=real_nav2_config,
        description='Nav2 params; default is nav2_params_real_mavros.yaml (MAVROS odom).',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Auto-start Nav2 lifecycle',
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nav2 logging level',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2',
    )

    declare_rviz_cfg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Optional RViz config path',
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'map': map_arg,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'log_level': log_level,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_params_file,
        declare_autostart,
        declare_log_level,
        declare_use_rviz,
        declare_rviz_cfg,
        bringup,
        rviz,
    ])
