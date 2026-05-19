#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for bringing up the Nav2 navigation stack for the RoboBoat project.
#  It declares configurable launch arguments (map, params_file, use_sim_time, autostart, log_level)
#  and includes the upstream nav2_bringup bringup_launch.py with those arguments resolved.
#  Optionally starts RViz2 (default on) with Nav2 default RViz layout; closing RViz2 does not stop Nav2.
#  When use_mavros_odometry==true, merges nav2_mavros_odom_overlay.yaml so Nav2 subscribes to
#  /mavros/local_position/odom (PX4-fused position; typically includes GNSS).
# ----------------------------------------------------------------------------------------------- #

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _deep_merge(dst: dict, src: dict) -> dict:
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    nav2_config = PathJoinSubstitution([package_share, 'config', 'nav2_params.yaml'])
    map_config = PathJoinSubstitution([package_share, 'config', 'map.yaml'])

    map_topic = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_mavros_odometry = LaunchConfiguration('use_mavros_odometry')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    bringup_launch_path = os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_config,
        description='Full path to map yaml file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_config,
        description='Full path to the ROS2 parameters file'
    )

    declare_use_mavros_odometry_cmd = DeclareLaunchArgument(
        'use_mavros_odometry',
        default_value='false',
        description='Merge nav2_mavros_odom_overlay.yaml (~ /mavros/local_position/odom)',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for Nav2 nodes (e.g. debug, info, warn, error)'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='If true, start RViz2 alongside Nav2'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz2 config (.rviz); default is nav2_bringup nav2_default_view.rviz'
    )

    def nav2_bringup_with_optional_merge(context):
        base_path = os.path.abspath(params_file.perform(context))
        merge_mavros = use_mavros_odometry.perform(context).strip().lower() in (
            'true', '1', 'yes', 'on')
        if merge_mavros:
            with open(base_path, 'r', encoding='utf-8') as f:
                merged = yaml.safe_load(f)
            ovl_path = os.path.join(
                get_package_share_directory('workspace_nav'),
                'config',
                'nav2_mavros_odom_overlay.yaml',
            )
            with open(ovl_path, 'r', encoding='utf-8') as f:
                overlay = yaml.safe_load(f)
            _deep_merge(merged, overlay)
            tmp = tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', delete=False, encoding='utf-8')
            yaml.safe_dump(merged, tmp, sort_keys=False, allow_unicode=True)
            tmp.close()
            params_arg = TextSubstitution(text=tmp.name)
        else:
            params_arg = params_file

        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={
                'map': map_topic,
                'use_sim_time': use_sim_time,
                'params_file': params_arg,
                'autostart': autostart,
                'log_level': log_level
            }.items()
        )
        return [bringup_launch]

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_mavros_odometry_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(OpaqueFunction(function=nav2_bringup_with_optional_merge))
    ld.add_action(start_rviz_cmd)

    return ld
