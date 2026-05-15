#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  实船 MAVROS 相关 TF：静态 base_link→传感器，以及可选动态 map→odom（gnss_odom_map_tf）。
#  独立使用方式与旧名 real_boat_tf_static.launch.py 相同（该文件现为指向本文件的薄封装）。
#  - 默认加载 static_transform_real_boat.yaml（base_link→传感器 link）
#  - map→odom（二选一，勿同时开）：
#      * use_gnss_map_odom_tf:=true（默认）：gnss_odom_map_tf（可 initialize_once / republish_hz）
#      * use_gnss_map_odom_tf:=false：恒等静态 map→odom（旧行为）
# ----------------------------------------------------------------------------------------------- #
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'workspace_ros'


def generate_launch_description():
    share = Path(get_package_share_directory(PKG))
    static_yaml = LaunchConfiguration('static_transform_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss_tf = LaunchConfiguration('use_gnss_map_odom_tf')
    map_config_yaml = LaunchConfiguration('map_config_yaml')
    map_origin_ref_key = LaunchConfiguration('map_origin_ref_key')
    initialize_once = LaunchConfiguration('initialize_once')
    republish_hz = LaunchConfiguration('republish_hz')
    max_data_age_sec = LaunchConfiguration('max_data_age_sec')
    map_odom_yaw_deg = LaunchConfiguration('map_odom_yaw_deg')

    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare('workspace_nav'), 'config', 'map.yaml'])

    declare_static = DeclareLaunchArgument(
        'static_transform_file',
        default_value=str(share / 'config' / 'static_transform_real_boat.yaml'),
        description='实船传感器静态 TF（base_link→*_link）；仿真请用 localization.launch 的 static_transform.yaml',
    )
    declare_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='false on real vessel without Gazebo clock',
    )
    declare_gnss_tf = DeclareLaunchArgument(
        'use_gnss_map_odom_tf',
        default_value='true',
        description='true：gnss_odom_map_tf 发布 map→odom；false：恒等静态 map→odom',
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map_config_yaml',
        default_value=default_map_yaml,
        description='与 Nav2 map_server 相同的地图 YAML；从中解析 map_origin_ref_key 得锚点 [lon,lat]',
    )
    declare_ref_key = DeclareLaunchArgument(
        'map_origin_ref_key',
        default_value='ref_gnss_10',
        description='YAML 中对应 Nav2 map 原点 (0,0) 的角点键名（与制图 ref_gnss_* 一致）',
    )
    declare_init_once = DeclareLaunchArgument(
        'initialize_once',
        default_value='true',
        description='gnss_odom_map_tf：true 时首次有效 GNSS+odom 对后锁定 map→odom 并按 republish_hz 重发缓存',
    )
    declare_republish_hz = DeclareLaunchArgument(
        'republish_hz',
        default_value='20.0',
        description='initialize_once 为 true 时，锁定后重发 map→odom 的频率 (Hz)；≤0 则按 10Hz 重发',
    )
    declare_max_age = DeclareLaunchArgument(
        'max_data_age_sec',
        default_value='0.0',
        description='GNSS/Odom header 与 ROS 时钟最大允许时差（秒）；0=禁用（应对 MAVROS 时间戳漂移）',
    )
    declare_map_yaw = DeclareLaunchArgument(
        'map_odom_yaw_deg',
        default_value='0.0',
        description='map→odom：ENU 平移/姿态相对 Nav2 map 的固定绕 z 偏角（度）；常见 ±90。与 gnss_odom_map_tf 同源',
    )

    def map_odom_static_identity(context, *args, **kwargs):
        ydeg = float(context.perform_substitution(LaunchConfiguration('map_odom_yaw_deg')))
        yrad = str(math.radians(ydeg))
        return [
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
                    '--yaw', yrad,
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                ],
                condition=UnlessCondition(use_gnss_tf),
                output='screen',
            ),
        ]

    return LaunchDescription([
        declare_static,
        declare_time,
        declare_gnss_tf,
        declare_map_yaml,
        declare_ref_key,
        declare_init_once,
        declare_republish_hz,
        declare_max_age,
        declare_map_yaw,

        Node(
            package=PKG,
            executable='static_transform_publisher',
            name='static_transforms_publisher',
            parameters=[
                {'static_transform_file': static_yaml},
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package=PKG,
            executable='gnss_odom_map_tf',
            name='gnss_odom_map_tf',
            parameters=[
                {'use_sim_time': use_sim_time},
                {
                    'map_config_yaml': ParameterValue(map_config_yaml, value_type=str),
                    'map_origin_ref_key': ParameterValue(map_origin_ref_key, value_type=str),
                    'initialize_once': ParameterValue(initialize_once, value_type=bool),
                    'republish_hz': ParameterValue(republish_hz, value_type=float),
                    'max_data_age_sec': ParameterValue(max_data_age_sec, value_type=float),
                    'map_odom_yaw_deg': ParameterValue(map_odom_yaw_deg, value_type=float),
                },
            ],
            output='screen',
            condition=IfCondition(use_gnss_tf),
        ),

        OpaqueFunction(function=map_odom_static_identity),
    ])
