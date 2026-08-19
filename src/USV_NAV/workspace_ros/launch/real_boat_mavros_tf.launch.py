#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  实船 MAVROS 相关 TF：robot_state_publisher 从 URDF/xacro 发布 base_link→传感器 静态 TF，
#  以及可选动态 map→odom（gnss_odom_map_tf）。
#  - 默认从 m_common 包加载 usv_cf.xacro（实船外参）
#  - map→odom（二选一，勿同时开）：
#      * use_gnss_map_odom_tf:=true（默认）：gnss_odom_map_tf（可 initialize_once / republish_hz）
#      * use_gnss_map_odom_tf:=false：恒等静态 map→odom（旧行为）
# ----------------------------------------------------------------------------------------------- #
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gnss_tf = LaunchConfiguration('use_gnss_map_odom_tf')
    map_config_yaml = LaunchConfiguration('map_config_yaml')
    map_origin_ref_key = LaunchConfiguration('map_origin_ref_key')
    initialize_once = LaunchConfiguration('initialize_once')
    republish_hz = LaunchConfiguration('republish_hz')
    max_data_age_sec = LaunchConfiguration('max_data_age_sec')
    map_odom_yaw_deg = LaunchConfiguration('map_odom_yaw_deg')
    urdf_file = LaunchConfiguration('urdf_file')
    local_odom_topic = LaunchConfiguration('local_odom_topic')
    global_topic = LaunchConfiguration('global_topic')

    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare('workspace_nav'), 'config', 'map_real_boat_hk.yaml'])

    declare_urdf = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('m_common'), 'urdf', 'usv_cf.xacro']),
        description='xacro/URDF 路径（默认 m_common 的实船 usv_cf.xacro）',
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
    declare_local_odom_topic = DeclareLaunchArgument(
        'local_odom_topic',
        default_value='/mavros/gps_input/local',
        description='gnss_odom_map_tf 的局部里程话题；实船 ArduPilot/MAVROS 用 gps_input/local（与 nav2 参数一致）',
    )
    declare_global_topic = DeclareLaunchArgument(
        'global_topic',
        default_value='/mavros/gps_input/raw/fix',
        description='gnss_odom_map_tf 的 GNSS 全局位置话题；实船 RTK 经 gps_input 插件发布 raw/fix（global_position/global 无数据）',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
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
        declare_urdf,
        declare_time,
        declare_gnss_tf,
        declare_map_yaml,
        declare_ref_key,
        declare_init_once,
        declare_republish_hz,
        declare_max_age,
        declare_map_yaw,
        declare_local_odom_topic,
        declare_global_topic,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='workspace_ros',
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
                    'local_odom_topic': ParameterValue(local_odom_topic, value_type=str),
                    'global_topic': ParameterValue(global_topic, value_type=str),
                },
            ],
            output='screen',
            condition=IfCondition(use_gnss_tf),
        ),

        OpaqueFunction(function=map_odom_static_identity),
    ])
