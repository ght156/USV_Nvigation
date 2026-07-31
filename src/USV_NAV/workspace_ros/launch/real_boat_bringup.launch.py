#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# Real-boat stack starter（MAVROS 由嵌软负责启动）。
#
# 启动内容：
#   - robot_state_publisher（从 m_common/urdf/usv_cf.xacro 发布传感器 TF）
#   - gnss_odom_map_tf（map→odom 动态 TF）
#   - 可选：nav2_cmd_vel_to_mavros 速度桥
# ----------------------------------------------------------------------------------------------- #

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavros_tf_launch = PathJoinSubstitution(
        [FindPackageShare('workspace_ros'), 'launch', 'real_boat_mavros_tf.launch.py'])
    nav2_cmd_vel_mavros_launch = PathJoinSubstitution(
        [FindPackageShare('workspace_ros'), 'launch', 'nav2_cmd_vel_mavros.launch.py'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2_cmd_vel_to_mavros = LaunchConfiguration('enable_nav2_cmd_vel_to_mavros')
    urdf_file = LaunchConfiguration('urdf_file')
    use_gnss_map_odom_tf = LaunchConfiguration('use_gnss_map_odom_tf')
    map_config_yaml = LaunchConfiguration('map_config_yaml')
    map_origin_ref_key = LaunchConfiguration('map_origin_ref_key')
    initialize_once = LaunchConfiguration('initialize_once')
    republish_hz = LaunchConfiguration('republish_hz')
    max_data_age_sec = LaunchConfiguration('max_data_age_sec')
    map_odom_yaw_deg = LaunchConfiguration('map_odom_yaw_deg')

    default_map_yaml = PathJoinSubstitution(
        [FindPackageShare('workspace_nav'), 'config', 'map.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Must be false when not using Gazebo /clock',
        ),
        DeclareLaunchArgument(
            'enable_nav2_cmd_vel_to_mavros',
            default_value='false',
            description='Nav2 cmd_vel bridge (default /cmd_vel_nav, bypass smoother) → MAVROS setpoint_raw/local',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('m_common'), 'urdf', 'usv_cf.xacro']),
            description='xacro/URDF 路径（默认 m_common 的实船 usv_cf.xacro）',
        ),
        DeclareLaunchArgument(
            'use_gnss_map_odom_tf',
            default_value='true',
            description='true：dynamic map→odom（GNSS vs MAVROS local odom）；false：恒等 map→odom',
        ),
        DeclareLaunchArgument(
            'map_config_yaml',
            default_value=default_map_yaml,
            description='与 Nav2 map_server 相同的 YAML；gnss_odom_map_tf 从中解析锚点 ref 键',
        ),
        DeclareLaunchArgument(
            'map_origin_ref_key',
            default_value='ref_gnss_10',
            description='map YAML 中与栅格原点 (0,0) 对应的 ref_gnss* 键名',
        ),
        DeclareLaunchArgument(
            'initialize_once',
            default_value='true',
            description='gnss_odom_map_tf：true 时首次有效 GNSS+odom 对后锁定 map→odom 并按 republish_hz 重发',
        ),
        DeclareLaunchArgument(
            'republish_hz',
            default_value='20.0',
            description='initialize_once 为 true 时锁定后重发 map→odom 的频率 (Hz)；≤0 则由节点按 10Hz 重发',
        ),
        DeclareLaunchArgument(
            'max_data_age_sec',
            default_value='0.0',
            description='gnss_odom_map_tf：0=不做 now 与消息头时间新鲜度过滤（推荐实船）；>0 时需时间同步正确',
        ),
        DeclareLaunchArgument(
            'map_odom_yaw_deg',
            default_value='0.0',
            description='map→odom 固定绕 z 偏角（度）；栅格与 ENU 差常 ±90 时可设 90 或 -90 试',
        ),

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_cmd_vel_mavros_launch),
                    launch_arguments={'use_sim_time': use_sim_time}.items(),
                ),
            ],
            condition=IfCondition(enable_nav2_cmd_vel_to_mavros),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mavros_tf_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'urdf_file': urdf_file,
                'use_gnss_map_odom_tf': use_gnss_map_odom_tf,
                'map_config_yaml': map_config_yaml,
                'map_origin_ref_key': map_origin_ref_key,
                'initialize_once': initialize_once,
                'republish_hz': republish_hz,
                'max_data_age_sec': max_data_age_sec,
                'map_odom_yaw_deg': map_odom_yaw_deg,
            }.items(),
        ),
    ])
