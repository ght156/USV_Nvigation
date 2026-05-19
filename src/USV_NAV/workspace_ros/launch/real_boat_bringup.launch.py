#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# Real-boat stack starter（MAVROS 需另开终端先行启动）。
#
# localization_backend：
#   - robot_localization：引用 localization.launch.py（本仓未提供，勿用）
#   - mavros_odom（默认）：MAVROS 融合位姿 + gnss_odom_map_tf + static_transform_real_boat.yaml
# ----------------------------------------------------------------------------------------------- #

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    loc_launch = PathJoinSubstitution(
        [FindPackageShare('workspace_ros'), 'launch', 'localization.launch.py'])
    mavros_tf_launch = PathJoinSubstitution(
        [FindPackageShare('workspace_ros'), 'launch', 'real_boat_mavros_tf.launch.py'])
    nav2_cmd_vel_mavros_launch = PathJoinSubstitution(
        [FindPackageShare('workspace_ros'), 'launch', 'nav2_cmd_vel_mavros.launch.py'])

    ws_share = Path(get_package_share_directory('workspace_ros'))
    default_static_real_boat = str(ws_share / 'config' / 'static_transform_real_boat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2_cmd_vel_to_mavros = LaunchConfiguration('enable_nav2_cmd_vel_to_mavros')
    localization_backend = LaunchConfiguration('localization_backend')
    static_transform_file = LaunchConfiguration('static_transform_file')
    imu_src = LaunchConfiguration('imu_src')
    gps_src = LaunchConfiguration('gps_src')
    use_gnss_map_odom_tf = LaunchConfiguration('use_gnss_map_odom_tf')
    map_config_yaml = LaunchConfiguration('map_config_yaml')
    map_origin_ref_key = LaunchConfiguration('map_origin_ref_key')
    initialize_once = LaunchConfiguration('initialize_once')
    republish_hz = LaunchConfiguration('republish_hz')
    max_data_age_sec = LaunchConfiguration('max_data_age_sec')
    map_odom_yaw_deg = LaunchConfiguration('map_odom_yaw_deg')

    use_mavros_odom = EqualsSubstitution(localization_backend, 'mavros_odom')
    use_robot_loc = EqualsSubstitution(localization_backend, 'robot_localization')

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
            'localization_backend',
            default_value='mavros_odom',
            description='robot_localization | mavros_odom',
        ),
        DeclareLaunchArgument(
            'static_transform_file',
            default_value=default_static_real_boat,
            description='仅 mavros_odom：传给 real_boat_mavros_tf 的 YAML（默认 static_transform_real_boat.yaml）',
        ),
        DeclareLaunchArgument(
            'imu_src',
            default_value='/mavros/imu/data',
            description='模式 A (robot_localization) 的 IMU 源话题',
        ),
        DeclareLaunchArgument(
            'gps_src',
            default_value='/mavros/global_position/raw/fix',
            description='模式 A (robot_localization) 的 GPS NavSatFix 源话题',
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

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(loc_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'imu_src': imu_src,
                        'gps_src': gps_src,
                    }.items(),
                ),
            ],
            condition=IfCondition(use_robot_loc),
        ),

        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(mavros_tf_launch),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'static_transform_file': static_transform_file,
                        'use_gnss_map_odom_tf': use_gnss_map_odom_tf,
                        'map_config_yaml': map_config_yaml,
                        'map_origin_ref_key': map_origin_ref_key,
                        'initialize_once': initialize_once,
                        'republish_hz': republish_hz,
                        'max_data_age_sec': max_data_age_sec,
                        'map_odom_yaw_deg': map_odom_yaw_deg,
                    }.items(),
                ),
            ],
            condition=IfCondition(use_mavros_odom),
        ),
    ])
