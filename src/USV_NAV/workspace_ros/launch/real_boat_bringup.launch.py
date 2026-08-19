#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# Real-boat stack starter（MAVROS 由嵌软负责启动）。
#
# 启动内容：
#   - robot_state_publisher（从 m_common/urdf/usv_cf.xacro 发布传感器 TF）
#   - gnss_odom_map_tf（map→odom 动态 TF）
#   - 可选：nav2_cmd_vel_to_mavros 速度桥（PX4 setpoint_raw/local）
#   - 可选：usv_ardupilot_velocity_bridge/ardupilot_velocity_bridge 速度桥（ArduPilot setpoint_velocity/cmd_vel，TwistStamped）
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
    ardupilot_velocity_bridge_launch = PathJoinSubstitution(
        [FindPackageShare('usv_ardupilot_velocity_bridge'),
         'launch', 'ardupilot_velocity_bridge.launch.py'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2_cmd_vel_to_mavros = LaunchConfiguration('enable_nav2_cmd_vel_to_mavros')
    enable_ardupilot_velocity_bridge = LaunchConfiguration('enable_ardupilot_velocity_bridge')
    ardupilot_input_cmd_topic = LaunchConfiguration('ardupilot_input_cmd_topic')
    ardupilot_publish_rate_hz = LaunchConfiguration('ardupilot_publish_rate_hz')
    ardupilot_max_linear_x = LaunchConfiguration('ardupilot_max_linear_x')
    ardupilot_max_angular_z = LaunchConfiguration('ardupilot_max_angular_z')
    urdf_file = LaunchConfiguration('urdf_file')
    use_gnss_map_odom_tf = LaunchConfiguration('use_gnss_map_odom_tf')
    map_config_yaml = LaunchConfiguration('map_config_yaml')
    map_origin_ref_key = LaunchConfiguration('map_origin_ref_key')
    initialize_once = LaunchConfiguration('initialize_once')
    republish_hz = LaunchConfiguration('republish_hz')
    max_data_age_sec = LaunchConfiguration('max_data_age_sec')
    map_odom_yaw_deg = LaunchConfiguration('map_odom_yaw_deg')
    local_odom_topic = LaunchConfiguration('local_odom_topic')
    global_topic = LaunchConfiguration('global_topic')

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
            'enable_ardupilot_velocity_bridge',
            default_value='true',
            description='ArduPilot velocity bridge: /cmd_vel_nav (Twist) → /mavros/setpoint_velocity/cmd_vel (TwistStamped)',
        ),
        DeclareLaunchArgument(
            'ardupilot_input_cmd_topic',
            default_value='/cmd_vel_nav',
            description='ardupilot_velocity_bridge input cmd_vel topic',
        ),
        DeclareLaunchArgument(
            'ardupilot_publish_rate_hz',
            default_value='20.0',
            description='ardupilot_velocity_bridge publish rate (Hz)',
        ),
        DeclareLaunchArgument(
            'ardupilot_max_linear_x',
            default_value='1.0',
            description='ardupilot_velocity_bridge forward speed limit (m/s)',
        ),
        DeclareLaunchArgument(
            'ardupilot_max_angular_z',
            default_value='1.0',
            description='ardupilot_velocity_bridge yaw rate limit (rad/s)',
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
        DeclareLaunchArgument(
            'local_odom_topic',
            default_value='/mavros/gps_input/local',
            description='gnss_odom_map_tf 的局部里程话题；实船用 gps_input/local（与 nav2 参数一致）',
        ),
        DeclareLaunchArgument(
            'global_topic',
            default_value='/mavros/gps_input/raw/fix',
            description='gnss_odom_map_tf 的 GNSS 全局位置话题；实船 RTK 经 gps_input 插件发布 raw/fix（global_position/global 无数据）',
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
                    PythonLaunchDescriptionSource(ardupilot_velocity_bridge_launch),
                    launch_arguments={
                        'input_cmd_topic': ardupilot_input_cmd_topic,
                        'publish_rate_hz': ardupilot_publish_rate_hz,
                        'max_linear_x': ardupilot_max_linear_x,
                        'max_angular_z': ardupilot_max_angular_z,
                    }.items(),
                ),
            ],
            condition=IfCondition(enable_ardupilot_velocity_bridge),
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
                'local_odom_topic': local_odom_topic,
                'global_topic': global_topic,
            }.items(),
        ),
    ])
