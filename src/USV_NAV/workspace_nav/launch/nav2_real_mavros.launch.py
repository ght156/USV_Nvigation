#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 实船 Nav2：默认 nav2_params_real_mavros.yaml + map.yaml。
#
# 与官方 nav2_bringup/bringup_launch.py 的区别：
#   - 不启动 amcl。官方 bringup 会无条件 include localization_launch.py（map_server + amcl），
#     而实船定位（map→odom）由 real_boat_bringup 的 gnss_odom_map_tf 负责；AMCL 是第二套
#     定位源，一旦拿到 scan/initial pose 就会和 gnss_odom_map_tf 抢同一段 map→odom TF。
#     这里复制官方结构后只保留 map_server + navigation_launch（controller/planner/bt/...）。
#   - 默认 enable_mission_bridge:=false（先不起 mission；需要时 enable_mission_bridge:=true
#     一并启动 mission_bridge + nav_status_aggregator）。
# ----------------------------------------------------------------------------------------------- #

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    real_nav2_config = PathJoinSubstitution(
        [package_share, 'config', 'nav2_params_real_mavros.yaml'])
    map_default = PathJoinSubstitution([package_share, 'config', 'map.yaml'])
    mission_bridge_launch = PathJoinSubstitution(
        [package_share, 'launch', 'mission_bridge.launch.py'])
    mission_params_default = PathJoinSubstitution(
        [package_share, 'config', 'mission_stack.real_boat.yaml'])

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_mission_bridge = LaunchConfiguration('enable_mission_bridge')
    mission_params_file = LaunchConfiguration('mission_params_file')
    mission_odom_topic = LaunchConfiguration('mission_odom_topic')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')
    navigation_launch_path = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    # 与 nav2_bringup/bringup_launch.py 相同的参数重写：yaml_filename 替换为 map:= 路径。
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_arg,
            },
            convert_types=True),
        allow_substs=True)

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

    declare_enable_mission_bridge = DeclareLaunchArgument(
        'enable_mission_bridge',
        default_value='false',
        description='为 true 时一并启动 mission_bridge + nav_status_aggregator（GCS/上层对接；实船默认开启）',
    )

    declare_mission_params_file = DeclareLaunchArgument(
        'mission_params_file',
        default_value=mission_params_default,
        description='mission 栈参数 YAML（mission_stack.real_boat.yaml）',
    )

    declare_mission_odom_topic = DeclareLaunchArgument(
        'mission_odom_topic',
        default_value='/mavros/gps_input/local',
        description='mission_bridge / nav_status_aggregator 共用 odom 话题',
    )

    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen',
    )

    # 只加载 map_server（不含 amcl）：定位由 gnss_odom_map_tf 负责。
    load_map_server = LoadComposableNodes(
        target_container='/nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['map_server']}]),
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'use_composition': 'True',
            'use_respawn': 'False',
            'container_name': 'nav2_container',
            'log_level': log_level,
        }.items(),
    )

    mission_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_bridge_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_yaml_path': map_arg,
            'mission_stack_params_file': mission_params_file,
            'odom_topic': mission_odom_topic,
        }.items(),
        condition=IfCondition(enable_mission_bridge),
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
        declare_enable_mission_bridge,
        declare_mission_params_file,
        declare_mission_odom_topic,
        container,
        load_map_server,
        navigation,
        mission_bridge,
        rviz,
    ])
