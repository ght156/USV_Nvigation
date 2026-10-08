#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 免导航实时看图：只起 map_server + 局部代价图 + 位姿桥，不起导航栈。
#
# 用途：船端起全栈导航（nav2_boat.launch.py）会把图传带宽/CPU 打满导致断连。
#       本 launch 用于"只想在网页工具里看 /map + 局部代价图 + 船位"的场景：
#         ros2 launch workspace_nav map_view.launch.py
#
# 启动内容（3 个进程）：
#   1. nav2_container：map_server（发 /map，latched，一次）+ lifecycle_manager
#   2. nav2_costmap_2d 独立节点：跑局部代价图（/livox/lidar → 障碍标记），
#      话题 remap 到标准名 /local_costmap/costmap、/local_costmap/published_footprint 等。
#      注意：该可执行文件忽略 __node/__ns 改名（节点固定 /costmap/costmap），
#      所以参数文件里必须用双层 key costmap: costmap:（见 config/map_view_params.yaml），
#      lifecycle_manager 的 node_names 也写 'costmap/costmap'。
#   3. usv_map_pose_bridge.py（可选）：从 TF 查 map→base_link 发 /usv_map_pose 给网页端。
#
# 前提（本 launch 不管，需自行启动）：
#   - MAVROS：odom→base_link TF
#   - 定位栈：map→odom TF（usv_localization / SLAM 均可）
#   缺 TF 时代价图节点能激活但不发帧（rolling window 拿不到机器人位姿），属正常现象。
#
# 带宽预算（船端→主机，仅订阅时过 DDS）：
#   /map 一次 latch + 局部代价图 1Hz≈40KB/s + /usv_map_pose、footprint、/tf 小话题。
# ----------------------------------------------------------------------------------------------- #

from launch import LaunchDescription
import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _setup(context, *args, **kwargs):
    package_share = FindPackageShare('workspace_nav')

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    enable_pose_bridge = LaunchConfiguration('enable_pose_bridge')
    pose_bridge_script = LaunchConfiguration('pose_bridge_script').perform(context)

    # 与 nav2_boat.launch.py 相同：yaml_filename 替换为 map:= 路径
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

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[configured_params, {'autostart': autostart}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings,
        output='screen',
    )

    load_composables = LoadComposableNodes(
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
                name='lifecycle_manager_map_view',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['map_server', 'costmap/costmap']}]),
        ],
    )

    # 局部代价图：nav2_costmap_2d 独立可执行文件（节点名固定 /costmap/costmap，
    # __node/__ns 改名无效；话题 remap 有效，已实测）。
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        parameters=[configured_params],
        remappings=[
            ('/tf', 'tf'), ('/tf_static', 'tf_static'),
            ('/costmap/costmap', '/local_costmap/costmap'),
            ('/costmap/costmap_raw', '/local_costmap/costmap_raw'),
            ('/costmap/costmap_updates', '/local_costmap/costmap_updates'),
            ('/costmap/published_footprint', '/local_costmap/published_footprint'),
            ('/costmap/footprint', '/local_costmap/footprint'),
            # 调试话题（仅 publish_voxel_map:=true 时才有数据；默认关着省带宽）
            ('/costmap/voxel_marked_cloud', '/local_costmap/voxel_marked_cloud'),
            ('/costmap/voxel_grid', '/local_costmap/voxel_grid'),
            ('/costmap/clearing_endpoints', '/local_costmap/clearing_endpoints'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # 船位桥：map→base_link → /usv_map_pose（网页工具订阅）。脚本不在本机则跳过并告警
    # （主机与船端路径不同，可用 pose_bridge_script:= 覆盖）。
    if enable_pose_bridge.perform(context).strip().lower() in ('true', '1', 'yes') \
            and os.path.isfile(pose_bridge_script):
        pose_bridge = ExecuteProcess(
            cmd=['python3', pose_bridge_script],
            output='screen',
        )
    else:
        pose_bridge = LogInfo(
            msg=f'[map_view] 未启动位姿桥：脚本不存在 {pose_bridge_script}'
                '（用 pose_bridge_script:= 指定正确路径；不影响 /map 与代价图）')

    return [container, load_composables, costmap_node, pose_bridge]


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    map_default = PathJoinSubstitution([package_share, 'config', 'map.yaml'])
    params_default = PathJoinSubstitution([package_share, 'config', 'map_view_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_default,
            description='地图 yaml；可 map:=/绝对路径/其它.yaml 覆盖'),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_default,
            description='参数文件，默认 config/map_view_params.yaml'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Real boat: false. Set true only if playing bags with /clock.'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Auto-start lifecycle (map_server + costmap)'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'),
        DeclareLaunchArgument(
            'enable_pose_bridge',
            default_value='true',
            description='是否启动 usv_map_pose_bridge（/usv_map_pose 船位话题）'),
        DeclareLaunchArgument(
            'pose_bridge_script',
            default_value='/home/ght/ros_map_tool-main/usv_map_pose_bridge.py',
            description='usv_map_pose_bridge.py 路径（船端路径不同可用此参数覆盖）'),
        OpaqueFunction(function=_setup),
    ])
