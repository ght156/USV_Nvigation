#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 避障调试专用独立代价图（2026-10-22）：与主导航栈并存、互不影响。
#
# 用途：主栈 voxel_layer 联调期间默认关闭（水面 ghost 未调好），但避障参数要同步调。
#       本 launch 起一个独立的局部代价图节点，吃 livox_cloud_filter 遮蔽后的点云，
#       参数是 nav2_params_xingfei.yaml local_costmap 段的副本（debug_costmap_params.yaml），
#       话题全部 remap 到 /debug_costmap/*，不碰主栈任何东西。
#
# 启动内容（2 个进程）：
#   1. livox_cloud_filter：/livox/lidar → /livox/lidar_filtered（清扫杆/水雾遮蔽，
#      mask_boxes 支持在线改；这里固定启用，因为它就是要调的对象之一）
#   2. nav2_costmap_2d 独立节点 + 专属 lifecycle_manager（节点名固定 /costmap/costmap）
#
# 用法（船端）：
#   ros2 launch workspace_nav debug_costmap.launch.py
#   或本地一键：scripts/costmap_diag/debug_costmap.sh start|stop|status
#
# 网页看图：ros_map_tool_v3_nav2_live.html 连接后把"代价图话题"输入框改为
#   /debug_costmap/costmap（footprint 框改 /debug_costmap/published_footprint）
#
# 在线调参（立即生效的）：ros2 param set /costmap/costmap voxel_layer.mark_threshold 2
#   观测源参数（距离/高度窗口）改 debug_costmap_params.yaml 后需重启本 launch。
# 前提：主栈/定位（map→odom、odom→base_link TF）与 MAVROS 已在跑，否则 rolling
#   窗口拿不到位姿不发帧。
# ----------------------------------------------------------------------------------------------- #

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _setup(context, *args, **kwargs):
    package_share = FindPackageShare('workspace_nav')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True),
        allow_substs=True)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 点云遮蔽节点（CPU 已优化版；调试对象之一，这里固定启用）
    livox_filter = Node(
        package='workspace_nav',
        executable='livox_cloud_filter',
        name='livox_cloud_filter_debug',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # 独立代价图节点：节点名固定 /costmap/costmap，话题 remap 到 /debug_costmap/*
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        parameters=[configured_params],
        remappings=remappings + [
            ('/costmap/costmap', '/debug_costmap/costmap'),
            ('/costmap/costmap_raw', '/debug_costmap/costmap_raw'),
            ('/costmap/costmap_updates', '/debug_costmap/costmap_updates'),
            ('/costmap/published_footprint', '/debug_costmap/published_footprint'),
            ('/costmap/footprint', '/debug_costmap/footprint'),
            ('/costmap/voxel_marked_cloud', '/debug_costmap/voxel_marked_cloud'),
            ('/costmap/voxel_grid', '/debug_costmap/voxel_grid'),
            ('/costmap/clearing_endpoints', '/debug_costmap/clearing_endpoints'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
    )

    # 专属 lifecycle manager：只管这个调试代价图（名字与主栈的两个 manager 都不冲突）
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_debug_costmap',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['costmap/costmap']}],
        output='screen',
    )

    return [livox_filter, costmap_node, lifecycle_mgr]


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')
    params_default = PathJoinSubstitution(
        [package_share, 'config', 'debug_costmap_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_default,
            description='调试代价图参数文件，默认 config/debug_costmap_params.yaml'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Real boat: false. Set true only if playing bags with /clock.'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'),
        OpaqueFunction(function=_setup),
    ])
