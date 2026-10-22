#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 实船 Nav2：默认 nav2_params_xingfei.yaml + mission_stack.xingfei.yaml + app/map/map.yaml（兴飞船）。
# 地图统一从部署根 app 目录读：<USV_HOME>/app/map/map.yaml（船端 ~/usv-test/app/map/map.yaml）。
#
# 与官方 nav2_bringup/bringup_launch.py 的区别：
#   - 不启动 amcl。官方 bringup 会无条件 include localization_launch.py（map_server + amcl），
#     而实船定位（map→odom）由定位栈负责（gnss_odom_map_tf 或 SLAM slam_nav_bridge）；AMCL 是
#     第二套定位源，一旦拿到 scan/initial pose 就会抢同一段 map→odom TF。
#     这里复制官方结构后只保留 map_server + navigation_launch（controller/planner/bt/...）。
#   - 默认 enable_mission_bridge:=false（先不起 mission；需要时 enable_mission_bridge:=true
#     一并启动 mission_bridge + nav_status_aggregator）。
#
# 定位源切换（mavros ⇄ SLAM）：
#   odom_topic:=/slam/localization   # 接 SLAM（lightning / slam_nav_bridge）
#   odom_topic:=/mavros/gps_input/local 或留空  # 回 MAVROS（留空 = 尊重 params_file 里的值）
#   一个参数同时改写 Nav2 三处 odom_topic（bt_navigator / controller_server /
#   velocity_smoother）并透传给 mission 栈（mission_bridge / nav_status_aggregator /
#   zone_monitor）。归港栈同理：ros2 launch usv_docking docking.launch.py odom_topic:=...
# ----------------------------------------------------------------------------------------------- #

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _default_app_map_yaml() -> str:
    """统一地图来源：部署根 app/map/map.yaml（船端即 ~/usv-test/app/map）。

    部署根 USV_HOME 同时含 app/ 与 install/。优先用环境变量；systemd
    自启没带 USV_HOME 时，从本包安装位置逐级向上找 app/map/map.yaml。
    只有都没有时才回退到包内 config/map.yaml（开发机 / 仿真）。
    """
    candidates = []
    usv_home = os.environ.get('USV_HOME', '').strip()
    if usv_home:
        candidates.append(os.path.join(usv_home, 'app', 'map', 'map.yaml'))
    here = os.path.abspath(get_package_share_directory('workspace_nav'))
    while True:
        candidates.append(os.path.join(here, 'app', 'map', 'map.yaml'))
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    print(
        '[nav2_boat] app/map/map.yaml 未找到（USV_HOME=%r），回退包内 config/map.yaml'
        % usv_home, file=sys.stderr)
    return os.path.join(
        get_package_share_directory('workspace_nav'), 'config', 'map.yaml')


def _setup(context, *args, **kwargs):
    package_share = FindPackageShare('workspace_nav')

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_mission_bridge = LaunchConfiguration('enable_mission_bridge')
    mission_params_file = LaunchConfiguration('mission_params_file')
    enable_livox_cloud_filter = LaunchConfiguration('enable_livox_cloud_filter')

    # odom_topic：空 = 不注入（尊重 params_file）；非空 = 改写 Nav2 全部 odom_topic 并透传 mission 栈
    odom_topic = LaunchConfiguration('odom_topic').perform(context).strip()

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    navigation_launch_path = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    mission_bridge_launch = PathJoinSubstitution(
        [package_share, 'launch', 'mission_bridge.launch.py'])

    # 与 nav2_bringup/bringup_launch.py 相同的参数重写：yaml_filename 替换为 map:= 路径。
    # navigation_launch.py 会从磁盘重读 params_file 且只重写 use_sim_time/autostart，
    # 所以必须把这里 RewrittenYaml 的产物（临时文件）传给它，odom_topic 注入才会生效。
    param_rewrites = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_arg,
        # 2026-10-09：注入带 goal_checker_id 的 BT（默认 BT 会让 FollowPath 空
        # checker 立即 abort，船一步不动）。RewrittenYaml 只替换已存在的 key，
        # params_file 里须有 default_nav_to_pose_bt_xml 占位。
        'default_nav_to_pose_bt_xml': os.path.join(
            package_share.perform(context),
            'config', 'navigate_to_pose_w_goal_checker.xml'),
    }
    if odom_topic:
        param_rewrites['odom_topic'] = odom_topic
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_rewrites,
        convert_types=True)
    configured_params = ParameterFile(rewritten_params, allow_substs=True)

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

    # 只加载 map_server（不含 amcl）：定位由外部定位栈负责。
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
            'params_file': rewritten_params,
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
            # 兴飞：把 Nav2 实际加载的 params_file 传给 mission_bridge，
            # 使 waypoint_tolerance_m 自动读取与 Nav2 goal checker 同源
            # （否则回退到 nav2_params_chengfeng.yaml）。
            'nav2_params_path': params_file,
            'odom_topic': odom_topic,
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

    # 点云自用回波遮蔽：清扫杆/水雾区滤除后再喂给代价图。
    # 默认关闭；需要时显式传 enable_livox_cloud_filter:=true。
    livox_filter = Node(
        package='workspace_nav',
        executable='livox_cloud_filter',
        name='livox_cloud_filter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_livox_cloud_filter),
    )

    return [container, load_map_server, navigation, mission_bridge, rviz, livox_filter]


def generate_launch_description():
    package_share = FindPackageShare('workspace_nav')

    real_nav2_config = PathJoinSubstitution(
        [package_share, 'config', 'nav2_params_xingfei.yaml'])
    map_default = _default_app_map_yaml()
    mission_params_default = PathJoinSubstitution(
        [package_share, 'config', 'mission_stack.xingfei.yaml'])

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_rviz_config = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=map_default,
        description='默认 <USV_HOME>/app/map/map.yaml；可 map:=/绝对路径/其它.yaml 覆盖',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Real boat: false. Set true only if playing bags with /clock.',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=real_nav2_config,
        description='Nav2 params; default is nav2_params_xingfei.yaml (MAVROS odom).',
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
        description='mission 栈参数 YAML（mission_stack.xingfei.yaml）',
    )

    declare_enable_livox_cloud_filter = DeclareLaunchArgument(
        'enable_livox_cloud_filter',
        default_value='false',
        description='是否启动 Livox 自用回波过滤节点（默认关闭）',
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='',
        description='统一定位源里程计话题：空 = 尊重 params_file；'
                    '切 SLAM 用 /slam/localization，回 MAVROS 用 /mavros/gps_input/local。'
                    '同时改写 Nav2 三处 odom_topic 并透传 mission 栈。',
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
        declare_enable_livox_cloud_filter,
        declare_odom_topic,
        OpaqueFunction(function=_setup),
    ])
