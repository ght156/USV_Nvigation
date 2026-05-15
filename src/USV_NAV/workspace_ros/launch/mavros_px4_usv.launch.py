#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 实船 MAVROS（PX4）：在官方插件表 + px4_config 之后加载本仓库覆盖层
# workspace_ros/config/mavros_px4_overrides_usv.yaml
# 以便发布  odom → base_link  TF，接在  map→odom  静态之后。
#
# 用法（例）：
#   ros2 launch workspace_ros mavros_px4_usv.launch.py fcu_url:=/dev/ttyACM0:57600
#
# 【重要】本 launch **只起 MAVROS**，不发布 **`map→odom`** TF。后者由实船链路里 **`real_boat_bringup`**
#（包含 `localization.launch.py` 或 `localization_backend:=mavros_odom` 时的 `real_boat_mavros_tf.launch.py`）发布；
# **仅单机测 MAVROS** 时若没有 bringup，可临时：
#   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
#
# 仍可通过参数 pluginlists_yaml / base_config_yaml 指回官方文件（默认随 humble 安装路径）。
# ----------------------------------------------------------------------------------------------- #

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavros = FindPackageShare('mavros')
    ws = FindPackageShare('workspace_ros')

    plugin_default = PathJoinSubstitution([mavros, 'launch', 'px4_pluginlists.yaml'])
    base_cfg_default = PathJoinSubstitution([mavros, 'launch', 'px4_config.yaml'])
    override_cfg = PathJoinSubstitution([ws, 'config', 'mavros_px4_overrides_usv.yaml'])

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    pluginlists_yaml = LaunchConfiguration('pluginlists_yaml')
    base_config_yaml = LaunchConfiguration('base_config_yaml')
    override_config_yaml = LaunchConfiguration('override_config_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:57600',
            description='MAVLink 串口或 udp/tcp URL',
        ),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument(
            'pluginlists_yaml',
            default_value=plugin_default,
            description='官方 px4_pluginlists.yaml 路径',
        ),
        DeclareLaunchArgument(
            'base_config_yaml',
            default_value=base_cfg_default,
            description='官方 px4_config.yaml 路径（勿删，仅在其后叠加覆盖）',
        ),
        DeclareLaunchArgument(
            'override_config_yaml',
            default_value=override_cfg,
            description='本仓库 USV 覆盖层',
        ),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            output='screen',
            parameters=[
                ParameterFile(pluginlists_yaml, allow_substs=True),
                ParameterFile(base_config_yaml, allow_substs=True),
                ParameterFile(override_config_yaml, allow_substs=True),
                {
                    'fcu_url': fcu_url,
                    'gcs_url': gcs_url,
                    'tgt_system': tgt_system,
                    'tgt_component': tgt_component,
                    'fcu_protocol': fcu_protocol,
                },
            ],
        ),
    ])
