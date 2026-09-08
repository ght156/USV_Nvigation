"""Simulate map→odom TF with fake GI320 GNSS around /home/zza/maps/map.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('usv_localization')
    default_params = os.path.join(pkg_share, 'config', 'usv_map_odom_tf.yaml')
    map_yaml = LaunchConfiguration('map_config_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_config_yaml',
            default_value='/home/zza/maps/map.yaml',
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='usv_localization',
            executable='sim_gi320_gnss.py',
            name='sim_gi320_gnss',
            output='screen',
            parameters=[{
                'map_config_yaml': map_yaml,
                'map_origin_ref_key': 'ref_gnss_10',
                'rate_hz': 10.0,
                'path_radius_m': 20.0,
                'speed_mps': 1.0,
                'start_east_m': 30.0,
                'start_north_m': 40.0,
                'fix_type': 4,
                'hdop': 0.8,
            }],
        ),
        Node(
            package='usv_localization',
            executable='usv_map_odom_tf_node',
            name='usv_map_odom_tf',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'map_config_yaml': map_yaml,
                    'map_origin_ref_key': 'ref_gnss_10',
                    'fc_local_odom_topic': '',
                    'publish_odom_tf': True,
                    'publish_odom_topic': True,
                    'republish_hz': 20.0,
                },
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-lc',
                        'echo "=== tf map -> odom ==="; '
                        'timeout 4 ros2 run tf2_ros tf2_echo map odom || true; '
                        'echo "=== tf odom -> base_link ==="; '
                        'timeout 4 ros2 run tf2_ros tf2_echo odom base_link || true',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
