"""Launch usv_map_odom_tf (map→odom) + robot static TF from usv_transforms/cleaning_boat.xacro."""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_usv_xacro() -> str:
    try:
        return os.path.join(
            get_package_share_directory('usv_transforms'), 'urdf', 'cleaning_boat.xacro'
        )
    except PackageNotFoundError:
        return os.path.join(
            os.environ.get('HOME', '/home/jetson'),
            'usv', 'src', 'usv_transforms', 'urdf', 'cleaning_boat.xacro',
        )


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_yaml = LaunchConfiguration('map_config_yaml').perform(context)
    publish_robot_tf = LaunchConfiguration('publish_robot_tf').perform(context)

    node_params = [params_file, {'use_sim_time': use_sim_time == 'true'}]
    if map_yaml.strip():
        node_params.append({'map_config_yaml': map_yaml})

    nodes = [
        Node(
            package='usv_localization',
            executable='usv_map_odom_tf_node',
            name='usv_map_odom_tf',
            output='screen',
            parameters=node_params,
        ),
    ]

    if publish_robot_tf == 'true':
        robot_description = ParameterValue(
            Command(['xacro ', LaunchConfiguration('robot_description_file')]),
            value_type=str,
        )
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time == 'true',
                }],
            )
        )

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('usv_localization')
    default_params = os.path.join(pkg, 'config', 'usv_map_odom_tf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'map_config_yaml',
            default_value=PathJoinSubstitution(
                [FindPackageShare('workspace_nav'), 'config', 'map.yaml']
            ),
            description='Path to Nav2 map.yaml with ref_gnss* (default: workspace_nav/config/map.yaml)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'publish_robot_tf',
            default_value='true',
            description='Publish base_link→sensor static TF via robot_state_publisher',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=_default_usv_xacro(),
            description='USV xacro (default: usv_transforms/urdf/cleaning_boat.xacro)',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
