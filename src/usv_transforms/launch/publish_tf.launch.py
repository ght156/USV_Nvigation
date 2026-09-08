#!/usr/bin/env python3
# ----------------------------------------------------------------------------- #
#  清污船整车 TF：加载 cleaning_boat.xacro，由 robot_state_publisher 发布
#  base_link → 各传感器 link 的静态 TF（URDF 内均为 fixed joint）。
#
#  用法：
#    ros2 launch usv_transforms cleaning_boat_tf.launch.py
#    ros2 launch usv_transforms cleaning_boat_tf.launch.py robot_description_file:=/path/to/other.xacro
# ----------------------------------------------------------------------------- #
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_urdf = os.path.join(
        get_package_share_directory('usv_transforms'), 'urdf', 'cleaning_boat.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('robot_description_file')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value=default_urdf,
            description='Path to the USV xacro/urdf used by robot_state_publisher',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
