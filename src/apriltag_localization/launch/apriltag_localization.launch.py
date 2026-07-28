# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_name = 'apriltag_localization'
    package_prefix = get_package_prefix(package_name)
    package_share = get_package_share_directory(package_name)

    default_params = os.path.join(package_prefix, 'config', 'detection_cfg_sim.yml')
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = default_params

    rviz_config = os.path.join(package_share, 'rviz', 'apriltag_localization.rviz')

    apriltag_node = Node(
        package=package_name,
        executable='apriltag_localization_cpp',
        name='apriltag_node',
        parameters=[params_file],
        output='both',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='apriltag_rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return [apriltag_node, rviz_node]


def generate_launch_description():
    package_name = 'apriltag_localization'
    package_prefix = get_package_prefix(package_name)
    default_params = os.path.join(package_prefix, 'config', 'detection_cfg_sim.yml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to apriltag_localization YAML config',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Whether to launch RViz2 for TF/Image visualization',
        ),
        OpaqueFunction(function=launch_setup),
    ])
