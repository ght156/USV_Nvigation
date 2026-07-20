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

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_dir = get_package_prefix('apriltag_localization')
    profile = LaunchConfiguration('profile').perform(context)
    cfg_name = 'detection_cfg_sim.yml' if profile == 'sim' else 'detection_cfg.yml'
    params_file_path = os.path.join(package_dir, 'config', cfg_name)
    print(f'Loading apriltag parameters from: {params_file_path}')

    return [
        Node(
            package='apriltag_localization',
            executable='apriltag_localization_cpp',
            name='apriltag_node',
            parameters=[params_file_path],
            output='both',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile',
            default_value='sim',
            description='sim → detection_cfg_sim.yml；real → detection_cfg.yml',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
