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
from rcl_interfaces.srv import SetParameters
import rclpy
import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess
)

def generate_launch_description():
    
    # 获取你的包的路径
    package_name = 'apriltag_localization'
    package_dir = get_package_prefix(package_name)
    
    # 指定YAML配置文件的路径
    params_file_path = os.path.join(package_dir, 'config', 'detection_cfg.yml')
    print(f"Loading parameters from: {params_file_path}")
    # 启动节点并加载参数
    my_node_with_params = Node(
        package=package_name,
        executable='apriltag_localization_cpp', # 应在CMakeLists.txt或setup.py中定义
        name='apriltag_node', # 节点名，应与YAML文件中的顶层名称一致
        parameters=[params_file_path], # 加载YAML文件中的所有参数
        output='both'
    )
    
    return LaunchDescription([
        my_node_with_params
    ])