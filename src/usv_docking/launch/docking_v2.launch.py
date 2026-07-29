#!/usr/bin/env python3
"""docking_v2.launch.py — V2 归港四节点启动（不触碰旧 docking_controller）。

用法：
    ros2 launch usv_docking docking_v2.launch.py use_sim_time:=true
    ros2 launch usv_docking docking_v2.launch.py use_sim_time:=true test_only:=false

test_only=false 才会向 /cmd_vel_nav 发速度（真船/联调后期），
默认 true 只发 /docking_v2/cmd_vel_test。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_docking")
    params_file = os.path.join(pkg_share, "config", "docking_v2.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    test_only = LaunchConfiguration("test_only")

    common_params = {"use_sim_time": use_sim_time}

    nodes = [
        Node(
            package="usv_docking",
            executable="docking_pose_estimator_v2",
            name="docking_pose_estimator_v2",
            output="screen",
            parameters=[params_file, common_params],
        ),
        Node(
            package="usv_docking",
            executable="docking_fsm_v2",
            name="docking_fsm_v2",
            output="screen",
            parameters=[params_file, common_params],
        ),
        Node(
            package="usv_docking",
            executable="docking_motion_controller_v2",
            name="docking_motion_controller_v2",
            output="screen",
            parameters=[params_file, common_params, {"test_only": test_only}],
        ),
        Node(
            package="usv_docking",
            executable="docking_safety_v2",
            name="docking_safety_v2",
            output="screen",
            parameters=[params_file, common_params],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "test_only",
                default_value="true",
                description="true: 只发 /docking_v2/cmd_vel_test；false: 发 /cmd_vel_nav",
            ),
            *nodes,
        ]
    )
