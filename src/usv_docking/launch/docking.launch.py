#!/usr/bin/env python3
"""docking.launch.py — 归港四节点启动（实船版，旧 docking_controller 已于 2026-08-31 移除）。

用法：
    ros2 launch usv_docking docking.launch.py                      # 实船默认（use_sim_time=false）
    ros2 launch usv_docking docking.launch.py test_only:=false     # 接管 /cmd_vel_nav
    ros2 launch usv_docking docking.launch.py params_file:=/path/to/docking_real.yaml

test_only=false 才会向 /cmd_vel_nav 发速度（真船/联调后期），
默认 true 只发 /docking/cmd_vel_test。
params_file 可指向实船标定后的整份配置（如覆盖 odom_speed_topic 为
/mavros/gps_input/local、重校准速度/坞尺寸参数）；默认 config/docking.yaml
与代码 declare_parameter 默认值逐项一致（test_param_defaults.py 强制）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_docking")

    use_sim_time = LaunchConfiguration("use_sim_time")
    test_only = LaunchConfiguration("test_only")
    params_file = LaunchConfiguration("params_file")

    common_params = {"use_sim_time": use_sim_time}

    nodes = [
        Node(
            package="usv_docking",
            executable="docking_pose_estimator",
            name="docking_pose_estimator",
            output="screen",
            parameters=[params_file, common_params],
        ),
        Node(
            package="usv_docking",
            executable="docking_fsm",
            name="docking_fsm",
            output="screen",
            parameters=[params_file, common_params],
        ),
        Node(
            package="usv_docking",
            executable="docking_motion_controller",
            name="docking_motion_controller",
            output="screen",
            parameters=[params_file, common_params, {"test_only": test_only}],
        ),
        Node(
            package="usv_docking",
            executable="docking_safety",
            name="docking_safety",
            output="screen",
            parameters=[params_file, common_params],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock（实船 false）",
            ),
            DeclareLaunchArgument(
                "test_only",
                default_value="true",
                description="true: 只发 /docking/cmd_vel_test；false: 发 /cmd_vel_nav",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "docking.yaml"),
                description="整份参数文件路径（实船标定后传自己的 yaml）",
            ),
            *nodes,
        ]
    )
