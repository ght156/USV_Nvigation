#!/usr/bin/env python3
"""docking.launch.py — 归港四节点启动（实船版，旧 docking_controller 已于 2026-08-31 移除）。

用法：
    ros2 launch usv_docking docking.launch.py                      # 实船默认（use_sim_time=false）
    ros2 launch usv_docking docking.launch.py test_only:=false     # 接管 /cmd_vel_nav
    ros2 launch usv_docking docking.launch.py params_file:=/path/to/docking_real.yaml
    ros2 launch usv_docking docking.launch.py odom_topic:=/slam/localization  # 定位源切 SLAM

test_only=false 才会向 /cmd_vel_nav 发速度（真船/联调后期），
默认 true 只发 /docking/cmd_vel_test。
params_file 可指向实船标定后的整份配置；默认 config/docking.yaml
与代码 declare_parameter 默认值逐项一致（test_param_defaults.py 强制）。
odom_topic 为空时尊重 params_file / 代码默认（/mavros/gps_input/local）；
非空时覆盖 docking_fsm 的 odom_speed_topic（与导航侧 nav2_boat.launch.py
的 odom_topic 用法一致，建议两边传同一个值）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    test_only = LaunchConfiguration("test_only")
    params_file = LaunchConfiguration("params_file")

    # 定位源切换：空 = 尊重 params_file；非空 = 覆盖 docking_fsm.odom_speed_topic。
    # 只有 docking_fsm 声明了该参数，不能传给其余三个节点（undeclared 会启动失败）。
    odom_topic = LaunchConfiguration("odom_topic").perform(context).strip()

    common_params = {"use_sim_time": use_sim_time}

    fsm_params = [params_file, common_params]
    if odom_topic:
        fsm_params.append({"odom_speed_topic": odom_topic})

    return [
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
            parameters=fsm_params,
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


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_docking")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock（实船 false）",
            ),
            DeclareLaunchArgument(
                "test_only",
                default_value="false",
                description="true: 只发 /docking/cmd_vel_test；false: 发 /cmd_vel_nav",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "docking.yaml"),
                description="整份参数文件路径（实船标定后传自己的 yaml）",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="定位源里程计话题：空 = 尊重 params_file（默认 "
                            "/mavros/gps_input/local）；切 SLAM 传 /slam/localization",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
