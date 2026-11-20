#!/usr/bin/env python3
"""C++ 精靠泊：估计器独立进程，FSM/控制器/安全监督共享组件容器。

用法：
    ros2 launch usv_docking docking.launch.py
    ros2 launch usv_docking docking.launch.py test_only:=true
    ros2 launch usv_docking docking.launch.py params_file:=/path/to/docking_real.yaml
    ros2 launch usv_docking docking.launch.py odom_topic:=/slam/localization

test_only 默认 false（真实接管 /cmd_vel_nav）；代码裸跑默认 true 的安全联锁不变。
切定位源时 dock_mission.launch.py 必须传同一个 odom_topic。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def _setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    test_only = LaunchConfiguration("test_only")
    params_file = LaunchConfiguration("params_file")
    odom_topic = LaunchConfiguration("odom_topic").perform(context).strip()
    common_params = {"use_sim_time": use_sim_time}
    fsm_params = [params_file, common_params]
    if odom_topic:
        fsm_params.append({"odom_speed_topic": odom_topic})

    estimator = Node(
        package="usv_docking",
        executable="docking_pose_estimator",
        name="docking_pose_estimator",
        output="screen",
        parameters=[params_file, common_params],
    )
    container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name="docking_core_container",
        output="screen",
    )
    descriptions = [
        ComposableNode(
            package="usv_docking",
            plugin="usv_docking::DockingFsm",
            name="docking_fsm",
            parameters=fsm_params,
            extra_arguments=[{"use_intra_process_comms": False}],
        ),
        ComposableNode(
            package="usv_docking",
            plugin="usv_docking::DockingMotionController",
            name="docking_motion_controller",
            parameters=[params_file, common_params, {"test_only": test_only}],
            extra_arguments=[{"use_intra_process_comms": False}],
        ),
        ComposableNode(
            package="usv_docking",
            plugin="usv_docking::DockingSafety",
            name="docking_safety",
            parameters=[params_file, common_params],
            extra_arguments=[{"use_intra_process_comms": False}],
        ),
    ]
    loads = [
        LoadComposableNodes(
            target_container="/docking_core_container",
            composable_node_descriptions=[description],
        )
        for description in descriptions
    ]
    return [estimator, container] + loads


def generate_launch_description():
    pkg_share = get_package_share_directory("usv_docking")
    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock（实船 false）",
        ),
        DeclareLaunchArgument(
            "test_only", default_value="false",
            description="true: 只发 /docking/cmd_vel_test；false: 发 /cmd_vel_nav",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "docking.yaml"),
            description="整份参数文件路径（实船标定后传自己的 yaml）",
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="",
            description="定位源里程计话题：空 = 尊重 params_file；切 SLAM 传 /slam/localization",
        ),
        OpaqueFunction(function=_setup),
    ])
