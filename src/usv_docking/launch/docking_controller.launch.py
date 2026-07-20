#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    pkg_share = get_package_share_directory("usv_docking")
    params_file = os.path.join(pkg_share, "config", f"docking_controller_{profile}.yaml")
    if not os.path.isfile(params_file):
        raise RuntimeError(
            f"usv_docking parameter file not found: {params_file} "
            f"(profile={profile!r}; expected sim or real)"
        )

    return [
        Node(
            package="usv_docking",
            executable="docking_controller",
            name="docking_controller",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim",
                description="Parameter profile: sim | real",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
