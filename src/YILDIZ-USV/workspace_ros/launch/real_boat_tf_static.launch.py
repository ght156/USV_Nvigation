#!/usr/bin/env python3
# 兼容旧文件名：仅 Include 本包 real_boat_mavros_tf.launch.py。
# 注意：ROS 2 不会把命令行 launch 参数自动转给被 include 的文件，故本文件不能通过 `<arg>:=<value>` 改参；
# 需要传参时请用 real_boat_mavros_tf.launch.py，或经由 real_boat_bringup.launch.py 转发。

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PKG = 'workspace_ros'


def generate_launch_description():
    share = Path(get_package_share_directory(PKG))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / 'launch' / 'real_boat_mavros_tf.launch.py'),
            ),
        ),
    ])
