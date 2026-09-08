from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_share_dir = get_package_share_directory("bw_gi320_driver")

    config_file = os.path.join(
        package_share_dir,
        "config",
        "bw_gi320_config.yaml"
    )

    rtk_node = Node(
        package="bw_gi320_driver",
        executable="bw_gi320_driver_node",
        name="rtk_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[config_file]
    )

    return LaunchDescription([
        rtk_node
    ])