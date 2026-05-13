#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Launch file for initializing the Gazebo Garden simulation of the RoboBoat.
#  It sets up environment paths, generates the robot description from a Xacro file,
#  spawns the robot into the simulation world, and launches core ROS 2 publisher nodes.
#  The file also bridges key Gazebo topics—such as clock, sensors, and thruster commands—
#  enabling seamless ROS 2 interaction with the simulated environment.
# ----------------------------------------------------------------------------------------------- #

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, TextSubstitution

def generate_launch_description():
    package_name = 'workspace_gz'
    # 使用 ament 在运行时解析为当前已 source 工作空间下的绝对路径，避免系统里残留的旧 GZ_* 或旧工程目录
    share = os.path.normpath(get_package_share_directory(package_name))
    prefix = os.path.normpath(get_package_prefix(package_name))
    model_path = os.path.join(share, 'models')
    plugin_path = os.path.join(prefix, 'lib')
    world_sdf = os.path.join(share, 'worlds', 'world.sdf')
    xacro_file = os.path.join(share, 'description', 'roboboat', 'roboboat.xacro')

    # Command 对纯 str 列表会把参数直接拼在一起；用 TextSubstitution 保证与 xacro 可执行文件分离为两个 argv
    robot_description = ParameterValue(
        Command([
            TextSubstitution(text='xacro '),
            TextSubstitution(text=xacro_file),
        ]),
        value_type=str
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=plugin_path
        ),
        # 部分 GUI/老插件仍读 IGN_ 前缀，与 GZ_SIM_* 设成同一路径以免指向历史 install
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=model_path
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value=plugin_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_sdf],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_roboboat',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'roboboat',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
                "/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_gps/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            ],
            remappings=[
                ("/world/default/clock", "/clock"),
                ("/model/roboboat/joint/left_housing_link_to_left_prop_link/cmd_thrust", "/roboboat/thrusters/left/thrust"),
                ("/model/roboboat/joint/right_housing_link_to_right_prop_link/cmd_thrust", "/roboboat/thrusters/right/thrust"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_gps/navsat", "/roboboat/sensors/gps/navsat"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_imu/imu", "/roboboat/sensors/imu/imu"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_lidar/scan", "/roboboat/sensors/lidar/scan"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_lidar/scan/points", "/roboboat/sensors/lidar/scan/points"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/camera_info", "/roboboat/sensors/camera/camera_info"),
                ("/world/default/model/roboboat/link/base_link/sensor/sensor_camera/image", "/roboboat/sensors/camera/image"),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
