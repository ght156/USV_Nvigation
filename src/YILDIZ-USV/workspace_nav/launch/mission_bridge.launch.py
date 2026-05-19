#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("workspace_nav")
    default_map_yaml = PathJoinSubstitution([pkg, "config", "map.yaml"])

    def _decl(name: str, default_value, description: str) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(name, default_value=default_value, description=description)

    lc = LaunchConfiguration

    use_sim_time = lc("use_sim_time")

    return LaunchDescription(
        [
            _decl("use_sim_time", "true", "必须与 Gazebo localization / Nav2 一致"),
            _decl(
                "map_yaml_path",
                default_map_yaml,
                "与 Nav2 launch 的 map:= 同源；可被 include 时覆盖",
            ),
            _decl("follow_waypoints_action", "follow_waypoints", "FollowWaypoints action"),
            _decl("waypoint_topic", "/waypoint", "地面站航点话题"),
            _decl("color_topic", "/color_code", "地面站颜色/目标话题"),
            _decl("map_datum_ref_key", "ref_gnss_10", "map YAML 中 datum 锚点键"),
            _decl("global_frame", "map", "TF 全局帧"),
            _decl("robot_frame", "base_link", "TF 车体帧"),
            _decl(
                "waypoints_json_path",
                "",
                "为空则沿用 WAYPOINT_FILE_PATH / 默认同 waypoint_with_state",
            ),
            _decl(
                "target_buoy_json_path",
                "",
                "为空则沿用 TARGET_JSON_PATH / 默认 share json/target_buoy.json",
            ),
            _decl(
                "odom_topic",
                "/odometry/filtered",
                "逐点跳过时船位（仿真 EKF）；实机可在 include 时改为 MAVROS odom",
            ),
            _decl("datum_source", "map_yaml", "首版固定 map_yaml"),
            _decl("projection", "enu", "经纬→平面投影 enu | utm"),
            _decl("tf_check_period_sec", "1.0", "WAITING_SYSTEM 周期检查 TF/action"),
            _decl("waypoint_tolerance_m", "1.5", "接近航点阈值（米）"),
            _decl("debug_mode", "false", "打印原始 waypoint/color 负载"),
            _decl(
                "allow_replace_running_mission",
                "false",
                "RUNNING 中替换任务（未实现：仍为拒绝并告警）",
            ),
            _decl(
                "allow_repeat_identical_route",
                "false",
                "完成后是否允许同源 hash 立刻再跑一趟",
            ),
            Node(
                package="workspace_nav",
                executable="mission_bridge",
                name="mission_bridge",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {
                        "map_yaml_path": ParameterValue(lc("map_yaml_path"), value_type=str),
                        "follow_waypoints_action": ParameterValue(
                            lc("follow_waypoints_action"), value_type=str
                        ),
                        "waypoint_topic": ParameterValue(lc("waypoint_topic"), value_type=str),
                        "color_topic": ParameterValue(lc("color_topic"), value_type=str),
                        "map_datum_ref_key": ParameterValue(
                            lc("map_datum_ref_key"), value_type=str
                        ),
                        "global_frame": ParameterValue(lc("global_frame"), value_type=str),
                        "robot_frame": ParameterValue(lc("robot_frame"), value_type=str),
                        "waypoints_json_path": ParameterValue(
                            lc("waypoints_json_path"), value_type=str
                        ),
                        "target_buoy_json_path": ParameterValue(
                            lc("target_buoy_json_path"), value_type=str
                        ),
                        "odom_topic": ParameterValue(lc("odom_topic"), value_type=str),
                        "datum_source": ParameterValue(lc("datum_source"), value_type=str),
                        "projection": ParameterValue(lc("projection"), value_type=str),
                        "tf_check_period_sec": ParameterValue(
                            lc("tf_check_period_sec"), value_type=float
                        ),
                        "waypoint_tolerance_m": ParameterValue(
                            lc("waypoint_tolerance_m"), value_type=float
                        ),
                        "debug_mode": ParameterValue(lc("debug_mode"), value_type=bool),
                        "allow_replace_running_mission": ParameterValue(
                            lc("allow_replace_running_mission"), value_type=bool
                        ),
                        "allow_repeat_identical_route": ParameterValue(
                            lc("allow_repeat_identical_route"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
