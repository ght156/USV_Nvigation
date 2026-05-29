#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _decl(name: str, default_value, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def _setup_nodes(context, *args, **kwargs):
    lc = LaunchConfiguration
    use_sim_time = lc("use_sim_time")
    odom_topic = lc("odom_topic")

    params_file = lc("params_file").perform(context).strip()

    mission_bridge_params = []
    aggregator_params = []
    if params_file:
        mission_bridge_params.append(params_file)
        aggregator_params.append(params_file)

    mission_bridge_params.extend(
        [
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
                "odom_topic": ParameterValue(odom_topic, value_type=str),
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
                "target_buoy_force_rewrite": ParameterValue(
                    lc("target_buoy_force_rewrite"), value_type=bool
                ),
                "waypoint_command_mode": ParameterValue(
                    lc("waypoint_command_mode"), value_type=str
                ),
                "waypoint_commit_delay_sec": ParameterValue(
                    lc("waypoint_commit_delay_sec"), value_type=float
                ),
                "mission_start_topic": ParameterValue(
                    lc("mission_start_topic"), value_type=str
                ),
                "mission_cancel_topic": ParameterValue(
                    lc("mission_cancel_topic"), value_type=str
                ),
                "discard_watchdog_sec": ParameterValue(
                    lc("discard_watchdog_sec"), value_type=float
                ),
                "suppress_passive_waypoints_after_cancel": ParameterValue(
                    lc("suppress_passive_waypoints_after_cancel"), value_type=bool
                ),
                "target_buoy_min_write_period_sec": ParameterValue(
                    lc("target_buoy_min_write_period_sec"), value_type=float
                ),
            },
        ]
    )

    # aggregator odom_topic 始终与 mission_bridge 的 odom_topic launch 参数绑定
    aggregator_params.extend(
        [
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {
                "publish_rate": ParameterValue(
                    lc("nav_status_publish_rate"), value_type=float
                ),
                "vehicle_id": ParameterValue(
                    lc("nav_status_vehicle_id"), value_type=str
                ),
                "gps_topic": ParameterValue(
                    lc("aggregator_gps_topic"), value_type=str
                ),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
                "global_frame": ParameterValue(lc("global_frame"), value_type=str),
                "robot_frame": ParameterValue(lc("robot_frame"), value_type=str),
            },
        ]
    )

    return [
        Node(
            package="workspace_nav",
            executable="mission_bridge",
            name="mission_bridge",
            output="screen",
            parameters=mission_bridge_params,
        ),
        Node(
            package="workspace_nav",
            executable="nav_status_aggregator",
            name="nav_status_aggregator",
            output="screen",
            parameters=aggregator_params,
        ),
    ]


def generate_launch_description():
    pkg = FindPackageShare("workspace_nav")
    default_map_yaml = PathJoinSubstitution([pkg, "config", "map.yaml"])
    default_mission_params = PathJoinSubstitution(
        [pkg, "config", "mission_stack.real_boat.yaml"]
    )

    return LaunchDescription(
        [
            _decl("use_sim_time", "false", "实船 false；回放 bag 带 /clock 时设 true"),
            _decl(
                "params_file",
                default_mission_params,
                "mission_bridge / nav_status_aggregator 参数 YAML；"
                "默认 mission_stack.real_boat.yaml",
            ),
            _decl(
                "map_yaml_path",
                default_map_yaml,
                "与 Nav2 同源地图；可被 params_file 或 launch 覆盖",
            ),
            _decl("follow_waypoints_action", "follow_waypoints", "FollowWaypoints action"),
            _decl("waypoint_topic", "/waypoint", "航点话题"),
            _decl("color_topic", "/color_code", "目标颜色话题"),
            _decl("map_datum_ref_key", "ref_gnss_10", "map YAML datum 键"),
            _decl("global_frame", "map", "TF 全局帧（两节点共用）"),
            _decl("robot_frame", "base_link", "TF 车体帧（两节点共用）"),
            _decl("waypoints_json_path", "", "为空则用环境变量或默认路径"),
            _decl("target_buoy_json_path", "", "为空则用环境变量或默认路径"),
            _decl(
                "odom_topic",
                "/mavros/local_position/odom",
                "实船 MAVROS 里程计；与 nav_status_aggregator 共用（launch 内绑定）",
            ),
            _decl("datum_source", "map_yaml", "datum 来源"),
            _decl("projection", "enu", "enu | utm"),
            _decl("tf_check_period_sec", "1.0", "WAITING_SYSTEM 检查周期 (s)"),
            _decl("waypoint_tolerance_m", "1.5", "航点容差 (m)"),
            _decl("debug_mode", "false", "打印原始 waypoint/color"),
            _decl(
                "allow_replace_running_mission",
                "false",
                "运行中是否允许新 /waypoint 抢占",
            ),
            _decl(
                "allow_repeat_identical_route",
                "false",
                "完成后是否允许相同 hash 再跑",
            ),
            _decl(
                "target_buoy_force_rewrite",
                "false",
                "每次 /color_code 是否强制写 target_buoy.json",
            ),
            _decl(
                "waypoint_command_mode",
                "debounce",
                "immediate | debounce | start_pulse",
            ),
            _decl(
                "waypoint_commit_delay_sec",
                "0.45",
                "debounce 静默窗口 (s)",
            ),
            _decl(
                "mission_start_topic",
                "",
                'start_pulse 时必填，如 "/gcs_mission/start"',
            ),
            _decl(
                "mission_cancel_topic",
                "/gcs_mission/cancel",
                "Cancel 话题 (std_msgs/Empty)",
            ),
            _decl("discard_watchdog_sec", "4.0", "抢占丢弃 goal 看门狗 (s)"),
            _decl(
                "suppress_passive_waypoints_after_cancel",
                "true",
                "Cancel 后丢弃无 explicit_replan 的被动 /waypoint",
            ),
            _decl(
                "target_buoy_min_write_period_sec",
                "0.0",
                "target_buoy.json 最小写盘间隔 (s)",
            ),
            _decl("nav_status_publish_rate", "2.0", "/nav_status 发布频率 (Hz)"),
            _decl("nav_status_vehicle_id", "usv_001", "vehicle_id"),
            _decl(
                "aggregator_gps_topic",
                "/mavros/global_position/raw/fix",
                "实船 MAVROS GPS（可与 mission_bridge 独立配置）",
            ),
            OpaqueFunction(function=_setup_nodes),
        ]
    )
