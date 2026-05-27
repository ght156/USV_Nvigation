#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("workspace_nav")
    default_map_yaml = PathJoinSubstitution([pkg, "config", "map_hk.yaml"])

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
                "航行中看到新 /waypoint 是否自动 cancel 并重派（实船默认关；仿真需运动中换路时显式 true）",
            ),
            _decl(
                "allow_repeat_identical_route",
                "false",
                "完成后是否允许同源 hash 立刻再跑一趟",
            ),
            _decl(
                "target_buoy_force_rewrite",
                "false",
                "true 时每次 /color_code 都重写 target_buoy.json（默认仅语义变色才写）",
            ),
            _decl(
                "waypoint_command_mode",
                "debounce",
                "immediate | debounce | start_pulse；防抖可抑制地面站短时交替发布引发的反复 cancel",
            ),
            _decl(
                "waypoint_commit_delay_sec",
                "0.45",
                "debounce 模式下静默窗口（秒）；收到最后一条 /waypoint 后再执行",
            ),
            _decl(
                "mission_start_topic",
                "",
                'start_pulse 必填，例如 "/gcs_mission/start"（订阅 std_msgs/Empty)',
            ),
            _decl(
                "mission_cancel_topic",
                "/gcs_mission/cancel",
                "std_msgs/Empty：清空缓冲并在航行中断航；与地面站 Cancel Nav 对齐",
            ),
            _decl(
                "discard_watchdog_sec",
                "4.0",
                "抢占丢弃 goal 超时看门狗（秒），≤0 关闭；卡死时再发航点前应已恢复",
            ),
            _decl(
                "suppress_passive_waypoints_after_cancel",
                "true",
                "Cancel Nav 后置位：丢弃无 explicit_replan 的被动 /waypoint，避免又自动起航"
                "（start_pulse + Empty 仍可启动；JSON 中带 explicit_replan 会清除抑制）",
            ),
            _decl(
                "target_buoy_min_write_period_sec",
                "0.0",
                "target_buoy.json 最小写盘间隔（秒）；0 关闭。"
                "与「同色跳过」并行，可压住 GCS 红绿交替语义导致的刷屏写盘",
            ),
            # nav_status_aggregator launch arguments
            _decl(
                "nav_status_publish_rate",
                "2.0",
                "聚合器发布频率（Hz）",
            ),
            _decl(
                "nav_status_vehicle_id",
                "usv_001",
                "车辆 ID",
            ),
            _decl(
                "aggregator_gps_topic",
                "/gps/fixed_cov",
                "GPS fix 话题（聚合器用）",
            ),
            _decl(
                "aggregator_odom_topic",
                "/odometry/filtered",
                "里程计话题（聚合器用）",
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
                ],
            ),
            # nav_status_aggregator: pure observer, no goals sent
            Node(
                package="workspace_nav",
                executable="nav_status_aggregator",
                name="nav_status_aggregator",
                output="screen",
                parameters=[
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
                        "odom_topic": ParameterValue(
                            lc("aggregator_odom_topic"), value_type=str
                        ),
                    },
                ],
            ),
        ]
    )
