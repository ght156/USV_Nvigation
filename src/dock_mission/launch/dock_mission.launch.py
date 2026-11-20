"""Launch dock_mission Phase 2 nodes (实船版).

与仿真仓差异：
- use_sim_time 默认 false；
- profile 默认 real（overlay 缺失时不回退 sim）；
- dock_entry_validator（入口验收）已于 2026-09 移除：其 origin_map 吃飞控
  odom 系坐标、飞控每次重启原点都变需人工重填，而现有门控已放得很宽，
  防错价值抵不过维护成本；交接前的到点把关由 Nav2 docking_goal_checker
  （0.6m/0.15rad）承担。
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_node_params(config_path: str, node_key: str) -> list:
    if not os.path.isfile(config_path):
        return []
    with open(config_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    block = data.get(node_key, {})
    params = block.get("ros__parameters", block)
    return [params] if params else []


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("dock_mission")
    profile = LaunchConfiguration("profile").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context) == "true"
    # 定位源切换：空 = 尊重 yaml/代码默认；非空 = 覆盖 dock_mission_node 的
    # odom_speed_topic（SETTLE 停稳门速度源）。与 usv_docking docking.launch.py
    # 的 odom_topic 用法一致——两边必须传同一个话题，否则 SETTLE 与 FSM
    # 读不同里程计（2026-10-13 二审 P1-8）。
    odom_topic = LaunchConfiguration("odom_topic").perform(context).strip()

    base = os.path.join(pkg, "config", "dock_mission.yaml")
    overlay = os.path.join(pkg, "config", f"dock_mission_{profile}.yaml")
    if not os.path.isfile(overlay):
        overlay = None

    def params_for(key: str) -> list:
        p = _load_node_params(base, key)
        if overlay:
            o = _load_node_params(overlay, key)
            if o:
                merged = {**p[0], **o[0]} if p else o[0]
                return [merged]
        return p

    sim_param = [{"use_sim_time": use_sim}]
    nodes = []
    entries = [
        ("dock_mission_node", "dock_mission_node"),
    ]
    for exe, key in entries:
        params = params_for(key) + sim_param
        if odom_topic:
            params.append({"odom_speed_topic": odom_topic})
        nodes.append(
            Node(
                package="dock_mission",
                executable=exe,
                name=key,
                parameters=params,
                output="screen",
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOG_DIR", launch_config.log_dir),
            DeclareLaunchArgument("profile", default_value="real"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="定位源里程计话题：空 = 尊重 yaml 默认 "
                            "(/mavros/gps_input/local)；切 SLAM 传 "
                            "/slam/localization（与 docking.launch.py 传同一个值）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
