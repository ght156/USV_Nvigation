"""Launch dock_mission Phase 2 nodes (实船版).

与仿真仓差异：
- use_sim_time 默认 false；
- profile 默认 real（overlay 缺失时不回退 sim）；
- speed_arbitrator 默认不启动——实船速度链是 Nav2/docking 直发 /cmd_vel_nav
  经 usv_ardupilot_velocity_bridge（无 /cmd_vel_nav_raw 中间话题），
  互斥由本 FSM 的交接时序 + 速度桥 1s 看门狗保证；确需仲裁器时
  enable_speed_arbitrator:=true 并自行重接 Nav2 输出到 /cmd_vel_nav_raw。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    enable_arbitrator = (
        LaunchConfiguration("enable_speed_arbitrator").perform(context) == "true"
    )

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
        ("dock_entry_validator", "dock_entry_validator"),
    ]
    if enable_arbitrator:
        entries.append(("speed_arbitrator", "speed_arbitrator"))
    for exe, key in entries:
        nodes.append(
            Node(
                package="dock_mission",
                executable=exe,
                name=key,
                parameters=params_for(key) + sim_param,
                output="screen",
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="real"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_speed_arbitrator", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
