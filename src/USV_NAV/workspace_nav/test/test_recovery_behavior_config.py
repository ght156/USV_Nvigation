from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_behavior_commands_reach_velocity_bridge_input():
    launch = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text()
    for marker in ("executable='behavior_server'", "plugin='behavior_server::BehaviorServer'"):
        block = launch.split(marker, 1)[1].split("),", 1)[0]
        assert "('cmd_vel', 'cmd_vel_nav')" in block

    boat_launch = (PACKAGE_ROOT / "launch" / "nav2_boat.launch.py").read_text()
    assert "'launch', 'navigation.launch.py'" in boat_launch


def test_behavior_parameters_match_xingfei_velocity_bridge():
    params_path = PACKAGE_ROOT / "config" / "nav2_params_xingfei.yaml"
    params = yaml.safe_load(params_path.read_text())["behavior_server"]["ros__parameters"]

    assert params["cycle_frequency"] == 10.0
    for behavior_name in ("backup", "drive_on_heading"):
        behavior = params[behavior_name]
        assert behavior["acceleration_limit"] == 0.3
        assert behavior["deceleration_limit"] == -0.3
        assert behavior["minimum_speed"] == 0.0

    assert params["projection_time"] == 1.0
    assert params["simulation_time_step"] == 0.1
    assert params["cmd_vel_teleop"] == "cmd_vel_teleop"


def test_recovery_tree_uses_motion_without_clearing_costmaps():
    tree = ET.parse(PACKAGE_ROOT / "config" / "navigate_to_pose_w_goal_checker.xml")
    assert tree.find(".//ClearEntireCostmap") is None

    spin = tree.find(".//Spin").attrib
    assert spin["spin_dist"] == "1.57"
    assert spin["time_allowance"] == "10.0"
    assert tree.find(".//Wait").attrib["wait_duration"] == "10"

    backup = tree.find(".//BackUp").attrib
    assert backup["backup_dist"] == "0.50"
    assert backup["backup_speed"] == "0.2"
    assert backup["time_allowance"] == "10.0"

    drive = tree.find(".//DriveOnHeading").attrib
    assert drive["dist_to_travel"] == "0.30"
    assert drive["speed"] == "0.2"
    assert drive["time_allowance"] == "10.0"


def test_xingfei_clears_local_and_global_only_once_at_mission_start():
    params_path = PACKAGE_ROOT / "config" / "mission_stack.xingfei.yaml"
    params = yaml.safe_load(params_path.read_text())["mission_bridge"]["ros__parameters"]
    assert params["clear_costmap_on_mission_start"] is True
    assert params["costmap_clear_period_sec"] == 0.0

    source = (PACKAGE_ROOT / "src" / "mission_bridge_node.cpp").read_text()
    assert '"/local_costmap/clear_entirely_local_costmap"' in source
    assert '"/global_costmap/clear_entirely_global_costmap"' in source
    assert source.count('clear_local_costmap("new mission");') == 2
    assert source.count('clear_global_costmap("new mission");') == 2
    assert 'clear_global_costmap("periodic")' not in source


def test_xingfei_heading_alignment_threshold_and_padding():
    params_path = PACKAGE_ROOT / "config" / "nav2_params_xingfei.yaml"
    params = yaml.safe_load(params_path.read_text())
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["rotate_to_heading_min_angle"] == 0.5236
    assert params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint_padding"] == 0.1
    assert params["global_costmap"]["global_costmap"]["ros__parameters"]["footprint_padding"] == 0.1


def test_xingfei_global_inflation_is_three_metres():
    params_path = PACKAGE_ROOT / "config" / "nav2_params_xingfei.yaml"
    params = yaml.safe_load(params_path.read_text())
    global_layer = params["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]
    local_layer = params["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    assert global_layer["inflation_radius"] == 3.0
    assert local_layer["inflation_radius"] == 5.0


def test_global_voxel_matches_local_except_wider_blind_zone():
    params_path = PACKAGE_ROOT / "config" / "nav2_params_xingfei.yaml"
    params = yaml.safe_load(params_path.read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    global_voxel = params["global_costmap"]["global_costmap"]["ros__parameters"]["voxel_layer"]

    assert global_voxel["enabled"] is True
    for key in (
        "origin_z",
        "z_resolution",
        "z_voxels",
        "max_obstacle_height",
        "mark_threshold",
        "unknown_threshold",
        "combination_method",
        "observation_sources",
    ):
        assert global_voxel[key] == local[key]

    local_cloud = dict(local["livox_cloud"])
    global_cloud = dict(global_voxel["livox_cloud"])
    assert local_cloud.pop("obstacle_min_range") == 1.6
    assert global_cloud.pop("obstacle_min_range") == 2.0
    assert global_cloud == local_cloud
