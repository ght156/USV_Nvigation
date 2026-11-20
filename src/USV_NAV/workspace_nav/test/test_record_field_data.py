from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT.parents[2] / "scripts" / "record_field_data.sh"


def _topics_block(text):
    return text.split("TOPICS=(", 1)[1].split(")\n\nlog()", 1)[0]


def test_navigation_recorder_covers_active_navigation_chain():
    text = SCRIPT.read_text()
    topics_block = _topics_block(text)
    topics = [
        "/behavior_tree_log",
        "/global_costmap/costmap_updates",
        "/mission_bridge/safety_event",
        "/mavros/state",
        "/mavros/imu/data",
        "/mavros/rc/out",
        "/diagnostics",
        "/parameter_events",
        "/goal_checker_selector",
        "/received_global_plan",
        "/move_base_simple/goal",
    ]
    actions = [
        "/mission_bridge/navigate",
        "/follow_waypoints",
        "/navigate_to_pose",
        "/compute_path_through_poses",
        "/compute_path_to_pose",
        "/follow_path",
        "/spin",
        "/backup",
        "/wait",
        "/drive_on_heading",
    ]
    for action in actions:
        topics.extend((f"{action}/_action/feedback", f"{action}/_action/status"))
    for topic in topics:
        assert f'"{topic}"' in topics_block

    for unused in (
        "/mavros/global_position/global",
        "/mavros/local_position/odom",
        "/usv_localization/map_odom",
        "/local_plan",
        "/local_costmap/costmap_updates",
        "/plan_smoothed",
        "/unsmoothed_plan",
        "/cmd_vel_teleop",
        "/goal_pose",
        "/speed_limit",
        "/preempt_teleop",
        "/gcs_mission/cancel",
        "/patrol/command",
        "/patrol/status",
        "/navigate_through_poses/_action/feedback",
        "/assisted_teleop/_action/feedback",
        "/smooth_path/_action/feedback",
    ):
        assert f'"{unused}"' not in topics_block


def test_navigation_recorder_starts_without_querying_every_topic():
    text = SCRIPT.read_text()
    qos_writer = text.split("write_qos_yaml() {", 1)[1].split("cmd_start() {", 1)[0]
    start = text.split("cmd_start() {", 1)[1].split("cmd_status() {", 1)[0]
    assert "query_pub_qos" not in qos_writer
    assert "query_pub_qos" not in start
    assert "write_qos_yaml" in start
    assert "--include-hidden-topics" in start
    assert 'rel="best_effort"' in qos_writer
    assert 'rel="reliable"' in qos_writer
    assert 'dur="transient_local"' in qos_writer
    assert "reliability: $rel" in qos_writer
    assert "durability: $dur" in qos_writer


def test_navigation_recorder_uses_boat_workspace_and_short_split_files():
    text = SCRIPT.read_text()
    assert "/home/hkzl/usv-test/install/arm/setup.bash" in text
    assert "/home/hkzl/usv_nav_ws/install/arm/setup.bash" in text
    assert "SPLIT_SEC=600" in text
    assert "RMW=rmw_fastrtps_cpp" in text
