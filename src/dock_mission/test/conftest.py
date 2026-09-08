"""Shared pytest fixtures for dock_mission unit tests."""

from __future__ import annotations

import math
import textwrap
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
import yaml

from dock_mission.bay_loader import BayRecord, GnssStaging, MapStaging
from dock_mission.dock_enu import BayDefinition, DockEnuTransform, EntryCorridor, Pose2D


@pytest.fixture(scope="session")
def ros_context():
    """Initialize rclpy once per test session for Node-based tests."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def sample_dock_database(tmp_path: Path) -> Path:
    path = tmp_path / "dock_database.yaml"
    path.write_text(
        textwrap.dedent(
            """\
            bays:
              bay2:
                name: "Test bay2"
                standoff_m: 4.0
                map_staging:
                  x: 3.5
                  y: 0.0
                  yaw: 0.0
                gnss_staging:
                  latitude: 31.48618175
                  longitude: 120.36797148
                  yaw_deg: 0.0
                origin_map:
                  x: -0.5
                  y: 0.0
                  x_axis_yaw: 3.141592653589793
                staging_dock_enu:
                  x: -4.0
                  y: 0.0
                  yaw: 3.141592653589793
                entry_point_dock_enu:
                  x: -2.0
                  y: 0.0
                  yaw: 3.141592653589793
                entry_corridor:
                  x_min: -6.0
                  x_max: 0.0
                  y_max: 1.0
                  yaw_max: 0.15
              bay_missing_gnss:
                name: "No GNSS"
                map_staging:
                  x: 1.0
                  y: 2.0
                  yaw: 0.5
            """
        ),
        encoding="utf-8",
    )
    return path


@pytest.fixture
def sample_map_yaml(tmp_path: Path) -> Path:
    path = tmp_path / "map_hk.yaml"
    cfg = {
        "image": "map.pgm",
        "resolution": 1.0,
        "origin": [0.0, 0.0, 0.0],
        "ref_gnss_10": [120.36793468, 31.48618175],
    }
    with path.open("w", encoding="utf-8") as f:
        yaml.dump(cfg, f)
    return path


@pytest.fixture
def bay_record() -> BayRecord:
    return BayRecord(
        bay_id="bay2",
        name="Test bay2",
        gnss=GnssStaging(
            latitude=31.48618175,
            longitude=120.36797148,
            yaw_deg=0.0,
        ),
        map_pose=MapStaging(x=3.5, y=0.0, yaw=0.0),
        standoff_m=4.0,
    )


@pytest.fixture
def bay_definition() -> BayDefinition:
    return BayDefinition(
        bay_id="bay2",
        origin_map_x=-0.5,
        origin_map_y=0.0,
        x_axis_yaw_map=math.pi,
        staging=Pose2D(x=-4.0, y=0.0, yaw=math.pi),
        entry_point=Pose2D(x=-2.0, y=0.0, yaw=math.pi),
        corridor=EntryCorridor(
            x_min=-6.0,
            x_max=0.0,
            y_max=1.0,
            yaw_max=0.15,
        ),
        standoff_m=4.0,
    )


@pytest.fixture
def dock_transform(bay_definition: BayDefinition) -> DockEnuTransform:
    return DockEnuTransform(bay_definition)


@pytest.fixture
def mock_ros_node(ros_context):
    """Minimal rclpy Node with dock_mission parameters declared."""
    node = rclpy.create_node("dock_mission_test")
    defaults = {
        "bay_id": "bay2",
        "dock_database_path": "",
        "map_yaml_path": "",
        "use_gnss_staging": False,
        "staging_retry_max": 3,
        "settle_sec": 2.5,
        "dock_home_topic": "/dock/home",
        "send_waypoints_service": "/mission_bridge/send_waypoints",
        "nav2_controller_node": "/controller_server",
        "goal_checker_selector_topic": "goal_checker_selector",
        "use_goal_checker_selector": True,
        "cruise_goal_checker_id": "general_goal_checker",
        "docking_goal_checker_id": "docking_goal_checker",
        "cruise_xy_goal_tolerance": 1.0,
        "cruise_yaw_goal_tolerance": 1.0,
        "docking_xy_goal_tolerance": 0.6,
        "docking_yaw_goal_tolerance": 0.15,
        "dock_mission_id": "dock_staging",
    }
    for name, value in defaults.items():
        if isinstance(value, bool):
            node.declare_parameter(name, value)
        elif isinstance(value, int):
            node.declare_parameter(name, value)
        elif isinstance(value, float):
            node.declare_parameter(name, value)
        else:
            node.declare_parameter(name, value)
    yield node
    node.destroy_node()
