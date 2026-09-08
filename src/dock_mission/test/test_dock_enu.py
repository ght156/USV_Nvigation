"""Tests for dock_enu frame transforms."""

from __future__ import annotations

import math

import pytest

from dock_mission.dock_enu import (
    BayDefinition,
    DockEnuTransform,
    EntryCorridor,
    Pose2D,
    load_bay_from_dict,
    wrap_yaw,
)


def test_wrap_yaw():
    assert wrap_yaw(0.0) == pytest.approx(0.0)
    assert wrap_yaw(3 * math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_yaw(-3 * math.pi) == pytest.approx(-math.pi, abs=1e-9)


def test_map_to_dock_at_spawn(dock_transform):
    pose = dock_transform.map_to_dock(0.0, 0.0, 0.0)
    assert pose.x == pytest.approx(-0.5, abs=1e-9)
    assert pose.y == pytest.approx(0.0, abs=1e-9)


def test_dock_to_map_roundtrip(dock_transform):
    dock = Pose2D(x=-4.0, y=0.5, yaw=math.pi)
    back = dock_transform.dock_to_map(dock)
    again = dock_transform.map_to_dock(back.x, back.y, back.yaw)
    assert again.x == pytest.approx(dock.x, abs=1e-9)
    assert again.y == pytest.approx(dock.y, abs=1e-9)
    assert wrap_yaw(again.yaw - dock.yaw) == pytest.approx(0.0, abs=1e-9)


def test_default_staging_in_dock(dock_transform):
    staging = dock_transform.default_staging_in_dock()
    assert staging.x == pytest.approx(-4.0)
    assert staging.y == pytest.approx(0.0)
    assert staging.yaw == pytest.approx(math.pi)


def test_load_bay_from_dict():
    data = {
        "origin_map": {"x": 1.0, "y": 2.0, "x_axis_yaw": 0.1},
        "staging_dock_enu": {"x": -3.0, "y": 0.1, "yaw": 3.0},
        "entry_corridor": {
            "x_min": -5.0,
            "x_max": -0.1,
            "y_max": 0.8,
            "yaw_max": 0.2,
        },
        "standoff_m": 3.5,
    }
    bay = load_bay_from_dict("test_bay", data)
    assert isinstance(bay, BayDefinition)
    assert bay.bay_id == "test_bay"
    assert bay.origin_map_x == pytest.approx(1.0)
    assert bay.standoff_m == pytest.approx(3.5)
    assert bay.corridor.y_max == pytest.approx(0.8)


def test_load_bay_entry_point_default():
    data = {
        "origin_map": {"x": 0.0, "y": 0.0, "x_axis_yaw": 0.0},
        "staging_dock_enu": {"x": -4.0, "y": 0.0, "yaw": math.pi},
        "entry_corridor": {"x_min": -6.0, "x_max": 0.0, "y_max": 1.0, "yaw_max": 0.15},
    }
    bay = load_bay_from_dict("bay2", data)
    assert bay.entry_point.x == pytest.approx(-2.0)
    assert bay.entry_point.y == pytest.approx(0.0)


def test_load_bay_entry_point_explicit():
    data = {
        "origin_map": {"x": 0.0, "y": 0.0, "x_axis_yaw": 0.0},
        "staging_dock_enu": {"x": -4.0, "y": 0.0, "yaw": math.pi},
        "entry_point_dock_enu": {"x": -2.5, "y": 0.0, "yaw": math.pi},
        "entry_corridor": {"x_min": -6.0, "x_max": 0.0, "y_max": 1.0, "yaw_max": 0.15},
    }
    bay = load_bay_from_dict("bay2", data)
    assert bay.entry_point.x == pytest.approx(-2.5)
