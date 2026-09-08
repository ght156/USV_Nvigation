"""Tests for bay_loader.load_bay."""

from __future__ import annotations

import pytest

from dock_mission.bay_loader import BayRecord, load_bay


def test_load_bay_from_fixture_database(sample_dock_database):
    bay = load_bay("bay2", sample_dock_database)
    assert isinstance(bay, BayRecord)
    assert bay.bay_id == "bay2"
    assert bay.name == "Test bay2"
    assert bay.standoff_m == 4.0
    assert bay.map_pose.x == pytest.approx(3.5)
    assert bay.map_pose.y == pytest.approx(0.0)
    assert bay.gnss.latitude == pytest.approx(31.48618175)
    assert bay.gnss.longitude == pytest.approx(120.36797148)


def test_load_bay_missing_key_raises(sample_dock_database):
    with pytest.raises(KeyError, match="bay 'unknown'"):
        load_bay("unknown", sample_dock_database)


def test_load_bay_defaults_for_partial_entry(tmp_path):
    path = tmp_path / "partial.yaml"
    path.write_text(
        "bays:\n  bay1:\n    name: minimal\n",
        encoding="utf-8",
    )
    bay = load_bay("bay1", path)
    assert bay.bay_id == "bay1"
    assert bay.standoff_m == pytest.approx(4.0)
    assert bay.gnss.latitude == pytest.approx(0.0)
    assert bay.map_pose.x == pytest.approx(0.0)
