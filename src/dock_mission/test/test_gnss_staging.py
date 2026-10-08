"""Tests for gnss_staging WGS84 waypoint generation."""

from __future__ import annotations

import pytest

from dock_mission.bay_loader import BayRecord, GnssStaging, MapStaging
from dock_mission.gnss_staging import make_staging_waypoint, map_xy_to_gnss


def test_map_xy_to_gnss_at_datum(sample_map_yaml):
    # map 原点（datum）反解应回到 ref_gnss
    gnss = map_xy_to_gnss(0.0, 0.0, sample_map_yaml)
    assert gnss.latitude == pytest.approx(31.48618175, rel=1e-6)
    assert gnss.longitude == pytest.approx(120.36793468, rel=1e-6)


def test_make_staging_waypoint_gnss_mode(bay_record):
    lat, lon, yaw = make_staging_waypoint(bay_record, use_gnss=True)
    assert lat == pytest.approx(31.48618175)
    assert lon == pytest.approx(120.36797148)
    assert yaw == pytest.approx(0.0)


def test_make_staging_waypoint_gnss_needs_no_map_yaml(bay_record):
    # 实船路径：WGS84 直发 mission_bridge，不读 map_yaml
    make_staging_waypoint(bay_record, use_gnss=True, map_yaml=None)


def test_make_staging_waypoint_gnss_rejects_zero(bay_record):
    bay = BayRecord(
        bay_id="empty",
        name="empty",
        gnss=GnssStaging(latitude=0.0, longitude=0.0, yaw_deg=0.0),
        map_pose=MapStaging(x=1.0, y=2.0, yaw=0.0),
        standoff_m=4.0,
    )
    with pytest.raises(ValueError, match="gnss_staging not configured"):
        make_staging_waypoint(bay, use_gnss=True)


def test_make_staging_waypoint_map_mode(bay_record, sample_map_yaml):
    lat, lon, yaw = make_staging_waypoint(
        bay_record, use_gnss=False, map_yaml=sample_map_yaml
    )
    # map_staging (3.5, 0)：datum 正东 3.5m → 纬度不变、经度略增
    assert lat == pytest.approx(31.48618175, rel=1e-6)
    assert lon > 120.36793468
    assert yaw == pytest.approx(bay_record.map_pose.yaw)


def test_make_staging_waypoint_map_mode_requires_map_yaml(bay_record):
    with pytest.raises(ValueError, match="map_yaml required"):
        make_staging_waypoint(bay_record, use_gnss=False, map_yaml=None)
