"""Tests for gnss_staging coordinate conversion and pose building."""

from __future__ import annotations

import math

import pytest
from geometry_msgs.msg import PoseStamped

from dock_mission.bay_loader import BayRecord, GnssStaging, MapStaging
from dock_mission.gnss_staging import (
    gnss_to_map_xy,
    make_staging_pose,
    make_staging_waypoint,
    map_xy_to_gnss,
)


def test_gnss_to_map_xy_at_datum(sample_map_yaml):
    gnss = GnssStaging(
        latitude=31.48618175,
        longitude=120.36793468,
        yaw_deg=0.0,
    )
    x, y = gnss_to_map_xy(gnss, sample_map_yaml)
    assert x == pytest.approx(0.0, abs=0.01)
    assert y == pytest.approx(0.0, abs=0.01)


def test_map_xy_to_gnss_roundtrip(sample_map_yaml):
    gnss = GnssStaging(
        latitude=31.48618175,
        longitude=120.36797148,
        yaw_deg=0.0,
    )
    x, y = gnss_to_map_xy(gnss, sample_map_yaml)
    back = map_xy_to_gnss(x, y, sample_map_yaml)
    assert back.latitude == pytest.approx(gnss.latitude, rel=1e-6)
    assert back.longitude == pytest.approx(gnss.longitude, rel=1e-6)


def test_make_staging_pose_map_mode(bay_record):
    pose = make_staging_pose(bay_record, use_gnss=False)
    assert isinstance(pose, PoseStamped)
    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == pytest.approx(3.5)
    assert pose.pose.position.y == pytest.approx(0.0)
    assert pose.pose.orientation.w == pytest.approx(1.0)


def test_make_staging_pose_gnss_mode(bay_record, sample_map_yaml):
    pose = make_staging_pose(
        bay_record,
        use_gnss=True,
        map_yaml=sample_map_yaml,
    )
    assert pose.header.frame_id == "map"
    assert math.isfinite(pose.pose.position.x)
    assert math.isfinite(pose.pose.position.y)


def test_make_staging_pose_gnss_requires_map_yaml(bay_record):
    with pytest.raises(ValueError, match="map_yaml required"):
        make_staging_pose(bay_record, use_gnss=True, map_yaml=None)


def test_make_staging_pose_rejects_zero_gnss(tmp_path, sample_map_yaml):
    bay = BayRecord(
        bay_id="empty",
        name="empty",
        gnss=GnssStaging(latitude=0.0, longitude=0.0, yaw_deg=0.0),
        map_pose=MapStaging(x=1.0, y=2.0, yaw=0.0),
        standoff_m=4.0,
    )
    with pytest.raises(ValueError, match="gnss_staging not configured"):
        make_staging_pose(bay, use_gnss=True, map_yaml=sample_map_yaml)


def test_make_staging_waypoint_gnss_mode(bay_record):
    lat, lon, yaw = make_staging_waypoint(bay_record, use_gnss=True)
    assert lat == pytest.approx(31.48618175)
    assert lon == pytest.approx(120.36797148)
    assert yaw == pytest.approx(0.0)


def test_make_staging_waypoint_gnss_needs_no_map_yaml(bay_record):
    # 实船路径：WGS84 直发 mission_bridge，不读 map_yaml
    make_staging_waypoint(bay_record, use_gnss=True, map_yaml=None)


def test_make_staging_waypoint_map_mode_roundtrip(bay_record, sample_map_yaml):
    lat, lon, yaw = make_staging_waypoint(
        bay_record, use_gnss=False, map_yaml=sample_map_yaml
    )
    # map_staging (3.5, 0) 反解 WGS84 后再正解应回到原 map 点
    gnss = GnssStaging(latitude=lat, longitude=lon, yaw_deg=0.0)
    x, y = gnss_to_map_xy(gnss, sample_map_yaml)
    assert x == pytest.approx(3.5, abs=0.01)
    assert y == pytest.approx(0.0, abs=0.01)
    assert yaw == pytest.approx(bay_record.map_pose.yaw)


def test_make_staging_waypoint_map_mode_requires_map_yaml(bay_record):
    with pytest.raises(ValueError, match="map_yaml required"):
        make_staging_waypoint(bay_record, use_gnss=False, map_yaml=None)
