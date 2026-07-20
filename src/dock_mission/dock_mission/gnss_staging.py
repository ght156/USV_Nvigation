"""GNSS / map staging pose → geometry_msgs/PoseStamped for Nav2."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header

from dock_mission.bay_loader import BayRecord, GnssStaging


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _load_map_datum(map_yaml: Path) -> Tuple[float, float, float, float, float]:
    with map_yaml.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    ref = cfg.get("ref_gnss_10") or cfg.get("ref_gnss")
    if not ref:
        raise ValueError(f"no ref_gnss in {map_yaml}")
    lon0 = float(ref[0])
    lat0 = float(ref[1])
    origin = cfg.get("origin") or [0.0, 0.0, 0.0]
    ox = float(origin[0]) if len(origin) > 0 else 0.0
    oy = float(origin[1]) if len(origin) > 1 else 0.0
    oyaw = float(origin[2]) if len(origin) > 2 else 0.0
    return lat0, lon0, ox, oy, oyaw


def _geodetic_delta_enu_m(
    lat0: float, lon0: float, lat: float, lon: float
) -> Tuple[float, float]:
    r_earth = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = r_earth * math.cos(math.radians(lat0)) * dlon
    north = r_earth * dlat
    return east, north


def _enu_to_map_xy(
    east: float, north: float, ox: float, oy: float, origin_yaw: float
) -> Tuple[float, float]:
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    mx = ox + east * c - north * s
    my = oy + east * s + north * c
    return mx, my


def gnss_to_map_xy(
    gnss: GnssStaging,
    map_yaml: Path,
) -> Tuple[float, float]:
    lat0, lon0, ox, oy, oyaw = _load_map_datum(map_yaml)
    east, north = _geodetic_delta_enu_m(lat0, lon0, gnss.latitude, gnss.longitude)
    return _enu_to_map_xy(east, north, ox, oy, oyaw)


def map_xy_to_gnss(
    x: float,
    y: float,
    map_yaml: Path,
) -> GnssStaging:
    """Inverse ENU (flat earth) for staging calibration from map pose."""
    lat0, lon0, ox, oy, oyaw = _load_map_datum(map_yaml)
    dx = x - ox
    dy = y - oy
    c = math.cos(origin_yaw := oyaw)
    s = math.sin(origin_yaw)
    east = dx * c + dy * s
    north = -dx * s + dy * c
    r_earth = 6378137.0
    lat = lat0 + math.degrees(north / r_earth)
    lon = lon0 + math.degrees(
        east / (r_earth * math.cos(math.radians(lat0)))
    )
    return GnssStaging(latitude=lat, longitude=lon, yaw_deg=0.0)


def make_staging_pose(
    bay: BayRecord,
    *,
    frame_id: str = "map",
    use_gnss: bool = True,
    map_yaml: Optional[Path] = None,
) -> PoseStamped:
    if use_gnss:
        if bay.gnss.latitude == 0.0 and bay.gnss.longitude == 0.0:
            raise ValueError(
                f"bay {bay.bay_id}: gnss_staging not configured in dock_database.yaml"
            )
        if map_yaml is None:
            raise ValueError("map_yaml required when use_gnss=True")
        x, y = gnss_to_map_xy(bay.gnss, map_yaml)
        yaw = math.radians(bay.gnss.yaw_deg)
    else:
        x, y = bay.map_pose.x, bay.map_pose.y
        yaw = bay.map_pose.yaw
    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = _yaw_to_quaternion(yaw)
    return pose
