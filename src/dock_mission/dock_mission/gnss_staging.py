"""Staging 航点生成：实船契约 = WGS84 (lat, lon, yaw)。

use_gnss=True 直接透传泊位库 gnss_staging；False 时把 map_staging 经
map_yaml(ref_gnss) 反解为 WGS84（标定/调试用）。经纬度→map 正解由
mission_bridge 内部完成，本模块不含正解。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional, Tuple

import yaml

from dock_mission.bay_loader import BayRecord, GnssStaging


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


def make_staging_waypoint(
    bay: BayRecord,
    *,
    use_gnss: bool = True,
    map_yaml: Optional[Path] = None,
) -> Tuple[float, float, float]:
    """返回 (latitude, longitude, yaw_deg)，对应实船 m_common/MissionWaypoint 契约。

    实船 mission_bridge 的 SendWaypoints 只收 WGS84 航点（经纬度→map 换算
    由 mission_bridge 按地图 datum 完成），与仿真仓的 map 系 PoseStamped 不同。
    use_gnss=True 直接用泊位库 gnss_staging；False 时把 map_staging 经
    map_yaml(ref_gnss_10) 反解为 WGS84（标定/调试用）。
    **yaw 为度**（ENU/map 系航向，MissionWaypoint.yaw 语义；接口统一用度）。
    """
    if use_gnss:
        if bay.gnss.latitude == 0.0 and bay.gnss.longitude == 0.0:
            raise ValueError(
                f"bay {bay.bay_id}: gnss_staging not configured in dock_database.yaml"
            )
        return (
            float(bay.gnss.latitude),
            float(bay.gnss.longitude),
            float(bay.gnss.yaw_deg),
        )
    if map_yaml is None:
        raise ValueError("map_yaml required when use_gnss=False (map→WGS84 反解)")
    gnss = map_xy_to_gnss(bay.map_pose.x, bay.map_pose.y, map_yaml)
    return (gnss.latitude, gnss.longitude, math.degrees(float(bay.map_pose.yaw)))
