"""WGS84 ↔ dock-relative frame for usv_docking GNSS approach (no map/Nav2)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from usv_docking.dock_enu import Pose2D

EARTH_RADIUS_M = 6378137.0


def wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


@dataclass(frozen=True)
class GnssPoint:
    latitude: float
    longitude: float


@dataclass(frozen=True)
class GnssBayGeometry:
    """Dock center anchor + virtual entry target in dock-relative frame."""

    bay_id: str
    dock_center: GnssPoint
    virtual_entry: GnssPoint
    entry_yaw_rad: float
    into_dock_yaw_rad: float
    entry_pose_dock: Pose2D
    standoff_m: float


def geodetic_delta_enu_m(
    lat0: float, lon0: float, lat: float, lon: float
) -> Tuple[float, float]:
    """East/north offset (m) from (lat0, lon0) to (lat, lon)."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = EARTH_RADIUS_M * math.cos(math.radians(lat0)) * dlon
    north = EARTH_RADIUS_M * dlat
    return east, north


def offset_gnss_flat(
    lat0: float, lon0: float, east_m: float, north_m: float
) -> GnssPoint:
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat0)))
    return GnssPoint(
        latitude=lat0 + math.degrees(dlat),
        longitude=lon0 + math.degrees(dlon),
    )


def seaward_offset_gnss(
    center: GnssPoint, boat_yaw_rad: float, standoff_m: float
) -> GnssPoint:
    """Virtual entry standoff_m toward sea along boat forward axis (yaw)."""
    east = standoff_m * math.cos(boat_yaw_rad)
    north = standoff_m * math.sin(boat_yaw_rad)
    return offset_gnss_flat(center.latitude, center.longitude, east, north)


def enu_to_dock_xy(east: float, north: float, into_dock_yaw: float) -> Tuple[float, float]:
    """Rotate ENU into dock frame (+x = into dock)."""
    c = math.cos(-into_dock_yaw)
    s = math.sin(-into_dock_yaw)
    x = c * east - s * north
    y = s * east + c * north
    return x, y


def latlon_to_dock_pose(
    lat: float,
    lon: float,
    center: GnssPoint,
    into_dock_yaw: float,
    yaw_rad: float,
) -> Pose2D:
    east, north = geodetic_delta_enu_m(
        center.latitude, center.longitude, lat, lon
    )
    x, y = enu_to_dock_xy(east, north, into_dock_yaw)
    return Pose2D(x=x, y=y, yaw=wrap_yaw(yaw_rad))


def build_bay_geometry(
    bay_id: str,
    center_lat: float,
    center_lon: float,
    yaw_deg: float,
    virtual_lat: Optional[float] = None,
    virtual_lon: Optional[float] = None,
    standoff_m: Optional[float] = None,
) -> GnssBayGeometry:
    center = GnssPoint(latitude=center_lat, longitude=center_lon)
    entry_yaw = math.radians(yaw_deg)
    into_dock = wrap_yaw(entry_yaw + math.pi)

    if virtual_lat is not None and virtual_lon is not None:
        entry_gnss = GnssPoint(latitude=virtual_lat, longitude=virtual_lon)
        entry_pose = latlon_to_dock_pose(
            virtual_lat, virtual_lon, center, into_dock, entry_yaw
        )
        resolved_standoff_m = abs(entry_pose.x)
    elif standoff_m is not None and standoff_m > 0.0:
        entry_gnss = seaward_offset_gnss(center, entry_yaw, standoff_m)
        entry_pose = Pose2D(x=-standoff_m, y=0.0, yaw=entry_yaw)
        resolved_standoff_m = standoff_m
    else:
        raise ValueError(
            f"bay {bay_id}: set virtual_entry_gnss or virtual_entry_standoff_m"
        )

    return GnssBayGeometry(
        bay_id=bay_id,
        dock_center=center,
        virtual_entry=entry_gnss,
        entry_yaw_rad=entry_yaw,
        into_dock_yaw_rad=into_dock,
        entry_pose_dock=entry_pose,
        standoff_m=resolved_standoff_m,
    )
