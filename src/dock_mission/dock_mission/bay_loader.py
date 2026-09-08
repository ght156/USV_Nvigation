"""Load dock_database.yaml bay definitions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import yaml
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class GnssStaging:
    latitude: float
    longitude: float
    yaw_deg: float


@dataclass(frozen=True)
class MapStaging:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class BayRecord:
    bay_id: str
    name: str
    gnss: GnssStaging
    map_pose: MapStaging
    standoff_m: float


def default_database_path() -> Path:
    share = Path(get_package_share_directory("dock_mission"))
    return share / "config" / "dock_database.yaml"


def load_bay(
    bay_id: str,
    database_path: Optional[str | Path] = None,
) -> BayRecord:
    path = Path(database_path) if database_path else default_database_path()
    with path.open(encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    bays: dict[str, Any] = root.get("bays") or {}
    if bay_id not in bays:
        raise KeyError(f"bay '{bay_id}' not in {path}")
    data = bays[bay_id]
    gnss_raw = data.get("gnss_staging") or {}
    map_raw = data.get("map_staging") or {}
    return BayRecord(
        bay_id=bay_id,
        name=str(data.get("name", bay_id)),
        gnss=GnssStaging(
            latitude=float(gnss_raw.get("latitude", 0.0)),
            longitude=float(gnss_raw.get("longitude", 0.0)),
            yaw_deg=float(gnss_raw.get("yaw_deg", 0.0)),
        ),
        map_pose=MapStaging(
            x=float(map_raw.get("x", 0.0)),
            y=float(map_raw.get("y", 0.0)),
            yaw=float(map_raw.get("yaw", 0.0)),
        ),
        standoff_m=float(data.get("standoff_m", 4.0)),
    )
