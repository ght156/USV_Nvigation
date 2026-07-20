"""Load usv_docking dock_geometry_*.yaml (GNSS virtual entry, no dock_mission)."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml

from usv_docking.gnss_geo import GnssBayGeometry, build_bay_geometry


def default_geometry_path(geometry_file: str = "dock_geometry_sim.yaml") -> Path:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("usv_docking"))
    return share / "config" / geometry_file


def load_gnss_bay_geometry(
    bay_id: str,
    geometry_path: Optional[str | Path] = None,
    geometry_file: str = "dock_geometry_sim.yaml",
) -> GnssBayGeometry:
    if geometry_path:
        path = Path(geometry_path)
    else:
        path = default_geometry_path(geometry_file)
    with path.open(encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    bays = root.get("bays") or {}
    if bay_id not in bays:
        raise KeyError(f"bay '{bay_id}' not in {path}")
    data = bays[bay_id]
    center = data.get("dock_center_gnss") or {}
    entry = data.get("virtual_entry_gnss") or {}
    standoff = data.get("virtual_entry_standoff_m")
    yaw_deg = float(
        center.get(
            "yaw_deg",
            entry.get("yaw_deg", data.get("entry_yaw_deg", 0.0)),
        )
    )
    return build_bay_geometry(
        bay_id=bay_id,
        center_lat=float(center["latitude"]),
        center_lon=float(center["longitude"]),
        yaw_deg=yaw_deg,
        virtual_lat=float(entry["latitude"]) if entry.get("latitude") is not None else None,
        virtual_lon=float(entry["longitude"]) if entry.get("longitude") is not None else None,
        standoff_m=float(standoff) if standoff is not None else None,
    )
