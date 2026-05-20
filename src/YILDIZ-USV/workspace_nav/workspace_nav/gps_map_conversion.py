#!/usr/bin/env python3
# Pure helpers shared by waypoint_transform and mission_bridge (no rclpy).

from __future__ import annotations

import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Tuple

import utm


class WaypointParseResult(NamedTuple):
    """地面站 JSON 解析结果：`explicit_replan` 表示操纵员显式下令重规划（如 GCS Run Mission）。"""

    waypoints: List[Tuple[float, float]]
    explicit_replan: bool


def parse_waypoint_message(data_str: str) -> Optional[WaypointParseResult]:
    """与 `parse_waypoint_payload` 相同几何解析，并从字典载荷读取显式重申航迹标志。"""
    data_str = str(data_str).strip()
    if data_str.startswith("data: "):
        data_str = data_str[6:].strip()
    try:
        json_data = json.loads(data_str)
    except json.JSONDecodeError:
        return None

    explicit = False

    waypoints_list: Optional[List[Any]] = None
    if isinstance(json_data, list):
        waypoints_list = json_data
    elif isinstance(json_data, dict) and "waypoints" in json_data:
        wl = json_data["waypoints"]
        if isinstance(wl, list):
            waypoints_list = wl
        for key in (
            "explicit_replan",
            "from_start_mission",
            "start_mission",
            "force_replan",
            "restart_mission",
        ):
            v = json_data.get(key)
            if isinstance(v, bool):
                explicit = explicit or v
            elif isinstance(v, (int, float)) and v != 0:
                explicit = True
            elif isinstance(v, str) and v.strip().lower() in ("true", "1", "yes", "on"):
                explicit = True

        cmd = json_data.get("command")
        if isinstance(cmd, str):
            lc = cmd.strip().lower()
            if lc in ("start", "replan", "restart", "reload"):
                explicit = True
    if not waypoints_list:
        return None

    parsed: List[Tuple[float, float]] = []
    for wp in waypoints_list:
        if isinstance(wp, dict) and "latitude" in wp and "longitude" in wp:
            lat = float(wp["latitude"])
            lon = float(wp["longitude"])
        elif isinstance(wp, (list, tuple)) and len(wp) >= 2:
            lat = float(wp[0])
            lon = float(wp[1])
        else:
            return None
        if lat == 0.0 and lon == 0.0:
            return None
        if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
            return None
        parsed.append((lat, lon))
    if not parsed:
        return None
    return WaypointParseResult(parsed, explicit)



def read_map_ref_lon_lat(cfg: dict, ref_key: str) -> Tuple[float, float]:
    """读取 ref_gnss* 条目。

    约定：数组为 **[longitude_deg, latitude_deg]**，
    返回 **(纬度, 经度)**。
    """
    if ref_key not in cfg:
        raise ValueError(f"map.yaml 中未找到 {ref_key}")
    arr = cfg[ref_key]
    lon = float(arr[0])
    lat = float(arr[1])
    return lat, lon


def datum_lat_lon_from_cfg(cfg: dict, ref_key: str) -> Tuple[float, float]:
    if ref_key and ref_key.strip() and ref_key in cfg:
        return read_map_ref_lon_lat(cfg, ref_key)
    for key in ("ref_gnss_10", "ref_gnss"):
        if key not in cfg:
            continue
        arr = cfg[key]
        lon = float(arr[0])
        lat = float(arr[1])
        return lat, lon
    raise ValueError("map.yaml 中未找到 ref_gnss_10 / ref_gnss 或指定 datum_ref_key")


def read_map_origin(cfg: dict) -> Tuple[float, float, float]:
    """map_server 风格 origin: [ox, oy, yaw_rad]。缺省为 (0,0,0)。"""
    origin = cfg.get("origin")
    if not origin:
        return 0.0, 0.0, 0.0
    if isinstance(origin, (list, tuple)):
        ox = float(origin[0]) if len(origin) > 0 else 0.0
        oy = float(origin[1]) if len(origin) > 1 else 0.0
        oyaw = float(origin[2]) if len(origin) > 2 else 0.0
        return ox, oy, oyaw
    return 0.0, 0.0, 0.0


def geodetic_delta_enu_m(
    lat0: float, lon0: float, lat: float, lon: float
) -> Tuple[float, float]:
    r_earth = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east = r_earth * math.cos(math.radians(lat0)) * dlon
    north = r_earth * dlat
    return east, north


def enu_delta_to_map_xy(
    east: float, north: float, ox: float, oy: float, origin_yaw: float
) -> Tuple[float, float]:
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    mx = ox + east * c - north * s
    my = oy + east * s + north * c
    return mx, my


def parse_waypoint_payload(data_str: str) -> Optional[List[Tuple[float, float]]]:
    """地面站载荷：顶层 list、或 {\"waypoints\": [...]}；点可为 {lat,lon} 或 [lat,lon]。

    （忽略字典中的重申标志；如需显式重申语义请使用 `parse_waypoint_message`。）
    """
    r = parse_waypoint_message(data_str)
    return r.waypoints if r else None


def lat_lon_list_to_waypoints_document(
    waypoints_ll: List[Tuple[float, float]],
    datum_lat: float,
    datum_lon: float,
    datum_easting: float,
    datum_northing: float,
    projection: str,
    datum_source: str,
    map_yaml_resolved: str,
    datum_ref_key: str,
    map_ox: float,
    map_oy: float,
    map_origin_yaw: float,
) -> Dict[str, Any]:
    proj = projection if projection in ("enu", "utm") else "enu"
    output: Dict[str, Any] = {
        "waypoints": [],
        "datum": {"latitude": float(datum_lat), "longitude": float(datum_lon)},
        "datum_source_meta": {
            "datum_source": datum_source,
            "map_yaml": map_yaml_resolved or None,
            "map_datum_ref_key": datum_ref_key if datum_source == "map_yaml" else None,
            "projection": proj,
        },
        "map_frame_meta": {
            "frame_id": "map",
            "description": (
                "x,y 为 Nav2 map 坐标（米）。datum_source=map_yaml 时已应用 map.yaml 的 origin 平移与旋转；"
                "first_gps 时为相对首帧 GPS 的平面坐标，不一定与 map 一致。"
            ),
            "origin_x": float(map_ox),
            "origin_y": float(map_oy),
            "origin_yaw_rad": float(map_origin_yaw),
            "applied_origin_transform": bool(datum_source == "map_yaml"),
        },
    }
    for lat, lon in waypoints_ll:
        if proj == "utm":
            easting, northing, _, _ = utm.from_latlon(float(lat), float(lon))
            east = float(easting - datum_easting)
            north = float(northing - datum_northing)
        else:
            east, north = geodetic_delta_enu_m(
                float(datum_lat), float(datum_lon), float(lat), float(lon)
            )
        if datum_source == "map_yaml":
            x, y = enu_delta_to_map_xy(east, north, map_ox, map_oy, map_origin_yaw)
        else:
            x, y = east, north
        output["waypoints"].append(
            {"latitude": float(lat), "longitude": float(lon), "x": round(x, 4), "y": round(y, 4)}
        )
    return output


def atomic_write_json(output_dir: Path, output_file: Path, data: Dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fd = None
    tmp_path = None
    try:
        fd, tmp_path = tempfile.mkstemp(
            dir=str(output_dir), prefix="json_", suffix=".tmp"
        )
        with os.fdopen(fd, "w") as tf:
            json.dump(data, tf, indent=2)
            tf.flush()
            os.fsync(tf.fileno())
        fd = None
        if output_file.exists():
            try:
                output_file.unlink()
            except Exception:
                pass
        os.replace(tmp_path, str(output_file))
        tmp_path = None
    finally:
        if tmp_path and os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception:
                pass


def verify_waypoints_file(path: Path) -> bool:
    try:
        with path.open("r", encoding="utf-8") as vf:
            loaded = json.load(vf)
        if not isinstance(loaded, dict):
            return False
        if "waypoints" not in loaded or "datum" not in loaded:
            return False
        if not isinstance(loaded.get("waypoints"), list):
            return False
        return isinstance(loaded.get("datum"), dict)
    except Exception:
        return False
