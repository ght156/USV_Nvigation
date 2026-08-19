#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# zone_manager：电子围栏（作业区/禁止区）+ 硬边界 → Nav2 KeepoutFilter 动态掩码。
#
# 围栏模型（与 decision 对接说明 §6 对齐）：
#   GeoFence{fence_id, type: exclusion|inclusion, shape: polygon|circle, points/center/radius_m}
#   - inclusion（作业区）：船不得驶出；多个 inclusion 取并集
#   - exclusion（禁航区/禁止区）：船不得进入；优先级高于 inclusion
#   - 硬边界：不闭合折线（本项目扩展，decision 文档未覆盖），不可穿越
#
# 输入：
#   - service mission_bridge/set_geofence   (m_common/srv/SetGeoFence，全量快照，空数组=清除围栏)
#   - service mission_bridge/get_geofence   (m_common/srv/GetGeoFence，查询围栏快照)
#   - service mission_bridge/set_nav_zones  (m_common/srv/SetNavZones，旧接口兼容，自动转围栏)
#   - service mission_bridge/clear_nav_zones(std_srvs/srv/Trigger，清除围栏+硬边界)
#   - service mission_bridge/get_nav_zones  (m_common/srv/GetNavZones，旧接口查询)
#   - topic   /nav_zones (std_msgs/String JSON，GCS 通道；{"clear": true} 清除全部)
# 输出（RELIABLE + TRANSIENT_LOCAL）：
#   - /keepout_filter/costmap_filter_info (nav2_msgs/CostmapFilterInfo)
#   - /keepout_filter/keepout_filter_mask (nav_msgs/OccupancyGrid，100=禁行)
#   - /nav_zones/current (std_msgs/String JSON，当前生效围栏+硬边界，带 fence_id)
# 持久化：成功变更后原子写 zones_persist_path（默认 workspace_nav/json/nav_zones.json），
#   启动时自动恢复；只有平台显式覆盖/清除才失效。
# ----------------------------------------------------------------------------------------------- #
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from m_common.msg import GeoFence, GeoPoint, GeoPolygon, NavZones
from m_common.srv import GetGeoFence, GetNavZones, SetGeoFence, SetNavZones

from workspace_nav.gps_map_conversion import (
    atomic_write_json,
    datum_lat_lon_from_cfg,
    enu_delta_to_map_xy,
    geodetic_delta_enu_m,
    read_map_origin,
)
from workspace_nav.zone_geometry import KEEPOUT, build_fence_mask

LatLon = Tuple[float, float]  # (lon, lat)
Fence = Dict[str, Any]  # {fence_id, type, shape, points:[LatLon], center:LatLon|None, radius_m}

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_FENCE_TYPES = ("exclusion", "inclusion")
_FENCE_SHAPES = ("polygon", "circle")


def _default_persist_path() -> Path:
    """与 mission_bridge 同一套 workspace 查找：定位 workspace_nav/json/ 目录。"""
    try:
        start = Path(__file__).resolve()
    except Exception:
        start = Path.cwd().resolve()
    for p in [start] + list(start.parents):
        for rel in (
            ("src", "USV_NAV", "workspace_nav", "json"),
            ("USV_NAV", "workspace_nav", "json"),
        ):
            d = p.joinpath(*rel)
            if d.is_dir():
                return (d / "nav_zones.json").resolve()
    share = Path(get_package_share_directory("workspace_nav"))
    return (share / "json" / "nav_zones.json").resolve()


class ZoneManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("zone_manager")

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_datum_ref_key", "ref_gnss_10")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("filter_info_topic", "/keepout_filter/costmap_filter_info")
        self.declare_parameter("filter_mask_topic", "/keepout_filter/keepout_filter_mask")
        self.declare_parameter("zones_input_topic", "/nav_zones")
        self.declare_parameter("zones_current_topic", "/nav_zones/current")
        self.declare_parameter("boundary_dilation_cells", 2)
        # 周期重发掩码/filter_info 的间隔（s）；0 = 关闭（TRANSIENT_LOCAL 已保证晚加入者补发）
        self.declare_parameter("republish_period_sec", 0.0)
        self.declare_parameter("zones_persist_path", "")

        self._global_frame = (
            self.get_parameter("global_frame").get_parameter_value().string_value.strip() or "map"
        )
        ref_key = (
            self.get_parameter("map_datum_ref_key").get_parameter_value().string_value.strip()
            or "ref_gnss_10"
        )
        self._dilation = max(0, int(self.get_parameter("boundary_dilation_cells").value))

        persist_param = (
            self.get_parameter("zones_persist_path").get_parameter_value().string_value.strip()
        )
        self._persist_path = (
            Path(persist_param).expanduser().resolve()
            if persist_param
            else _default_persist_path()
        )

        # ---- 地图锚点（与 mission_bridge 同一约定：ref_gnss_* 为 [lon, lat]） ----
        map_yaml_param = (
            self.get_parameter("map_yaml_path").get_parameter_value().string_value.strip()
        )
        if map_yaml_param:
            map_path = Path(map_yaml_param).expanduser().resolve()
        else:
            map_path = (
                Path(get_package_share_directory("workspace_nav")) / "config" / "map.yaml"
            ).resolve()
        try:
            with map_path.open("r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            self._datum_lat, self._datum_lon = datum_lat_lon_from_cfg(cfg, ref_key)
            self._map_ox, self._map_oy, self._map_origin_yaw = read_map_origin(cfg)
        except Exception as e:
            self.get_logger().fatal(f"读取地图锚点失败 {map_path}: {e}")
            raise SystemExit(1) from e
        self.get_logger().info(
            f"map yaml: {map_path} datum=({self._datum_lat}, {self._datum_lon}) "
            f"origin=({self._map_ox}, {self._map_oy}, {self._map_origin_yaw})"
        )

        # ---- 状态 ----
        self._fences: List[Fence] = []
        self._hard_boundaries: List[List[LatLon]] = []
        # /map 栅格元信息：(width, height, resolution, origin_x, origin_y)
        self._map_info: Optional[Tuple[int, int, float, float, float]] = None
        self._mask: Optional[List[int]] = None

        # ---- 发布 ----
        self._filter_info_pub = self.create_publisher(
            CostmapFilterInfo,
            self.get_parameter("filter_info_topic").value,
            _LATCHED_QOS,
        )
        self._mask_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("filter_mask_topic").value,
            _LATCHED_QOS,
        )
        self._zones_current_pub = self.create_publisher(
            String,
            self.get_parameter("zones_current_topic").value,
            _LATCHED_QOS,
        )

        # ---- 订阅 ----
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._cb_map, _LATCHED_QOS
        )
        self.create_subscription(
            String, self.get_parameter("zones_input_topic").value, self._cb_zones_json, 10
        )

        # ---- 服务 ----
        self.create_service(SetGeoFence, "mission_bridge/set_geofence", self._cb_set_geofence)
        self.create_service(GetGeoFence, "mission_bridge/get_geofence", self._cb_get_geofence)
        self.create_service(SetNavZones, "mission_bridge/set_nav_zones", self._cb_set_nav_zones)
        self.create_service(Trigger, "mission_bridge/clear_nav_zones", self._cb_clear_nav_zones)
        self.create_service(GetNavZones, "mission_bridge/get_nav_zones", self._cb_get_nav_zones)

        period = float(self.get_parameter("republish_period_sec").value)
        if period > 0.0:
            self.create_timer(max(0.2, period), self._republish_timer)

        # ---- 启动恢复持久化区域 ----
        self._load_persisted()

        self.get_logger().info(
            "zone_manager ready: set/get_geofence, set/clear/get_nav_zones, /nav_zones; "
            f"persist={self._persist_path}"
        )

    # ------------------------------------------------------------------ #
    # 坐标换算
    # ------------------------------------------------------------------ #
    def _latlon_to_map_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        east, north = geodetic_delta_enu_m(self._datum_lat, self._datum_lon, lat, lon)
        return enu_delta_to_map_xy(east, north, self._map_ox, self._map_oy, self._map_origin_yaw)

    def _map_xy_to_grid(self, x: float, y: float) -> Tuple[float, float]:
        assert self._map_info is not None
        _, _, res, ox, oy = self._map_info
        return (x - ox) / res, (y - oy) / res

    # ------------------------------------------------------------------ #
    # 围栏校验与转换
    # ------------------------------------------------------------------ #
    @staticmethod
    def _valid_lat(lat: float) -> bool:
        return math.isfinite(lat) and -90.0 <= lat <= 90.0

    @staticmethod
    def _valid_lon(lon: float) -> bool:
        return math.isfinite(lon) and -180.0 <= lon <= 180.0

    def _validate_fence(self, f: Fence) -> Optional[str]:
        fid = f.get("fence_id", "")
        if not fid:
            return "fence_id 不能为空"
        if f.get("type") not in _FENCE_TYPES:
            return f"围栏 {fid}: type 必须为 exclusion|inclusion（收到 {f.get('type')!r}）"
        shape = f.get("shape") or "polygon"
        if shape not in _FENCE_SHAPES:
            return f"围栏 {fid}: shape 必须为 polygon|circle（收到 {shape!r}）"
        if shape == "polygon":
            pts = f.get("points") or []
            if len(pts) < 3:
                return f"围栏 {fid}: 多边形至少需要 3 个点（收到 {len(pts)} 个）"
            for lon, lat in pts:
                if not self._valid_lat(lat) or not self._valid_lon(lon):
                    return f"围栏 {fid}: 经纬度非法 ({lon}, {lat})"
            if len(set(pts)) != len(pts):
                return f"围栏 {fid}: 多边形顶点存在重复点"
        else:
            c = f.get("center")
            r = float(f.get("radius_m") or 0.0)
            if c is None or not self._valid_lat(c[1]) or not self._valid_lon(c[0]):
                return f"围栏 {fid}: 圆心非法"
            if not math.isfinite(r) or r <= 0.0:
                return f"围栏 {fid}: radius_m 必须 > 0（收到 {f.get('radius_m')}）"
        return None

    def _validate_fences(self, fences: Sequence[Fence]) -> Optional[str]:
        seen = set()
        for f in fences:
            err = self._validate_fence(f)
            if err:
                return err
            if f["fence_id"] in seen:
                return f"fence_id 重复: {f['fence_id']}"
            seen.add(f["fence_id"])
        return None

    # ------------------------------------------------------------------ #
    # /map 回调
    # ------------------------------------------------------------------ #
    def _cb_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        new_info = (
            int(info.width),
            int(info.height),
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
        )
        if new_info != self._map_info:
            self._map_info = new_info
            self.get_logger().info(
                f"/map 就绪: {new_info[0]}x{new_info[1]} res={new_info[2]} "
                f"origin=({new_info[3]}, {new_info[4]})"
            )
            self._rasterize_and_publish()

    # ------------------------------------------------------------------ #
    # 区域应用（统一入口：校验 → 更新状态 → 持久化 → 发布）
    # ------------------------------------------------------------------ #
    def _apply(
        self,
        fences: Optional[List[Fence]],
        hard_boundaries: Optional[List[List[LatLon]]],
        source: str,
    ) -> Tuple[bool, str]:
        """fences / hard_boundaries 为 None 表示该项不变，否则全量替换。"""
        if fences is not None:
            err = self._validate_fences(fences)
            if err:
                return False, err
        if hard_boundaries is not None:
            for i, line in enumerate(hard_boundaries):
                if len(line) < 2:
                    return False, f"硬边界 #{i} 至少需要 2 个点（收到 {len(line)} 个）"
                for lon, lat in line:
                    if not self._valid_lat(lat) or not self._valid_lon(lon):
                        return False, f"硬边界 #{i} 经纬度非法 ({lon}, {lat})"

        if fences is not None:
            self._fences = fences
        if hard_boundaries is not None:
            self._hard_boundaries = hard_boundaries

        self.get_logger().info(
            f"[{source}] 区域已更新: 围栏 {len(self._fences)} 个 "
            f"(inclusion={sum(1 for f in self._fences if f['type'] == 'inclusion')}, "
            f"exclusion={sum(1 for f in self._fences if f['type'] == 'exclusion')}), "
            f"硬边界 {len(self._hard_boundaries)} 条"
        )
        self._persist()
        self._publish_zones_current()
        if self._map_info is None:
            self.get_logger().warning("尚未收到 /map，掩码将在地图就绪后发布")
            return True, "zones accepted (mask pending /map)"
        self._rasterize_and_publish()
        return True, (
            f"applied: fences={len(self._fences)} hard_boundaries={len(self._hard_boundaries)}"
        )

    # ------------------------------------------------------------------ #
    # 服务回调
    # ------------------------------------------------------------------ #
    def _cb_set_geofence(
        self, request: SetGeoFence.Request, response: SetGeoFence.Response
    ) -> SetGeoFence.Response:
        fences = [self._msg_to_fence(f) for f in request.fences]
        ok, msg = self._apply(fences, None, "set_geofence")
        response.success = ok
        response.message = msg
        if not ok:
            self.get_logger().warning(f"set_geofence 被拒绝（保留旧围栏）: {msg}")
        return response

    def _cb_get_geofence(
        self, request: GetGeoFence.Request, response: GetGeoFence.Response
    ) -> GetGeoFence.Response:
        response.success = True
        response.fences = [self._fence_to_msg(f) for f in self._fences]
        return response

    def _cb_set_nav_zones(
        self, request: SetNavZones.Request, response: SetNavZones.Response
    ) -> SetNavZones.Response:
        """旧接口兼容：work_area → inclusion 围栏，forbidden_zones → exclusion 围栏。"""
        fences: List[Fence] = []
        wa = self._msg_poly_to_list(request.zones.work_area)
        if wa:
            fences.append(
                {"fence_id": "work_area", "type": "inclusion", "shape": "polygon",
                 "points": wa, "center": None, "radius_m": 0.0}
            )
        for i, p in enumerate(request.zones.forbidden_zones):
            pts = self._msg_poly_to_list(p)
            if pts:
                fences.append(
                    {"fence_id": f"forbidden_{i}", "type": "exclusion", "shape": "polygon",
                     "points": pts, "center": None, "radius_m": 0.0}
                )
        hb = [self._msg_poly_to_list(p) for p in request.zones.hard_boundaries]
        ok, msg = self._apply(fences, hb, "set_nav_zones")
        response.success = ok
        response.message = msg
        if not ok:
            self.get_logger().warning(f"set_nav_zones 被拒绝: {msg}")
        return response

    def _cb_clear_nav_zones(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, _ = self._apply([], [], "clear_nav_zones")
        response.success = ok
        response.message = "all nav zones cleared"
        return response

    def _cb_get_nav_zones(
        self, request: GetNavZones.Request, response: GetNavZones.Response
    ) -> GetNavZones.Response:
        response.success = True
        response.zones = self._fences_to_nav_zones_msg()
        return response

    # ------------------------------------------------------------------ #
    # GCS JSON 通道
    # ------------------------------------------------------------------ #
    def _cb_zones_json(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f"/nav_zones JSON 解析失败: {e}")
            return
        if isinstance(data, dict) and data.get("clear"):
            self._apply([], [], "/nav_zones")
            return
        if not isinstance(data, dict):
            self.get_logger().warning("/nav_zones 载荷须为 JSON 对象")
            return

        fences: Optional[List[Fence]] = None
        if "fences" in data:
            fences = [self._json_to_fence(f) for f in (data.get("fences") or [])]
        elif "work_area" in data or "forbidden_zones" in data or "forbidden" in data:
            # 旧格式兼容
            fences = []
            wa = self._json_poly(data.get("work_area"))
            if wa:
                fences.append(
                    {"fence_id": "work_area", "type": "inclusion", "shape": "polygon",
                     "points": wa, "center": None, "radius_m": 0.0}
                )
            for i, p in enumerate(data.get("forbidden_zones") or data.get("forbidden") or []):
                pts = self._json_poly(p)
                if pts:
                    fences.append(
                        {"fence_id": f"forbidden_{i}", "type": "exclusion", "shape": "polygon",
                         "points": pts, "center": None, "radius_m": 0.0}
                    )
        hb = None
        if "hard_boundaries" in data or "boundaries" in data:
            hb = [self._json_poly(p) for p in (data.get("hard_boundaries") or data.get("boundaries") or [])]

        if fences is None and hb is None:
            self.get_logger().warning("/nav_zones 载荷无 fences/work_area/hard_boundaries 字段")
            return
        ok, msg_txt = self._apply(fences, hb, "/nav_zones")
        if not ok:
            self.get_logger().warning(f"/nav_zones 区域被拒绝: {msg_txt}")

    @staticmethod
    def _json_poly(raw: Any) -> List[LatLon]:
        """点可为 [lon, lat] 或 {"longitude": .., "latitude": ..} 或 {"lon": .., "lat": ..}。"""
        pts: List[LatLon] = []
        if not isinstance(raw, list):
            return pts
        for p in raw:
            try:
                if isinstance(p, dict):
                    lon = p.get("longitude", p.get("lon"))
                    lat = p.get("latitude", p.get("lat"))
                    pts.append((float(lon), float(lat)))
                elif isinstance(p, (list, tuple)) and len(p) >= 2:
                    pts.append((float(p[0]), float(p[1])))
            except (TypeError, ValueError):
                continue
        return pts

    def _json_to_fence(self, raw: Any) -> Fence:
        if not isinstance(raw, dict):
            return {"fence_id": "", "type": "", "shape": "polygon",
                    "points": [], "center": None, "radius_m": 0.0}
        center = None
        c = raw.get("center")
        if isinstance(c, dict):
            try:
                center = (float(c.get("lon", c.get("longitude"))), float(c.get("lat", c.get("latitude"))))
            except (TypeError, ValueError):
                center = None
        elif isinstance(c, (list, tuple)) and len(c) >= 2:
            center = (float(c[0]), float(c[1]))
        return {
            "fence_id": str(raw.get("fence_id", "")),
            "type": str(raw.get("type", "")),
            "shape": str(raw.get("shape", "polygon") or "polygon"),
            "points": self._json_poly(raw.get("points")),
            "center": center,
            "radius_m": float(raw.get("radius_m") or 0.0),
        }

    @staticmethod
    def _msg_poly_to_list(poly: GeoPolygon) -> List[LatLon]:
        n = min(len(poly.longitude), len(poly.latitude))
        return [(float(poly.longitude[i]), float(poly.latitude[i])) for i in range(n)]

    @staticmethod
    def _msg_to_fence(msg: GeoFence) -> Fence:
        return {
            "fence_id": msg.fence_id.strip(),
            "type": msg.type.strip(),
            "shape": msg.shape.strip() or "polygon",
            "points": [(float(p.lon), float(p.lat)) for p in msg.points],
            "center": (float(msg.center.lon), float(msg.center.lat))
            if (msg.center.lon != 0.0 or msg.center.lat != 0.0)
            else None,
            "radius_m": float(msg.radius_m),
        }

    @staticmethod
    def _fence_to_msg(f: Fence) -> GeoFence:
        msg = GeoFence()
        msg.fence_id = f["fence_id"]
        msg.type = f["type"]
        msg.shape = f["shape"]
        msg.points = [GeoPoint(lat=lat, lon=lon) for lon, lat in f["points"]]
        if f["center"] is not None:
            msg.center = GeoPoint(lat=f["center"][1], lon=f["center"][0])
        msg.radius_m = float(f["radius_m"])
        return msg

    def _fences_to_nav_zones_msg(self) -> NavZones:
        """旧接口查询：circle 围栏以 16 边形近似返回。"""
        def to_poly(pts: Sequence[LatLon]) -> GeoPolygon:
            poly = GeoPolygon()
            poly.longitude = [float(lon) for lon, _ in pts]
            poly.latitude = [float(lat) for _, lat in pts]
            return poly

        msg = NavZones()
        for f in self._fences:
            pts = list(f["points"])
            if f["shape"] == "circle" and f["center"] is not None:
                clon, clat = f["center"]
                pts = []
                for k in range(16):
                    ang = 2.0 * math.pi * k / 16.0
                    east = f["radius_m"] * math.cos(ang)
                    north = f["radius_m"] * math.sin(ang)
                    lat = clat + math.degrees(north / 6378137.0)
                    lon = clon + math.degrees(
                        east / (6378137.0 * math.cos(math.radians(clat)))
                    )
                    pts.append((lon, lat))
            if f["type"] == "inclusion" and not msg.work_area.longitude:
                msg.work_area = to_poly(pts)
            elif f["type"] == "exclusion":
                msg.forbidden_zones.append(to_poly(pts))
        msg.hard_boundaries = [to_poly(b) for b in self._hard_boundaries]
        return msg

    # ------------------------------------------------------------------ #
    # 掩码栅格化与发布
    # ------------------------------------------------------------------ #
    def _fence_to_fill(self, f: Fence) -> Optional[Tuple]:
        """围栏 → 栅格填充描述 ("polygon", pts) / ("circle", cx, cy, r_cells)。"""
        assert self._map_info is not None
        _, _, res, _, _ = self._map_info
        if f["shape"] == "circle":
            if f["center"] is None:
                return None
            x, y = self._latlon_to_map_xy(f["center"][0], f["center"][1])
            gx, gy = self._map_xy_to_grid(x, y)
            return ("circle", gx, gy, f["radius_m"] / res)
        pts = [
            self._map_xy_to_grid(*self._latlon_to_map_xy(lon, lat)) for lon, lat in f["points"]
        ]
        return ("polygon", pts)

    def _rasterize_and_publish(self) -> None:
        if self._map_info is None:
            return
        width, height, res, ox, oy = self._map_info
        inclusion = []
        exclusion = []
        for f in self._fences:
            fill = self._fence_to_fill(f)
            if fill is None:
                continue
            (inclusion if f["type"] == "inclusion" else exclusion).append(fill)
        hb = [
            [self._map_xy_to_grid(*self._latlon_to_map_xy(lon, lat)) for lon, lat in line]
            for line in self._hard_boundaries
        ]
        self._mask = build_fence_mask(width, height, inclusion, exclusion, hb, self._dilation)
        keepout_cells = sum(1 for v in self._mask if v == KEEPOUT)
        self.get_logger().info(
            f"掩码已更新: {width}x{height}, 禁行格 {keepout_cells} "
            f"({100.0 * keepout_cells / max(1, width * height):.1f}%)"
        )
        self._publish_filter()

    def _publish_filter(self) -> None:
        if self._map_info is None or self._mask is None:
            return
        width, height, res, ox, oy = self._map_info
        stamp = self.get_clock().now().to_msg()

        info_msg = CostmapFilterInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self._global_frame
        info_msg.type = 0  # keepout
        info_msg.filter_mask_topic = (
            self.get_parameter("filter_mask_topic").get_parameter_value().string_value
        )
        info_msg.base = 0.0
        info_msg.multiplier = 1.0
        self._filter_info_pub.publish(info_msg)

        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._global_frame
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.info.origin.orientation.w = 1.0
        grid.data = self._mask
        self._mask_pub.publish(grid)

    def _republish_timer(self) -> None:
        if self._mask is not None:
            self._publish_filter()

    # ------------------------------------------------------------------ #
    # 持久化
    # ------------------------------------------------------------------ #
    def _persist(self) -> None:
        data = {
            "fences": [self._fence_to_dict(f) for f in self._fences],
            "hard_boundaries": [
                [[lon, lat] for lon, lat in line] for line in self._hard_boundaries
            ],
        }
        try:
            atomic_write_json(self._persist_path.parent, self._persist_path, data)
        except Exception as e:
            self.get_logger().error(f"区域持久化失败 {self._persist_path}: {e}")

    def _load_persisted(self) -> None:
        if not self._persist_path.exists():
            self.get_logger().info(f"无持久化区域文件（{self._persist_path}），空区域启动")
            return
        try:
            with self._persist_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            fences = [self._json_to_fence(x) for x in data.get("fences", [])]
            hb = [self._json_poly(p) for p in data.get("hard_boundaries", [])]
            err = self._validate_fences(fences)
            if err:
                self.get_logger().error(f"持久化区域校验失败（忽略，空区域启动）: {err}")
                return
            self._fences = fences
            self._hard_boundaries = hb
            self.get_logger().info(
                f"已从 {self._persist_path} 恢复: 围栏 {len(fences)} 个, 硬边界 {len(hb)} 条"
            )
            self._publish_zones_current()
        except Exception as e:
            self.get_logger().error(f"读取持久化区域失败 {self._persist_path}: {e}")

    # ------------------------------------------------------------------ #
    # 当前区域上报
    # ------------------------------------------------------------------ #
    @staticmethod
    def _fence_to_dict(f: Fence) -> Dict[str, Any]:
        return {
            "fence_id": f["fence_id"],
            "type": f["type"],
            "shape": f["shape"],
            "points": [[lon, lat] for lon, lat in f["points"]],
            "center": [f["center"][0], f["center"][1]] if f["center"] else None,
            "radius_m": float(f["radius_m"]),
        }

    def _zones_to_dict(self) -> Dict[str, Any]:
        return {
            "fences": [self._fence_to_dict(f) for f in self._fences],
            "hard_boundaries": [
                [[lon, lat] for lon, lat in line] for line in self._hard_boundaries
            ],
        }

    def _publish_zones_current(self) -> None:
        out = String()
        out.data = json.dumps(self._zones_to_dict(), ensure_ascii=False)
        self._zones_current_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
