#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# zone_manager：作业区 / 禁止区 / 硬边界（经纬度多边形/折线）→ Nav2 KeepoutFilter 动态掩码。
#
# 输入：
#   - service mission_bridge/set_nav_zones   (m_common/srv/SetNavZones，全量替换)
#   - service mission_bridge/clear_nav_zones (std_srvs/srv/Trigger，清除全部区域)
#   - service mission_bridge/get_nav_zones   (m_common/srv/GetNavZones，查询当前区域)
#   - topic   /nav_zones (std_msgs/String JSON，GCS 通道；{"clear": true} 清除)
# 输出（RELIABLE + TRANSIENT_LOCAL）：
#   - /keepout_filter/costmap_filter_info (nav2_msgs/CostmapFilterInfo)
#   - /keepout_filter/keepout_filter_mask (nav_msgs/OccupancyGrid，100=禁行)
#   - /nav_zones/current (std_msgs/String JSON，当前生效区域，供 mission_bridge/zone_monitor 使用)
# 语义：作业区外不可行 / 禁止区内不可入 / 硬边界折线不可穿越。
# ----------------------------------------------------------------------------------------------- #
from __future__ import annotations

import json
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

from m_common.msg import GeoPolygon, NavZones
from m_common.srv import GetNavZones, SetNavZones

from workspace_nav.gps_map_conversion import (
    datum_lat_lon_from_cfg,
    enu_delta_to_map_xy,
    geodetic_delta_enu_m,
    read_map_origin,
)
from workspace_nav.zone_geometry import KEEPOUT, build_zone_mask

LatLon = Tuple[float, float]  # (lon, lat)

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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
        # 周期重发掩码/filter_info 的间隔（s）；0 = 关闭。
        # 输出均为 TRANSIENT_LOCAL，晚加入/重启的订阅方会自动收到最后一帧，
        # 通常无需周期重发；每次 filter_info 变更都会让 KeepoutFilter 重建掩码订阅，
        # 高频重发会造成日志刷屏，默认关闭。
        self.declare_parameter("republish_period_sec", 0.0)

        self._global_frame = (
            self.get_parameter("global_frame").get_parameter_value().string_value.strip() or "map"
        )
        ref_key = (
            self.get_parameter("map_datum_ref_key").get_parameter_value().string_value.strip()
            or "ref_gnss_10"
        )
        self._dilation = max(0, int(self.get_parameter("boundary_dilation_cells").value))

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
        self._zones: Dict[str, Any] = {
            "work_area": [],        # List[LatLon]，空 = 不限制
            "forbidden_zones": [],  # List[List[LatLon]]
            "hard_boundaries": [],  # List[List[LatLon]]
        }
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
        self.create_service(SetNavZones, "mission_bridge/set_nav_zones", self._cb_set_nav_zones)
        self.create_service(Trigger, "mission_bridge/clear_nav_zones", self._cb_clear_nav_zones)
        self.create_service(GetNavZones, "mission_bridge/get_nav_zones", self._cb_get_nav_zones)

        period = float(self.get_parameter("republish_period_sec").value)
        if period > 0.0:
            self.create_timer(max(0.2, period), self._republish_timer)

        self.get_logger().info(
            "zone_manager ready: set/clear/get_nav_zones + /nav_zones; "
            f"filter_info={self.get_parameter('filter_info_topic').value} "
            f"mask={self.get_parameter('filter_mask_topic').value}"
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
    # 区域设置 / 清除 / 查询
    # ------------------------------------------------------------------ #
    @staticmethod
    def _validate_zones(zones: Dict[str, Any]) -> Optional[str]:
        wa = zones["work_area"]
        if wa and len(wa) < 3:
            return f"作业区至少需要 3 个点（收到 {len(wa)} 个）"
        for i, poly in enumerate(zones["forbidden_zones"]):
            if len(poly) < 3:
                return f"禁止区 #{i} 至少需要 3 个点（收到 {len(poly)} 个）"
        for i, line in enumerate(zones["hard_boundaries"]):
            if len(line) < 2:
                return f"硬边界 #{i} 至少需要 2 个点（收到 {len(line)} 个）"
        return None

    def _apply_zones(self, zones: Dict[str, Any], source: str) -> Tuple[bool, str]:
        err = self._validate_zones(zones)
        if err:
            return False, err
        self._zones = zones
        n_wa = len(zones["work_area"])
        n_fb = len(zones["forbidden_zones"])
        n_hb = len(zones["hard_boundaries"])
        self.get_logger().info(
            f"[{source}] 区域已更新: 作业区 {n_wa} 点, 禁止区 {n_fb} 个, 硬边界 {n_hb} 条"
        )
        self._publish_zones_current()
        if self._map_info is None:
            self.get_logger().warning("尚未收到 /map，掩码将在地图就绪后发布")
            return True, "zones accepted (mask pending /map)"
        self._rasterize_and_publish()
        return True, f"zones applied: work_area={n_wa}pt forbidden={n_fb} hard_boundary={n_hb}"

    def _cb_set_nav_zones(
        self, request: SetNavZones.Request, response: SetNavZones.Response
    ) -> SetNavZones.Response:
        zones = {
            "work_area": self._msg_poly_to_list(request.zones.work_area),
            "forbidden_zones": [
                self._msg_poly_to_list(p) for p in request.zones.forbidden_zones
            ],
            "hard_boundaries": [
                self._msg_poly_to_list(p) for p in request.zones.hard_boundaries
            ],
        }
        ok, msg = self._apply_zones(zones, "set_nav_zones")
        response.success = ok
        response.message = msg
        if not ok:
            self.get_logger().warning(f"set_nav_zones 被拒绝: {msg}")
        return response

    def _cb_clear_nav_zones(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        empty = {"work_area": [], "forbidden_zones": [], "hard_boundaries": []}
        ok, _ = self._apply_zones(empty, "clear_nav_zones")
        response.success = ok
        response.message = "all nav zones cleared"
        return response

    def _cb_get_nav_zones(
        self, request: GetNavZones.Request, response: GetNavZones.Response
    ) -> GetNavZones.Response:
        response.success = True
        response.zones = self._zones_to_msg()
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
            self._apply_zones(
                {"work_area": [], "forbidden_zones": [], "hard_boundaries": []}, "/nav_zones"
            )
            return
        if not isinstance(data, dict):
            self.get_logger().warning("/nav_zones 载荷须为 JSON 对象")
            return
        zones = {
            "work_area": self._json_poly(data.get("work_area")),
            "forbidden_zones": [
                self._json_poly(p)
                for p in (data.get("forbidden_zones") or data.get("forbidden") or [])
            ],
            "hard_boundaries": [
                self._json_poly(p)
                for p in (data.get("hard_boundaries") or data.get("boundaries") or [])
            ],
        }
        ok, msg_txt = self._apply_zones(zones, "/nav_zones")
        if not ok:
            self.get_logger().warning(f"/nav_zones 区域被拒绝: {msg_txt}")

    @staticmethod
    def _json_poly(raw: Any) -> List[LatLon]:
        """点可为 [lon, lat] 或 {"longitude": .., "latitude": ..}。"""
        pts: List[LatLon] = []
        if not isinstance(raw, list):
            return pts
        for p in raw:
            try:
                if isinstance(p, dict):
                    pts.append((float(p["longitude"]), float(p["latitude"])))
                elif isinstance(p, (list, tuple)) and len(p) >= 2:
                    pts.append((float(p[0]), float(p[1])))
            except (KeyError, TypeError, ValueError):
                continue
        return pts

    @staticmethod
    def _msg_poly_to_list(poly: GeoPolygon) -> List[LatLon]:
        n = min(len(poly.longitude), len(poly.latitude))
        return [(float(poly.longitude[i]), float(poly.latitude[i])) for i in range(n)]

    # ------------------------------------------------------------------ #
    # 掩码栅格化与发布
    # ------------------------------------------------------------------ #
    def _poly_map_to_grid(self, pts: Sequence[LatLon]) -> List[Tuple[float, float]]:
        return [self._map_xy_to_grid(*self._latlon_to_map_xy(lon, lat)) for lon, lat in pts]

    def _rasterize_and_publish(self) -> None:
        if self._map_info is None:
            return
        width, height, res, ox, oy = self._map_info
        wa = self._poly_map_to_grid(self._zones["work_area"]) if self._zones["work_area"] else []
        fb = [self._poly_map_to_grid(p) for p in self._zones["forbidden_zones"]]
        hb = [self._poly_map_to_grid(p) for p in self._zones["hard_boundaries"]]
        self._mask = build_zone_mask(width, height, wa, fb, hb, self._dilation)
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
        # TRANSIENT_LOCAL 已对晚加入者补发；周期重发用于 costmap 重启等恢复场景
        if self._mask is not None:
            self._publish_filter()

    # ------------------------------------------------------------------ #
    # 当前区域上报
    # ------------------------------------------------------------------ #
    def _zones_to_dict(self) -> Dict[str, Any]:
        return {
            "work_area": [[lon, lat] for lon, lat in self._zones["work_area"]],
            "forbidden_zones": [
                [[lon, lat] for lon, lat in p] for p in self._zones["forbidden_zones"]
            ],
            "hard_boundaries": [
                [[lon, lat] for lon, lat in p] for p in self._zones["hard_boundaries"]
            ],
        }

    def _zones_to_msg(self) -> NavZones:
        def to_poly(pts: Sequence[LatLon]) -> GeoPolygon:
            poly = GeoPolygon()
            poly.longitude = [float(lon) for lon, _ in pts]
            poly.latitude = [float(lat) for _, lat in pts]
            return poly

        msg = NavZones()
        msg.work_area = to_poly(self._zones["work_area"])
        msg.forbidden_zones = [to_poly(p) for p in self._zones["forbidden_zones"]]
        msg.hard_boundaries = [to_poly(p) for p in self._zones["hard_boundaries"]]
        return msg

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
