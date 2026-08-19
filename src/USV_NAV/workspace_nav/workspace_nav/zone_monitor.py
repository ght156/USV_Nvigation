#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# zone_monitor：运行时电子围栏越界监控（SAFE-001，对齐 decision 对接说明 §6）。
#
# 订阅船位里程计 + zone_manager 的 /nav_zones/current（带 fence_id 的围栏快照），持续检查船位：
#   - 存在 inclusion（作业区）围栏且船位驶出 → 违规（EXIT）
#   - 船位闯入任一 exclusion（禁航区） → 违规（ENTER）
# 连续 violation_threshold 次违规后：
#   - 无论有无任务都调用 mission_bridge/emergency_stop 急停（锁存，区域更新后解除）
#   - 无活动任务时（/mission_bridge/state 非 RUNNING/PAUSED）发布
#     /mission_bridge/safety_event（m_common/msg/NavSafetyEvent）；
#     有活动任务时由任务接口的终态承担上报（NavigateTask Result / mission 状态事件）
# 违规条件消失且告警曾触发时，再发一条 enabled=false 表示告警解除。
# 未设置任何围栏时节点空转。
# ----------------------------------------------------------------------------------------------- #
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from m_common.msg import NavSafetyEvent
from m_common.srv import EmergencyStop

from workspace_nav.gps_map_conversion import (
    datum_lat_lon_from_cfg,
    enu_delta_to_map_xy,
    geodetic_delta_enu_m,
    read_map_origin,
)
from workspace_nav.zone_geometry import fence_violation

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_ACTIVE_MISSION_STATES = ("RUNNING", "PAUSED")


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _point_seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _distance_to_fence(px: float, py: float, fence: Dict[str, Any]) -> float:
    """到围栏边界的近似距离（m）：多边形取到各边最小距离，圆取 |d - r|。"""
    if fence.get("shape") == "circle" and fence.get("center") is not None:
        cx, cy = fence["center"]
        return abs(math.hypot(px - cx, py - cy) - float(fence["radius_m"]))
    pts = fence.get("points") or []
    if len(pts) < 2:
        return 0.0
    n = len(pts)
    return min(
        _point_seg_dist(px, py, pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
        for i in range(n)
    )


class ZoneMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("zone_monitor")

        self.declare_parameter("enabled", True)
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_datum_ref_key", "ref_gnss_10")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_topic", "/mavros/gps_input/local")
        self.declare_parameter("nav_zones_topic", "/nav_zones/current")
        self.declare_parameter("mission_state_topic", "/mission_bridge/state")
        self.declare_parameter("safety_event_topic", "/mission_bridge/safety_event")
        self.declare_parameter("check_period_sec", 0.5)
        self.declare_parameter("violation_threshold", 5)
        self.declare_parameter("emergency_stop_service", "mission_bridge/emergency_stop")

        self._enabled = bool(self.get_parameter("enabled").value)
        self._global_frame = (
            self.get_parameter("global_frame").get_parameter_value().string_value.strip() or "map"
        )
        self._threshold = max(1, int(self.get_parameter("violation_threshold").value))

        # ---- 地图锚点（与 mission_bridge / zone_manager 同一约定） ----
        map_yaml_param = (
            self.get_parameter("map_yaml_path").get_parameter_value().string_value.strip()
        )
        if map_yaml_param:
            map_path = Path(map_yaml_param).expanduser().resolve()
        else:
            map_path = (
                Path(get_package_share_directory("workspace_nav")) / "config" / "map.yaml"
            ).resolve()
        ref_key = (
            self.get_parameter("map_datum_ref_key").get_parameter_value().string_value.strip()
            or "ref_gnss_10"
        )
        try:
            with map_path.open("r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            self._datum_lat, self._datum_lon = datum_lat_lon_from_cfg(cfg, ref_key)
            self._map_ox, self._map_oy, self._map_origin_yaw = read_map_origin(cfg)
        except Exception as e:
            self.get_logger().fatal(f"读取地图锚点失败 {map_path}: {e}")
            raise SystemExit(1) from e

        # ---- 状态 ----
        # map 系围栏：[{fence_id, type, shape, points:[(x,y)], center:(x,y)|None, radius_m}]
        self._fences: List[Dict[str, Any]] = []
        self._odom: Optional[Odometry] = None
        self._mission_state = ""
        self._violation_streak = 0
        self._triggered = False        # 急停锁存
        self._event_active = False     # 是否已发 enabled=true 告警
        self._last_violation: Optional[Tuple[Dict[str, Any], str]] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._cb_odom, 10
        )
        self.create_subscription(
            String,
            self.get_parameter("nav_zones_topic").value,
            self._cb_nav_zones_current,
            _LATCHED_QOS,
        )
        self.create_subscription(
            String,
            self.get_parameter("mission_state_topic").value,
            self._cb_mission_state,
            10,
        )
        self._safety_pub = self.create_publisher(
            NavSafetyEvent, self.get_parameter("safety_event_topic").value, 1
        )

        self._emerg_client = self.create_client(
            EmergencyStop, self.get_parameter("emergency_stop_service").value
        )

        period = max(0.1, float(self.get_parameter("check_period_sec").value))
        self.create_timer(period, self._check_timer)

        self.get_logger().info(
            f"zone_monitor ready: enabled={self._enabled} period={period}s "
            f"threshold={self._threshold}"
        )

    # ------------------------------------------------------------------ #
    # 坐标换算
    # ------------------------------------------------------------------ #
    def _latlon_to_map_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        east, north = geodetic_delta_enu_m(self._datum_lat, self._datum_lon, lat, lon)
        return enu_delta_to_map_xy(east, north, self._map_ox, self._map_oy, self._map_origin_yaw)

    def _map_xy_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """map → 经纬度（enu_delta_to_map_xy 的逆变换）。返回 (lat, lon)。"""
        c = math.cos(self._map_origin_yaw)
        s = math.sin(self._map_origin_yaw)
        east = (x - self._map_ox) * c + (y - self._map_oy) * s
        north = -(x - self._map_ox) * s + (y - self._map_oy) * c
        lat = self._datum_lat + math.degrees(north / 6378137.0)
        lon = self._datum_lon + math.degrees(
            east / (6378137.0 * math.cos(math.radians(self._datum_lat)))
        )
        return lat, lon

    # ------------------------------------------------------------------ #
    # 订阅回调
    # ------------------------------------------------------------------ #
    def _cb_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _cb_mission_state(self, msg: String) -> None:
        self._mission_state = msg.data.strip()

    def _cb_nav_zones_current(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("/nav_zones/current JSON 解析失败，忽略")
            return

        fences: List[Dict[str, Any]] = []
        for raw in data.get("fences") or []:
            try:
                f: Dict[str, Any] = {
                    "fence_id": str(raw.get("fence_id", "")),
                    "type": str(raw.get("type", "")),
                    "shape": str(raw.get("shape", "polygon") or "polygon"),
                    "points": [
                        self._latlon_to_map_xy(float(p[0]), float(p[1]))
                        for p in (raw.get("points") or [])
                    ],
                    "center": None,
                    "radius_m": float(raw.get("radius_m") or 0.0),
                }
                c = raw.get("center")
                if c:
                    f["center"] = self._latlon_to_map_xy(float(c[0]), float(c[1]))
                fences.append(f)
            except (TypeError, ValueError, IndexError):
                continue
        self._fences = fences
        # 区域更新解除急停锁存与违规计数
        self._triggered = False
        self._violation_streak = 0
        if self._event_active:
            self._publish_safety_event(None, "", enabled=False)
            self._event_active = False
        self.get_logger().info(
            f"围栏更新: {len(fences)} 个 "
            f"(inclusion={sum(1 for f in fences if f['type'] == 'inclusion')}, "
            f"exclusion={sum(1 for f in fences if f['type'] == 'exclusion')})"
        )

    # ------------------------------------------------------------------ #
    # 周期检查
    # ------------------------------------------------------------------ #
    def _pose_in_map(self) -> Optional[Tuple[float, float]]:
        if self._odom is None:
            return None
        src_frame = self._odom.header.frame_id or "odom"
        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        if src_frame == self._global_frame:
            return x, y
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, src_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warning(f"TF {self._global_frame}←{src_frame} 不可用: {e}")
            return None
        t = tf.transform.translation
        yaw = _quat_to_yaw(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        return t.x + c * x - s * y, t.y + s * x + c * y

    def _check_timer(self) -> None:
        if not self._enabled or not self._fences:
            return
        pose = self._pose_in_map()
        if pose is None:
            return
        violation = fence_violation(pose[0], pose[1], self._fences)
        if violation is None:
            if self._violation_streak > 0:
                self.get_logger().info("违规条件消失，计数清零")
            self._violation_streak = 0
            if self._event_active:
                self._publish_safety_event(self._last_violation[0] if self._last_violation else None,
                                           self._last_violation[1] if self._last_violation else "",
                                           enabled=False, pose=pose)
                self._event_active = False
            return
        fence, transition = violation
        self._last_violation = violation
        self._violation_streak += 1
        self.get_logger().warning(
            f"越界 {self._violation_streak}/{self._threshold}: 围栏 {fence['fence_id']} "
            f"({fence['type']}, {transition}), 船位 ({pose[0]:.2f}, {pose[1]:.2f})"
        )
        if self._violation_streak < self._threshold:
            return
        if not self._triggered:
            self._triggered = True
            self.get_logger().error(
                f"越界确认，触发急停: 围栏 {fence['fence_id']} ({fence['type']}, {transition})"
            )
            self._call_emergency_stop()
            # 有活动任务时终态由任务接口承担；无任务时发 NavSafetyEvent
            if self._mission_state not in _ACTIVE_MISSION_STATES and not self._event_active:
                self._publish_safety_event(fence, transition, enabled=True, pose=pose)
                self._event_active = True

    def _publish_safety_event(
        self,
        fence: Optional[Dict[str, Any]],
        transition: str,
        enabled: bool,
        pose: Optional[Tuple[float, float]] = None,
    ) -> None:
        msg = NavSafetyEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame
        msg.severity = 2
        msg.event_code = "GEOFENCE_VIOLATION"
        msg.enabled = enabled
        if fence:
            msg.fence_id = fence["fence_id"]
            msg.fence_type = fence["type"]
            if pose is not None:
                msg.distance_to_boundary_m = float(_distance_to_fence(pose[0], pose[1], fence))
        msg.transition = transition
        if pose is not None:
            lat, lon = self._map_xy_to_latlon(pose[0], pose[1])
            msg.latitude = lat
            msg.longitude = lon
        msg.message = "electronic geofence violation" if enabled else "geofence violation cleared"
        self._safety_pub.publish(msg)
        self.get_logger().info(
            f"safety_event: enabled={enabled} fence={msg.fence_id} transition={msg.transition}"
        )

    def _call_emergency_stop(self) -> None:
        if not self._emerg_client.service_is_ready():
            self.get_logger().error("emergency_stop 服务不可用，无法急停！")
            return
        future = self._emerg_client.call_async(EmergencyStop.Request())
        future.add_done_callback(self._emerg_done)

    def _emerg_done(self, future) -> None:
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"急停已执行: {resp.message}")
            else:
                self.get_logger().error(f"急停调用被拒绝: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"急停调用异常: {e}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneMonitorNode()
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
