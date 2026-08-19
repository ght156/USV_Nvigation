#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# zone_monitor：运行时越界监控（SAFE-001）。
# 订阅船位里程计 + zone_manager 的 /nav_zones/current，持续检查船位：
#   - 作业区已设置且船位在作业区外 → 违规
#   - 船位落入任一禁止区 → 违规
# 连续 violation_threshold 次违规后调用 mission_bridge/emergency_stop 急停并锁存，
# 锁存在区域更新后解除。未设置任何区域时节点空转。
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

from m_common.srv import EmergencyStop

from workspace_nav.gps_map_conversion import (
    datum_lat_lon_from_cfg,
    enu_delta_to_map_xy,
    geodetic_delta_enu_m,
    read_map_origin,
)
from workspace_nav.zone_geometry import point_in_polygon

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ZoneMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("zone_monitor")

        self.declare_parameter("enabled", True)
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_datum_ref_key", "ref_gnss_10")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_topic", "/mavros/gps_input/local")
        self.declare_parameter("nav_zones_topic", "/nav_zones/current")
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
        self._zone_polys: Dict[str, Any] = {"work_area": [], "forbidden_zones": []}
        self._zones_active = False
        self._odom: Optional[Odometry] = None
        self._violation_streak = 0
        self._triggered = False

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
    # 订阅回调
    # ------------------------------------------------------------------ #
    def _cb_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _cb_nav_zones_current(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("/nav_zones/current JSON 解析失败，忽略")
            return

        def _conv(pts) -> List[Tuple[float, float]]:
            out: List[Tuple[float, float]] = []
            for p in pts or []:
                try:
                    lon, lat = float(p[0]), float(p[1])
                except (TypeError, ValueError, IndexError):
                    continue
                east, north = geodetic_delta_enu_m(self._datum_lat, self._datum_lon, lat, lon)
                out.append(
                    enu_delta_to_map_xy(
                        east, north, self._map_ox, self._map_oy, self._map_origin_yaw
                    )
                )
            return out

        self._zone_polys = {
            "work_area": _conv(data.get("work_area")),
            "forbidden_zones": [_conv(p) for p in (data.get("forbidden_zones") or [])],
        }
        self._zones_active = bool(
            self._zone_polys["work_area"] or self._zone_polys["forbidden_zones"]
        )
        # 区域更新解除急停锁存与违规计数
        self._triggered = False
        self._violation_streak = 0
        self.get_logger().info(
            f"区域更新: 作业区 {len(self._zone_polys['work_area'])} 点, "
            f"禁止区 {len(self._zone_polys['forbidden_zones'])} 个, active={self._zones_active}"
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

    def _is_violation(self, x: float, y: float) -> Optional[str]:
        wa = self._zone_polys["work_area"]
        if wa and not point_in_polygon(x, y, wa):
            return f"船位 ({x:.2f}, {y:.2f}) 在作业区外"
        for j, poly in enumerate(self._zone_polys["forbidden_zones"]):
            if point_in_polygon(x, y, poly):
                return f"船位 ({x:.2f}, {y:.2f}) 闯入禁止区 #{j}"
        return None

    def _check_timer(self) -> None:
        if not self._enabled or not self._zones_active or self._triggered:
            return
        pose = self._pose_in_map()
        if pose is None:
            return
        violation = self._is_violation(*pose)
        if violation is None:
            self._violation_streak = 0
            return
        self._violation_streak += 1
        self.get_logger().warning(
            f"越界 {self._violation_streak}/{self._threshold}: {violation}"
        )
        if self._violation_streak >= self._threshold:
            self._triggered = True
            self.get_logger().error(f"越界确认，触发急停: {violation}")
            self._call_emergency_stop(violation)

    def _call_emergency_stop(self, reason: str) -> None:
        if not self._emerg_client.service_is_ready():
            self.get_logger().error(
                f"emergency_stop 服务不可用，无法急停！原因: {reason}"
            )
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
