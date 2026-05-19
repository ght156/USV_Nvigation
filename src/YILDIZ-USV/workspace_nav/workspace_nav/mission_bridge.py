#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# 仿真 / 调试：订阅 GCS /waypoint、/color_code，与 waypoint_transform 同源写 waypoints.json，
# 写 target_buoy.json，并逐点调 Nav2 FollowWaypoints（与地面站/USV_NAV 同类载荷兼容）。
# ----------------------------------------------------------------------------------------------- #
from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import utm
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time as RTime
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from workspace_nav.gps_map_conversion import (
    atomic_write_json,
    datum_lat_lon_from_cfg,
    lat_lon_list_to_waypoints_document,
    parse_waypoint_payload,
    read_map_origin,
    verify_waypoints_file,
)
from workspace_nav.waypoint_with_state import make_waypoint_path

GREEN = "\x1b[32m"
RESET = "\x1b[0m"

HEX_TO_COLOR = {
    "#FF0000": "red",
    "#ff0000": "red",
    "#00FF00": "green",
    "#00ff00": "green",
    "#000000": "black",
}
VALID_SEMANTIC = {"green", "red", "black"}


class MissionState(str, Enum):
    WAITING_SYSTEM = "WAITING_SYSTEM"
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


def _find_workspace_root() -> Optional[Path]:
    try:
        script_path = Path(__file__).resolve()
    except Exception:
        script_path = Path.cwd().resolve()
    candidates = [script_path, Path.cwd().resolve()]
    seen = set()
    for start in candidates:
        for p in [start] + list(start.parents):
            if p in seen:
                continue
            seen.add(p)
            if (p / "src" / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
            if (p / "YILDIZ-USV" / "workspace_nav").is_dir():
                return p
            if (p / "src" / "USV_NAV" / "workspace_nav").is_dir():
                return p
            if (p / "USV_NAV" / "workspace_nav").is_dir():
                return p
    return None


def _find_workspace_nav_json_dir() -> Path:
    ws_root = _find_workspace_root()
    if ws_root is not None:
        for rel in (
            ("src", "YILDIZ-USV", "workspace_nav", "json"),
            ("YILDIZ-USV", "workspace_nav", "json"),
            ("src", "USV_NAV", "workspace_nav", "json"),
            ("USV_NAV", "workspace_nav", "json"),
        ):
            d = ws_root.joinpath(*rel).resolve()
            if d.exists():
                return d
        return ws_root.joinpath("src", "YILDIZ-USV", "workspace_nav", "json").resolve()
    return (Path.cwd().resolve() / "src" / "YILDIZ-USV" / "workspace_nav" / "json").resolve()


def resolve_target_buoy_paths_param_or_env(nav: Node, param_wp: str) -> Tuple[Path, Path]:
    import os

    env_path = os.environ.get("TARGET_JSON_PATH")
    if env_path:
        p = Path(env_path).expanduser().resolve()
        return p.parent, p
    if param_wp.strip():
        p = Path(param_wp).expanduser().resolve()
        return p.parent, p
    try:
        base = get_package_share_directory("workspace_nav")
        cand = Path(base) / "json" / "target_buoy.json"
        return cand.parent.resolve(), cand.resolve()
    except Exception:
        pass
    d = _find_workspace_nav_json_dir()
    return d, (d / "target_buoy.json").resolve()


def normalize_color(nav: Optional[Node], raw: str, debug: bool) -> Optional[str]:
    raw = raw.strip()
    if not raw:
        return None
    low = raw.lower()
    if low in VALID_SEMANTIC:
        return low
    key = raw if raw.startswith("#") else raw
    if key in HEX_TO_COLOR:
        return HEX_TO_COLOR[key]
    lk = raw.lower()
    if lk in HEX_TO_COLOR:
        return HEX_TO_COLOR[lk]
    if nav is not None:
        nav.get_logger().warning(f"Unknown color payload: '{raw}', skipped")
        if debug:
            nav.get_logger().info(f"[debug] color raw bytes: {raw!r}")
    return None


def waypoint_mission_hash(waypoints: List[Tuple[float, float]]) -> str:
    norm = [{"latitude": lat, "longitude": lon} for lat, lon in waypoints]
    blob = json.dumps(norm, sort_keys=True).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


class MissionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_bridge")

        self.declare_parameter("waypoint_topic", "/waypoint")
        self.declare_parameter("color_topic", "/color_code")
        self.declare_parameter("follow_waypoints_action", "follow_waypoints")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("waypoints_json_path", "")
        self.declare_parameter("target_buoy_json_path", "")
        self.declare_parameter("datum_source", "map_yaml")
        self.declare_parameter("map_datum_ref_key", "ref_gnss_10")
        self.declare_parameter("projection", "enu")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("waypoint_tolerance_m", 1.5)
        self.declare_parameter("tf_check_period_sec", 1.0)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("allow_replace_running_mission", False)
        self.declare_parameter("allow_repeat_identical_route", False)

        self._dbg = bool(self.get_parameter("debug_mode").value)
        wf_param = (
            self.get_parameter("waypoints_json_path").get_parameter_value().string_value
        )
        if wf_param.strip():
            self._waypoints_path = Path(wf_param).expanduser().resolve()
        else:
            self._waypoints_path = make_waypoint_path()

        tn_param = (
            self.get_parameter("target_buoy_json_path").get_parameter_value().string_value
        )
        self._target_dir, self._target_path = resolve_target_buoy_paths_param_or_env(
            self, tn_param
        )

        self._datum_source = (
            self.get_parameter("datum_source").get_parameter_value().string_value.strip()
            or "map_yaml"
        )
        self._ref_key = (
            self.get_parameter("map_datum_ref_key").get_parameter_value().string_value.strip()
            or "ref_gnss_10"
        )
        proj = (
            self.get_parameter("projection").get_parameter_value().string_value.strip().lower()
            or "enu"
        )
        self._projection = proj if proj in ("enu", "utm") else "enu"
        if self._projection == "utm":
            self.get_logger().warning(
                "projection=utm is not recommended; use enu unless you know the datum/zone align."
            )

        map_yaml_param = (
            self.get_parameter("map_yaml_path").get_parameter_value().string_value.strip()
        )
        if map_yaml_param:
            map_path = Path(map_yaml_param).expanduser().resolve()
        else:
            try:
                share = Path(get_package_share_directory("workspace_nav"))
                map_path = (share / "config" / "map.yaml").resolve()
            except Exception as e:
                self.get_logger().fatal(f"无法解析默认 map_yaml: {e}")
                raise SystemExit(1) from e

        if self._datum_source != "map_yaml":
            self.get_logger().fatal("mission_bridge 首版仅支持 datum_source=map_yaml")
            raise SystemExit(1)

        self._datum_lat = 0.0
        self._datum_lon = 0.0
        self._datum_easting = 0.0
        self._datum_northing = 0.0
        self._map_ox = 0.0
        self._map_oy = 0.0
        self._map_origin_yaw = 0.0
        self._map_yaml_resolved = ""

        try:
            with map_path.open("r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            lat, lon = datum_lat_lon_from_cfg(cfg, self._ref_key)
            self._map_ox, self._map_oy, self._map_origin_yaw = read_map_origin(cfg)
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            self._datum_lat, self._datum_lon = lat, lon
            self._datum_easting, self._datum_northing = easting, northing
            self._map_yaml_resolved = str(map_path)
        except Exception as e:
            self.get_logger().fatal(f"读取地图失败 {map_path}: {e}")
            raise SystemExit(1) from e

        self.get_logger().info(f"loaded map yaml: {self._map_yaml_resolved}")
        self.get_logger().info(
            f"datum latitude: {self._datum_lat}, datum longitude: {self._datum_lon}"
        )
        self.get_logger().info(
            f"origin (ox oy yaw_rad): {self._map_ox}, {self._map_oy}, {self._map_origin_yaw}"
        )

        gf = self.get_parameter("global_frame").value
        rf = self.get_parameter("robot_frame").value
        self._global_frame = gf
        self._robot_frame = rf

        self.tf_buffer = Buffer(cache_time=RDuration(seconds=30.0), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_timeout_sec = 5.0

        cg = MutuallyExclusiveCallbackGroup()
        self._waypoint_client = ActionClient(
            self,
            FollowWaypoints,
            self.get_parameter("follow_waypoints_action").value,
            callback_group=cg,
        )

        wp_topic = self.get_parameter("waypoint_topic").value
        cg_topic = self.get_parameter("color_topic").value
        self.create_subscription(String, wp_topic, self._cb_waypoint, 10)
        self.create_subscription(String, cg_topic, self._cb_color, 10)

        self._odom_topic = self.get_parameter("odom_topic").value
        self._tolerance = float(self.get_parameter("waypoint_tolerance_m").value)

        self._sm_lock = threading.Lock()
        self._state = MissionState.WAITING_SYSTEM
        self._running_mission_hash: Optional[str] = None
        self._last_completed_mission_hash: Optional[str] = None
        self._allow_replace_running_mission = bool(
            self.get_parameter("allow_replace_running_mission").value
        )
        self._allow_repeat_identical_route = bool(
            self.get_parameter("allow_repeat_identical_route").value
        )

        self._nav_xy: List[Tuple[float, float]] = []
        self.current_index = 0
        self.navigating = False
        self._current_pose_xy = (0.0, 0.0)
        self._pose_lock = threading.Lock()
        self._odom_sub: Optional[Any] = None
        self._send_timer: Optional[Any] = None
        self._idle_transition_timer: Optional[Any] = None

        self.get_logger().info(f"Writing waypoints to: {self._waypoints_path}")
        self.get_logger().info(f"Writing target buoy to: {self._target_path}")

        tf_per = float(self.get_parameter("tf_check_period_sec").value)
        if tf_per < 0.1:
            tf_per = 0.5
        self.create_timer(tf_per, self._tick_ready)

    def _log_green(self, s: str) -> None:
        self.get_logger().info(f"{GREEN}{s}{RESET}")

    def _tick_ready(self) -> None:
        with self._sm_lock:
            if self._state != MissionState.WAITING_SYSTEM:
                return
            if self._tf_ok() and self._waypoint_client.wait_for_server(timeout_sec=0.2):
                self._state = MissionState.IDLE
                self._log_green(f"TF ready: {self._global_frame} -> {self._robot_frame}")
                self.get_logger().info("FollowWaypoints action server ready")
                self.get_logger().info("STATE -> IDLE")

    def _tf_ok(self) -> bool:
        try:
            return self.tf_buffer.can_transform(
                self._global_frame,
                self._robot_frame,
                RTime(),
                timeout=RDuration(seconds=0.05),
            )
        except Exception:
            return False

    def _cb_color(self, msg: String) -> None:
        if self._dbg:
            self.get_logger().info(f"[debug] /color_code raw: {msg.data!r}")
        sem = normalize_color(self, msg.data, self._dbg)
        if sem is None:
            return
        ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        target_data = {
            "color": sem,
            "target": {"color": sem, "timestamp": ts},
        }
        try:
            atomic_write_json(self._target_dir, self._target_path, target_data)
            self._log_green(f"Updated target_buoy.json ({sem}) -> {self._target_path}")
        except Exception as e:
            self.get_logger().error(f"Failed writing target_buoy.json: {e}")

    def _cb_waypoint(self, msg: String) -> None:
        if self._dbg:
            self.get_logger().info(f"[debug] /waypoint raw: {msg.data!r}")

        wps = parse_waypoint_payload(msg.data)
        if not wps:
            self.get_logger().warning("Invalid or empty waypoint message; skipped")
            return

        mh = waypoint_mission_hash(wps)

        with self._sm_lock:
            if self._state == MissionState.WAITING_SYSTEM:
                self.get_logger().warning(
                    "System not ready (WAITING_SYSTEM); waypoint ignored."
                )
                return

            if self._state == MissionState.RUNNING:
                if mh == self._running_mission_hash:
                    _twall = getattr(self, "_last_dup_wall", 0.0)
                    if time.time() - _twall > 4.0:
                        setattr(self, "_last_dup_wall", time.time())
                        self.get_logger().info("duplicate mission ignored")
                    return
                if self._allow_replace_running_mission:
                    self.get_logger().warning(
                        "allow_replace_running_mission is true but in-run replacement "
                        "is not implemented yet; rejecting new mission"
                    )
                self.get_logger().warning("mission running, new mission rejected")
                return

            if self._state not in (
                MissionState.IDLE,
                MissionState.COMPLETED,
                MissionState.FAILED,
            ):
                self.get_logger().warning(f"Waypoint ignored in state {self._state}")
                return

            if (
                mh == self._last_completed_mission_hash
                and not self._allow_repeat_identical_route
            ):
                _tc = getattr(self, "_last_done_dup_wall", 0.0)
                if time.time() - _tc > 4.0:
                    setattr(self, "_last_done_dup_wall", time.time())
                    self.get_logger().info(
                        "same mission as last successful run ignored (waiting for new plan)"
                    )
                return

        self._execute_mission_atomic(wps, mh)

    def _execute_mission_atomic(self, wps: List[Tuple[float, float]], mh: str) -> None:
        try:
            document = lat_lon_list_to_waypoints_document(
                wps,
                self._datum_lat,
                self._datum_lon,
                self._datum_easting,
                self._datum_northing,
                self._projection,
                self._datum_source,
                self._map_yaml_resolved,
                self._ref_key,
                self._map_ox,
                self._map_oy,
                self._map_origin_yaw,
            )
            out_dir = self._waypoints_path.parent
            atomic_write_json(out_dir, self._waypoints_path, document)

            if not verify_waypoints_file(self._waypoints_path):
                self.get_logger().error(
                    "waypoints.json verification failed after write; aborted mission."
                )
                return

            nav_xy = [
                (
                    float(e["x"]),
                    float(e["y"]),
                )
                for e in document["waypoints"]
            ]

            with self._sm_lock:
                if self._state not in (
                    MissionState.IDLE,
                    MissionState.COMPLETED,
                    MissionState.FAILED,
                ):
                    return

                self._nav_xy = nav_xy
                self.current_index = 0
                self.navigating = False
                self._running_mission_hash = mh
                self._state = MissionState.RUNNING
                self._log_green(
                    f"STATE -> RUNNING (mission hash {mh[:12]}…) {len(nav_xy)} poses"
                )

            self._start_nav_stack()

        except Exception as e:
            self.get_logger().error(f"Failed to execute mission: {e}")
            with self._sm_lock:
                self._state = MissionState.FAILED
                self._running_mission_hash = None
            self._state_to_idle_relaxed()

    def _start_nav_stack(self) -> None:
        try:
            if self._send_timer is not None:
                try:
                    self._send_timer.cancel()
                except Exception:
                    pass
                self._send_timer = None
        except Exception:
            pass

        self._current_pose_xy = (0.0, 0.0)
        if self._odom_sub is None:
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._odom_cb, 10
            )

        self._send_timer = self.create_timer(2.0, self._send_next_waypoint)

    def _odom_cb(self, msg: Odometry) -> None:
        with self._pose_lock:
            self._current_pose_xy = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

    def _robot_xy(self) -> Tuple[float, float]:
        with self._pose_lock:
            return self._current_pose_xy

    def create_pose_msg(self, x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_next_waypoint(self) -> None:
        with self._sm_lock:
            if self._state != MissionState.RUNNING:
                return

        if self.navigating:
            return
        if self.current_index >= len(self._nav_xy):
            return

        x, y = self._nav_xy[self.current_index]
        rx, ry = self._robot_xy()
        dist = math.hypot(x - rx, y - ry)
        if dist <= self._tolerance:
            self.get_logger().info(
                f"Waypoint {self.current_index + 1} within tolerance; skipping."
            )
            self.current_index += 1
            self._reset_send_timer(0.5)
            self._finalize_if_done()
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [self.create_pose_msg(x, y, 0.0, 0.0)]
        self.navigating = True
        self._log_green(
            f"Sending waypoint {self.current_index + 1}/{len(self._nav_xy)} x={x:.2f}, y={y:.2f}"
        )

        if not self._waypoint_client.wait_for_server(timeout_sec=self._action_timeout_sec):
            self.get_logger().error("FollowWaypoints server not available")
            self._on_nav_fatal()
            return

        gh_fut = self._waypoint_client.send_goal_async(goal)
        gh_fut.add_done_callback(self._goal_response_cb)

    def _reset_send_timer(self, sec: float) -> None:
        try:
            if self._send_timer is not None:
                self._send_timer.cancel()
        except Exception:
            pass
        self._send_timer = self.create_timer(sec, self._send_next_waypoint)

    def _goal_response_cb(self, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            self.get_logger().error("goal future failed")
            self.navigating = False
            self._reset_send_timer(2.0)
            return
        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected")
            self.navigating = False
            self._on_nav_fatal()
            return
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future: Future) -> None:
        self.navigating = False
        try:
            raw = future.result()
            status = raw.status if raw else GoalStatus.STATUS_UNKNOWN
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Waypoint failed status={status}")
            self._on_nav_failed()
            return

        self._log_green(f"Waypoint {self.current_index + 1} reached successfully.")
        self.current_index += 1

        done = False
        with self._sm_lock:
            if self.current_index >= len(self._nav_xy):
                done = True

        if done:
            self._finish_all_waypoints_success()
            return

        self._reset_send_timer(1.0)

    def _finalize_if_done(self) -> None:
        with self._sm_lock:
            if self.current_index >= len(self._nav_xy):
                pass
            else:
                return
        self._finish_all_waypoints_success()

    def _finish_all_waypoints_success(self) -> None:
        try:
            if self._send_timer is not None:
                try:
                    self._send_timer.cancel()
                except Exception:
                    pass
                self._send_timer = None
        except Exception:
            pass

        self._clear_waypoint_file()
        with self._sm_lock:
            hc = self._running_mission_hash
            self._last_completed_mission_hash = hc
            self._running_mission_hash = None
            self._state = MissionState.COMPLETED
        self.get_logger().info("All waypoints completed. STATE -> COMPLETED")

        try:
            if self._odom_sub is not None:
                self.destroy_subscription(self._odom_sub)
                self._odom_sub = None
        except Exception:
            pass

        self._state_to_idle_relaxed()

    def _clear_waypoint_file(self) -> None:
        try:
            self._waypoints_path.parent.mkdir(parents=True, exist_ok=True)
            with self._waypoints_path.open("w", encoding="utf-8") as f:
                f.write("{}")
            self.get_logger().info(f"Waypoint file cleared: {self._waypoints_path}")
        except Exception as e:
            self.get_logger().error(f"Failed clearing waypoint file: {e}")

    def _on_nav_fatal(self) -> None:
        with self._sm_lock:
            self._state = MissionState.FAILED
            self._running_mission_hash = None
        self.navigating = False
        try:
            if self._send_timer is not None:
                try:
                    self._send_timer.cancel()
                except Exception:
                    pass
                self._send_timer = None
        except Exception:
            pass
        try:
            if self._odom_sub is not None:
                self.destroy_subscription(self._odom_sub)
                self._odom_sub = None
        except Exception:
            pass

        self.get_logger().error("MISSION FAILED — STATE -> FAILED")
        self._state_to_idle_relaxed()

    def _on_nav_failed(self) -> None:
        self._on_nav_fatal()

    def _idle_once_cb(self) -> None:
        self._defer_idle()
        if self._idle_transition_timer is not None:
            try:
                self._idle_transition_timer.cancel()
            except Exception:
                pass
            self._idle_transition_timer = None

    def _state_to_idle_relaxed(self) -> None:
        if self._idle_transition_timer is not None:
            try:
                self._idle_transition_timer.cancel()
            except Exception:
                pass
            self._idle_transition_timer = None
        self._idle_transition_timer = self.create_timer(0.05, self._idle_once_cb)

    def _defer_idle(self) -> None:
        with self._sm_lock:
            if self._state in (MissionState.COMPLETED, MissionState.FAILED):
                self._state = MissionState.IDLE
                self.get_logger().info("STATE -> IDLE")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
