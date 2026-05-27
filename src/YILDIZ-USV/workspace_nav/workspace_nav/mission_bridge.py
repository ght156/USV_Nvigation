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
from std_msgs.msg import Empty
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from workspace_nav.gps_map_conversion import (
    atomic_write_json,
    datum_lat_lon_from_cfg,
    lat_lon_list_to_waypoints_document,
    parse_waypoint_message,
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


def waypoint_mission_hash(waypoints: List[Tuple[float, float]], mission_id: str = "") -> str:
    norm: List[Dict[str, Any]] = [{"latitude": lat, "longitude": lon} for lat, lon in waypoints]
    if mission_id:
        norm.append({"mission_id": mission_id})
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
        self.declare_parameter("target_buoy_force_rewrite", False)
        self.declare_parameter("waypoint_command_mode", "immediate")
        self.declare_parameter("waypoint_commit_delay_sec", 0.45)
        self.declare_parameter("mission_start_topic", "")
        self.declare_parameter("mission_cancel_topic", "")
        self.declare_parameter("discard_watchdog_sec", 4.0)
        self.declare_parameter("suppress_passive_waypoints_after_cancel", True)
        self.declare_parameter("target_buoy_min_write_period_sec", 0.0)

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
                map_path = (share / "config" / "map_hk.yaml").resolve()
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
        self._target_buoy_force_rewrite = bool(
            self.get_parameter("target_buoy_force_rewrite").value
        )

        wm = (
            self.get_parameter("waypoint_command_mode")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )
        if wm in ("debounce", "debounced"):
            self._wp_cmd_mode = "debounce"
        elif wm in ("start_pulse", "start", "pulse"):
            self._wp_cmd_mode = "start_pulse"
        else:
            self._wp_cmd_mode = "immediate"
        self._wp_commit_delay = float(
            self.get_parameter("waypoint_commit_delay_sec").value
        )
        if self._wp_commit_delay < 0.05:
            self._wp_commit_delay = 0.05

        ms_top = (
            self.get_parameter("mission_start_topic").get_parameter_value().string_value.strip()
        )
        mc_top = (
            self.get_parameter("mission_cancel_topic")
            .get_parameter_value()
            .string_value.strip()
        )
        self._discard_watchdog_sec = max(
            0.0, float(self.get_parameter("discard_watchdog_sec").value)
        )

        self._suppress_passive_after_cancel = bool(
            self.get_parameter("suppress_passive_waypoints_after_cancel").value
        )
        self._target_buoy_min_period = max(
            0.0, float(self.get_parameter("target_buoy_min_write_period_sec").value)
        )
        self._nav_xy: List[Tuple[float, float]] = []
        self.current_index = 0
        self.navigating = False
        self._current_pose_xy = (0.0, 0.0)
        self._current_mission_id: Optional[str] = None
        self._current_command_id: Optional[str] = None
        self._mission_start_wall: float = 0.0
        self._have_odom = False
        self._pose_lock = threading.Lock()
        self._odom_sub: Optional[Any] = None
        self._send_timer: Optional[Any] = None
        self._idle_transition_timer: Optional[Any] = None
        # 当前发往 Nav2 的 FollowWaypoints goal（用于地面站换新任务时 cancel）
        self._active_goal_handle: Optional[Any] = None
        self._mission_token = 0
        self._delayed_mission_timer: Optional[Any] = None
        self._delayed_mission: Optional[Tuple[List[Tuple[float, float]], str]] = None
        self._waypoint_commit_timer: Optional[Any] = None
        self._deb_route: Optional[
            Tuple[List[Tuple[float, float]], str, bool]
        ] = None
        self._startpulse_route: Optional[
            Tuple[List[Tuple[float, float]], str, bool]
        ] = None

        self._mission_start_topic = ms_top
        self._mission_cancel_topic = mc_top.strip()

        self.get_logger().info(
            "waypoint_command_mode=%s (commit_delay=%.2fs, start_topic=%s, cancel_topic=%s)"
            % (
                self._wp_cmd_mode,
                self._wp_commit_delay,
                self._mission_start_topic or "(empty)",
                self._mission_cancel_topic or "(empty)",
            )
        )

        if self._wp_cmd_mode == "start_pulse":
            if not self._mission_start_topic:
                self.get_logger().fatal(
                    'waypoint_command_mode=start_pulse 时必须设置 mission_start_topic（例如 "/gcs_mission/start")'
                )
                raise SystemExit(1)

        _wp_in = self.get_parameter("waypoint_topic").value
        _cg_in = self.get_parameter("color_topic").value
        self.create_subscription(String, _wp_in, self._cb_waypoint, 10)
        self.create_subscription(String, _cg_in, self._cb_color, 10)
        if self._wp_cmd_mode == "start_pulse":
            self.create_subscription(
                Empty,
                self._mission_start_topic,
                self._cb_mission_start,
                10,
            )
            self._log_green(f"listening start_pulse.Empty on {self._mission_start_topic!r}")
        if self._mission_cancel_topic:
            self.create_subscription(
                Empty,
                self._mission_cancel_topic,
                self._cb_mission_cancel,
                10,
            )

        # Publish internal state so GCS can monitor true mission progress (vs dispatch-only state)
        self._state_pub = self.create_publisher(String, '/mission_bridge/state', 10)

        # Publish enriched status detail for nav_status_aggregator consumption
        self._status_detail_pub = self.create_publisher(String, '/mission_bridge/status_detail', 10)
        self._status_detail_timer = self.create_timer(0.5, self._status_detail_heartbeat)

        self._suppress_passive_waypoints = False
        self._last_passive_waypoint_wall = 0.0

        # 地面站可能对 /color_code 高频重复发布同色；仅在语义变化时写盘，避免刷屏与无意义改写
        self._last_written_target_sem: Optional[str] = None
        self._last_target_buoy_any_write_wall = 0.0
        self._last_stage_log_wall = 0.0

        self.get_logger().info(f"Writing waypoints to: {self._waypoints_path}")
        self.get_logger().info(f"Writing target buoy to: {self._target_path}")

        tf_per = float(self.get_parameter("tf_check_period_sec").value)
        if tf_per < 0.1:
            tf_per = 0.5
        self.create_timer(tf_per, self._tick_ready)

    def _log_green(self, s: str) -> None:
        self.get_logger().info(f"{GREEN}{s}{RESET}")

    def _publish_status_detail(self, state_override: Optional[str] = None, error_code: Optional[str] = None) -> None:
        """Publish /mission_bridge/status_detail JSON for nav_status_aggregator."""
        with self._sm_lock:
            if state_override is None and self._state != MissionState.RUNNING:
                return
            state = state_override or self._state.value
            task_id = self._current_mission_id or ""
            command_id = self._current_command_id or ""
            waypoint_total = len(self._nav_xy) if self._nav_xy else 0
            waypoint_current_index = self.current_index
            waypoint_completed = self.current_index
            elapsed = 0.0
            if self._mission_start_wall > 0.0:
                elapsed = time.time() - self._mission_start_wall
        msg = String()
        msg.data = json.dumps({
            "state": state,
            "task_id": task_id,
            "command_id": command_id,
            "waypoint_total": waypoint_total,
            "waypoint_completed": waypoint_completed,
            "waypoint_current_index": waypoint_current_index,
            "elapsed_sec": round(elapsed, 1),
            "error_code": error_code,
        })
        self._status_detail_pub.publish(msg)

    def _status_detail_heartbeat(self) -> None:
        """2 Hz heartbeat — only publishes status_detail when RUNNING."""
        self._publish_status_detail()

    def _transition_state(self, new_state: MissionState) -> None:
        """Set internal state and publish to /mission_bridge/state for GCS."""
        prev = self._state
        self._state = new_state
        if new_state != prev:
            msg = String()
            msg.data = str(new_state.value)
            self._state_pub.publish(msg)
            self.get_logger().info(f"STATE -> {new_state.value}")

    def _tick_ready(self) -> None:
        with self._sm_lock:
            if self._state != MissionState.WAITING_SYSTEM:
                return
            if self._tf_ok() and self._waypoint_client.wait_for_server(timeout_sec=0.2):
                self._transition_state(MissionState.IDLE)
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

    def _cancel_waypoint_commit_timer(self) -> None:
        wt = getattr(self, "_waypoint_commit_timer", None)
        self._waypoint_commit_timer = None
        if wt is not None:
            try:
                wt.cancel()
            except Exception:
                pass

    def _reschedule_waypoint_commit_timer(self) -> None:
        self._cancel_waypoint_commit_timer()

        def _flush_debounced() -> None:
            t_inner = getattr(self, "_waypoint_commit_timer", None)
            self._waypoint_commit_timer = None
            if t_inner is not None:
                try:
                    t_inner.cancel()
                except Exception:
                    pass
            bundle = getattr(self, "_deb_route", None)
            if bundle is None:
                return
            bwps, bmh, bexplicit = bundle
            self._consume_waypoint_command(
                bwps, bmh, from_start_pulse=False, explicit_replan=bexplicit
            )

        self._waypoint_commit_timer = self.create_timer(
            float(self._wp_commit_delay),
            _flush_debounced,
        )

    def _cb_mission_start(self, _: Empty) -> None:
        bundle = getattr(self, "_startpulse_route", None)
        if bundle is None or not bundle[0]:
            self.get_logger().warning(
                "start_pulse: buffered route empty — publish /waypoint first, then pulse start topic."
            )
            return
        wps, mh, _buffered_explicit = bundle
        self.get_logger().info(
            f"start_pulse: executing buffered route ({len(wps)} points, mission hash {mh[:12]}…)"
        )
        self._consume_waypoint_command(
            wps, mh, from_start_pulse=True, explicit_replan=True
        )

    def _cb_mission_cancel(self, _: Empty) -> None:
        self.get_logger().info("mission_cancel: clearing waypoint buffers")
        if self._suppress_passive_after_cancel:
            self._suppress_passive_waypoints = True
            self.get_logger().info(
                "mission_cancel: suppress_passive_waypoints=True until explicit_replan or start_pulse"
            )
        self._deb_route = None
        self._startpulse_route = None
        self._cancel_waypoint_commit_timer()
        self._clear_waypoint_file()

        dmt = getattr(self, "_delayed_mission_timer", None)
        if dmt is not None:
            try:
                dmt.cancel()
            except Exception:
                pass
            self._delayed_mission_timer = None
        self._delayed_mission = None

        with self._sm_lock:
            running = self._state == MissionState.RUNNING

        if running:
            self._preempt_running_mission_for_new_waypoints()

    def _consume_waypoint_command(
        self,
        wps: List[Tuple[float, float]],
        mh: str,
        *,
        from_start_pulse: bool,
        explicit_replan: bool = False,
    ) -> None:
        operator_explicit = bool(from_start_pulse or explicit_replan)
        if operator_explicit:
            self._suppress_passive_waypoints = False
        elif (
            self._suppress_passive_after_cancel
            and self._suppress_passive_waypoints
        ):
            _tp = getattr(self, "_last_passive_waypoint_wall", 0.0)
            if time.time() - _tp > 4.0:
                setattr(self, "_last_passive_waypoint_wall", time.time())
                self.get_logger().info(
                    "passive /waypoint discarded after Cancel Nav "
                    "(need explicit_replan in JSON or start_pulse Empty)"
                )
            return
        # 取消遗留的延后启动定时器（防连续改点时任务叠加）
        dmt = getattr(self, "_delayed_mission_timer", None)
        if dmt is not None:
            try:
                dmt.cancel()
            except Exception:
                pass
            self._delayed_mission_timer = None
        self._delayed_mission = None

        preempt = False
        with self._sm_lock:
            if self._state == MissionState.WAITING_SYSTEM:
                self.get_logger().warning(
                    "System not ready (WAITING_SYSTEM); waypoint ignored."
                )
                return

            if self._state == MissionState.RUNNING:
                if mh == self._running_mission_hash:
                    if operator_explicit:
                        preempt = True
                    else:
                        _twall = getattr(self, "_last_dup_wall", 0.0)
                        if time.time() - _twall > 4.0:
                            setattr(self, "_last_dup_wall", time.time())
                            self.get_logger().info("duplicate mission ignored")
                        return
                elif (
                    not self._allow_replace_running_mission and not operator_explicit
                ):
                    self.get_logger().warning(
                        "mission running, new mission rejected "
                        "(enable allow_replace_running_mission, publish explicit_replan/start_pulse, "
                        "or Run Mission from GCS)"
                    )
                    return
                else:
                    preempt = True
            elif self._state not in (
                MissionState.IDLE,
                MissionState.COMPLETED,
                MissionState.FAILED,
            ):
                self.get_logger().warning(f"Waypoint ignored in state {self._state}")
                return
            elif (
                not from_start_pulse
                and not explicit_replan
                and mh == self._last_completed_mission_hash
                and not self._allow_repeat_identical_route
            ):
                _tc = getattr(self, "_last_done_dup_wall", 0.0)
                if time.time() - _tc > 4.0:
                    setattr(self, "_last_done_dup_wall", time.time())
                    self.get_logger().info(
                        "same mission as last successful run ignored (waiting for new plan)"
                    )
                return

        if preempt:
            self._preempt_running_mission_for_new_waypoints()
            self._delayed_mission = (wps, mh)

            def _deferred_execute() -> None:
                """一次性定时：开头 cancel，避免 ROS 2 Timer 周期性误触发。"""
                tmr = getattr(self, "_delayed_mission_timer", None)
                self._delayed_mission_timer = None
                if tmr is not None:
                    try:
                        tmr.cancel()
                    except Exception:
                        pass
                dm = getattr(self, "_delayed_mission", None)
                if dm is None:
                    return
                if dm[1] != mh:
                    return
                self._delayed_mission = None
                self._execute_mission_atomic(dm[0], dm[1])

            self._delayed_mission_timer = self.create_timer(0.22, _deferred_execute)
            return

        self._execute_mission_atomic(wps, mh)

    def _cb_color(self, msg: String) -> None:
        if self._dbg:
            self.get_logger().info(f"[debug] /color_code raw: {msg.data!r}")
        sem = normalize_color(self, msg.data, self._dbg)
        if sem is None:
            return
        if sem == self._last_written_target_sem and not self._target_buoy_force_rewrite:
            if self._dbg:
                self.get_logger().info(
                    f"[debug] target_buoy unchanged ({sem}), skip rewrite (same as last write)"
                )
            return

        if self._target_buoy_min_period > 0.0:
            _now = time.time()
            _lw = getattr(self, "_last_target_buoy_any_write_wall", 0.0)
            if _lw > 0.0 and (_now - _lw) < float(self._target_buoy_min_period):
                if self._dbg:
                    self.get_logger().info(
                        "[debug] target_buoy write throttled "
                        f"(min period {self._target_buoy_min_period:.2f}s)"
                    )
                return

        ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        target_data = {
            "color": sem,
            "target": {"color": sem, "timestamp": ts},
        }
        try:
            atomic_write_json(self._target_dir, self._target_path, target_data)
            self._last_written_target_sem = sem
            self._last_target_buoy_any_write_wall = time.time()
            self._log_green(f"Updated target_buoy.json ({sem}) -> {self._target_path}")
        except Exception as e:
            self.get_logger().error(f"Failed writing target_buoy.json: {e}")

    def _cb_waypoint(self, msg: String) -> None:
        if self._dbg:
            self.get_logger().info(f"[debug] /waypoint raw: {msg.data!r}")

        parsed = parse_waypoint_message(msg.data)
        if not parsed or not parsed.waypoints:
            self.get_logger().warning("Invalid or empty waypoint message; skipped")
            return

        wps = parsed.waypoints
        explicit = parsed.explicit_replan
        mission_id = parsed.mission_id

        # Parse command_id from raw JSON for status_detail publishing
        command_id = ""
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                cid = payload.get("command_id")
                if isinstance(cid, str) and cid.strip():
                    command_id = cid.strip()
        except Exception:
            pass

        self._current_mission_id = mission_id if mission_id else None
        self._current_command_id = command_id if command_id else None

        mh = waypoint_mission_hash(wps, mission_id)

        if mission_id:
            self.get_logger().info(
                f"Received mission_id={mission_id}  "
                f"waypoints={len(wps)}  explicit={explicit}  hash={mh[:12]}…"
            )

        if (
            self._suppress_passive_after_cancel
            and self._suppress_passive_waypoints
            and not explicit
            and self._wp_cmd_mode in ("immediate", "debounce")
        ):
            _tp = getattr(self, "_last_passive_waypoint_wall", 0.0)
            if time.time() - _tp > 4.0:
                setattr(self, "_last_passive_waypoint_wall", time.time())
                self.get_logger().info(
                    "passive /waypoint ignored after Cancel Nav "
                    f"(mode={self._wp_cmd_mode!r})"
                )
            return

        if self._wp_cmd_mode == "start_pulse":
            self._startpulse_route = (wps, mh, explicit)
            if self._dbg or (
                time.time() - getattr(self, "_last_stage_log_wall", 0.0) > 12.0
            ):
                self._last_stage_log_wall = time.time()
                self.get_logger().info(
                    f"start_pulse: staged {len(wps)} waypoint(s), hash={mh[:12]}… "
                    f"pulse std_msgs/msg/Empty on {self._mission_start_topic!r} to navigate"
                )
            return

        if self._wp_cmd_mode == "debounce":
            self._deb_route = (wps, mh, explicit)
            self._reschedule_waypoint_commit_timer()
            return

        self._consume_waypoint_command(
            wps, mh, from_start_pulse=False, explicit_replan=explicit
        )

    def _preempt_running_mission_for_new_waypoints(self) -> None:
        """取消当前发往 Nav2 的 FollowWaypoints，使后续新航线能重新触发全局规划。"""
        self.get_logger().info(
            "New GCS waypoint set while navigating — canceling active FollowWaypoints goal"
        )
        gh = self._active_goal_handle
        self._active_goal_handle = None

        if gh is not None or self.navigating:
            self._mission_token += 1
            self.get_logger().info(
                f"Mission token bumped to {self._mission_token} (preempt)"
            )
        with self._sm_lock:
            self.navigating = False

        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as ex:
                self.get_logger().warning(f"cancel_goal_async: {ex}")

        try:
            if self._send_timer is not None:
                try:
                    self._send_timer.cancel()
                except Exception:
                    pass
                self._send_timer = None
        except Exception:
            pass

        with self._sm_lock:
            self._running_mission_hash = None
            self._transition_state(MissionState.IDLE)

        self._publish_status_detail(state_override="IDLE")

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

            self._mission_token += 1
            self.get_logger().info(
                f"Mission token bumped to {self._mission_token} (new mission)"
            )

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
                self._transition_state(MissionState.RUNNING)
                self._log_green(
                    f"STATE -> RUNNING (mission hash {mh[:12]}…) {len(nav_xy)} poses"
                )

            self._mission_start_wall = time.time()
            self._publish_status_detail()

            self._start_nav_stack()

        except Exception as e:
            self.get_logger().error(f"Failed to execute mission: {e}")
            with self._sm_lock:
                self._transition_state(MissionState.FAILED)
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

        if self._odom_sub is None:
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._odom_cb, 10
            )

        self._send_timer = self.create_timer(2.0, self._send_next_waypoint)

    def _odom_cb(self, msg: Odometry) -> None:
        self._have_odom = True
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
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_next_waypoint(self) -> None:
        if not self._have_odom:
            self.get_logger().warning("Waiting for odometry before sending waypoint")
            self._reset_send_timer(0.5)
            return
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
            with self._sm_lock:
                self.current_index += 1
            self._reset_send_timer(0.5)
            self._finalize_if_done()
            return

        goal = FollowWaypoints.Goal()
        goal.poses = [self.create_pose_msg(x, y, 0.0, 0.0)]
        with self._sm_lock:
            self.navigating = True
        self._log_green(
            f"Sending waypoint {self.current_index + 1}/{len(self._nav_xy)} x={x:.2f}, y={y:.2f}"
        )

        if not self._waypoint_client.wait_for_server(timeout_sec=self._action_timeout_sec):
            self.get_logger().error("FollowWaypoints server not available")
            self._on_nav_fatal()
            return

        send_token = self._mission_token
        gh_fut = self._waypoint_client.send_goal_async(goal)
        gh_fut.add_done_callback(
            lambda future, t=send_token: self._goal_response_cb(future, t)
        )

    def _reset_send_timer(self, sec: float) -> None:
        try:
            if self._send_timer is not None:
                self._send_timer.cancel()
        except Exception:
            pass
        self._send_timer = self.create_timer(sec, self._send_next_waypoint)

    def _goal_response_cb(self, future: Future, send_token: int) -> None:
        if send_token != self._mission_token:
            try:
                goal_handle = future.result()
            except Exception:
                return
            if goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                rf = goal_handle.get_result_async()
                rf.add_done_callback(
                    lambda f, t=send_token: self._goal_result_cb(f, t)
                )
            return

        try:
            goal_handle = future.result()
        except Exception:
            self.get_logger().error("goal future failed")
            with self._sm_lock:
                self.navigating = False
            self._active_goal_handle = None
            self._reset_send_timer(2.0)
            return

        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected")
            with self._sm_lock:
                self.navigating = False
            self._active_goal_handle = None
            self._on_nav_fatal()
            return

        self._active_goal_handle = goal_handle
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(
            lambda f, t=send_token: self._goal_result_cb(f, t)
        )

    def _goal_result_cb(self, future: Future, send_token: int) -> None:
        with self._sm_lock:
            self.navigating = False
        self._active_goal_handle = None

        if send_token != self._mission_token:
            try:
                raw = future.result()
                status = raw.status if raw else GoalStatus.STATUS_UNKNOWN
            except Exception:
                status = GoalStatus.STATUS_UNKNOWN
            self.get_logger().info(
                f"Discarding stale goal result (token {send_token} != {self._mission_token}, status={status})"
            )
            return

        try:
            raw = future.result()
            status = raw.status if raw else GoalStatus.STATUS_UNKNOWN
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"Waypoint failed status={status}")
            self._on_nav_failed()
            return

        # Nav2 FollowWaypoints may return SUCCEEDED even when a waypoint was
        # skipped (planner failed, waypoint marked as "missed").  Check the
        # result for missed_waypoints.
        missed = getattr(raw.result, "missed_waypoints", [])
        if missed:
            self.get_logger().error(
                f"Waypoint {self.current_index + 1} MISSED "
                f"(planner could not reach target, moving to next)"
            )
            self._on_nav_failed()
            return

        self._log_green(f"Waypoint {self.current_index + 1} reached successfully.")
        done = False
        with self._sm_lock:
            self.current_index += 1
            if self.current_index >= len(self._nav_xy):
                done = True

        self._publish_status_detail()

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

        self._active_goal_handle = None

        self._clear_waypoint_file()
        with self._sm_lock:
            hc = self._running_mission_hash
            self._last_completed_mission_hash = hc
            self._running_mission_hash = None
            self._transition_state(MissionState.COMPLETED)
        self.get_logger().info("All waypoints completed.")

        self._publish_status_detail(state_override="COMPLETED")

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
                json.dump({"waypoints": []}, f)
            self.get_logger().info(f"Waypoint file cleared: {self._waypoints_path}")
        except Exception as e:
            self.get_logger().error(f"Failed clearing waypoint file: {e}")

    def _on_nav_fatal(self) -> None:
        with self._sm_lock:
            self._transition_state(MissionState.FAILED)
            self._running_mission_hash = None
            self.navigating = False
        self._active_goal_handle = None
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
        self._publish_status_detail(state_override="FAILED", error_code="MISSION_FAILED")
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
                self._transition_state(MissionState.IDLE)


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
