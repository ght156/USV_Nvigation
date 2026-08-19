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
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration as RDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time as RTime
from std_msgs.msg import Empty
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from m_common.action import NavigateTask
from m_common.msg import NavSafetyEvent
from m_common.srv import CancelMission, SendWaypoints, SetPause, EmergencyStop

from workspace_nav.gps_map_conversion import (
    atomic_write_json,
    datum_lat_lon_from_cfg,
    enu_delta_to_map_xy,
    geodetic_delta_enu_m,
    lat_lon_list_to_waypoints_document,
    parse_waypoint_message,
    read_map_origin,
    verify_waypoints_file,
)
from workspace_nav.waypoint_with_state import make_waypoint_path
from workspace_nav.zone_geometry import fence_violation

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

# yaw 哨兵约定：合法航向为 [-2π, 2π] rad；超出该范围（如 Decision 下发的 65536）
# 一律视为"不指定朝向"，导航取行进方向作为到点朝向。
YAW_UNSPECIFIED = 65536.0


def yaw_is_specified(yaw: float) -> bool:
    return math.isfinite(yaw) and -2.0 * math.pi <= yaw <= 2.0 * math.pi


class MissionState(str, Enum):
    WAITING_SYSTEM = "WAITING_SYSTEM"
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    EMERGENCY = "EMERGENCY"


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
        self.declare_parameter("waypoint_commit_delay_sec", 0.45)
        self.declare_parameter("mission_cancel_topic", "")
        self.declare_parameter("discard_watchdog_sec", 4.0)
        self.declare_parameter("suppress_passive_waypoints_after_cancel", True)
        self.declare_parameter("target_buoy_min_write_period_sec", 0.0)
        self.declare_parameter("nav_zones_topic", "/nav_zones/current")

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

        self._sm_lock = threading.RLock()
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

        self._wp_commit_delay = float(
            self.get_parameter("waypoint_commit_delay_sec").value
        )
        if self._wp_commit_delay < 0.05:
            self._wp_commit_delay = 0.05

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
        self._nav_xy: List[Tuple[float, float, float]] = []  # (map_x, map_y, yaw_rad)
        self.current_index = 0
        self._paused_nav_xy: List[Tuple[float, float, float]] = []
        self._paused_index: int = 0
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

        self._mission_cancel_topic = mc_top.strip()

        self.get_logger().info(
            "GCS /waypoint debounce enabled (commit_delay=%.2fs, cancel_topic=%s)"
            % (
                self._wp_commit_delay,
                self._mission_cancel_topic or "(empty)",
            )
        )

        # Ground station (GCS) — Topic control; kept parallel to upper-layer Services.
        _wp_in = self.get_parameter("waypoint_topic").value
        _cg_in = self.get_parameter("color_topic").value
        self.create_subscription(String, _wp_in, self._cb_waypoint, 10)
        self.create_subscription(String, _cg_in, self._cb_color, 10)
        # 当前生效导航围栏（zone_manager 发布，TRANSIENT_LOCAL），用于航点预校验
        # 元素为 map 系 fence dict（见 zone_geometry.fence_violation）；空列表 = 不校验
        self._zone_fences: List[Dict[str, Any]] = []
        _zones_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            self.get_parameter("nav_zones_topic").value,
            self._cb_nav_zones_current,
            _zones_qos,
        )
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

        # Service servers for upper-layer task management
        self._srv_cg = MutuallyExclusiveCallbackGroup()
        self._srv_send = self.create_service(
            SendWaypoints, 'mission_bridge/send_waypoints',
            self._cb_send_waypoints, callback_group=self._srv_cg)
        self._srv_pause = self.create_service(
            SetPause, 'mission_bridge/set_pause',
            self._cb_set_pause, callback_group=self._srv_cg)
        self._srv_emerg = self.create_service(
            EmergencyStop, 'mission_bridge/emergency_stop',
            self._cb_emergency_stop, callback_group=self._srv_cg)
        self._srv_cancel = self.create_service(
            CancelMission, 'mission_bridge/cancel_mission',
            self._cb_cancel_mission, callback_group=self._srv_cg)
        self.get_logger().info(
            "Service servers ready: send_waypoints, set_pause, emergency_stop, cancel_mission")
        self._publish_status_detail()

        # ------------------------------------------------------------------ #
        # NavigateTask action server（Decision 对接契约）
        # 独立 ReentrantCallbackGroup：execute 阻塞等待终止事件，
        # 避免与服务组 / FollowWaypoints client 回调相互死锁。
        # 锁顺序约定：_sm_lock 可嵌套 _nt_lock，反向禁止。
        # ------------------------------------------------------------------ #
        self._nt_cb_group = ReentrantCallbackGroup()
        self._nt_lock = threading.Lock()
        self._nt_done = threading.Event()
        self._nt_goal_handle: Optional[Any] = None
        # (result_code, error_code, message, how, final_current_seq, final_reached_seq)
        self._nt_result_info: Optional[Tuple[int, str, str, str, int, int]] = None
        self._nt_seqs: List[int] = []
        self._nt_phase: int = NavigateTask.Feedback.PHASE_VALIDATING
        self._nt_nav2_feedback_seen = False
        self._nt_feedback_timer: Optional[Any] = None
        # 越界原因挂起：safety_event 与 emergency_stop 竞速时优先报 GEOFENCE
        self._nt_geofence_pending = False
        self._nav_action_server = ActionServer(
            self,
            NavigateTask,
            "/mission_bridge/navigate",
            execute_callback=self._nt_execute,
            goal_callback=self._nt_goal_cb,
            cancel_callback=self._nt_cancel_cb,
            callback_group=self._nt_cb_group,
        )
        _safety_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            NavSafetyEvent,
            "/mission_bridge/safety_event",
            self._cb_safety_event,
            _safety_qos,
            callback_group=self._nt_cb_group,
        )
        self.get_logger().info(
            "NavigateTask action server ready: /mission_bridge/navigate")

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

    def _publish_status_detail(self, state_override: Optional[str] = None,
                               error_code: Optional[str] = None,
                               nav2_error_code: int = 0,
                               nav2_error_msg: str = "",
                               transition_reason: Optional[str] = None) -> None:
        """Publish /mission_bridge/status_detail JSON for nav_status_aggregator."""
        with self._sm_lock:
            state = state_override or self._state.value
            task_id = self._current_mission_id or ""
            command_id = self._current_command_id or ""
            waypoint_total = len(self._nav_xy) if self._nav_xy else 0
            waypoint_current_index = self.current_index
            waypoint_completed = self.current_index
            elapsed = 0.0
            if self._mission_start_wall > 0.0:
                elapsed = time.time() - self._mission_start_wall
        payload: Dict[str, Any] = {
            "state": state,
            "task_id": task_id,
            "command_id": command_id,
            "waypoint_total": waypoint_total,
            "waypoint_completed": waypoint_completed,
            "waypoint_current_index": waypoint_current_index,
            "elapsed_sec": round(elapsed, 1),
            "error_code": error_code,
        }
        if nav2_error_code or nav2_error_msg:
            payload["nav2_error_code"] = nav2_error_code
            payload["nav2_error_msg"] = nav2_error_msg
        if transition_reason:
            payload["transition_reason"] = transition_reason
        msg = String()
        msg.data = json.dumps(payload)
        self._status_detail_pub.publish(msg)

    def _status_detail_heartbeat(self) -> None:
        """2 Hz heartbeat — keep /nav_status in sync for all task states."""
        self._publish_status_detail()

    def _transition_state(self, new_state: MissionState,
                          transition_reason: Optional[str] = None) -> None:
        """Set internal state and publish to /mission_bridge/state + status_detail."""
        with self._sm_lock:
            prev = self._state
            self._state = new_state
        if new_state != prev:
            msg = String()
            msg.data = str(new_state.value)
            self._state_pub.publish(msg)
            self.get_logger().info(f"STATE -> {new_state.value}")
            self._publish_status_detail(transition_reason=transition_reason)

    def _cancel_active_nav2_goal(self) -> None:
        """Cancel in-flight FollowWaypoints goal and stop send timer."""
        gh = self._active_goal_handle
        self._active_goal_handle = None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as ex:
                self.get_logger().warning(f"cancel_goal_async: {ex}")
        try:
            if self._send_timer is not None:
                self._send_timer.cancel()
                self._send_timer = None
        except Exception:
            pass
        with self._sm_lock:
            self.navigating = False

    def _on_system_not_ready(self) -> None:
        """TF or Nav2 action unavailable — stop execution and enter WAITING_SYSTEM."""
        with self._sm_lock:
            prev = self._state
            if prev == MissionState.WAITING_SYSTEM:
                return
            if prev in (MissionState.RUNNING, MissionState.PAUSED):
                self._mission_token += 1
        self._cancel_active_nav2_goal()
        self._nt_terminate(
            NavigateTask.Result.RESULT_LOCALIZATION_LOST,
            "SYSTEM_NOT_READY",
            "TF or FollowWaypoints unavailable during mission",
            "abort",
        )
        with self._sm_lock:
            self._transition_state(MissionState.WAITING_SYSTEM)
        self.get_logger().warning(
            "System not ready (TF or FollowWaypoints) — STATE -> WAITING_SYSTEM"
        )

    def _tick_ready(self) -> None:
        tf_ok = self._tf_ok()
        action_ok = self._waypoint_client.wait_for_server(timeout_sec=0.2)

        with self._sm_lock:
            state = self._state

        if state == MissionState.WAITING_SYSTEM:
            if tf_ok and action_ok:
                self._transition_state(MissionState.IDLE)
                self._log_green(f"TF ready: {self._global_frame} -> {self._robot_frame}")
                self.get_logger().info("FollowWaypoints action server ready")
            return

        if not tf_ok or not action_ok:
            if state in (
                MissionState.IDLE,
                MissionState.RUNNING,
                MissionState.PAUSED,
                MissionState.COMPLETED,
                MissionState.FAILED,
            ):
                self._on_system_not_ready()
            return

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
                bwps, bmh, explicit_replan=bexplicit
            )

        self._waypoint_commit_timer = self.create_timer(
            float(self._wp_commit_delay),
            _flush_debounced,
        )

    def _apply_mission_cancel(self) -> Tuple[bool, str]:
        """Cancel mission / clear EMERGENCY.

        Shared by upper-layer ``cancel_mission`` Service and GCS ``mission_cancel_topic``
        (default ``/gcs_mission/cancel``).
        """
        with self._sm_lock:
            state = self._state

        if state == MissionState.WAITING_SYSTEM:
            return False, "System not ready (WAITING_SYSTEM)"

        if self._suppress_passive_after_cancel:
            self._suppress_passive_waypoints = True
        self._deb_route = None
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

        if state == MissionState.EMERGENCY:
            self._suppress_passive_waypoints = False
            self._paused_nav_xy = []
            self._paused_index = 0
            with self._sm_lock:
                self._nav_xy = []
                self.current_index = 0
                self._running_mission_hash = None
                self._transition_state(MissionState.IDLE, transition_reason="cancel")
            self._nt_terminate(
                NavigateTask.Result.RESULT_CANCELED,
                "CANCELED",
                "mission cleared via cancel_mission",
                "abort",
            )
            self.get_logger().info("EMERGENCY cleared via cancel → IDLE")
            return True, "Emergency cleared, state IDLE"

        if state == MissionState.PAUSED:
            self._paused_nav_xy = []
            self._paused_index = 0
            with self._sm_lock:
                self._nav_xy = []
                self.current_index = 0
                self._running_mission_hash = None
                self._transition_state(MissionState.IDLE, transition_reason="cancel")
            self._nt_terminate(
                NavigateTask.Result.RESULT_CANCELED,
                "CANCELED",
                "paused mission cancelled via cancel_mission",
                "abort",
            )
            self.get_logger().info("PAUSED cancelled → IDLE")
            return True, "Paused mission cancelled, state IDLE"

        if state == MissionState.RUNNING:
            self._preempt_running_mission_for_new_waypoints(for_replace=False)
            with self._sm_lock:
                self._nav_xy = []
                self.current_index = 0
            return True, "Running mission cancelled, state IDLE"

        if state == MissionState.IDLE:
            return True, "Already idle (idempotent)"

        # COMPLETED / FAILED — already settling to IDLE
        with self._sm_lock:
            self._transition_state(MissionState.IDLE, transition_reason="cancel")
        return True, "Mission cleared, state IDLE"

    def _cb_mission_cancel(self, _: Empty) -> None:
        self.get_logger().info("mission_cancel (topic): clearing mission")
        success, message = self._apply_mission_cancel()
        if not success:
            self.get_logger().warning(message)

    def _consume_waypoint_command(
        self,
        wps: List[Tuple[float, float]],
        mh: str,
        *,
        explicit_replan: bool = False,
    ) -> None:
        operator_explicit = bool(explicit_replan)
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
                    "(need explicit_replan in JSON)"
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

            if self._state == MissionState.EMERGENCY:
                self.get_logger().warning(
                    "EMERGENCY active — waypoint ignored (send cancel to clear)"
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
                        "(enable allow_replace_running_mission, publish explicit_replan, "
                        "or Run Mission from GCS)"
                    )
                    return
                else:
                    preempt = True
            elif self._state == MissionState.PAUSED:
                if operator_explicit:
                    # Discard paused progress, execute new mission
                    self._paused_nav_xy = []
                    self._paused_index = 0
                    preempt = True
                else:
                    self.get_logger().warning(
                        "PAUSED — waypoint ignored (send explicit_replan to replace)"
                    )
                    return
            elif self._state not in (
                MissionState.IDLE,
                MissionState.COMPLETED,
                MissionState.FAILED,
            ):
                self.get_logger().warning(f"Waypoint ignored in state {self._state}")
                return
            elif (
                not explicit_replan
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
            self._preempt_running_mission_for_new_waypoints(for_replace=True)
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
        ):
            _tp = getattr(self, "_last_passive_waypoint_wall", 0.0)
            if time.time() - _tp > 4.0:
                setattr(self, "_last_passive_waypoint_wall", time.time())
                self.get_logger().info(
                    "passive /waypoint ignored after Cancel Nav"
                )
            return

        self._deb_route = (wps, mh, explicit)
        self._reschedule_waypoint_commit_timer()

    def _preempt_running_mission_for_new_waypoints(self, *, for_replace: bool = False) -> None:
        """Cancel active FollowWaypoints so a new route or cancel can proceed."""
        reason = "replace" if for_replace else "cancel"
        self.get_logger().info(
            f"Preempt active mission (reason={reason}) — canceling FollowWaypoints goal"
        )
        if self._active_goal_handle is not None or self.navigating:
            self._mission_token += 1
            self.get_logger().info(
                f"Mission token bumped to {self._mission_token} (preempt)"
            )
        self._cancel_active_nav2_goal()
        with self._sm_lock:
            self._running_mission_hash = None
            self._transition_state(MissionState.IDLE, transition_reason=reason)
        self._nt_terminate(
            NavigateTask.Result.RESULT_CANCELED,
            "PREEMPTED",
            f"mission preempted by legacy channel ({reason})",
            "abort",
        )

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
                    YAW_UNSPECIFIED,
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
            x, y = self._nav_xy[self.current_index][:2]
            yaw = float(self._nav_xy[self.current_index][2])

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

        if not yaw_is_specified(yaw):
            # 不指定朝向：取行进方向（上一航点/当前船位 → 目标点）
            if self.current_index > 0:
                px, py = self._nav_xy[self.current_index - 1][:2]
            else:
                px, py = rx, ry
            yaw = math.atan2(y - py, x - px)

        goal = FollowWaypoints.Goal()
        goal.poses = [self.create_pose_msg(x, y, 0.0, yaw)]
        with self._sm_lock:
            self.navigating = True
        self._log_green(
            f"Sending waypoint {self.current_index + 1}/{len(self._nav_xy)} "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.3f}"
        )

        if not self._waypoint_client.wait_for_server(timeout_sec=self._action_timeout_sec):
            self.get_logger().error("FollowWaypoints server not available")
            self._on_nav_fatal()
            return

        send_token = self._mission_token
        gh_fut = self._waypoint_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback_cb
        )
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
            n2_code = getattr(raw.result, "error_code", 0)
            n2_msg = getattr(raw.result, "error_msg", "")
            self.get_logger().error(
                f"Waypoint failed status={status} "
                f"nav2_error_code={n2_code} nav2_error_msg={n2_msg}"
            )
            self._on_nav_fatal(error_code="MISSION_FAILED",
                               nav2_error_code=int(n2_code),
                               nav2_error_msg=str(n2_msg))
            return

        # Nav2 FollowWaypoints may return SUCCEEDED even when a waypoint was
        # skipped (planner failed, waypoint marked as "missed").  Check the
        # result for missed_waypoints.
        missed = getattr(raw.result, "missed_waypoints", [])
        if missed:
            n2_code = getattr(raw.result, "error_code", 0)
            n2_msg = getattr(raw.result, "error_msg", "")
            self.get_logger().error(
                f"Waypoint {self.current_index + 1} MISSED "
                f"(planner could not reach target, moving to next) "
                f"nav2_error_code={n2_code} nav2_error_msg={n2_msg}"
            )
            self._on_nav_fatal(error_code="MISSION_FAILED",
                               nav2_error_code=int(n2_code),
                               nav2_error_msg=str(n2_msg),
                               nt_result_code=NavigateTask.Result.RESULT_PLANNING_FAILED)
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
        self._nt_terminate(
            NavigateTask.Result.RESULT_SUCCESS, "", "all waypoints reached", "succeed"
        )

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

    def _on_nav_fatal(self, error_code: str = "MISSION_FAILED",
                      nav2_error_code: int = 0,
                      nav2_error_msg: str = "",
                      nt_result_code: Optional[int] = None) -> None:
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

        self.get_logger().error(
            f"MISSION FAILED — STATE -> FAILED "
            f"error_code={error_code} nav2_error_code={nav2_error_code} nav2_error_msg={nav2_error_msg}"
        )
        self._publish_status_detail(
            error_code=error_code,
            nav2_error_code=nav2_error_code,
            nav2_error_msg=nav2_error_msg,
        )
        self._nt_terminate(
            nt_result_code
            if nt_result_code is not None
            else self._map_nav2_failure(nav2_error_code, nav2_error_msg),
            error_code,
            nav2_error_msg or "FollowWaypoints mission failed",
            "abort",
        )
        self._state_to_idle_relaxed()

    def _on_nav_failed(self, **kwargs: Any) -> None:
        self._on_nav_fatal(**kwargs)

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
                self._transition_state(MissionState.IDLE, transition_reason="complete")
            # PAUSED and EMERGENCY do NOT auto-transition to IDLE


    # ----------------------------------------------------------------------- #
    # Service callbacks
    # ----------------------------------------------------------------------- #

    def _cb_nav_zones_current(self, msg: String) -> None:
        """缓存 zone_manager 发布的当前生效围栏（经纬度 → map 系 fence dict）。

        仅 inclusion（作业区）/exclusion（禁止区）围栏参与航点预校验；
        硬边界交给 KeepoutFilter + 规划器绕行，无路时由 FollowWaypoints
        失败路径上报，不在此预先拒绝。
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("/nav_zones/current JSON 解析失败，忽略")
            return
        if not isinstance(data, dict):
            self.get_logger().warning("/nav_zones/current 顶层非 JSON object，忽略")
            return

        def _ll_to_map(lon: float, lat: float) -> Tuple[float, float]:
            east, north = geodetic_delta_enu_m(
                self._datum_lat, self._datum_lon, lat, lon
            )
            return enu_delta_to_map_xy(
                east, north, self._map_ox, self._map_oy, self._map_origin_yaw
            )

        fences: List[Dict[str, Any]] = []
        for f in data.get("fences") or []:
            try:
                fence_id = str(f.get("fence_id", ""))
                ftype = f.get("type")
                shape = f.get("shape")
                if ftype not in ("inclusion", "exclusion"):
                    continue
                if shape == "polygon":
                    pts: List[Tuple[float, float]] = []
                    for p in f.get("points") or []:
                        lon, lat = float(p[0]), float(p[1])
                        pts.append(_ll_to_map(lon, lat))
                    if len(pts) < 3:
                        continue
                    fences.append({
                        "fence_id": fence_id,
                        "type": ftype,
                        "shape": "polygon",
                        "points": pts,
                        "center": None,
                        "radius_m": 0.0,
                    })
                elif shape == "circle":
                    c = f.get("center")
                    if not c:
                        continue
                    center = _ll_to_map(float(c[0]), float(c[1]))
                    radius = float(f.get("radius_m") or 0.0)
                    if radius <= 0.0:
                        continue
                    fences.append({
                        "fence_id": fence_id,
                        "type": ftype,
                        "shape": "circle",
                        "points": [],
                        "center": center,
                        "radius_m": radius,
                    })
            except (TypeError, ValueError, IndexError, AttributeError):
                continue

        self._zone_fences = fences
        if fences:
            n_in = sum(1 for f in fences if f["type"] == "inclusion")
            n_ex = len(fences) - n_in
            self.get_logger().info(
                f"导航围栏已生效（预校验开启）: 作业区 {n_in} 个, 禁止区 {n_ex} 个"
            )

    def _nav_zones_violation(self, ll: List[Tuple[float, float]]) -> Optional[str]:
        """按围栏模型检查航点；返回 None 表示合法，否则为拒绝原因。

        无围栏数据时不校验（零回归）。
        """
        fences = self._zone_fences
        if not fences:
            return None
        for i, (lat, lon) in enumerate(ll):
            east, north = geodetic_delta_enu_m(self._datum_lat, self._datum_lon, lat, lon)
            x, y = enu_delta_to_map_xy(
                east, north, self._map_ox, self._map_oy, self._map_origin_yaw
            )
            violation = fence_violation(x, y, fences)
            if violation is None:
                continue
            fence, _transition = violation
            fence_id = fence.get("fence_id", "")
            if fence.get("type") == "inclusion":
                return f"航点 #{i} ({lat:.7f}, {lon:.7f}) 在作业区外（围栏 {fence_id}）"
            return f"航点 #{i} ({lat:.7f}, {lon:.7f}) 落在禁止区内（围栏 {fence_id}）"
        return None

    def _cb_send_waypoints(self, request: SendWaypoints.Request,
                           response: SendWaypoints.Response) -> SendWaypoints.Response:
        """Service: 下发航线 (SendWaypoints). Service 调用等价于 explicit_replan，可抢占 RUNNING/PAUSED。

        航点为 WGS84 lat/lon + yaw(rad)；内部转换为 map 坐标后执行。
        """
        wps = request.waypoints
        mission_id = request.mission_id.strip() if request.mission_id else ""
        command_id = request.command_id.strip() if request.command_id else ""

        if not wps:
            response.success = False
            response.message = "Waypoints list is empty"
            self.get_logger().warning(response.message)
            return response

        self.get_logger().info(
            f"[service] send_waypoints: {len(wps)} waypoints, "
            f"mission_id={mission_id!r} command_id={command_id!r}"
        )

        with self._sm_lock:
            current_state = self._state

        if current_state == MissionState.WAITING_SYSTEM:
            response.success = False
            response.message = "System not ready (WAITING_SYSTEM)"
            self.get_logger().warning(response.message)
            return response

        if current_state == MissionState.EMERGENCY:
            response.success = False
            response.message = "In EMERGENCY — call cancel_mission before sending waypoints"
            self.get_logger().warning(response.message)
            return response

        ll: List[Tuple[float, float]] = []
        yaws: List[float] = []
        for wp in wps:
            lat = float(wp.latitude)
            lon = float(wp.longitude)
            if lat == 0.0 and lon == 0.0:
                response.success = False
                response.message = "Invalid waypoint (0, 0)"
                self.get_logger().warning(response.message)
                return response
            if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
                response.success = False
                response.message = f"Lat/lon out of range: ({lat}, {lon})"
                self.get_logger().warning(response.message)
                return response
            ll.append((lat, lon))
            yaws.append(float(wp.yaw))

        # 作业区/禁止区预校验（在抢占当前任务与写盘之前，拒绝不产生影响）
        zone_violation = self._nav_zones_violation(ll)
        if zone_violation:
            response.success = False
            response.message = f"航点违反导航区域限制，已拒绝: {zone_violation}"
            self.get_logger().warning(response.message)
            return response

        mh = waypoint_mission_hash(ll, mission_id)

        with self._sm_lock:
            if self._state in (MissionState.RUNNING, MissionState.PAUSED):
                self.get_logger().info(
                    f"Service replacing active mission (state={self._state.value})"
                )
                self._preempt_running_mission_for_new_waypoints(for_replace=True)
                self._paused_nav_xy = []
                self._paused_index = 0

        try:
            self._write_and_start_mission(ll, yaws, mission_id, command_id)
        except Exception as e:
            response.success = False
            response.message = f"Failed to convert/write waypoints: {e}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"Mission started: {len(ll)} waypoints, hash={mh[:12]}…"
        self._log_green(response.message)
        return response

    def _write_and_start_mission(
        self,
        ll: List[Tuple[float, float]],
        yaws: List[float],
        mission_id: str,
        command_id: str,
    ) -> str:
        """写 waypoints.json、装载航线并启动 FollowWaypoints 执行（不校验、不抢占）。

        send_waypoints 服务与 NavigateTask action 共用。返回 mission hash。
        """
        document = lat_lon_list_to_waypoints_document(
            ll,
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
        for i, e in enumerate(document["waypoints"]):
            e["yaw"] = float(yaws[i])
        atomic_write_json(self._waypoints_path.parent, self._waypoints_path, document)

        nav_xy: List[Tuple[float, float, float]] = [
            (float(e["x"]), float(e["y"]), float(yaws[i]))
            for i, e in enumerate(document["waypoints"])
        ]

        mh = waypoint_mission_hash(ll, mission_id)
        self._current_mission_id = mission_id if mission_id else None
        self._current_command_id = command_id if command_id else None
        self._suppress_passive_waypoints = False

        self._mission_token += 1
        with self._sm_lock:
            self._nav_xy = nav_xy
            self.current_index = 0
            self.navigating = False
            self._running_mission_hash = mh
            self._transition_state(MissionState.RUNNING)
        self._mission_start_wall = time.time()
        self._start_nav_stack()
        return mh

    def _cb_set_pause(self, request: SetPause.Request,
                      response: SetPause.Response) -> SetPause.Response:
        """Service: 暂停/继续 (SetPause)."""
        pause = request.pause

        with self._sm_lock:
            current_state = self._state

        if current_state == MissionState.WAITING_SYSTEM:
            response.success = False
            response.message = "System not ready (WAITING_SYSTEM)"
            return response

        if current_state == MissionState.EMERGENCY:
            response.success = False
            response.message = "In EMERGENCY — call cancel_mission first"
            return response

        if pause:
            if current_state == MissionState.RUNNING:
                self._pause_mission()
                response.success = True
                response.message = "Mission paused; progress saved"
                self.get_logger().info("[service] set_pause: true → PAUSED")
            elif current_state == MissionState.PAUSED:
                response.success = True
                response.message = "Already paused (idempotent)"
            elif current_state == MissionState.IDLE:
                response.success = False
                response.message = "No active mission to pause"
            else:
                response.success = False
                response.message = f"Cannot pause in state {current_state.value}"
                self.get_logger().warning(response.message)
        else:
            if current_state == MissionState.PAUSED:
                self._resume_mission()
                response.success = True
                response.message = "Mission resumed from saved progress"
                self.get_logger().info("[service] set_pause: false → RUNNING")
            elif current_state == MissionState.RUNNING:
                response.success = True
                response.message = "Already running (idempotent)"
            else:
                response.success = False
                response.message = f"Cannot resume in state {current_state.value}"
                self.get_logger().warning(response.message)

        return response

    def _cb_cancel_mission(self, _request: CancelMission.Request,
                           response: CancelMission.Response) -> CancelMission.Response:
        """Service: 取消任务 / 退出急停 (CancelMission)."""
        self.get_logger().info("[service] cancel_mission")
        success, message = self._apply_mission_cancel()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(f"[service] cancel_mission: {message}")
        else:
            self.get_logger().warning(f"[service] cancel_mission: {message}")
        return response

    def _cb_emergency_stop(self, request: EmergencyStop.Request,
                           response: EmergencyStop.Response) -> EmergencyStop.Response:
        """Service: 急停 (EmergencyStop)."""
        with self._sm_lock:
            current_state = self._state

        if current_state == MissionState.EMERGENCY:
            response.success = True
            response.message = "Already in EMERGENCY (idempotent)"
            return response

        self._emergency_stop()
        response.success = True
        response.message = "Emergency stop executed"
        self.get_logger().error("[service] emergency_stop → EMERGENCY")
        return response

    # ----------------------------------------------------------------------- #
    # PAUSED / EMERGENCY internal logic
    # ----------------------------------------------------------------------- #

    def _pause_mission(self) -> None:
        """Save progress, cancel current goal, enter PAUSED. No auto-IDLE."""
        with self._sm_lock:
            self._paused_nav_xy = list(self._nav_xy)
            self._paused_index = self.current_index

        self._mission_token += 1
        self._cancel_active_nav2_goal()
        with self._sm_lock:
            self._transition_state(MissionState.PAUSED)
        self.get_logger().info(
            f"PAUSED — saved {len(self._paused_nav_xy)} waypoints, "
            f"index={self._paused_index}"
        )

    def _resume_mission(self) -> None:
        """Restore saved waypoints and continue from breakpoint."""
        with self._sm_lock:
            self._nav_xy = list(self._paused_nav_xy)
            self.current_index = self._paused_index
            self._paused_nav_xy = []
            self._paused_index = 0
            self._transition_state(MissionState.RUNNING)

        self._start_nav_stack()
        self.get_logger().info(
            f"RESUMED — continuing from waypoint {self.current_index + 1}/{len(self._nav_xy)}"
        )

    def _emergency_stop(self) -> None:
        """Cancel everything, clear state, enter EMERGENCY. No auto-IDLE."""
        for tmr_attr in ("_waypoint_commit_timer", "_delayed_mission_timer"):
            tmr = getattr(self, tmr_attr, None)
            if tmr is not None:
                try:
                    tmr.cancel()
                except Exception:
                    pass
                setattr(self, tmr_attr, None)

        self._mission_token += 1
        self._cancel_active_nav2_goal()

        with self._sm_lock:
            self._nav_xy = []
            self.current_index = 0
            self._paused_nav_xy = []
            self._paused_index = 0
            self._running_mission_hash = None
            self._transition_state(MissionState.EMERGENCY)

        self._suppress_passive_waypoints = True
        self._deb_route = None
        self._startpulse_route = None
        self._delayed_mission = None
        self._clear_waypoint_file()

        # zone_monitor 越界与急停竞速时优先报 GEOFENCE_VIOLATION
        with self._nt_lock:
            geofence_pending = self._nt_geofence_pending
        if geofence_pending:
            self._nt_terminate(
                NavigateTask.Result.RESULT_GEOFENCE_VIOLATION,
                "GEOFENCE_VIOLATION",
                "electronic geofence violation",
                "abort",
            )
        else:
            self._nt_terminate(
                NavigateTask.Result.RESULT_EMERGENCY_STOP,
                "EMERGENCY_STOP",
                "emergency stop during mission",
                "abort",
            )

        self._publish_status_detail(error_code="EMERGENCY_STOP")
        self.get_logger().error("EMERGENCY STOP — all state cleared")

    # ----------------------------------------------------------------------- #
    # NavigateTask action server（Decision 对接）
    # ----------------------------------------------------------------------- #

    def _nt_goal_cb(self, goal_request: NavigateTask.Goal) -> GoalResponse:
        """Goal 校验：不合格即 REJECT（无 Result），warning 日志说明原因。"""
        wps = goal_request.waypoints
        if not wps:
            self.get_logger().warning("[navigate] reject: waypoints empty")
            return GoalResponse.REJECT

        for i, wp in enumerate(wps):
            lat = float(wp.latitude)
            lon = float(wp.longitude)
            if not (math.isfinite(lat) and math.isfinite(lon)):
                self.get_logger().warning(
                    f"[navigate] reject: waypoint #{i} lat/lon not finite ({lat}, {lon})"
                )
                return GoalResponse.REJECT
            if lat == 0.0 and lon == 0.0:
                self.get_logger().warning(f"[navigate] reject: waypoint #{i} is (0, 0)")
                return GoalResponse.REJECT
            if lat < -90.0 or lat > 90.0 or lon < -180.0 or lon > 180.0:
                self.get_logger().warning(
                    f"[navigate] reject: waypoint #{i} lat/lon out of range ({lat}, {lon})"
                )
                return GoalResponse.REJECT

        with self._sm_lock:
            state = self._state
        if state == MissionState.WAITING_SYSTEM:
            self.get_logger().warning("[navigate] reject: system not ready (WAITING_SYSTEM)")
            return GoalResponse.REJECT
        if state == MissionState.EMERGENCY:
            self.get_logger().warning("[navigate] reject: EMERGENCY active")
            return GoalResponse.REJECT
        if state in (MissionState.RUNNING, MissionState.PAUSED):
            # 不自动抢占（含 NavigateTask 自身任务与老通道任务），由 Decision 先 Cancel
            self.get_logger().warning(
                f"[navigate] reject: mission busy (state={state.value})"
            )
            return GoalResponse.REJECT
        with self._nt_lock:
            if self._nt_goal_handle is not None:
                self.get_logger().warning(
                    "[navigate] reject: another NavigateTask goal active"
                )
                return GoalResponse.REJECT

        ll = [(float(wp.latitude), float(wp.longitude)) for wp in wps]
        violation = self._nav_zones_violation(ll)
        if violation:
            self.get_logger().warning(
                f"[navigate] reject: 航点违反导航区域限制: {violation}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(f"[navigate] accept goal: {len(wps)} waypoints")
        return GoalResponse.ACCEPT

    def _nt_cancel_cb(self, _goal_handle: Any) -> CancelResponse:
        self.get_logger().info("[navigate] cancel requested")
        return CancelResponse.ACCEPT

    def _nt_execute(self, goal_handle: Any) -> NavigateTask.Result:
        """阻塞式执行（依赖 MultiThreadedExecutor + ReentrantCallbackGroup）。

        启动任务后等待终止事件；终止由成功/失败/取消/急停/越界各路径通过
        _nt_terminate 触发（先发先赢），此处保证被接受的 goal 恰好返回一次 Result。
        """
        R = NavigateTask.Result
        wps = goal_handle.request.waypoints
        ll = [(float(w.latitude), float(w.longitude)) for w in wps]
        yaws = [float(w.yaw) for w in wps]
        seqs = [int(w.seq) for w in wps]

        with self._nt_lock:
            slot_taken = self._nt_goal_handle is not None
            if not slot_taken:
                self._nt_goal_handle = goal_handle
                self._nt_seqs = seqs
                self._nt_result_info = None
                self._nt_phase = NavigateTask.Feedback.PHASE_VALIDATING
                self._nt_nav2_feedback_seen = False
                self._nt_geofence_pending = False
        if slot_taken:
            # 与另一 goal 的极端竞态：后到者直接终止
            self.get_logger().warning("[navigate] abort: another goal registered first")
            result = R()
            result.result = R.RESULT_BUSY
            result.error_code = "BUSY"
            result.message = "another NavigateTask goal active"
            result.final_current_seq = -1
            result.final_reached_seq = -1
            try:
                goal_handle.abort()
            except Exception:
                pass
            return result

        self._nt_done.clear()
        self._nt_start_feedback_timer()

        if goal_handle.is_cancel_requested:
            # 接受后、启动前即收到取消
            self._nt_terminate(R.RESULT_CANCELED, "CANCELED",
                               "canceled before start", "canceled")
        else:
            with self._sm_lock:
                state = self._state
            if state in (MissionState.IDLE, MissionState.COMPLETED, MissionState.FAILED):
                try:
                    mh = self._write_and_start_mission(ll, yaws, "", "")
                except Exception as e:
                    self._nt_terminate(R.RESULT_INTERNAL_ERROR, "INTERNAL_ERROR",
                                       f"failed to start mission: {e}", "abort")
                else:
                    self._nt_phase = NavigateTask.Feedback.PHASE_PLANNING
                    self.get_logger().info(
                        f"[navigate] mission started: {len(ll)} waypoints "
                        f"hash={mh[:12]}…"
                    )
            else:
                # goal_cb 之后状态发生变化的竞态兜底
                code = (
                    R.RESULT_NOT_READY
                    if state in (MissionState.WAITING_SYSTEM, MissionState.EMERGENCY)
                    else R.RESULT_BUSY
                )
                self._nt_terminate(code, "STATE_" + state.value,
                                   f"cannot start mission in state {state.value}", "abort")

        while not self._nt_done.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                # 先标记 CANCELED 结果，再安全停车（停车路径的抢占 hook 不得覆盖它）
                self._nt_terminate(R.RESULT_CANCELED, "CANCELED",
                                   "canceled by decision", "canceled")
                self._apply_mission_cancel()

        with self._nt_lock:
            info = self._nt_result_info
            self._nt_goal_handle = None
            self._nt_seqs = []
        self._nt_stop_feedback_timer()

        result = R()
        if info is None:
            # 兜底：正常不会发生
            result.result = R.RESULT_INTERNAL_ERROR
            result.error_code = "INTERNAL_ERROR"
            result.message = "mission terminated without result info"
            result.final_current_seq = -1
            result.final_reached_seq = -1
            if goal_handle.is_active:
                try:
                    goal_handle.abort()
                except Exception:
                    pass
        else:
            result.result = info[0]
            result.error_code = info[1]
            result.message = info[2]
            result.final_current_seq = info[4]
            result.final_reached_seq = info[5]
        self.get_logger().info(
            f"[navigate] goal finished: result={result.result} "
            f"error_code={result.error_code!r} message={result.message!r}"
        )
        return result

    def _nt_terminate(self, result_code: int, error_code: str,
                      message: str, how: str) -> None:
        """终止当前 NavigateTask goal（幂等，先发先赢）。

        由所有任务终止路径（成功/失败/取消/抢占/急停/越界/失联）调用；
        无活动 goal 时为空操作，不影响老通道行为。
        """
        with self._sm_lock:
            idx = self.current_index
        with self._nt_lock:
            gh = self._nt_goal_handle
            if gh is None or self._nt_result_info is not None:
                return
            seqs = self._nt_seqs
            cur = seqs[idx] if 0 <= idx < len(seqs) else (seqs[-1] if seqs else -1)
            reached = seqs[idx - 1] if 1 <= idx <= len(seqs) else -1
            self._nt_result_info = (
                int(result_code), str(error_code), str(message), str(how),
                int(cur), int(reached),
            )
        try:
            if how == "succeed":
                gh.succeed()
            elif how == "canceled":
                gh.canceled()
            else:
                gh.abort()
        except Exception as ex:
            self.get_logger().warning(f"[navigate] terminal transition ({how}): {ex}")
        self._nt_done.set()

    @staticmethod
    def _map_nav2_failure(nav2_error_code: int, nav2_error_msg: str) -> int:
        """把 Nav2 FollowWaypoints 失败尽量映射到 NavigateTask Result 码。

        Humble 的 FollowWaypoints.Result 无 error_code 字段（恒为 0），
        按错误消息关键词归类；无法判定时归 INTERNAL_ERROR。
        """
        R = NavigateTask.Result
        msg = (nav2_error_msg or "").lower()
        if any(k in msg for k in ("plan", "path", "route", "missed")):
            return R.RESULT_PLANNING_FAILED
        if any(k in msg for k in ("controller", "control", "follow", "stuck", "collision")):
            return R.RESULT_CONTROLLER_FAILED
        if 100 <= nav2_error_code < 200:
            return R.RESULT_PLANNING_FAILED
        if 200 <= nav2_error_code < 300:
            return R.RESULT_CONTROLLER_FAILED
        return R.RESULT_INTERNAL_ERROR

    def _nav2_feedback_cb(self, _feedback_msg: Any) -> None:
        """收到 FollowWaypoints feedback 即视为进入 TRACKING 阶段。"""
        self._nt_nav2_feedback_seen = True

    def _nt_start_feedback_timer(self) -> None:
        self._nt_stop_feedback_timer()
        self._nt_feedback_timer = self.create_timer(
            0.5, self._nt_publish_feedback, callback_group=self._nt_cb_group
        )

    def _nt_stop_feedback_timer(self) -> None:
        t = self._nt_feedback_timer
        self._nt_feedback_timer = None
        if t is not None:
            try:
                t.cancel()
            except Exception:
                pass

    def _nt_publish_feedback(self) -> None:
        """2 Hz 发布 NavigateTask feedback（phase/seq/剩余里程）。"""
        with self._nt_lock:
            gh = self._nt_goal_handle
            seqs = self._nt_seqs
            phase = self._nt_phase
        if gh is None:
            return
        with self._sm_lock:
            state = self._state
            idx = self.current_index
            nav = list(self._nav_xy)

        fb = NavigateTask.Feedback()
        if state == MissionState.PAUSED:
            fb.phase = NavigateTask.Feedback.PHASE_PAUSED
            fb.message = "paused"
        elif phase == NavigateTask.Feedback.PHASE_VALIDATING:
            fb.phase = NavigateTask.Feedback.PHASE_VALIDATING
            fb.message = "validating"
        elif not self._nt_nav2_feedback_seen:
            fb.phase = NavigateTask.Feedback.PHASE_PLANNING
            fb.message = "planning"
        else:
            fb.phase = NavigateTask.Feedback.PHASE_TRACKING
            fb.message = "tracking"

        fb.current_seq = seqs[idx] if 0 <= idx < len(seqs) else (seqs[-1] if seqs else -1)
        fb.reached_seq = seqs[idx - 1] if 1 <= idx <= len(seqs) else -1

        rx, ry = self._robot_xy()
        # 口径（与 decision 对接说明 §9.3 一致）：当前船位到当前目标航点的距离
        dist = 0.0
        if 0 <= idx < len(nav):
            dist = math.hypot(nav[idx][0] - rx, nav[idx][1] - ry)
        fb.distance_remaining_m = float(dist)

        try:
            gh.publish_feedback(fb)
        except Exception:
            pass

    def _cb_safety_event(self, msg: NavSafetyEvent) -> None:
        """zone_monitor 异步安全事件：越界时终止活动 NavigateTask goal。"""
        if msg.event_code != "GEOFENCE_VIOLATION":
            return
        if msg.enabled:
            with self._nt_lock:
                self._nt_geofence_pending = True
                active = self._nt_goal_handle is not None
            if active:
                self.get_logger().error(
                    f"[navigate] geofence violation: fence={msg.fence_id} "
                    f"type={msg.fence_type} transition={msg.transition}"
                )
                self._nt_terminate(
                    NavigateTask.Result.RESULT_GEOFENCE_VIOLATION,
                    "GEOFENCE_VIOLATION",
                    "electronic geofence violation",
                    "abort",
                )
        else:
            with self._nt_lock:
                self._nt_geofence_pending = False


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = MissionBridgeNode()
    # NavigateTask execute 为阻塞式等待，需多线程执行器；
    # 共享状态由 _sm_lock / _pose_lock / _nt_lock 保护。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("interrupt")
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
