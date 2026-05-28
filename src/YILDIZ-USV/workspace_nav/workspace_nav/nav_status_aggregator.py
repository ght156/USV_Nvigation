#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
# nav_status_aggregator — Pure observer node.
#
# Subscribes to mission, localization, and navigation topics, then publishes:
#   /nav_status  (2Hz snapshot, String/JSON, RELIABLE + TRANSIENT_LOCAL, depth=10)
#   /task_event  (event-triggered, String/JSON, RELIABLE, depth=50)
#
# Phase 1: 5 error codes (PLAN_FAILED, CTRL_STUCK, LOC_LOST, LOC_DEGRADED, MISSION_FAILED).
# No costmap queries. No goal/action sending.
#
# /nav_status JSON schema: see docs/nav_gcs_refactor_plan.md §3.2
# /task_event  JSON schema: see docs/nav_gcs_refactor_plan.md §4.2
# ----------------------------------------------------------------------------------------------- #

import json
import math
import time
from collections import deque
from enum import Enum
from typing import Any, Deque, Dict, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_system_default
from rclpy.time import Time as RTime
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# --------------------------------------------------------------------------- #
# Enums
# --------------------------------------------------------------------------- #

class LocHealth(str, Enum):
    GOOD = "GOOD"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


# rcl_interfaces/Log.msg severity (Log.WARN etc. may be bytes in some installs)
_LOG_DEBUG = 10
_LOG_INFO = 20
_LOG_WARN = 30
_LOG_ERROR = 40
_LOG_FATAL = 50


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #

class NavStatusAggregator(Node):
    """Pure observer: subscribes to topics, publishes aggregated status / events."""

    def __init__(self) -> None:
        super().__init__("nav_status_aggregator")

        # ---- parameters -------------------------------------------------- #
        self._declare_params()

        self._vehicle_id = str(self.get_parameter("vehicle_id").value)
        self._publish_rate = max(0.5, min(10.0,
                                          float(self.get_parameter("publish_rate").value)))
        self._odom_timeout = max(0.5, float(self.get_parameter("odom_timeout").value))
        self._gps_timeout = max(0.5, float(self.get_parameter("gps_timeout").value))
        self._mb_timeout = max(1.0, float(self.get_parameter("mission_bridge_timeout").value))
        self._cov_lost = float(self.get_parameter("cov_lost_threshold").value)
        self._cov_degraded = float(self.get_parameter("cov_degraded_threshold").value)
        self._stuck_timeout = max(1.0, float(self.get_parameter("stuck_progress_timeout").value))
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)

        _status_detail_topic = str(self.get_parameter("status_detail_topic").value)
        _odom_topic = str(self.get_parameter("odom_topic").value)
        _gps_topic = str(self.get_parameter("gps_topic").value)
        _nav_status_topic = str(self.get_parameter("nav_status_topic").value)
        _task_event_topic = str(self.get_parameter("task_event_topic").value)
        _fw_status_topic = str(self.get_parameter("follow_waypoints_status_topic").value)
        _planner_status_topic = str(self.get_parameter("planner_status_topic").value)

        # ---- QoS profiles ----------------------------------------------- #
        nav_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        task_event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=50,
        )

        # ---- publishers ------------------------------------------------- #
        self._nav_status_pub = self.create_publisher(
            String, _nav_status_topic, nav_status_qos)
        self._task_event_pub = self.create_publisher(
            String, _task_event_topic, task_event_qos)

        # ---- callback group for concurrent handling --------------------- #
        cg = ReentrantCallbackGroup()

        # ---- subscriptions ---------------------------------------------- #
        self.create_subscription(
            String, _status_detail_topic, self._cb_status_detail, 10,
            callback_group=cg)
        self.create_subscription(
            Odometry, _odom_topic, self._cb_odom, 10,
            callback_group=cg)
        self.create_subscription(
            NavSatFix, _gps_topic, self._cb_gps, 10,
            callback_group=cg)
        self.create_subscription(
            GoalStatusArray, _fw_status_topic, self._cb_action_status, 10,
            callback_group=cg)
        self.create_subscription(
            GoalStatusArray, _planner_status_topic, self._cb_planner_status, 10,
            callback_group=cg)
        self.create_subscription(
            Log, "/rosout", self._cb_rosout, qos_profile_system_default,
            callback_group=cg)

        # ---- TF --------------------------------------------------------- #
        self._tf_buffer = Buffer(cache_time=RDuration(seconds=10.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- internal state — mission ----------------------------------- #
        self._mission_state: str = "IDLE"
        self._task_id: str = ""
        self._command_id: str = ""
        self._waypoint_total: int = 0
        self._waypoint_completed: int = 0
        self._waypoint_current_index: int = 0
        self._mission_elapsed_sec: float = 0.0
        self._last_error: Optional[str] = None
        self._nav2_error_code: int = 0
        self._nav2_error_msg: str = ""
        self._mb_last_stamp: float = 0.0
        self._prev_mission_state: Optional[str] = None

        # ---- internal state — localization / odom / GPS ----------------- #
        self._odom: Optional[Odometry] = None
        self._gps: Optional[NavSatFix] = None
        self._odom_last_stamp: float = 0.0
        self._gps_last_stamp: float = 0.0
        self._loc_health: LocHealth = LocHealth.GOOD
        self._tf_ok: bool = False
        self._position_cov_max: float = 0.0
        self._orientation_cov_max: float = 0.0
        self._odom_hz: float = 0.0

        # ---- internal state — pose -------------------------------------- #
        self._pose_x: float = 0.0
        self._pose_y: float = 0.0
        self._pose_yaw: float = 0.0
        self._linear_v: float = 0.0
        self._angular_w: float = 0.0

        # ---- odom frequency tracking ------------------------------------ #
        self._odom_stamps: Deque[float] = deque(maxlen=50)

        # ---- stuck detection -------------------------------------------- #
        self._last_progress_pose: Optional[Tuple[float, float]] = None
        self._last_progress_time: float = 0.0
        self._controller_stuck: bool = False

        # ---- waypoint action status tracking ---------------------------- #
        self._waypoint_goal_active: bool = False
        self._planner_failed: bool = False
        self._planner_last_error_time: float = 0.0

        # ---- active alarms set ------------------------------------------ #
        self._active_alarms: Set[str] = set()

        # ---- startup grace — suppress loc alarms until sensors arrive --- #
        self._startup_ts: float = time.time()
        self._startup_grace_sec: float = 10.0

        # ---- aggregated log capture (synthetic from health observers) --- #
        self._recent_logs: Deque[Dict[str, Any]] = deque(maxlen=200)

        # ---- nav_phase tracking ----------------------------------------- #
        self._nav_phase: str = "IDLE"

        # ---- timers ----------------------------------------------------- #
        self._publish_timer = self.create_timer(1.0 / self._publish_rate,
                                                self._publish_nav_status)
        self._tf_timer = self.create_timer(1.0, self._check_tf)

        self.get_logger().info(
            f"nav_status_aggregator started  rate={self._publish_rate}Hz  "
            f"vehicle={self._vehicle_id}  "
            f"status_detail={_status_detail_topic}  nav_status={_nav_status_topic}"
        )

    # ----------------------------------------------------------------------- #
    # Parameter declarations
    # ----------------------------------------------------------------------- #

    def _declare_params(self) -> None:
        self.declare_parameter("status_detail_topic", "/mission_bridge/status_detail")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("gps_topic", "/gps/fixed_cov")
        self.declare_parameter("follow_waypoints_status_topic",
                               "/follow_waypoints/_action/status")
        self.declare_parameter("planner_status_topic",
                               "/compute_path_to_pose/_action/status")
        self.declare_parameter("nav_status_topic", "/nav_status")
        self.declare_parameter("task_event_topic", "/task_event")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("vehicle_id", "usv_001")
        self.declare_parameter("odom_timeout", 2.0)
        self.declare_parameter("gps_timeout", 5.0)
        self.declare_parameter("mission_bridge_timeout", 5.0)
        self.declare_parameter("cov_lost_threshold", 10.0)
        self.declare_parameter("cov_degraded_threshold", 1.0)
        self.declare_parameter("stuck_progress_timeout", 12.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

    # ----------------------------------------------------------------------- #
    # Subscription callbacks
    # ----------------------------------------------------------------------- #

    def _cb_status_detail(self, msg: String) -> None:
        """Receive mission state snapshot from mission_bridge."""
        try:
            data: dict = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                f"[nav_status_aggregator]: invalid JSON in status_detail "
                f"({msg.data[:100] if msg.data else 'empty'})"
            )
            return

        now = time.time()
        self._mb_last_stamp = now

        prev_state = self._mission_state

        self._mission_state = data.get("state", "IDLE")
        self._task_id = data.get("task_id", "") or ""
        self._command_id = data.get("command_id", "") or ""
        self._waypoint_total = int(data.get("waypoint_total", 0))
        self._waypoint_completed = int(data.get("waypoint_completed", 0))
        self._waypoint_current_index = int(data.get("waypoint_current_index", 0))
        self._mission_elapsed_sec = float(data.get("elapsed_sec", 0.0))

        new_error = data.get("error_code")
        if new_error:
            self._last_error = str(new_error)
        elif self._mission_state in ("COMPLETED",):
            self._last_error = None

        # Capture Nav2 FollowWaypoints native error details
        n2_code = data.get("nav2_error_code", 0)
        n2_msg = data.get("nav2_error_msg", "")
        if n2_msg:
            self._nav2_error_code = int(n2_code)
            self._nav2_error_msg = str(n2_msg)
        elif self._mission_state in ("COMPLETED", "IDLE"):
            self._nav2_error_code = 0
            self._nav2_error_msg = ""

        # Detect mission state transitions and fire events
        if prev_state != self._mission_state:
            self._detect_mission_transition(prev_state, self._mission_state)
            self._prev_mission_state = prev_state

    def _cb_odom(self, msg: Odometry) -> None:
        """Receive odometry: pose, covariance, stuck detection."""
        now = time.time()
        self._odom_last_stamp = now
        self._odom = msg

        # Track frequency (wall-clock based)
        self._odom_stamps.append(now)
        if len(self._odom_stamps) >= 10:
            window = self._odom_stamps[-1] - self._odom_stamps[0]
            if window > 1.0:
                self._odom_hz = (len(self._odom_stamps) - 1) / window

        # Pose
        p = msg.pose.pose.position
        self._pose_x = p.x
        self._pose_y = p.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Twist
        self._linear_v = msg.twist.twist.linear.x
        self._angular_w = msg.twist.twist.angular.z

        # Covariance
        cov = msg.pose.covariance
        self._position_cov_max = max(cov[0], cov[7], cov[14])
        self._orientation_cov_max = max(cov[21], cov[28], cov[35])

        # Update health and stuck
        self._update_loc_health()
        self._update_stuck_detection()

    def _cb_gps(self, msg: NavSatFix) -> None:
        """Receive GPS fix status."""
        self._gps_last_stamp = time.time()
        self._gps = msg
        self._update_loc_health()

    def _cb_action_status(self, msg: GoalStatusArray) -> None:
        """Monitor FollowWaypoints action status.

        Used only to observe goal state; never sends goals.
        """
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        s = latest.status

        if s in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
            self._waypoint_goal_active = True
        elif s in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            self._waypoint_goal_active = False

    def _cb_planner_status(self, msg: GoalStatusArray) -> None:
        """Monitor planner server action status for real-time plan failures.

        Nav2 internally retries ComputePathToPose inside the BT, so the
        FollowWaypoints goal may stay RUNNING even while planning repeatedly
        fails.  This callback catches those intermediate failures.
        """
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        s = latest.status

        if s == GoalStatus.STATUS_ABORTED:
            self._planner_failed = True
            self._planner_last_error_time = time.time()
            self._push_log("WARN", "planner_server",
                           "GridBased: failed to create plan, no valid path found.")
            self._push_log("WARN", "planner_server",
                           "[compute_path_to_pose] [ActionServer] Aborting handle.")
            self.get_logger().warn(
                f"[nav_status_aggregator]: planner FAILED | "
                f"goal_status=ABORTED | target was unreachable or in obstacle"
            )
            self._fire_alarm(
                "PLAN_FAILED", "WARN",
                "Planner failed to find valid path — target may be in obstacle or outside map"
            )
        elif s == GoalStatus.STATUS_SUCCEEDED:
            if self._planner_failed:
                self.get_logger().info("[nav_status_aggregator]: planner recovered — plan OK")
            self._planner_failed = False
            self._fire_alarm_clear("PLAN_FAILED")

    @staticmethod
    def _normalize_log_level(level: Any) -> int:
        """Humble /rosout may deliver level as int or single-byte buffer."""
        if isinstance(level, bytes):
            return int(level[0]) if level else 0
        return int(level)

    def _cb_rosout(self, msg: Log) -> None:
        """Capture WARN/ERROR from /rosout (may be empty in component-container setups).

        Falls back gracefully — log entries are also generated by the
        aggregator's own health observers (_cb_planner_status,
        _update_stuck_detection, _update_loc_health).
        """
        level = self._normalize_log_level(msg.level)
        if level < _LOG_WARN:
            return
        node = str(getattr(msg, "name", "") or "")
        if not node:
            return
        nav_keywords = (
            "planner", "controller", "behavior", "bt_navigator",
            "costmap", "recovery", "waypoint", "mission_bridge",
        )
        if not any(kw in node for kw in nav_keywords):
            return

        level_name = {
            _LOG_DEBUG: "DEBUG",
            _LOG_INFO: "INFO",
            _LOG_WARN: "WARN",
            _LOG_ERROR: "ERROR",
            _LOG_FATAL: "FATAL",
        }
        self._push_log(
            level_name.get(level, str(level)),
            node,
            str(getattr(msg, "msg", "") or ""),
        )

    def _push_log(self, level: str, node: str, message: str) -> None:
        self._recent_logs.append({
            "stamp": time.time(),
            "level": level,
            "node": node,
            "message": message,
        })

    # ----------------------------------------------------------------------- #
    # Health logic
    # ----------------------------------------------------------------------- #

    def _update_loc_health(self) -> None:
        """Evaluate localization health from covariance, GPS, TF, timeouts."""
        prev_loc = self._loc_health

        odom_stale = (time.time() - self._odom_last_stamp) > self._odom_timeout
        gps_stale = (time.time() - self._gps_last_stamp) > self._gps_timeout

        if not self._tf_ok or odom_stale or self._position_cov_max > self._cov_lost:
            self._loc_health = LocHealth.LOST
        elif (
            self._position_cov_max > self._cov_degraded
            or (self._gps is not None and self._gps.status.status < 3)
            or (self._odom_hz > 0.0 and self._odom_hz < 20.0)
            or gps_stale
        ):
            self._loc_health = LocHealth.DEGRADED
        else:
            self._loc_health = LocHealth.GOOD

        if prev_loc == self._loc_health:
            return

        in_grace = (time.time() - self._startup_ts) < self._startup_grace_sec

        # Fire alarm on transition (suppress log push during startup grace)
        if self._loc_health == LocHealth.LOST:
            self._last_error = "LOC_LOST"
            gps_fix = self._gps.status.status if self._gps else -1
            if not in_grace:
                self._push_log("ERROR", "localization",
                               f"Localization LOST — tf_ok={self._tf_ok} cov_max={self._position_cov_max:.2f}m²")
            self._fire_alarm(
                "LOC_LOST", "ERROR",
                f"定位丢失: tf_ok={self._tf_ok} "
                f"cov_max={self._position_cov_max:.2f}m² "
                f"odom_hz={self._odom_hz:.1f}Hz  gps_fix={gps_fix}"
            )
            self.get_logger().error(
                f"[nav_status_aggregator]: localization LOST | "
                f"tf_ok={self._tf_ok} cov_max={self._position_cov_max:.2f}m²"
            )

        elif self._loc_health == LocHealth.DEGRADED:
            self._last_error = "LOC_DEGRADED"
            gps_fix = self._gps.status.status if self._gps else -1
            if not in_grace:
                self._push_log("WARN", "localization",
                               f"Localization DEGRADED — cov_max={self._position_cov_max:.2f}m² gps_fix={gps_fix}")
            self._fire_alarm(
                "LOC_DEGRADED", "WARN",
                f"定位退化: cov_max={self._position_cov_max:.2f}m² "
                f"gps_fix={gps_fix} odom_hz={self._odom_hz:.1f}Hz"
            )
            self.get_logger().warn(
                f"[nav_status_aggregator]: localization DEGRADED | "
                f"cov_max={self._position_cov_max:.2f}m² gps_fix={gps_fix} "
                f"odom_hz={self._odom_hz:.1f}"
            )

        else:  # Recovered to GOOD
            if prev_loc == LocHealth.LOST:
                self._fire_alarm_clear("LOC_LOST")
                self._fire_alarm_clear("LOC_DEGRADED")
            else:
                self._fire_alarm_clear("LOC_DEGRADED")
            if self._last_error in ("LOC_LOST", "LOC_DEGRADED"):
                self._last_error = None
            self.get_logger().info(
                f"[nav_status_aggregator]: localization GOOD (recovered from {prev_loc.value})"
            )

    def _update_stuck_detection(self) -> None:
        """Detect if the robot is stuck (no progress while mission RUNNING)."""
        if self._mission_state != "RUNNING":
            self._controller_stuck = False
            self._last_progress_pose = None
            return

        current_pose = (self._pose_x, self._pose_y)
        now = time.time()

        if self._last_progress_pose is None:
            self._last_progress_pose = current_pose
            self._last_progress_time = now
            return

        dx = current_pose[0] - self._last_progress_pose[0]
        dy = current_pose[1] - self._last_progress_pose[1]
        dist = math.hypot(dx, dy)

        if dist > 0.3:
            # Made progress — reset stuck timer
            self._last_progress_pose = current_pose
            self._last_progress_time = now
            if self._controller_stuck:
                self._controller_stuck = False
                self._push_log("INFO", "controller_server", "Controller recovered from STUCK")
                self.get_logger().info(
                    "[nav_status_aggregator]: controller recovered from STUCK"
                )
                self._fire_alarm_clear("CTRL_STUCK")
                if self._last_error == "CTRL_STUCK":
                    self._last_error = None
            return

        elapsed_no_progress = now - self._last_progress_time
        if elapsed_no_progress > self._stuck_timeout and not self._controller_stuck:
            self._controller_stuck = True
            self._last_error = "CTRL_STUCK"
            self._push_log("ERROR", "controller_server",
                           f"Controller STUCK — no progress for {elapsed_no_progress:.1f}s")
            self.get_logger().error(
                f"[nav_status_aggregator]: controller STUCK | "
                f"no_progress={elapsed_no_progress:.1f}s last_move={dist:.2f}m"
            )
            self._fire_alarm("CTRL_STUCK", "ERROR",
                             f"控制器无进展 {elapsed_no_progress:.1f}s")

    # ----------------------------------------------------------------------- #
    # TF monitoring
    # ----------------------------------------------------------------------- #

    def _check_tf(self) -> None:
        """Periodically check TF map→base_link availability."""
        try:
            self._tf_ok = self._tf_buffer.can_transform(
                self._global_frame,
                self._robot_frame,
                RTime(),
                timeout=RDuration(seconds=0.05),
            )
        except Exception:
            self._tf_ok = False

    # ----------------------------------------------------------------------- #
    # Mission transition detection
    # ----------------------------------------------------------------------- #

    def _detect_mission_transition(self, prev: str, curr: str) -> None:
        """Fire task events based on mission state transitions."""
        if prev == "IDLE" and curr == "RUNNING":
            self._last_error = None
            self._fire_event("TASK_STARTED", {
                "task_id": self._task_id,
                "total_waypoints": self._waypoint_total,
            })

        elif curr == "COMPLETED":
            self._last_error = None
            self._fire_event("TASK_COMPLETED", {
                "task_id": self._task_id,
                "elapsed_sec": self._mission_elapsed_sec,
            })

        elif curr == "FAILED":
            error_code = self._last_error or "MISSION_FAILED"
            n2_msg = getattr(self, "_nav2_error_msg", "") or ""
            detail: Dict[str, Any] = {
                "task_id": self._task_id,
                "error_code": error_code,
                "failed_waypoint_index": self._waypoint_completed,
                "reason": f"Mission failed with error: {error_code}",
            }
            if n2_msg:
                detail["nav2_error_code"] = getattr(self, "_nav2_error_code", 0)
                detail["nav2_error_msg"] = n2_msg
                self._push_log(
                    "ERROR", "mission_bridge",
                    f"Nav2 FollowWaypoints error ({error_code}): "
                    f"code={detail['nav2_error_code']} msg={n2_msg}"
                )
            self._fire_event("TASK_FAILED", detail)

        elif prev == "RUNNING" and curr == "IDLE":
            self._fire_event("TASK_CANCELLED", {
                "task_id": self._task_id,
                "source": "system",
            })

    # ----------------------------------------------------------------------- #
    # Event publishing
    # ----------------------------------------------------------------------- #

    def _fire_event(self, event_type: str, detail: Dict[str, Any]) -> None:
        """Publish a /task_event JSON message."""
        now = time.time()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        payload = {
            "schema_version": 1,
            "stamp": {"sec": sec, "nanosec": nsec},
            "vehicle_id": self._vehicle_id,
            "task_id": self._task_id,
            "command_id": self._command_id,
            "event": event_type,
            "detail": detail,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._task_event_pub.publish(msg)
        self.get_logger().info(
            f"[nav_status_aggregator]: /task_event {event_type}  "
            f"task={self._task_id or 'none'}"
        )

    def _fire_alarm(self, code: str, level: str, message: str) -> None:
        """Fire ALARM_RAISED event (if not already active)."""
        if code in self._active_alarms:
            return
        self._active_alarms.add(code)
        self._fire_event("ALARM_RAISED", {
            "alarm_code": code,
            "level": level,
            "message": message,
            "suggested_action": self._suggested_action(code),
        })

    def _fire_alarm_clear(self, code: str) -> None:
        """Fire ALARM_CLEARED event (if currently active)."""
        if code not in self._active_alarms:
            return
        self._active_alarms.discard(code)
        self._fire_event("ALARM_CLEARED", {"alarm_code": code})

    @staticmethod
    def _suggested_action(code: str) -> str:
        suggestions = {
            "LOC_LOST": "检查 GPS/EKF 状态，确认传感器数据正常",
            "LOC_DEGRADED": "检查 GPS 信号强度，确认定位数据质量",
            "CTRL_STUCK": "检查推进器状态，确认无障碍物卡死",
            "PLAN_FAILED": "检查目标航点坐标，确认不在障碍层内",
            "MISSION_FAILED": "检查导航系统状态，确认任务参数正确",
        }
        return suggestions.get(code, "")

    # ----------------------------------------------------------------------- #
    # Nav status publishing (periodic)
    # ----------------------------------------------------------------------- #

    def _publish_nav_status(self) -> None:
        """Compose and publish the /nav_status 2Hz snapshot per §3.2 schema."""
        now = time.time()
        odom_stale = (now - self._odom_last_stamp) > self._odom_timeout
        gps_stale = (now - self._gps_last_stamp) > self._gps_timeout
        mb_alive = (now - self._mb_last_stamp) < self._mb_timeout
        gps_fix = self._gps.status.status if self._gps is not None else 0

        nav_phase = self._derive_nav_phase()
        planner_status = self._derive_planner_status()
        controller_status = self._derive_controller_status()

        sec = int(now)
        nsec = int((now - sec) * 1e9)

        progress = 0.0
        if self._waypoint_total > 0:
            progress = round(
                (self._waypoint_completed / self._waypoint_total) * 100.0, 1
            )

        payload = {
            "schema_version": 1,
            "stamp": {"sec": sec, "nanosec": nsec},
            "vehicle_id": self._vehicle_id,
            "task": {
                "state": self._mission_state,
                "task_id": self._task_id or None,
                "command_id": self._command_id or None,
                "nav_phase": nav_phase,
                "current_waypoint": self._waypoint_completed,
                "total_waypoints": self._waypoint_total,
                "progress_percent": progress,
                "elapsed_sec": round(self._mission_elapsed_sec, 1),
                "distance_to_goal_m": 0.0,
                "eta_sec": None,
                "last_error": self._last_error,
            },
            "planner": {
                "status": planner_status,
                "last_plan_time_ms": 0.0,
                "last_error": self._derive_planner_last_error(),
            },
            "controller": {
                "status": controller_status,
                "tracking_error_m": 0.0,
                "last_error": self._last_error
                if self._last_error == "CTRL_STUCK" else None,
            },
            "localization": {
                "overall": self._loc_health.value,
                "position_cov_max": round(self._position_cov_max, 4),
                "orientation_cov_max": round(self._orientation_cov_max, 4),
                "gps_fix": gps_fix,
                "tf_ok": self._tf_ok,
                "odom_hz": round(self._odom_hz, 1),
            },
            "pose": {
                "x": round(self._pose_x, 4),
                "y": round(self._pose_y, 4),
                "yaw": round(self._pose_yaw, 4),
                "v": round(self._linear_v, 4),
                "w": round(self._angular_w, 4),
            },
            "flags": {
                "manual_override": False,
                "emergency_stop": False,
                "recovery_active": nav_phase == "RECOVERY",
            },
            "alerts": {
                "odom_stale": odom_stale,
                "gps_stale": gps_stale,
                "mission_bridge_alive": mb_alive,
                "planner_error": planner_status == "FAILED",
                "controller_error": controller_status in ("FAILED", "STUCK"),
            },
            "recent_logs": list(self._recent_logs),
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._nav_status_pub.publish(msg)

    # ----------------------------------------------------------------------- #
    # Phase derivation helpers
    # ----------------------------------------------------------------------- #

    def _derive_nav_phase(self) -> str:
        """Derive navigation phase from available data."""
        if self._mission_state == "IDLE":
            return "IDLE"
        if self._controller_stuck:
            return "STUCK"
        if self._waypoint_goal_active:
            return "TRACKING"
        if self._mission_state == "RUNNING":
            return "TRACKING"
        return "IDLE"

    def _derive_planner_status(self) -> str:
        """Derive planner health status from real-time action monitoring."""
        if self._planner_failed:
            return "FAILED"
        if self._last_error in ("PLAN_FAILED", "MISSION_FAILED"):
            return "FAILED"
        return "OK"

    def _derive_planner_last_error(self) -> Optional[str]:
        if self._planner_failed:
            return "PLAN_FAILED"
        if self._last_error in ("PLAN_FAILED", "MISSION_FAILED"):
            return self._last_error
        return None

    def _derive_controller_status(self) -> str:
        """Derive controller health status."""
        if self._controller_stuck:
            return "STUCK"
        if self._last_error == "CTRL_STUCK":
            return "STUCK"
        return "OK"


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = NavStatusAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
