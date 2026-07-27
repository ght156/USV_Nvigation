"""TF-based USV docking controller.

Uses ``lookup_transform("dock_frame", "base_link")`` to obtain the boat's
pose in the dock coordinate frame and drives the boat into the dock
stern-first.

Search strategy: once TF has been obtained at least once, the dock bearing
is continuously estimated via odometry even when the tag is lost.  Search
direction always points toward the estimated bearing, making search and
alignment a continuous same-direction motion.

Designed to coexist with the legacy ``docking_controller`` node — launch
one or the other, not both.  External interfaces (``/dock/start``,
``/dock/status``, ``/cmd_vel_nav``) are kept compatible so that
``dock_mission`` works unchanged.
"""

from __future__ import annotations

import json
import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener

from usv_docking.pose_filter import PoseEmaFilter
from usv_docking.tf_pose_provider import PoseData, PoseQuality, TfPoseProvider

_DT = 0.1  # control period (s)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# states
# ---------------------------------------------------------------------------

class TfDockState(Enum):
    IDLE = "TF_DOCK_IDLE"
    ACQUIRE_TAG = "TF_DOCK_ACQUIRE_TAG"
    SEARCH_TAG = "TF_DOCK_SEARCH_TAG"
    RECORD_POSE = "TF_DOCK_RECORD_POSE"
    APPROACH_ALIGN = "TF_DOCK_APPROACH_ALIGN"
    ALIGN = "TF_DOCK_ALIGN"
    BACK_IN = "TF_DOCK_BACK_IN"
    CORRECTING = "TF_DOCK_CORRECTING"
    DOCKED = "TF_DOCK_DOCKED"
    ABORT = "TF_DOCK_ABORT"


# ---------------------------------------------------------------------------
# node
# ---------------------------------------------------------------------------

class TfDockingNode(Node):
    def __init__(self) -> None:
        super().__init__("tf_docking_node")
        self._declare_params()
        self._load_params()

        # --- TF ---
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pose_provider = TfPoseProvider(
            self._tf_buffer,
            self,
            dock_frame=self._dock_frame,
            robot_frame=self._robot_frame,
            max_age_sec=self._tf_max_age,
            jump_threshold_m=self._tf_jump_threshold,
            min_range_m=self._tf_min_range,
            max_range_m=self._tf_max_range,
        )
        self._pose_filter = PoseEmaFilter(self._filter_alpha)

        # --- ROS I/O ---
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self.create_subscription(Bool, self._start_topic, self._start_cb, 10)
        self.create_subscription(Empty, self._cancel_topic, self._cancel_cb, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self._odom_x: Optional[float] = None
        self._odom_y: Optional[float] = None
        self._odom_yaw: Optional[float] = None

        # --- session state (reset per attempt) ---
        self._reset_session()

        self._timer = self.create_timer(_DT, self._tick)
        self.get_logger().info("tf_docking_node ready")

    # ==================================================================
    # parameters
    # ==================================================================

    def _declare_params(self) -> None:
        d = self.declare_parameter
        # frames / topics
        d("dock_frame", "dock_frame")
        d("robot_frame", "base_link")
        d("cmd_vel_topic", "/cmd_vel_nav")
        d("start_topic", "/dock/start")
        d("cancel_topic", "/dock/cancel")
        d("status_topic", "/dock/status")
        d("odom_topic", "/odometry/filtered")
        # TF quality
        d("tf_max_age_sec", 0.3)
        d("tf_max_age_back_in_sec", 0.2)
        d("tf_jump_threshold_m", 1.0)
        d("tf_min_range_m", 0.3)
        d("tf_max_range_m", 30.0)
        # filter
        d("pose_filter_alpha", 0.35)
        d("yaw_outlier_reject_rad", 1.5)
        # dock geometry
        d("entry_heading_rad", math.pi)
        # ACQUIRE_TAG
        d("acquire_cycles", 5)
        d("acquire_timeout_sec", 5.0)
        # SEARCH_TAG
        d("search_spin_speed", 0.35)
        d("search_kyaw", 0.40)
        d("search_timeout_sec", 90.0)
        d("search_angle_rad", 2.0 * math.pi)
        d("search_confirm_frames", 2)
        d("search_dwell_sec", 1.0)
        d("search_servo_gain", 0.5)
        d("max_search_retries", 2)
        # RECORD_POSE
        d("max_lateral_offset_m", 3.0)
        d("max_heading_offset_rad", 1.0)
        # APPROACH_ALIGN
        d("approach_speed", 0.30)
        d("approach_kyaw", 0.40)
        d("approach_entry_x_m", 3.0)
        d("approach_timeout_sec", 90.0)
        # ALIGN
        d("align_ky", 0.20)
        d("align_kyaw", 0.40)
        d("align_y_tol_m", 0.10)
        d("align_yaw_tol_rad", 0.08)
        d("align_settle_cycles", 5)
        d("align_creep_speed", 0.08)
        d("align_creep_yaw_rad", 0.26)
        d("align_timeout_sec", 45.0)
        # BACK_IN
        d("back_in_kx", 0.15)
        d("back_in_ky", 0.20)
        d("back_in_kyaw", 0.40)
        d("back_in_max_speed", 0.30)
        d("back_in_min_speed", 0.05)
        d("back_in_corridor_y_m", 0.30)
        d("back_in_corridor_yaw_rad", 0.26)
        d("back_in_corridor_cycles", 10)
        d("back_in_timeout_sec", 90.0)
        # BACK_IN blind backing (tag lost near dock is expected)
        d("back_in_blind_depth_m", 1.5)
        d("back_in_blind_speed", 0.05)
        d("back_in_blind_timeout_sec", 15.0)
        # CORRECTING
        d("correct_timeout_sec", 30.0)
        d("correct_max_retries", 2)
        d("correct_creep_speed", 0.03)
        # DOCKED thresholds
        d("stop_x_m", 0.30)
        d("stop_y_m", 0.15)
        d("stop_yaw_rad", 0.08)
        # tag loss
        d("tag_loss_hold_sec", 5.0)
        # global limits
        d("max_docking_duration_sec", 240.0)
        d("max_yaw_rate", 0.30)
        d("max_reverse_speed", 0.50)

    def _load_params(self) -> None:
        g = lambda n: self.get_parameter(n)  # noqa: E731
        self._dock_frame = g("dock_frame").value
        self._robot_frame = g("robot_frame").value
        self._cmd_vel_topic = g("cmd_vel_topic").value
        self._start_topic = g("start_topic").value
        self._cancel_topic = g("cancel_topic").value
        self._status_topic = g("status_topic").value
        self._odom_topic = g("odom_topic").value

        self._tf_max_age = float(g("tf_max_age_sec").value)
        self._tf_max_age_back_in = float(g("tf_max_age_back_in_sec").value)
        self._tf_jump_threshold = float(g("tf_jump_threshold_m").value)
        self._tf_min_range = float(g("tf_min_range_m").value)
        self._tf_max_range = float(g("tf_max_range_m").value)

        self._filter_alpha = float(g("pose_filter_alpha").value)
        self._yaw_outlier_rad = float(g("yaw_outlier_reject_rad").value)

        self._entry_heading = float(g("entry_heading_rad").value)

        self._acquire_cycles = int(g("acquire_cycles").value)
        self._acquire_timeout = float(g("acquire_timeout_sec").value)

        self._search_spin = float(g("search_spin_speed").value)
        self._search_kyaw = float(g("search_kyaw").value)
        self._search_timeout = float(g("search_timeout_sec").value)
        self._search_angle = float(g("search_angle_rad").value)
        self._search_confirm_frames = int(g("search_confirm_frames").value)
        self._search_dwell = float(g("search_dwell_sec").value)
        self._search_servo_gain = float(g("search_servo_gain").value)
        self._max_search_retries = int(g("max_search_retries").value)

        self._max_lateral = float(g("max_lateral_offset_m").value)
        self._max_heading_off = float(g("max_heading_offset_rad").value)

        self._app_speed = float(g("approach_speed").value)
        self._app_kyaw = float(g("approach_kyaw").value)
        self._app_entry_x = float(g("approach_entry_x_m").value)
        self._app_timeout = float(g("approach_timeout_sec").value)

        self._al_ky = float(g("align_ky").value)
        self._al_kyaw = float(g("align_kyaw").value)
        self._al_y_tol = float(g("align_y_tol_m").value)
        self._al_yaw_tol = float(g("align_yaw_tol_rad").value)
        self._al_settle = int(g("align_settle_cycles").value)
        self._al_creep_v = float(g("align_creep_speed").value)
        self._al_creep_yaw = float(g("align_creep_yaw_rad").value)
        self._al_timeout = float(g("align_timeout_sec").value)

        self._bi_kx = float(g("back_in_kx").value)
        self._bi_ky = float(g("back_in_ky").value)
        self._bi_kyaw = float(g("back_in_kyaw").value)
        self._bi_max_v = float(g("back_in_max_speed").value)
        self._bi_min_v = float(g("back_in_min_speed").value)
        self._bi_corr_y = float(g("back_in_corridor_y_m").value)
        self._bi_corr_yaw = float(g("back_in_corridor_yaw_rad").value)
        self._bi_corr_cycles = int(g("back_in_corridor_cycles").value)
        self._bi_timeout = float(g("back_in_timeout_sec").value)
        self._bi_blind_depth = float(g("back_in_blind_depth_m").value)
        self._bi_blind_speed = float(g("back_in_blind_speed").value)
        self._bi_blind_timeout = float(g("back_in_blind_timeout_sec").value)

        self._corr_timeout = float(g("correct_timeout_sec").value)
        self._corr_max_retries = int(g("correct_max_retries").value)
        self._corr_creep_v = float(g("correct_creep_speed").value)

        self._stop_x = float(g("stop_x_m").value)
        self._stop_y = float(g("stop_y_m").value)
        self._stop_yaw = float(g("stop_yaw_rad").value)

        self._tag_loss_hold = float(g("tag_loss_hold_sec").value)
        self._max_duration = float(g("max_docking_duration_sec").value)
        self._max_w = float(g("max_yaw_rate").value)
        self._max_rev_v = float(g("max_reverse_speed").value)

    # ==================================================================
    # session
    # ==================================================================

    def _reset_session(self) -> None:
        self._state = TfDockState.IDLE
        self._state_time = 0.0
        self._session_time = 0.0
        self._abort_reason = ""
        self._needs_reapproach = False

        # pose (filtered)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._pose_valid = False

        # ACQUIRE_TAG
        self._acq_count = 0

        # SEARCH_TAG
        self._search_retries = 0
        self._search_total_angle = 0.0
        self._search_confirm_count = 0
        self._search_dir = 1.0
        self._search_yaw_prev: Optional[float] = None
        self._resume_state: Optional[TfDockState] = None

        # bearing estimator (persists across search cycles within session)
        self._est_bearing: Optional[float] = None
        self._est_range: Optional[float] = None
        self._est_odom_yaw: Optional[float] = None

        # ALIGN
        self._align_count = 0

        # BACK_IN / CORRECTING
        self._corr_violation_count = 0
        self._corr_retries = 0
        self._blind_time = 0.0

        # tag loss
        self._tag_loss_time = 0.0

        # cmd cache
        self._cmd_v = 0.0
        self._cmd_w = 0.0

        self._pose_provider.reset()
        self._pose_filter.reset()

    # ==================================================================
    # callbacks
    # ==================================================================

    def _start_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._state not in (TfDockState.IDLE, TfDockState.DOCKED, TfDockState.ABORT):
            self.get_logger().warn(f"ignoring /dock/start in state {self._state.value}")
            return
        self._reset_session()
        self._transition(TfDockState.ACQUIRE_TAG, "start requested")

    def _cancel_cb(self, _msg: Empty) -> None:
        if self._state not in (TfDockState.IDLE, TfDockState.DOCKED, TfDockState.ABORT):
            self._abort("USER_CANCEL")

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._odom_yaw = math.atan2(siny, cosy)

    # ==================================================================
    # main tick
    # ==================================================================

    def _tick(self) -> None:
        self._session_time += _DT
        self._state_time += _DT

        if self._state == TfDockState.IDLE:
            self._publish_cmd(0.0, 0.0)
            self._publish_status()
            return

        if self._state in (TfDockState.DOCKED, TfDockState.ABORT):
            self._publish_cmd(0.0, 0.0)
            self._publish_status()
            return

        # global timeout
        if self._session_time > self._max_duration:
            self._abort("MAX_DOCKING_DURATION")
            return

        # --- get pose ---
        max_age = self._tf_max_age
        if self._state in (TfDockState.BACK_IN, TfDockState.CORRECTING):
            max_age = self._tf_max_age_back_in
        raw = self._pose_provider.get_pose(max_age_override=max_age)

        # --- update filtered pose + bearing estimate ---
        if raw.quality == PoseQuality.GOOD:
            yaw = raw.yaw
            if self._pose_valid and abs(_wrap_yaw(yaw - self._yaw)) > self._yaw_outlier_rad:
                yaw = self._yaw
            self._x, self._y, self._yaw = self._pose_filter.update(raw.x, raw.y, yaw)
            self._pose_valid = True
            self._tag_loss_time = 0.0
            self._update_bearing_estimate()
        else:
            self._pose_valid = False

        # --- dispatch ---
        handler = {
            TfDockState.ACQUIRE_TAG: self._run_acquire_tag,
            TfDockState.SEARCH_TAG: self._run_search_tag,
            TfDockState.RECORD_POSE: self._run_record_pose,
            TfDockState.APPROACH_ALIGN: self._run_approach_align,
            TfDockState.ALIGN: self._run_align,
            TfDockState.BACK_IN: self._run_back_in,
            TfDockState.CORRECTING: self._run_correcting,
        }.get(self._state)

        if handler is not None:
            handler(raw)

        self._publish_status()

    # ==================================================================
    # bearing estimator
    # ==================================================================

    def _update_bearing_estimate(self) -> None:
        """Record dock bearing in boat frame whenever TF is GOOD."""
        if self._odom_yaw is None:
            return
        bearing_dock = math.atan2(-self._y, -self._x)
        bearing_boat = _wrap_yaw(bearing_dock - self._yaw)
        offset = _wrap_yaw(bearing_boat - math.pi)
        self._est_bearing = offset
        self._est_range = math.hypot(self._x, self._y)
        self._est_odom_yaw = self._odom_yaw

    def _estimated_bearing(self) -> Optional[float]:
        """Current estimated dock bearing offset from stern, via odom dead-reckoning.

        Returns angle in [-pi, pi]: positive = dock left of stern, negative = right.
        Returns None if no estimate available.
        """
        if self._est_bearing is None or self._est_odom_yaw is None:
            return None
        if self._odom_yaw is None:
            return self._est_bearing
        delta_yaw = _wrap_yaw(self._odom_yaw - self._est_odom_yaw)
        return _wrap_yaw(self._est_bearing - delta_yaw)

    # ==================================================================
    # state handlers
    # ==================================================================

    # ---- ACQUIRE_TAG ------------------------------------------------

    def _run_acquire_tag(self, raw: PoseData) -> None:
        if raw.quality == PoseQuality.GOOD:
            self._acq_count += 1
            if self._acq_count >= self._acquire_cycles:
                self._transition(TfDockState.RECORD_POSE, "tag acquired")
                return
        else:
            self._acq_count = 0

        self._publish_cmd(0.0, 0.0)

        if self._state_time > self._acquire_timeout:
            self._enter_search(from_state=TfDockState.ACQUIRE_TAG)

    # ---- SEARCH_TAG -------------------------------------------------

    def _run_search_tag(self, raw: PoseData) -> None:
        if raw.quality == PoseQuality.GOOD:
            # TF available: servo heading toward alignment (stern = π in dock frame)
            heading_err = self._heading_error()
            w = _clamp(-self._search_kyaw * heading_err, -self._max_w, self._max_w)
            self._publish_cmd(0.0, w)

            if abs(heading_err) < self._al_creep_yaw:
                self._search_confirm_count += 1
                if self._search_confirm_count >= self._search_confirm_frames:
                    self._transition(
                        TfDockState.RECORD_POSE,
                        f"tag found, heading_err={heading_err:.3f}",
                    )
                    return
            else:
                self._search_confirm_count = 0
            return

        # --- TF lost: blind spin in locked direction ---
        self._search_confirm_count = 0
        w = self._search_dir * abs(self._search_spin)

        if self._odom_yaw is not None and self._search_yaw_prev is not None:
            self._search_total_angle += abs(_wrap_yaw(self._odom_yaw - self._search_yaw_prev))
        else:
            self._search_total_angle += abs(w) * _DT
        self._search_yaw_prev = self._odom_yaw

        self._publish_cmd(0.0, w)

        if self._search_total_angle >= self._search_angle:
            if self._search_retries < self._max_search_retries:
                self._search_retries += 1
                self._search_dir *= -1.0
                self._search_total_angle = 0.0
                self._search_yaw_prev = self._odom_yaw
                self.get_logger().info(
                    f"SEARCH: retry {self._search_retries}/{self._max_search_retries}, "
                    f"dir={'CCW' if self._search_dir > 0 else 'CW'}"
                )
                return
            self._abort("TAG_SEARCH_NO_TAG", needs_reapproach=True)
            return

        if self._state_time > self._search_timeout:
            self._abort("TAG_SEARCH_TIMEOUT", needs_reapproach=True)

    # ---- RECORD_POSE ------------------------------------------------

    def _run_record_pose(self, _raw: PoseData) -> None:
        if not self._pose_valid:
            self._publish_cmd(0.0, 0.0)
            return

        depth = abs(self._x)
        lateral = abs(self._y)
        heading_err = abs(self._heading_error())

        self.get_logger().info(
            f"RECORD_POSE: x={self._x:.2f} y={self._y:.2f} yaw={self._yaw:.2f} "
            f"depth={depth:.2f} lateral={lateral:.2f} heading_err={heading_err:.3f}"
        )

        if lateral > self._max_lateral or heading_err > self._max_heading_off:
            self._abort(
                f"POSITION_TOO_FAR (y={lateral:.2f} heading={heading_err:.2f})",
                needs_reapproach=True,
            )
            return

        if depth < self._stop_x:
            self._transition(TfDockState.DOCKED, "already inside dock")
            return

        next_state = self._resume_state or TfDockState.APPROACH_ALIGN
        self._resume_state = None
        self._transition(next_state, f"pose recorded, depth={depth:.2f}")

    # ---- APPROACH_ALIGN ---------------------------------------------

    def _run_approach_align(self, raw: PoseData) -> None:
        if self._check_tag_loss(raw):
            return

        depth = abs(self._x)

        # transition to ALIGN when close enough
        if depth < self._app_entry_x:
            self._transition(TfDockState.ALIGN, f"entered approach zone, depth={depth:.2f}")
            return

        # bearing-based curved approach: aim stern at entrance (dock origin)
        bearing = math.atan2(-self._y, -self._x)
        desired_yaw = _wrap_yaw(bearing + math.pi)
        heading_err = _wrap_yaw(self._yaw - desired_yaw)

        v = -self._app_speed
        w = _clamp(-self._app_kyaw * heading_err, -self._max_w, self._max_w)
        self._publish_cmd(v, w)

        if self._state_time > self._app_timeout:
            self._abort("APPROACH_ALIGN_TIMEOUT")

    # ---- ALIGN ------------------------------------------------------

    def _run_align(self, raw: PoseData) -> None:
        if self._check_tag_loss(raw):
            return

        heading_err = self._heading_error()
        w = self._steer(heading_err, self._y, self._al_ky, self._al_kyaw)

        # creep backward when heading is close
        if abs(heading_err) < self._al_creep_yaw:
            v = -self._al_creep_v
        else:
            v = 0.0

        self._publish_cmd(v, w)

        if abs(self._y) < self._al_y_tol and abs(heading_err) < self._al_yaw_tol:
            self._align_count += 1
            if self._align_count >= self._al_settle:
                self._transition(TfDockState.BACK_IN, "aligned")
                return
        else:
            self._align_count = 0

        if self._state_time > self._al_timeout:
            self._abort("ALIGN_TIMEOUT")

    # ---- BACK_IN ----------------------------------------------------

    def _run_back_in(self, raw: PoseData) -> None:
        depth = abs(self._x)

        # --- blind backing: tag loss near dock is expected ---
        if raw.quality != PoseQuality.GOOD and depth < self._bi_blind_depth:
            self._blind_time += _DT
            self._publish_cmd(-self._bi_blind_speed, 0.0)
            if self._blind_time > self._bi_blind_timeout:
                self._transition(TfDockState.DOCKED, "blind back-in complete")
            return

        # tag recovered → reset blind timer
        self._blind_time = 0.0

        if self._check_tag_loss(raw):
            return

        heading_err = self._heading_error()

        # docked?
        if depth < self._stop_x and abs(self._y) < self._stop_y and abs(heading_err) < self._stop_yaw:
            self._transition(TfDockState.DOCKED, "docked")
            return

        remaining = max(depth - self._stop_x, 0.0)
        v = -_clamp(self._bi_kx * remaining, self._bi_min_v, self._bi_max_v)
        w = self._steer(heading_err, self._y, self._bi_ky, self._bi_kyaw)
        self._publish_cmd(v, w)

        # corridor check
        if abs(self._y) > self._bi_corr_y or abs(heading_err) > self._bi_corr_yaw:
            self._corr_violation_count += 1
            if self._corr_violation_count >= self._bi_corr_cycles:
                self._corr_violation_count = 0
                self._transition(TfDockState.CORRECTING, "corridor violation")
                return
        else:
            self._corr_violation_count = 0

        if self._state_time > self._bi_timeout:
            self._abort("BACK_IN_TIMEOUT")

    # ---- CORRECTING -------------------------------------------------

    def _run_correcting(self, raw: PoseData) -> None:
        if self._check_tag_loss(raw):
            return

        heading_err = self._heading_error()
        w = self._steer(heading_err, self._y, self._bi_ky, self._bi_kyaw)
        self._publish_cmd(-self._corr_creep_v, w)

        if abs(self._y) < self._bi_corr_y and abs(heading_err) < self._bi_corr_yaw:
            self._transition(TfDockState.BACK_IN, "corrected, resuming back-in")
            return

        if self._state_time > self._corr_timeout:
            self._corr_retries += 1
            if self._corr_retries > self._corr_max_retries:
                self._abort("CORRECTING_TIMEOUT")
            else:
                self._state_time = 0.0

    # ==================================================================
    # tag-loss guard (shared by APPROACH_ALIGN / ALIGN / BACK_IN / CORRECTING)
    # ==================================================================

    def _check_tag_loss(self, raw: PoseData) -> bool:
        """Three-level tag loss response. Return True if handler should skip."""
        if raw.quality == PoseQuality.GOOD:
            self._tag_loss_time = 0.0
            return False

        self._tag_loss_time += _DT

        # Level 1: brief loss — maintain momentum with decay
        if self._tag_loss_time < self._search_dwell:
            self._publish_cmd(self._cmd_v * 0.5, self._cmd_w * 0.5)
            return True

        # Level 2: medium loss — servo toward estimated bearing via odom
        est = self._estimated_bearing()
        if est is not None and self._tag_loss_time < self._tag_loss_hold:
            w = _clamp(-self._search_servo_gain * est, -self._max_w, self._max_w)
            self._publish_cmd(0.0, w)
            return True

        # Level 3: prolonged loss — enter directed search
        self._enter_search(from_state=self._state)
        return True

    def _enter_search(self, from_state: TfDockState) -> None:
        self._resume_state = from_state if from_state != TfDockState.ACQUIRE_TAG else None
        self._search_total_angle = 0.0
        self._search_confirm_count = 0
        self._search_yaw_prev = self._odom_yaw
        # lock direction from bearing estimate (won't change during search)
        est = self._estimated_bearing()
        if est is not None and abs(est) > 0.05:
            self._search_dir = math.copysign(1.0, est)
        elif self._cmd_w != 0.0:
            self._search_dir = math.copysign(1.0, self._cmd_w)
        else:
            self._search_dir = 1.0
        self._transition(TfDockState.SEARCH_TAG, f"tag lost in {from_state.value}")

    # ==================================================================
    # control helpers
    # ==================================================================

    def _heading_error(self) -> float:
        return _wrap_yaw(self._yaw - self._entry_heading)

    def _steer(self, heading_err: float, y: float, ky: float, kyaw: float) -> float:
        """Angular velocity for stern-first docking (reversing)."""
        w = ky * y - kyaw * heading_err
        return _clamp(w, -self._max_w, self._max_w)

    # ==================================================================
    # output
    # ==================================================================

    def _publish_cmd(self, v: float, w: float) -> None:
        v = _clamp(v, -self._max_rev_v, 0.0)
        w = _clamp(w, -self._max_w, self._max_w)
        self._cmd_v = v
        self._cmd_w = w
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._cmd_pub.publish(msg)

    def _publish_status(self) -> None:
        payload = {
            "state": self._state.value,
            "success": self._state == TfDockState.DOCKED,
            "needs_reapproach": self._needs_reapproach,
            "abort_reason": self._abort_reason,
            "x": round(self._x, 3),
            "y": round(self._y, 3),
            "heading_error": round(self._heading_error(), 3),
            "pose_valid": self._pose_valid,
            "cmd_v": round(self._cmd_v, 3),
            "cmd_w": round(self._cmd_w, 3),
            "session_time": round(self._session_time, 1),
            "est_bearing": round(self._estimated_bearing() or 0.0, 3),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    # ==================================================================
    # transitions
    # ==================================================================

    def _transition(self, new: TfDockState, reason: str = "") -> None:
        old = self._state
        self._state = new
        self._state_time = 0.0
        self._align_count = 0
        self._corr_violation_count = 0
        self._acq_count = 0
        if reason:
            self.get_logger().info(f"{old.value} -> {new.value}: {reason}")

    def _abort(self, reason: str, needs_reapproach: bool = False) -> None:
        self._abort_reason = reason
        self._needs_reapproach = needs_reapproach
        self._publish_cmd(0.0, 0.0)
        self._transition(TfDockState.ABORT, reason)
        self.get_logger().error(f"ABORT: {reason} (reapproach={needs_reapproach})")


# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
