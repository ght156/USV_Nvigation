"""USV docking v5: GNSS virtual entry → Tag search/align → visual back-in."""

from __future__ import annotations

import json
import math
import time
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformListener

from usv_docking.bay_config import load_gnss_bay_geometry
from usv_docking.gnss_geo import GnssBayGeometry
from usv_docking.dock_feedback import error_code_from_reason, is_auto_retry_reason, phase_from_state
from usv_docking.gnss_approach import (
    at_virtual_entry,
    bearing_to_gnss_target,
    compute_leg_errors,
    compute_waypoint_errors,
    gnss_back_creep_speed,
    gnss_back_motion_cmd,
    heading_align_omega,
    heading_aligned,
    stern_toward_yaw,
)
from usv_docking.pose_filter import PoseEmaFilter
from usv_docking.pose_transform import DockPoseTransformer
from usv_docking.undock_controller import (
    OdomReading,
    UndockConfig,
    UndockController,
    UNDOCK_PARAM_DEFAULTS,
    UndockState,
)


class DockState(str, Enum):
    IDLE = "DOCK_IDLE"
    PRECHECK = "DOCK_PRECHECK"
    WAIT_TAG = "DOCK_WAIT_TAG"
    SEARCH_SPIN = "DOCK_SEARCH_SPIN"
    GNSS_HEADING_ALIGN = "DOCK_GNSS_HEADING_ALIGN"
    GNSS_BACK_TO_ENTRY = "DOCK_GNSS_BACK_TO_ENTRY"
    GNSS_ENTRY_SETTLE = "DOCK_GNSS_ENTRY_SETTLE"
    VISION_SEARCH_TAG = "DOCK_VISION_SEARCH_TAG"
    APPROACH_ENTRY = "DOCK_APPROACH_ENTRY"
    ALIGN_ENTRY = "DOCK_ALIGN_ENTRY"
    BACK_IN = "DOCK_BACK_IN"
    WAIT_CHARGE = "DOCK_WAIT_CHARGE"
    STOP = "DOCK_STOP"
    UNDOCK_OUT = "DOCK_UNDOCK_OUT"
    UNDOCK_SETTLE = "DOCK_UNDOCK_SETTLE"
    UNDOCK_STOP = "DOCK_UNDOCK_STOP"
    ABORT = "DOCK_ABORT"


_UNDOCK_TO_DOCK: dict[UndockState, DockState] = {
    UndockState.OUT: DockState.UNDOCK_OUT,
    UndockState.SETTLE: DockState.UNDOCK_SETTLE,
    UndockState.STOP: DockState.UNDOCK_STOP,
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


class DockingController(Node):
    def __init__(self) -> None:
        super().__init__("docking_controller")
        self._declare_parameters()
        self._load_parameters()
        self._undock = UndockController(UndockConfig.from_ros(self.get_parameter))

        self._state = DockState.IDLE
        self._abort_reason: Optional[str] = None
        self._needs_reapproach = False
        self._mission_state = "UNKNOWN"

        self._last_tag_time: Optional[rclpy.time.Time] = None
        self._last_raw_pose: Optional[list[float]] = None
        self._x_base = 0.0
        self._y_base = 0.0
        self._yaw_base = 0.0
        self._pose_valid = False
        self._tf_error: Optional[str] = None

        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._settle_count = 0
        self._entry_settle_count = 0
        self._docking_start_time: Optional[rclpy.time.Time] = None
        self._wait_tag_start_time: Optional[rclpy.time.Time] = None
        self._search_spin_start_time: Optional[rclpy.time.Time] = None
        self._search_spin_accum_rad = 0.0
        self._search_spin_last_odom_yaw: Optional[float] = None
        self._approach_entry_start_time: Optional[rclpy.time.Time] = None
        self._align_entry_start_time: Optional[rclpy.time.Time] = None
        self._align_settle_count = 0
        self._resume_state_after_search: Optional[DockState] = None
        self._search_spin_bias_dir: Optional[float] = None
        self._tag_acquire_count = 0
        self._tag_reacquire_count = 0
        self._tag_reacquire_stable_start_time = None
        self._tag_was_acquired = False
        self._align_entry_holding_for_tag = False
        self._back_in_blind_active = False
        self._back_in_blind_start_time: Optional[rclpy.time.Time] = None
        self._back_in_holding_for_tag = False
        self._back_in_last_heading_error: Optional[float] = None
        self._back_in_last_y_base: Optional[float] = None
        self._have_odom = False
        self._odom_x: Optional[float] = None
        self._odom_y: Optional[float] = None
        self._odom_yaw: Optional[float] = None
        self._last_odom_time: Optional[rclpy.time.Time] = None
        self._gnss_bay: Optional[GnssBayGeometry] = None
        self._gnss_lat: Optional[float] = None
        self._gnss_lon: Optional[float] = None
        self._gnss_status: int = -1
        self._have_gnss = False
        self._last_gnss_time: Optional[rclpy.time.Time] = None
        self._gnss_virtual_entry_reached = False
        self._gnss_settle_count = 0
        self._gnss_heading_align_start_time: Optional[rclpy.time.Time] = None
        self._gnss_back_start_time: Optional[rclpy.time.Time] = None
        self._gnss_back_correcting = False
        self._gnss_back_mode: str = "hold"
        self._gnss_pos_source: str = "gnss"
        self._gnss_entry_settle_start_time: Optional[rclpy.time.Time] = None
        self._gnss_leg_locked = False
        self._gnss_leg_start_lat: Optional[float] = None
        self._gnss_leg_start_lon: Optional[float] = None
        self._gnss_locked_stern_yaw: Optional[float] = None
        self._vision_search_start_time: Optional[rclpy.time.Time] = None
        self._last_dock_pose_msg_time: Optional[rclpy.time.Time] = None
        self._back_in_start_time: Optional[rclpy.time.Time] = None
        self._back_in_violation_count = 0
        self._num_retries = 0
        self._wait_charge_start_time: Optional[rclpy.time.Time] = None
        self._charging_reported = False
        self._charging_true_start: Optional[rclpy.time.Time] = None
        self._last_charging_msg_time: Optional[rclpy.time.Time] = None
        self._have_charging_status = False
        self._sensor_hold_active = False
        self._sensor_hold_reason: Optional[str] = None
        self._sensor_hold_start_time: Optional[rclpy.time.Time] = None

        self._pose_filter = PoseEmaFilter(self._pose_filter_alpha)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._pose_transformer = DockPoseTransformer(
            tf_buffer=self._tf_buffer,
            camera_frame=self._camera_frame,
            robot_frame=self._robot_frame,
            tf_timeout_sec=self._tf_timeout_sec,
            allow_camera_frame_fallback=self._allow_camera_frame_fallback,
            invert_x=self._invert_x,
            invert_y=self._invert_y,
            invert_yaw=self._invert_yaw,
        )

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)

        self.create_subscription(
            Float64MultiArray, self._dock_pose_topic, self._dock_pose_cb, 10
        )
        self.create_subscription(Bool, self._start_topic, self._start_cb, 10)
        self.create_subscription(Bool, self._undock_topic, self._undock_cb, 10)
        self.create_subscription(Empty, self._cancel_topic, self._cancel_cb, 10)
        self.create_subscription(
            String, self._mission_state_topic, self._mission_state_cb, 10
        )
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 10)
        self.create_subscription(NavSatFix, self._gnss_topic, self._gnss_cb, 10)
        if self._require_charging_confirm:
            self.create_subscription(
                Bool,
                self._charging_status_topic,
                self._charging_status_cb,
                10,
            )

        period = 1.0 / max(self._control_rate, 1.0)
        self.create_timer(period, self._control_timer_cb)

        self._init_gnss_bay()

        self.get_logger().info(
            f"usv_docking v5 GNSS ready gnss_approach={self._gnss_approach_enabled} "
            f"topic={self._gnss_topic} rate={self._control_rate}Hz"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "dock_pose_topic": "/apriltag_node/dock_pose",
            "cmd_vel_topic": "/cmd_vel_nav",
            "start_topic": "/dock/start",
            "undock_topic": "/dock/undock",
            "cancel_topic": "/dock/cancel",
            "status_topic": "/dock/status",
            "mission_state_topic": "/mission_bridge/state",
            "odom_topic": "/odometry/filtered",
            "gnss_topic": "/gps/fixed_cov",
            "gnss_min_fix_status": 0,
            "gnss_timeout_sec": 2.0,
            "gnss_startup_grace_sec": 3.0,
            "abort_on_gnss_loss": True,
            "camera_frame": "camera_left_link",
            "robot_frame": "base_link",
            "tf_timeout_sec": 0.1,
            "allow_camera_frame_fallback": False,
            "control_rate": 20.0,
            "pose_filter_alpha": 0.35,
            "tag_loss_grace_sec": 0.15,
            "tag_timeout": 0.5,
            "tag_loss_in_corridor_sec": 0.15,
            "max_docking_duration_sec": 240.0,
            "back_in_speed": 0.15,
            "min_back_in_speed": 0.05,
            "approach_entry_speed": 0.12,
            "kx": 0.25,
            "ky_back": 0.20,
            "kyaw_back": 0.40,
            "flip_lateral_yaw_when_reversing": True,
            "back_in_max_yaw_rate": 0.18,
            "back_in_y_limit": 0.50,
            "back_in_yaw_limit": 0.22,
            "back_in_grace_sec": 1.5,
            "back_in_violation_cycles": 10,
            "entry_standoff_m": 2.0,
            "entry_x_tolerance": 0.30,
            "entry_y_limit": 0.55,
            "entry_yaw_tolerance": 0.15,
            "entry_settle_cycles": 5,
            "align_settle_cycles": 5,
            "approach_entry_timeout_sec": 90.0,
            "align_entry_timeout_sec": 45.0,
            "tag_search_on_loss": True,
            "tag_reacquire_cycles": 12,
            "tag_reacquire_min_spin_sec": 2.5,
            # Hold time after tag reappears during recovery search (not before spin starts)
            "approach_tag_loss_hold_sec": 0.6,
            "align_tag_loss_hold_sec": 0.5,
            "in_dock_tag_search_timeout_sec": 120.0,
            "back_in_blind_back_sec": 3.0,
            "back_in_blind_speed": 0.06,
            "yaw_outlier_reject_rad": 1.5,
            "max_reverse_speed": 0.35,
            "max_yaw_rate": 0.35,
            "max_linear_accel": 0.12,
            "max_angular_accel": 0.35,
            "stop_x_threshold": 0.40,
            "stop_y_threshold": 0.25,
            "yaw_tolerance": 0.12,
            "settle_cycles": 5,
            "back_in_backward_projection_m": 0.25,
            "invert_x": False,
            "invert_y": False,
            "invert_yaw": False,
            "heading_offset_rad": 0.0,
            "require_mission_idle": True,
            "allow_unknown_mission_state": False,
            "wait_tag_timeout_sec": 10.0,
            "wait_tag_stationary_sec": 1.5,
            "enable_tag_search_spin": True,
            "tag_search_spin_speed": 0.25,
            "tag_search_spin_angle_rad": 6.283185307179586,
            "tag_search_spin_min_sec": 22.0,
            "tag_search_spin_timeout_sec": 90.0,
            "tag_acquire_cycles": 5,
            "odom_timeout_sec": 1.5,
            "odom_startup_grace_sec": 2.0,
            "dock_pose_msg_timeout_sec": 3.0,
            "abort_on_odom_loss": True,
            "abort_on_dock_pose_stream_loss": True,
            "halt_on_sensor_loss": True,
            "resume_on_sensor_recovery": True,
            "sensor_hold_abort_sec": 30.0,
            "max_retries": 2,
            "auto_retry_recoverable": True,
            "dock_bay_name": "bay2",
            "primary_tag_id": 43,
            "abort_on_mission_emergency": True,
            "publish_zero_when_idle": True,
            "require_charging_confirm": False,
            "charging_status_topic": "/wireless_charging/is_charging",
            "charge_confirm_hold_sec": 4.0,
            "charge_confirm_timeout_sec": 60.0,
            "charge_status_stale_sec": 5.0,
            "charge_pause_on_pose_settle": True,
            "abort_on_charge_pose_drift": False,
            "charge_pose_slack_factor": 1.5,
            "abort_on_charge_status_loss": False,
            "gnss_approach_enabled": True,
            "dock_geometry_path": "",
            "dock_geometry_file": "dock_geometry_sim.yaml",
            "gnss_align_yaw_tol": 0.08,
            "gnss_align_kpsi": 0.35,
            "gnss_align_omega_max": 0.30,
            "gnss_entry_dist_tol": 0.5,
            "gnss_entry_yaw_tol": 0.10,
            "gnss_overshoot_along_tol_m": 0.25,
            "gnss_back_creep_dist_m": 1.5,
            "gnss_settle_cycles": 5,
            "gnss_back_heading_ok_tol": 0.05,
            "gnss_back_heading_correct_tol": 0.12,
            "gnss_back_steer_deadband": 0.04,
            "gnss_back_steer_kpsi": 0.20,
            "gnss_back_walk_omega_max": 0.08,
            "gnss_back_walk_speed_scale": 0.75,
            "gnss_back_heading_check_min_dist_m": 2.0,
            "gnss_back_near_entry_correct_scale": 2.0,
            "gnss_heading_align_timeout_sec": 45.0,
            "gnss_back_timeout_sec": 90.0,
            "gnss_entry_settle_sec": 0.8,
            "vision_search_stationary_sec": 0.5,
            "vision_search_timeout_sec": 90.0,
        }
        defaults.update(UNDOCK_PARAM_DEFAULTS)
        for name, value in defaults.items():
            if isinstance(value, bool):
                self.declare_parameter(name, value)
            elif isinstance(value, int):
                self.declare_parameter(name, value)
            else:
                self.declare_parameter(name, float(value) if isinstance(value, float) else value)

    def _load_parameters(self) -> None:
        g = self.get_parameter
        self._dock_pose_topic = g("dock_pose_topic").value
        self._cmd_vel_topic = g("cmd_vel_topic").value
        self._start_topic = g("start_topic").value
        self._undock_topic = g("undock_topic").value
        self._cancel_topic = g("cancel_topic").value
        self._status_topic = g("status_topic").value
        self._mission_state_topic = g("mission_state_topic").value
        self._odom_topic = g("odom_topic").value
        self._gnss_topic = str(g("gnss_topic").value)
        self._gnss_min_fix_status = int(g("gnss_min_fix_status").value)
        self._gnss_timeout_sec = float(g("gnss_timeout_sec").value)
        self._gnss_startup_grace_sec = float(g("gnss_startup_grace_sec").value)
        self._abort_on_gnss_loss = bool(g("abort_on_gnss_loss").value)
        self._camera_frame = g("camera_frame").value
        self._robot_frame = g("robot_frame").value
        self._tf_timeout_sec = float(g("tf_timeout_sec").value)
        self._allow_camera_frame_fallback = bool(g("allow_camera_frame_fallback").value)
        self._control_rate = float(g("control_rate").value)
        self._pose_filter_alpha = float(g("pose_filter_alpha").value)
        self._tag_loss_grace_sec = float(g("tag_loss_grace_sec").value)
        self._tag_timeout = float(g("tag_timeout").value)
        self._tag_loss_in_corridor_sec = float(g("tag_loss_in_corridor_sec").value)
        self._max_docking_duration_sec = float(g("max_docking_duration_sec").value)
        self._back_in_speed = float(g("back_in_speed").value)
        self._min_back_in_speed = float(g("min_back_in_speed").value)
        self._approach_entry_speed = float(g("approach_entry_speed").value)
        self._kx = float(g("kx").value)
        self._ky_back = float(g("ky_back").value)
        self._kyaw_back = float(g("kyaw_back").value)
        self._flip_lateral_yaw_when_reversing = bool(
            g("flip_lateral_yaw_when_reversing").value
        )
        self._back_in_max_yaw_rate = float(g("back_in_max_yaw_rate").value)
        self._back_in_y_limit = float(g("back_in_y_limit").value)
        self._back_in_yaw_limit = float(g("back_in_yaw_limit").value)
        self._back_in_grace_sec = float(g("back_in_grace_sec").value)
        self._back_in_violation_cycles = int(g("back_in_violation_cycles").value)
        self._entry_standoff_m = float(g("entry_standoff_m").value)
        self._entry_x_tolerance = float(g("entry_x_tolerance").value)
        self._entry_y_limit = float(g("entry_y_limit").value)
        self._entry_yaw_tolerance = float(g("entry_yaw_tolerance").value)
        self._entry_settle_cycles = int(g("entry_settle_cycles").value)
        self._align_settle_cycles = int(g("align_settle_cycles").value)
        self._approach_entry_timeout_sec = float(g("approach_entry_timeout_sec").value)
        self._align_entry_timeout_sec = float(g("align_entry_timeout_sec").value)
        self._tag_search_on_loss = bool(g("tag_search_on_loss").value)
        self._tag_reacquire_cycles = int(g("tag_reacquire_cycles").value)
        self._tag_reacquire_min_spin_sec = float(g("tag_reacquire_min_spin_sec").value)
        self._approach_tag_loss_hold_sec = float(g("approach_tag_loss_hold_sec").value)
        self._align_tag_loss_hold_sec = float(g("align_tag_loss_hold_sec").value)
        self._in_dock_tag_search_timeout_sec = float(
            g("in_dock_tag_search_timeout_sec").value
        )
        self._back_in_blind_back_sec = float(g("back_in_blind_back_sec").value)
        self._back_in_blind_speed = float(g("back_in_blind_speed").value)
        self._yaw_outlier_reject_rad = float(g("yaw_outlier_reject_rad").value)
        self._max_reverse_speed = float(g("max_reverse_speed").value)
        self._max_yaw_rate = float(g("max_yaw_rate").value)
        self._max_linear_accel = float(g("max_linear_accel").value)
        self._max_angular_accel = float(g("max_angular_accel").value)
        self._stop_x_threshold = float(g("stop_x_threshold").value)
        self._stop_y_threshold = float(g("stop_y_threshold").value)
        self._yaw_tolerance = float(g("yaw_tolerance").value)
        self._settle_cycles = int(g("settle_cycles").value)
        self._back_in_backward_projection_m = float(
            g("back_in_backward_projection_m").value
        )
        self._invert_x = bool(g("invert_x").value)
        self._invert_y = bool(g("invert_y").value)
        self._invert_yaw = bool(g("invert_yaw").value)
        self._heading_offset_rad = float(g("heading_offset_rad").value)
        self._require_mission_idle = bool(g("require_mission_idle").value)
        self._allow_unknown_mission_state = bool(
            g("allow_unknown_mission_state").value
        )
        self._wait_tag_timeout_sec = float(g("wait_tag_timeout_sec").value)
        self._wait_tag_stationary_sec = float(g("wait_tag_stationary_sec").value)
        self._enable_tag_search_spin = bool(g("enable_tag_search_spin").value)
        self._tag_search_spin_speed = float(g("tag_search_spin_speed").value)
        self._tag_search_spin_angle_rad = float(g("tag_search_spin_angle_rad").value)
        self._tag_search_spin_min_sec = float(g("tag_search_spin_min_sec").value)
        self._tag_search_spin_timeout_sec = float(
            g("tag_search_spin_timeout_sec").value
        )
        self._tag_acquire_cycles = int(g("tag_acquire_cycles").value)
        self._odom_timeout_sec = float(g("odom_timeout_sec").value)
        self._odom_startup_grace_sec = float(g("odom_startup_grace_sec").value)
        self._dock_pose_msg_timeout_sec = float(g("dock_pose_msg_timeout_sec").value)
        self._abort_on_odom_loss = bool(g("abort_on_odom_loss").value)
        self._abort_on_dock_pose_stream_loss = bool(
            g("abort_on_dock_pose_stream_loss").value
        )
        self._halt_on_sensor_loss = bool(g("halt_on_sensor_loss").value)
        self._resume_on_sensor_recovery = bool(g("resume_on_sensor_recovery").value)
        self._sensor_hold_abort_sec = float(g("sensor_hold_abort_sec").value)
        self._max_retries = int(g("max_retries").value)
        self._auto_retry_recoverable = bool(g("auto_retry_recoverable").value)
        self._dock_bay_name = str(g("dock_bay_name").value)
        self._primary_tag_id = int(g("primary_tag_id").value)
        self._abort_on_mission_emergency = bool(g("abort_on_mission_emergency").value)
        self._publish_zero_when_idle = bool(g("publish_zero_when_idle").value)
        self._require_charging_confirm = bool(g("require_charging_confirm").value)
        self._charging_status_topic = str(g("charging_status_topic").value)
        self._charge_confirm_hold_sec = float(g("charge_confirm_hold_sec").value)
        self._charge_confirm_timeout_sec = float(g("charge_confirm_timeout_sec").value)
        self._charge_status_stale_sec = float(g("charge_status_stale_sec").value)
        self._charge_pause_on_pose_settle = bool(g("charge_pause_on_pose_settle").value)
        self._abort_on_charge_pose_drift = bool(g("abort_on_charge_pose_drift").value)
        self._charge_pose_slack_factor = float(g("charge_pose_slack_factor").value)
        self._abort_on_charge_status_loss = bool(g("abort_on_charge_status_loss").value)
        self._gnss_approach_enabled = bool(g("gnss_approach_enabled").value)
        self._dock_geometry_path = str(g("dock_geometry_path").value).strip()
        self._dock_geometry_file = str(g("dock_geometry_file").value).strip()
        self._gnss_align_yaw_tol = float(g("gnss_align_yaw_tol").value)
        self._gnss_align_kpsi = float(g("gnss_align_kpsi").value)
        self._gnss_align_omega_max = float(g("gnss_align_omega_max").value)
        self._gnss_entry_dist_tol = float(g("gnss_entry_dist_tol").value)
        self._gnss_entry_yaw_tol = float(g("gnss_entry_yaw_tol").value)
        self._gnss_overshoot_along_tol_m = float(g("gnss_overshoot_along_tol_m").value)
        self._gnss_back_creep_dist_m = float(g("gnss_back_creep_dist_m").value)
        self._gnss_settle_cycles = int(g("gnss_settle_cycles").value)
        self._gnss_back_heading_ok_tol = float(g("gnss_back_heading_ok_tol").value)
        self._gnss_back_heading_correct_tol = float(
            g("gnss_back_heading_correct_tol").value
        )
        self._gnss_back_steer_deadband = float(g("gnss_back_steer_deadband").value)
        self._gnss_back_steer_kpsi = float(g("gnss_back_steer_kpsi").value)
        self._gnss_back_walk_omega_max = float(g("gnss_back_walk_omega_max").value)
        self._gnss_back_walk_speed_scale = float(g("gnss_back_walk_speed_scale").value)
        self._gnss_back_heading_check_min_dist_m = float(
            g("gnss_back_heading_check_min_dist_m").value
        )
        self._gnss_back_near_entry_correct_scale = float(
            g("gnss_back_near_entry_correct_scale").value
        )
        self._gnss_heading_align_timeout_sec = float(
            g("gnss_heading_align_timeout_sec").value
        )
        self._gnss_back_timeout_sec = float(g("gnss_back_timeout_sec").value)
        self._gnss_entry_settle_sec = float(g("gnss_entry_settle_sec").value)
        self._vision_search_stationary_sec = float(
            g("vision_search_stationary_sec").value
        )
        self._vision_search_timeout_sec = float(g("vision_search_timeout_sec").value)

    def _odom_reading(self) -> OdomReading:
        return OdomReading(
            have=self._have_odom,
            x=self._odom_x,
            y=self._odom_y,
            yaw=self._odom_yaw,
            age_sec=self._odom_age_sec(),
        )

    def _is_undock_active(self) -> bool:
        return UndockController.is_active_state(self._state.value)

    def _dock_state_from_undock(self, state: UndockState) -> DockState:
        return _UNDOCK_TO_DOCK[state]

    def _init_gnss_bay(self) -> None:
        if not self._gnss_approach_enabled:
            return
        try:
            geo_path = self._dock_geometry_path or None
            self._gnss_bay = load_gnss_bay_geometry(
                self._dock_bay_name,
                geometry_path=geo_path,
                geometry_file=self._dock_geometry_file or "dock_geometry_sim.yaml",
            )
            c = self._gnss_bay.dock_center
            v = self._gnss_bay.virtual_entry
            self.get_logger().info(
                f"GNSS bay {self._gnss_bay.bay_id}: center=({c.latitude:.8f},{c.longitude:.8f}) "
                f"virtual=({v.latitude:.8f},{v.longitude:.8f}) "
                f"standoff={self._gnss_bay.standoff_m:.2f}m pos=gnss_latlon"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"GNSS bay load failed: {exc}")
            self._gnss_approach_enabled = False

    def _gnss_fix_ok(self) -> bool:
        if not self._have_gnss or self._gnss_lat is None or self._gnss_lon is None:
            return False
        return int(self._gnss_status) >= int(self._gnss_min_fix_status)

    def _gnss_age_sec(self) -> Optional[float]:
        if self._last_gnss_time is None:
            return None
        return (self.get_clock().now() - self._last_gnss_time).nanoseconds * 1e-9

    def _gnss_is_stale(self) -> bool:
        if not self._abort_on_gnss_loss or not self._gnss_phase_active():
            return False
        if self._session_elapsed_sec() < self._gnss_startup_grace_sec:
            return False
        age = self._gnss_age_sec()
        if age is None:
            return True
        return age > self._gnss_timeout_sec or not self._gnss_fix_ok()

    def _lock_gnss_approach_leg(self) -> None:
        if (
            self._gnss_bay is None
            or self._gnss_lat is None
            or self._gnss_lon is None
        ):
            return
        entry = self._gnss_bay.virtual_entry
        self._gnss_leg_start_lat = float(self._gnss_lat)
        self._gnss_leg_start_lon = float(self._gnss_lon)
        bearing = bearing_to_gnss_target(
            self._gnss_leg_start_lat,
            self._gnss_leg_start_lon,
            entry.latitude,
            entry.longitude,
        )
        self._gnss_locked_stern_yaw = stern_toward_yaw(bearing)
        self._gnss_leg_locked = True
        self.get_logger().info(
            f"GNSS leg locked: start=({self._gnss_leg_start_lat:.8f},"
            f"{self._gnss_leg_start_lon:.8f}) stern_yaw="
            f"{self._gnss_locked_stern_yaw:.3f} rad",
            throttle_duration_sec=2.0,
        )

    def _gnss_errors(self):
        if (
            self._gnss_bay is None
            or not self._gnss_fix_ok()
            or self._gnss_lat is None
            or self._gnss_lon is None
            or self._odom_yaw is None
        ):
            return None

        entry = self._gnss_bay.virtual_entry
        self._gnss_pos_source = "gnss"
        if (
            self._gnss_leg_locked
            and self._gnss_leg_start_lat is not None
            and self._gnss_leg_start_lon is not None
            and self._gnss_locked_stern_yaw is not None
        ):
            return compute_leg_errors(
                self._gnss_lat,
                self._gnss_lon,
                self._odom_yaw,
                entry.latitude,
                entry.longitude,
                self._gnss_leg_start_lat,
                self._gnss_leg_start_lon,
                self._gnss_locked_stern_yaw,
            )
        return compute_waypoint_errors(
            self._gnss_lat,
            self._gnss_lon,
            self._odom_yaw,
            entry.latitude,
            entry.longitude,
        )

    def _gnss_heading_align_omega(self, deyaw: float) -> float:
        return heading_align_omega(deyaw, self._gnss_align_kpsi, self._gnss_align_omega_max)

    def _gnss_at_virtual_entry(self, err) -> bool:
        return at_virtual_entry(
            err,
            self._gnss_entry_dist_tol,
            self._gnss_entry_yaw_tol,
            overshoot_remaining_tol_m=self._gnss_overshoot_along_tol_m,
        )

    def _gnss_back_speed(self, err) -> float:
        return gnss_back_creep_speed(
            err.dist_m,
            err.leg_remaining_m,
            self._kx,
            self._min_back_in_speed,
            self._approach_entry_speed,
            self._gnss_back_creep_dist_m,
            self._gnss_overshoot_along_tol_m,
        )

    def _gnss_back_control(self, err):
        speed = self._gnss_back_speed(err)
        near_entry = err.dist_m < self._gnss_back_heading_check_min_dist_m
        correct_tol = self._gnss_back_heading_correct_tol
        if near_entry:
            correct_tol *= self._gnss_back_near_entry_correct_scale
        return gnss_back_motion_cmd(
            err.deyaw,
            speed,
            steer_deadband=self._gnss_back_steer_deadband,
            ok_tol=self._gnss_back_heading_ok_tol,
            correct_tol=correct_tol,
            steer_kpsi=self._gnss_back_steer_kpsi,
            walk_omega_max=self._gnss_back_walk_omega_max,
            align_kpsi=self._gnss_align_kpsi,
            align_omega_max=self._gnss_align_omega_max,
            walk_speed_scale=self._gnss_back_walk_speed_scale,
        )

    def _gnss_phase_active(self) -> bool:
        return self._state in (
            DockState.GNSS_HEADING_ALIGN,
            DockState.GNSS_BACK_TO_ENTRY,
            DockState.GNSS_ENTRY_SETTLE,
        )

    def _vision_acquire_phase(self) -> bool:
        return self._state in (
            DockState.VISION_SEARCH_TAG,
            DockState.SEARCH_SPIN,
        )

    def _charging_status_cb(self, msg: Bool) -> None:
        self._have_charging_status = True
        self._last_charging_msg_time = self.get_clock().now()
        charging = bool(msg.data)
        if charging and not self._charging_reported:
            self._charging_true_start = self.get_clock().now()
        elif not charging:
            self._charging_true_start = None
        self._charging_reported = charging

    def _dock_pose_cb(self, msg: Float64MultiArray) -> None:
        self._last_dock_pose_msg_time = self.get_clock().now()
        if not msg.data:
            return
        self._last_raw_pose = list(msg.data)
        self._last_tag_time = self.get_clock().now()
        self._update_pose_estimate()

    def _start_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._is_undock_active():
            self.get_logger().warn(
                f"Ignore /dock/start during undock ({self._state.value})"
            )
            return
        if self._state not in (DockState.IDLE, DockState.STOP, DockState.ABORT):
            self.get_logger().warn(f"Ignore /dock/start in state {self._state.value}")
            return
        self._reset_session()
        self._transition(DockState.PRECHECK)
        self.get_logger().info("Docking started → DOCK_PRECHECK")

    def _undock_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not UndockController.can_start_from(self._state.value):
            self.get_logger().warn(
                f"Ignore /dock/undock in state {self._state.value}"
            )
            return
        if (
            self._undock.config.require_mission_idle
            and not self._mission_allows_docking()
        ):
            self.get_logger().warn("Undock rejected: mission not idle")
            self._abort("MISSION_NOT_IDLE", needs_reapproach=False)
            return
        self._docking_start_time = self.get_clock().now()
        self._undock.begin(self._odom_reading(), time.monotonic())
        self._transition(DockState.UNDOCK_OUT, "undock started")
        self.get_logger().info("Undock started → DOCK_UNDOCK_OUT")

    def _cancel_cb(self, _msg: Empty) -> None:
        if self._state == DockState.IDLE:
            return
        self.get_logger().info("Docking cancelled → DOCK_IDLE")
        self._publish_zero_velocity()
        self._transition(DockState.IDLE)

    def _mission_state_cb(self, msg: String) -> None:
        self._mission_state = msg.data.strip()

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom_x = float(msg.pose.pose.position.x)
        self._odom_y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._odom_yaw = _wrap_yaw(yaw)
        self._have_odom = True
        self._last_odom_time = self.get_clock().now()

    def _gnss_cb(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self._gnss_lat = float(msg.latitude)
        self._gnss_lon = float(msg.longitude)
        self._gnss_status = int(msg.status.status)
        self._have_gnss = True
        self._last_gnss_time = self.get_clock().now()

    def _is_active_docking_state(self) -> bool:
        return self._state not in (
            DockState.IDLE,
            DockState.STOP,
            DockState.UNDOCK_STOP,
            DockState.ABORT,
        )

    def _session_elapsed_sec(self) -> float:
        if self._docking_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._docking_start_time
        ).nanoseconds * 1e-9

    def _odom_age_sec(self) -> Optional[float]:
        if self._last_odom_time is None:
            return None
        return (self.get_clock().now() - self._last_odom_time).nanoseconds * 1e-9

    def _dock_pose_msg_age_sec(self) -> Optional[float]:
        if self._last_dock_pose_msg_time is None:
            return None
        return (
            self.get_clock().now() - self._last_dock_pose_msg_time
        ).nanoseconds * 1e-9

    def _odom_is_stale(self) -> bool:
        if not self._abort_on_odom_loss:
            return False
        if self._session_elapsed_sec() < self._odom_startup_grace_sec:
            return False
        age = self._odom_age_sec()
        if age is None:
            return True
        return age > self._odom_timeout_sec

    def _dock_pose_stream_is_stale(self) -> bool:
        if not self._abort_on_dock_pose_stream_loss:
            return False
        if self._gnss_phase_active() or self._vision_acquire_phase():
            return False
        if self._state in (
            DockState.WAIT_TAG,
            DockState.PRECHECK,
            DockState.WAIT_CHARGE,
            DockState.GNSS_ENTRY_SETTLE,
        ):
            return False
        if self._state == DockState.SEARCH_SPIN:
            return False
        if self._state == DockState.BACK_IN and (
            self._back_in_blind_active or self._back_in_holding_for_tag
        ):
            return False
        if self._state == DockState.ALIGN_ENTRY and self._align_pose_usable_for_control():
            return False
        if self._state == DockState.APPROACH_ENTRY and (
            self._approach_pose_usable_for_control()
            or self._approach_depth_settle_ok()
        ):
            return False
        if self._state in (
            DockState.APPROACH_ENTRY,
            DockState.ALIGN_ENTRY,
            DockState.BACK_IN,
        ):
            age = self._dock_pose_msg_age_sec()
            if age is None:
                return self._tag_was_acquired
            return age > self._dock_pose_msg_timeout_sec
        return False

    def _motion_sensors_ok(self) -> bool:
        if not self._is_active_docking_state():
            return True
        if self._is_undock_active():
            return not self._undock.odom_lost(
                self._odom_reading(), time.monotonic()
            )
        if self._odom_is_stale():
            return False
        if self._gnss_is_stale():
            return False
        if self._dock_pose_stream_is_stale():
            return False
        return True

    def _publish_zero_velocity(self) -> None:
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._publish_cmd_vel()

    def _clear_sensor_hold(self) -> None:
        if not self._sensor_hold_active:
            return
        reason = self._sensor_hold_reason or "SENSOR_LOST"
        elapsed = self._sensor_hold_elapsed_sec()
        self._sensor_hold_active = False
        self._sensor_hold_reason = None
        self._sensor_hold_start_time = None
        self.get_logger().info(
            f"Sensor recovered after {reason}, resume {self._state.value} "
            f"(hold {elapsed:.1f}s)"
        )

    def _enter_sensor_hold(self, reason: str) -> None:
        if self._sensor_hold_active:
            return
        self._sensor_hold_active = True
        self._sensor_hold_reason = reason
        self._sensor_hold_start_time = self.get_clock().now()
        self.get_logger().warning(
            f"Sensor hold: {reason} in {self._state.value} "
            f"(zero velocity, resume when recovered)"
        )

    def _sensor_hold_elapsed_sec(self) -> float:
        if self._sensor_hold_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._sensor_hold_start_time
        ).nanoseconds * 1e-9

    def _sensor_loss_reason(self) -> Optional[str]:
        if self._odom_is_stale():
            return "ODOM_LOST"
        if self._gnss_is_stale():
            return "GNSS_LOST"
        if self._dock_pose_stream_is_stale():
            return "DOCK_POSE_STREAM_LOST"
        return None

    def _check_sensor_health(self) -> bool:
        if not self._halt_on_sensor_loss or not self._is_active_docking_state():
            return True

        reason = self._sensor_loss_reason()
        if reason is None:
            self._clear_sensor_hold()
            return True

        self._publish_zero_velocity()

        if self._resume_on_sensor_recovery:
            if not self._sensor_hold_active:
                self._enter_sensor_hold(reason)
            elif self._sensor_hold_elapsed_sec() >= self._sensor_hold_abort_sec:
                self._abort(reason, needs_reapproach=True)
                return False
            return False

        self._abort(reason, needs_reapproach=True)
        return False

    def _reset_session(self) -> None:
        self._abort_reason = None
        self._needs_reapproach = False
        self._num_retries = 0
        self._settle_count = 0
        self._entry_settle_count = 0
        self._docking_start_time = self.get_clock().now()
        self._wait_tag_start_time = None
        self._search_spin_start_time = None
        self._search_spin_accum_rad = 0.0
        self._search_spin_last_odom_yaw = None
        self._approach_entry_start_time = None
        self._align_entry_start_time = None
        self._align_settle_count = 0
        self._resume_state_after_search = None
        self._search_spin_bias_dir = None
        self._tag_acquire_count = 0
        self._tag_reacquire_count = 0
        self._tag_reacquire_stable_start_time = None
        self._tag_was_acquired = False
        self._align_entry_holding_for_tag = False
        self._back_in_blind_active = False
        self._back_in_blind_start_time = None
        self._back_in_holding_for_tag = False
        self._back_in_last_heading_error = None
        self._back_in_last_y_base = None
        self._back_in_start_time = None
        self._back_in_violation_count = 0
        self._last_tag_time = None
        self._last_raw_pose = None
        self._last_dock_pose_msg_time = None
        self._pose_valid = False
        self._tf_error = None
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._pose_filter.reset()
        self._wait_charge_start_time = None
        self._charging_reported = False
        self._charging_true_start = None
        self._last_charging_msg_time = None
        self._have_charging_status = False
        self._sensor_hold_active = False
        self._sensor_hold_reason = None
        self._sensor_hold_start_time = None
        self._gnss_virtual_entry_reached = False
        self._gnss_settle_count = 0
        self._gnss_heading_align_start_time = None
        self._gnss_back_start_time = None
        self._gnss_back_correcting = False
        self._gnss_back_mode = "hold"
        self._gnss_entry_settle_start_time = None
        self._gnss_leg_locked = False
        self._gnss_leg_start_lat = None
        self._gnss_leg_start_lon = None
        self._gnss_locked_stern_yaw = None
        self._vision_search_start_time = None

    def _transition(self, new_state: DockState, reason: Optional[str] = None) -> None:
        old = self._state
        self._state = new_state
        if new_state == DockState.WAIT_TAG and self._wait_tag_start_time is None:
            self._wait_tag_start_time = self.get_clock().now()
        if new_state == DockState.SEARCH_SPIN and old != DockState.SEARCH_SPIN:
            self._search_spin_start_time = self.get_clock().now()
            self._search_spin_accum_rad = 0.0
            self._search_spin_last_odom_yaw = self._odom_yaw
            if self._resume_state_after_search is not None:
                self._tag_reacquire_count = 0
                self._tag_reacquire_stable_start_time = None
        elif old == DockState.SEARCH_SPIN and new_state != DockState.SEARCH_SPIN:
            self._search_spin_start_time = None
            self._search_spin_accum_rad = 0.0
            self._search_spin_last_odom_yaw = None
            self._search_spin_bias_dir = None  # 修改：退出搜索时重置方向偏置
        if new_state == DockState.APPROACH_ENTRY and old != DockState.APPROACH_ENTRY:
            self._approach_entry_start_time = self.get_clock().now()
            self._entry_settle_count = 0
        elif new_state != DockState.APPROACH_ENTRY:
            self._approach_entry_start_time = None
        if new_state == DockState.ALIGN_ENTRY and old != DockState.ALIGN_ENTRY:
            self._align_entry_start_time = self.get_clock().now()
            self._align_settle_count = 0
            self._align_entry_holding_for_tag = False
        elif new_state != DockState.ALIGN_ENTRY:
            self._align_entry_start_time = None
            self._align_entry_holding_for_tag = False
        if new_state == DockState.BACK_IN and old != DockState.BACK_IN:
            self._back_in_start_time = self.get_clock().now()
            self._back_in_violation_count = 0
            self._settle_count = 0
            self._back_in_blind_active = False
            self._back_in_blind_start_time = None
            self._back_in_holding_for_tag = False
            self._back_in_last_heading_error = None
            self._back_in_last_y_base = None
        elif new_state != DockState.BACK_IN:
            self._back_in_start_time = None
            self._back_in_violation_count = 0
        if new_state == DockState.GNSS_HEADING_ALIGN and old != DockState.GNSS_HEADING_ALIGN:
            self._gnss_heading_align_start_time = self.get_clock().now()
            self._gnss_settle_count = 0
            self._lock_gnss_approach_leg()
        elif new_state != DockState.GNSS_HEADING_ALIGN:
            self._gnss_heading_align_start_time = None
        if new_state == DockState.GNSS_BACK_TO_ENTRY and old != DockState.GNSS_BACK_TO_ENTRY:
            self._gnss_back_start_time = self.get_clock().now()
            self._gnss_settle_count = 0
            self._gnss_back_correcting = False
        elif new_state != DockState.GNSS_BACK_TO_ENTRY:
            self._gnss_back_start_time = None
        if new_state == DockState.GNSS_ENTRY_SETTLE and old != DockState.GNSS_ENTRY_SETTLE:
            self._gnss_entry_settle_start_time = self.get_clock().now()
            self._gnss_settle_count = 0
        elif new_state != DockState.GNSS_ENTRY_SETTLE:
            self._gnss_entry_settle_start_time = None
        if new_state == DockState.VISION_SEARCH_TAG and old != DockState.VISION_SEARCH_TAG:
            self._vision_search_start_time = self.get_clock().now()
            self._tag_acquire_count = 0
        elif new_state != DockState.VISION_SEARCH_TAG:
            self._vision_search_start_time = None
        if new_state == DockState.WAIT_CHARGE and old != DockState.WAIT_CHARGE:
            self._wait_charge_start_time = self.get_clock().now()
            self._charging_true_start = None
        elif new_state != DockState.WAIT_CHARGE:
            self._wait_charge_start_time = None
        if new_state in (DockState.IDLE, DockState.STOP, DockState.UNDOCK_STOP):
            self._publish_zero_velocity()
        if new_state == DockState.ABORT:
            self._publish_zero_velocity()
            if reason:
                self._abort_reason = reason
        if old != new_state:
            self.get_logger().info(
                f"State {old.value} → {new_state.value}"
                + (f" ({reason})" if reason else "")
            )

    def _tag_age_sec(self) -> Optional[float]:
        if self._last_tag_time is None:
            return None
        return (self.get_clock().now() - self._last_tag_time).nanoseconds * 1e-9

    def _fresh_tag_observation(self) -> bool:
        age = self._tag_age_sec()
        return bool(
            self._pose_valid
            and age is not None
            and age <= self._tag_loss_grace_sec
        )

    def _update_tag_acquire_count(self) -> None:
        if self._fresh_tag_observation():
            self._tag_acquire_count += 1
            if self._tag_acquire_count >= self._tag_acquire_cycles:
                self._tag_was_acquired = True
        else:
            self._tag_acquire_count = 0

    def _tag_ready_for_align(self) -> bool:
        return self._tag_acquire_count >= self._tag_acquire_cycles

    def _update_tag_reacquire_count(self) -> None:
        if self._fresh_tag_observation():
            if self._tag_reacquire_count == 0:
                self._tag_reacquire_stable_start_time = self.get_clock().now()
            self._tag_reacquire_count += 1
        else:
            self._tag_reacquire_count = 0
            self._tag_reacquire_stable_start_time = None

    def _tag_reacquire_stable_sec(self) -> float:
        if self._tag_reacquire_stable_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._tag_reacquire_stable_start_time
        ).nanoseconds * 1e-9

    def _tag_ready_to_resume_after_search(self) -> bool:
        if self._tag_reacquire_count < self._tag_reacquire_cycles:
            return False
        return self._tag_reacquire_stable_sec() >= self._tag_reacquire_min_spin_sec

    def _tag_search_spin_cmd(self) -> float:
        # During a recovery search (lost tag mid-docking), keep spinning the
        # same way the boat was already turning when the tag disappeared,
        # instead of a fixed configured direction. A fixed direction can be
        # exactly opposite the correction ALIGN_ENTRY/APPROACH_ENTRY needs,
        # which makes the boat sweep past the tag, briefly reacquire it on
        # the wrong side, and immediately lose it again once control resumes
        # and steers the other way — an endless search/align loop.
        if self._resume_state_after_search is not None and self._search_spin_bias_dir is not None:
            spin_dir = self._search_spin_bias_dir
        else:
            spin_dir = 1.0 if self._tag_search_spin_speed >= 0.0 else -1.0
        w_mag = abs(self._tag_search_spin_speed)
        return spin_dir * _clamp(w_mag, 0.0, self._max_yaw_rate)

    def _tag_lost_for_approach_search(self) -> bool:
        """Debounced tag loss — triggers search spin in APPROACH_ENTRY."""
        age = self._tag_age_sec()
        if age is None:
            return self._tag_was_acquired
        return age > self._approach_tag_loss_hold_sec

    def _tag_lost_for_align_search(self) -> bool:
        """Debounced tag loss — triggers search spin in ALIGN_ENTRY."""
        age = self._tag_age_sec()
        if age is None:
            return self._tag_was_acquired
        return age > self._align_tag_loss_hold_sec

    def _tag_ok_for_control(self) -> bool:
        return self._fresh_tag_observation() and self._pose_valid

    def _approach_pose_usable_for_control(self) -> bool:
        """Fresh tag or brief loss: keep backing / depth settle with filtered pose."""
        if not self._pose_valid:
            return False
        if self._fresh_tag_observation():
            return True
        age = self._tag_age_sec()
        if age is None:
            return False
        return age <= self._approach_tag_loss_hold_sec

    def _align_pose_usable_for_control(self) -> bool:
        """Fresh tag or brief loss: keep in-place align using filtered pose."""
        if not self._pose_valid:
            return False
        if self._fresh_tag_observation():
            return True
        age = self._tag_age_sec()
        if age is None:
            return False
        return age <= self._align_tag_loss_hold_sec

    def _wait_tag_elapsed_sec(self) -> float:
        if self._wait_tag_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._wait_tag_start_time
        ).nanoseconds * 1e-9

    def _approach_entry_elapsed_sec(self) -> float:
        if self._approach_entry_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._approach_entry_start_time
        ).nanoseconds * 1e-9

    def _align_entry_elapsed_sec(self) -> float:
        if self._align_entry_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._align_entry_start_time
        ).nanoseconds * 1e-9

    def _update_search_spin_progress(self, cmd_w: float) -> None:
        if self._have_odom and self._odom_yaw is not None:
            if self._search_spin_last_odom_yaw is None:
                self._search_spin_last_odom_yaw = self._odom_yaw
                return
            delta = _wrap_yaw(self._odom_yaw - self._search_spin_last_odom_yaw)
            self._search_spin_last_odom_yaw = self._odom_yaw
            self._search_spin_accum_rad += abs(delta)
            return

        dt = 1.0 / max(self._control_rate, 1.0)
        self._search_spin_accum_rad += abs(cmd_w) * dt

    def _search_spin_elapsed_sec(self) -> float:
        if self._search_spin_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._search_spin_start_time
        ).nanoseconds * 1e-9

    def _update_pose_estimate(self) -> None:
        if self._last_raw_pose is None:
            self._pose_valid = False
            return

        result, tf_err = self._pose_transformer.transform(self._last_raw_pose, None)
        if result is None:
            self._pose_valid = False
            self._tf_error = tf_err
            return

        self._tf_error = None
        x, y, yaw = result
        if self._pose_valid and abs(_wrap_yaw(yaw - self._yaw_base)) > self._yaw_outlier_reject_rad:
            yaw = self._yaw_base
        x, y, yaw = self._pose_filter.update(x, y, yaw)
        self._x_base = x
        self._y_base = y
        self._yaw_base = yaw
        self._pose_valid = True

    def _heading_error(self) -> float:
        return _wrap_yaw(self._yaw_base - self._heading_offset_rad)

    def _backing_steer_rate(self, heading: float, y: float) -> float:
        """Cross-track + heading while reversing (diff-drive arc correction)."""
        y_gain = (
            -self._ky_back
            if self._flip_lateral_yaw_when_reversing
            else self._ky_back
        )
        return y_gain * y + self._kyaw_back * heading

    def _stationary_steer_rate(self, heading: float, y: float) -> float:
        """In-place rotation to fix lateral + heading before final back-in."""
        return -self._ky_back * y + self._kyaw_back * heading

    def _approach_depth_m(self) -> float:
        """Range to bay center along approach axis (same sign convention as BACK_IN)."""
        return abs(self._x_base)

    def _entry_depth_error(self) -> float:
        return (
            abs(self._approach_depth_m() - self._entry_standoff_m)
            + self._back_in_backward_projection_m
        )

    def _entry_depth_reached(self) -> bool:
        """Virtual entry: |x_base| ≈ standoff, or passed it while backing toward center."""
        depth = self._approach_depth_m()
        if abs(depth - self._entry_standoff_m) < self._entry_x_tolerance:
            return True
        return depth < self._entry_standoff_m - self._entry_x_tolerance

    def _approach_depth_settle_ok(self) -> bool:
        """At virtual entry: count settle on filtered pose, do not require fresh tag."""
        return self._pose_valid and self._entry_depth_reached()

    def _entry_aligned(self) -> bool:
        return (
            abs(self._y_base) < self._entry_y_limit
            and abs(self._heading_error()) < self._entry_yaw_tolerance
        )

    def _entry_settled(self) -> bool:
        return self._entry_depth_reached() and self._entry_aligned()

    def _back_in_corridor_violation(self, heading: float) -> bool:
        return (
            abs(self._y_base) > self._back_in_y_limit
            or abs(heading) > self._back_in_yaw_limit
        )

    def _back_in_grace_elapsed(self) -> bool:
        if self._back_in_start_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self._back_in_start_time
        ).nanoseconds * 1e-9
        return elapsed >= self._back_in_grace_sec

    def _settled(self) -> bool:
        return (
            abs(self._x_base) < self._stop_x_threshold
            and abs(self._y_base) < self._stop_y_threshold
            and abs(self._heading_error()) < self._yaw_tolerance
        )

    def _pose_acceptable_for_charge_wait(self) -> bool:
        if not self._pose_valid:
            return True
        slack = max(self._charge_pose_slack_factor, 1.0)
        return (
            abs(self._x_base) < self._stop_x_threshold * slack
            and abs(self._y_base) < self._stop_y_threshold * slack
            and abs(self._heading_error()) < self._yaw_tolerance * slack
        )

    def _charging_msg_age_sec(self) -> Optional[float]:
        if self._last_charging_msg_time is None:
            return None
        return (
            self.get_clock().now() - self._last_charging_msg_time
        ).nanoseconds * 1e-9

    def _charging_status_fresh(self) -> bool:
        if not self._have_charging_status:
            return False
        age = self._charging_msg_age_sec()
        if age is None:
            return False
        return age <= self._charge_status_stale_sec

    def _charging_active(self) -> bool:
        return self._charging_status_fresh() and self._charging_reported

    def _charging_confirmed(self) -> bool:
        if not self._charging_reported or not self._charging_status_fresh():
            return False
        if self._charging_true_start is None:
            return False
        held = (
            self.get_clock().now() - self._charging_true_start
        ).nanoseconds * 1e-9
        return held >= self._charge_confirm_hold_sec

    def _wait_charge_elapsed_sec(self) -> float:
        if self._wait_charge_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._wait_charge_start_time
        ).nanoseconds * 1e-9

    def _charging_hold_sec(self) -> float:
        if self._charging_true_start is None:
            return 0.0
        return (
            self.get_clock().now() - self._charging_true_start
        ).nanoseconds * 1e-9

    def _rate_limit(self, current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def _apply_limits(
        self, v_target: float, w_target: float, *, forward_only: bool = False
    ) -> tuple[float, float]:
        dt = 1.0 / max(self._control_rate, 1.0)
        v_min = 0.0 if forward_only else -self._max_reverse_speed
        v = _clamp(
            self._rate_limit(self._cmd_v, v_target, self._max_linear_accel * dt),
            v_min,
            self._max_reverse_speed,
        )
        w = _clamp(
            self._rate_limit(self._cmd_w, w_target, self._max_angular_accel * dt),
            -self._max_yaw_rate,
            self._max_yaw_rate,
        )
        return v, w

    def _prepare_tag_search_retry(self, reason: str) -> None:
        self._abort_reason = None
        self._needs_reapproach = False
        self._tag_acquire_count = 0
        self._tag_was_acquired = False
        self._settle_count = 0
        self._entry_settle_count = 0
        self._back_in_violation_count = 0
        self._last_tag_time = None
        self._last_raw_pose = None
        self._pose_valid = False
        self._tf_error = None
        self._wait_tag_start_time = None
        self._search_spin_start_time = None
        self._search_spin_accum_rad = 0.0
        self._search_spin_last_odom_yaw = None
        self._approach_entry_start_time = None
        self._align_entry_start_time = None
        self._align_settle_count = 0
        self._back_in_start_time = None
        self._pose_filter.reset()
        self._publish_zero_velocity()
        self._wait_charge_start_time = None
        self._charging_reported = False
        self._charging_true_start = None
        self._last_charging_msg_time = None
        self._have_charging_status = False
        self._sensor_hold_active = False
        self._sensor_hold_reason = None
        self._sensor_hold_start_time = None
        self._resume_state_after_search = None
        self._search_spin_bias_dir = None
        self._transition(
            DockState.WAIT_TAG,
            f"retry {self._num_retries}/{self._max_retries} after {reason}",
        )

    def _tag_lost_in_docking(self) -> bool:
        age = self._tag_age_sec()
        if age is None:
            return self._tag_was_acquired
        return age > self._tag_loss_in_corridor_sec

    def _start_tag_search_recovery(
        self, resume: Optional[DockState] = None
    ) -> None:
        """Search spin when tag lost in APPROACH_ENTRY or ALIGN_ENTRY only."""
        if self._state == DockState.SEARCH_SPIN:
            return
        if resume is None:
            if self._state in (DockState.APPROACH_ENTRY, DockState.ALIGN_ENTRY):
                resume = self._state
            else:
                return
        if resume not in (DockState.APPROACH_ENTRY, DockState.ALIGN_ENTRY):
            return
        self._resume_state_after_search = resume
        self._tag_acquire_count = 0
        self._tag_reacquire_count = 0
        self._tag_reacquire_stable_start_time = None
        self._align_entry_holding_for_tag = False
        # Bias the recovery spin toward the direction the boat was already
        # turning right before the tag disappeared (last commanded w), so
        # the search sweeps back toward where the tag was last seen instead
        # of a fixed configured direction that may point the wrong way.
        if abs(self._cmd_w) > 1e-4:
            self._search_spin_bias_dir = 1.0 if self._cmd_w >= 0.0 else -1.0
        else:
            self._search_spin_bias_dir = None
        self._publish_zero_velocity()
        self.get_logger().warning(
            f"Tag lost in {resume.value}, search spin now; "
            f"resume after tag returns + {self._tag_reacquire_cycles} stable frames "
            f"+ {self._tag_reacquire_min_spin_sec:.1f}s hold"
        )
        self._transition(DockState.SEARCH_SPIN, "tag lost, searching")

    def _back_in_blind_elapsed_sec(self) -> float:
        if self._back_in_blind_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._back_in_blind_start_time
        ).nanoseconds * 1e-9

    def _begin_back_in_blind(self) -> None:
        if self._back_in_blind_active or self._back_in_holding_for_tag:
            return
        self._back_in_blind_active = True
        self._back_in_blind_start_time = self.get_clock().now()
        self.get_logger().info(
            f"BACK_IN tag lost: blind reverse {self._back_in_blind_back_sec:.1f}s "
            f"at {self._back_in_blind_speed:.2f} m/s then hold"
        )

    def _abort(self, reason: str, needs_reapproach: bool = False) -> None:
        if (
            self._auto_retry_recoverable
            and is_auto_retry_reason(reason)
            and self._num_retries < self._max_retries
        ):
            self._num_retries += 1
            self.get_logger().warning(
                f"Docking retry {self._num_retries}/{self._max_retries} "
                f"after {reason} (resetApproach at pre-dock)"
            )
            self._prepare_tag_search_retry(reason)
            return

        self._needs_reapproach = needs_reapproach
        self._publish_zero_velocity()
        self.get_logger().error(
            f"Docking ABORT: {reason} "
            f"(state={self._state.value}, "
            f"retries={self._num_retries}/{self._max_retries}, "
            f"spin_accum={self._search_spin_accum_rad:.2f} rad, "
            f"spin_elapsed={self._search_spin_elapsed_sec():.1f}s)"
        )
        self._transition(DockState.ABORT, reason)

    def _back_in_approach_depth(self) -> float:
        return abs(self._x_base) + self._back_in_backward_projection_m

    def _gnss_heading_align_elapsed_sec(self) -> float:
        if self._gnss_heading_align_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._gnss_heading_align_start_time
        ).nanoseconds * 1e-9

    def _gnss_back_elapsed_sec(self) -> float:
        if self._gnss_back_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._gnss_back_start_time
        ).nanoseconds * 1e-9

    def _gnss_entry_settle_elapsed_sec(self) -> float:
        if self._gnss_entry_settle_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._gnss_entry_settle_start_time
        ).nanoseconds * 1e-9

    def _vision_search_elapsed_sec(self) -> float:
        if self._vision_search_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self._vision_search_start_time
        ).nanoseconds * 1e-9

    def _mission_allows_docking(self) -> bool:
        if not self._require_mission_idle:
            return True
        if self._mission_state == "IDLE":
            return True
        if self._allow_unknown_mission_state and self._mission_state == "UNKNOWN":
            return True
        return False

    def _check_global_timeouts(self) -> bool:
        if self._is_undock_active():
            return False
        if self._docking_start_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._docking_start_time
        ).nanoseconds * 1e-9
        if elapsed > self._max_docking_duration_sec:
            self._abort("MAX_DOCKING_DURATION", needs_reapproach=True)
            return True
        if self._state == DockState.WAIT_TAG:
            if self._tag_ready_for_align():
                return False
            wait_elapsed = self._wait_tag_elapsed_sec()
            if (
                not self._enable_tag_search_spin
                and wait_elapsed > self._wait_tag_timeout_sec
            ):
                self._abort("WAIT_TAG_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.WAIT_CHARGE:
            if self._wait_charge_elapsed_sec() > self._charge_confirm_timeout_sec:
                self._abort("CHARGE_CONFIRM_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.SEARCH_SPIN:
            recovering = self._resume_state_after_search is not None
            spin_elapsed = self._search_spin_elapsed_sec()
            spin_timeout = (
                self._in_dock_tag_search_timeout_sec
                if recovering
                else self._tag_search_spin_timeout_sec
            )
            if spin_elapsed > spin_timeout:
                reason = (
                    "IN_DOCK_TAG_SEARCH_TIMEOUT"
                    if recovering
                    else "TAG_SEARCH_TIMEOUT"
                )
                self._abort(reason, needs_reapproach=True)
                return True
            if (
                not recovering
                and spin_elapsed >= self._tag_search_spin_min_sec
                and self._search_spin_accum_rad >= self._tag_search_spin_angle_rad
            ):
                self._abort("TAG_SEARCH_NO_TAG", needs_reapproach=True)
                return True
        if self._state == DockState.APPROACH_ENTRY:
            if self._approach_entry_elapsed_sec() > self._approach_entry_timeout_sec:
                self.get_logger().error(
                    f"APPROACH_ENTRY timeout: x_base={self._x_base:.2f} "
                    f"target={self._entry_standoff_m:.2f} "
                    f"y={self._y_base:.2f} heading={self._heading_error():.2f}"
                )
                self._abort("APPROACH_ENTRY_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.ALIGN_ENTRY:
            if self._align_entry_elapsed_sec() > self._align_entry_timeout_sec:
                self._abort("ALIGN_ENTRY_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.GNSS_HEADING_ALIGN:
            if self._gnss_heading_align_elapsed_sec() > self._gnss_heading_align_timeout_sec:
                self._abort("GNSS_HEADING_ALIGN_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.GNSS_BACK_TO_ENTRY:
            if self._gnss_back_elapsed_sec() > self._gnss_back_timeout_sec:
                self._abort("GNSS_BACK_TIMEOUT", needs_reapproach=True)
                return True
        if self._state == DockState.VISION_SEARCH_TAG:
            if self._vision_search_elapsed_sec() > self._vision_search_timeout_sec:
                self._abort("TAG_SEARCH_TIMEOUT", needs_reapproach=True)
                return True
        return False

    def _check_tag_health(self) -> bool:
        if self._is_undock_active():
            return True
        if self._gnss_phase_active() or self._state in (
            DockState.PRECHECK,
            DockState.WAIT_TAG,
            DockState.SEARCH_SPIN,
            DockState.VISION_SEARCH_TAG,
            DockState.GNSS_ENTRY_SETTLE,
            DockState.WAIT_CHARGE,
            DockState.BACK_IN,
        ):
            return True

        if self._state == DockState.APPROACH_ENTRY:
            if self._tag_lost_for_approach_search():
                if self._tag_search_on_loss and self._enable_tag_search_spin:
                    self._start_tag_search_recovery(DockState.APPROACH_ENTRY)
                    return True
            return True

        if self._state == DockState.ALIGN_ENTRY:
            if self._tag_lost_for_align_search():
                if self._tag_search_on_loss and self._enable_tag_search_spin:
                    self._start_tag_search_recovery(DockState.ALIGN_ENTRY)
                    return True
            return True

        age = self._tag_age_sec()
        if age is None:
            return True
        if age > self._tag_timeout:
            self._abort("TAG_TIMEOUT", needs_reapproach=True)
            return False
        return True

    def _control_undock_cycle(self) -> None:
        now_mono = time.monotonic()
        odom = self._odom_reading()

        timeout = self._undock.check_timeout(now_mono)
        if timeout is not None:
            self._publish_zero_velocity()
            self._abort(timeout, needs_reapproach=False)
            self._publish_status()
            return

        if self._undock.odom_lost(odom, now_mono):
            self._publish_zero_velocity()
            self._abort("UNDOCK_ODOM_LOST", needs_reapproach=False)
            self._publish_status()
            return

        result = self._undock.step(UndockState(self._state.value), odom)
        if result.next_state is not None:
            self._transition(
                self._dock_state_from_undock(result.next_state),
                result.transition_reason,
            )

        if self._state == DockState.UNDOCK_STOP:
            self._publish_zero_velocity()
        elif self._state != DockState.ABORT:
            self._cmd_v, self._cmd_w = self._apply_limits(
                result.v_target, result.w_target, forward_only=True
            )
            self._publish_cmd_vel()

        self._publish_status()

    def _control_timer_cb(self) -> None:
        if self._state == DockState.IDLE:
            if self._publish_zero_when_idle:
                self._publish_zero_velocity()
            self._publish_status()
            return

        if (
            self._abort_on_mission_emergency
            and self._mission_state == "EMERGENCY"
            and self._state not in (
                DockState.ABORT,
                DockState.STOP,
                DockState.UNDOCK_STOP,
            )
        ):
            self._publish_zero_velocity()
            self._abort("MISSION_EMERGENCY", needs_reapproach=True)
            self._publish_status()
            return

        if self._is_undock_active():
            self._control_undock_cycle()
            return

        if not self._check_sensor_health():
            self._publish_status()
            return

        if self._last_raw_pose is not None:
            self._update_pose_estimate()

        self._update_tag_acquire_count()

        if self._check_global_timeouts():
            self._publish_status()
            return

        if not self._check_tag_health():
            self._publish_status()
            return

        v_target = 0.0
        w_target = 0.0

        if self._state == DockState.PRECHECK:
            if not self._mission_allows_docking():
                self._abort("MISSION_NOT_IDLE", needs_reapproach=False)
            elif not self._have_odom:
                v_target, w_target = 0.0, 0.0
            elif self._gnss_approach_enabled and self._gnss_bay is not None:
                if not self._gnss_fix_ok() or self._odom_yaw is None:
                    v_target, w_target = 0.0, 0.0
                else:
                    self._transition(
                        DockState.GNSS_HEADING_ALIGN,
                        "GNSS phase: align stern toward virtual entry",
                    )
            else:
                self._transition(DockState.WAIT_TAG)

        elif self._state == DockState.GNSS_HEADING_ALIGN:
            err = self._gnss_errors()
            if err is None:
                v_target, w_target = 0.0, 0.0
            elif heading_aligned(err, self._gnss_align_yaw_tol):
                self._gnss_settle_count += 1
                v_target, w_target = 0.0, 0.0
                if self._gnss_settle_count >= self._gnss_settle_cycles:
                    self._transition(
                        DockState.GNSS_BACK_TO_ENTRY,
                        "GNSS heading aligned",
                    )
            else:
                self._gnss_settle_count = 0
                v_target = 0.0
                w_target = self._gnss_heading_align_omega(err.deyaw)

        elif self._state == DockState.GNSS_BACK_TO_ENTRY:
            err = self._gnss_errors()
            if err is None:
                v_target, w_target = 0.0, 0.0
            elif self._gnss_at_virtual_entry(err):
                self._gnss_settle_count += 1
                v_target, w_target = 0.0, 0.0
                self._gnss_back_correcting = False
                self._gnss_back_mode = "hold"
                if self._gnss_settle_count >= self._gnss_settle_cycles:
                    self._gnss_virtual_entry_reached = True
                    self._transition(
                        DockState.GNSS_ENTRY_SETTLE,
                        f"GNSS at virtual entry dist={err.dist_m:.2f}m "
                        f"deyaw={err.deyaw:.2f} leg_rem={err.leg_remaining_m:.2f}",
                    )
            else:
                self._gnss_settle_count = 0
                v_target, w_target, mode = self._gnss_back_control(err)
                self._gnss_back_mode = mode
                self._gnss_back_correcting = mode == "correct"

        elif self._state == DockState.GNSS_ENTRY_SETTLE:
            v_target, w_target = 0.0, 0.0
            if self._gnss_entry_settle_elapsed_sec() >= self._gnss_entry_settle_sec:
                self._transition(
                    DockState.VISION_SEARCH_TAG,
                    "virtual entry settled, acquire tag",
                )

        elif self._state == DockState.VISION_SEARCH_TAG:
            v_target, w_target = 0.0, 0.0
            if self._tag_ready_for_align():
                self._transition(
                    DockState.ALIGN_ENTRY,
                    "tag stable at virtual entry",
                )
            elif (
                self._enable_tag_search_spin
                and self._vision_search_elapsed_sec() >= self._vision_search_stationary_sec
            ):
                self._resume_state_after_search = DockState.ALIGN_ENTRY
                self._transition(
                    DockState.SEARCH_SPIN,
                    "no tag at virtual entry, search spin",
                )

        elif self._state == DockState.WAIT_TAG:
            if self._tag_ready_for_align():
                self._transition(
                    DockState.APPROACH_ENTRY,
                    "tag ready, first docking to virtual entry",
                )
            elif (
                self._enable_tag_search_spin
                and self._wait_tag_elapsed_sec() >= self._wait_tag_stationary_sec
            ):
                self._transition(
                    DockState.SEARCH_SPIN,
                    "no tag at pre-dock, starting search spin",
                )

        elif self._state == DockState.SEARCH_SPIN:
            if (
                self._resume_state_after_search is not None
                and self._tag_ready_to_resume_after_search()
            ):
                resume = self._resume_state_after_search
                self._resume_state_after_search = None
                self._tag_reacquire_count = 0
                self._tag_reacquire_stable_start_time = None
                self._transition(
                    resume,
                    f"tag stable ({self._tag_reacquire_cycles} frames), "
                    "resume docking",
                )
            elif (
                self._resume_state_after_search is None
                and self._tag_ready_for_align()
            ):
                next_state = (
                    DockState.ALIGN_ENTRY
                    if self._gnss_approach_enabled
                    else DockState.APPROACH_ENTRY
                )
                self._transition(
                    next_state,
                    "tag found after search",
                )
            else:
                if self._resume_state_after_search is not None:
                    self._update_tag_reacquire_count()
                    if self._fresh_tag_observation():
                        # 修改：减慢旋转而不是完全停止
                        # 这样可以让tag在视野中稳定，同时减少姿态变化
                        spin_cmd = self._tag_search_spin_cmd()
                        w_target = spin_cmd * 0.2  # 减速到20%
                    else:
                        w_target = self._tag_search_spin_cmd()
                else:
                    w_target = self._tag_search_spin_cmd()

        elif self._state == DockState.APPROACH_ENTRY:
            pose_ok = self._approach_pose_usable_for_control()
            if self._approach_depth_settle_ok():
                self._entry_settle_count += 1
                v_target, w_target = 0.0, 0.0
                if self._entry_settle_count >= self._entry_settle_cycles:
                    self._transition(
                        DockState.ALIGN_ENTRY,
                        f"virtual entry depth {self._entry_standoff_m}m reached "
                        f"(x_base={self._x_base:.2f})",
                    )
            elif pose_ok:
                self._entry_settle_count = 0
                heading = self._heading_error()
                speed = _clamp(
                    self._kx * self._entry_depth_error(),
                    self._min_back_in_speed,
                    self._approach_entry_speed,
                )
                v_target = -speed
                w_target = _clamp(
                    self._backing_steer_rate(heading, self._y_base),
                    -self._back_in_max_yaw_rate,
                    self._back_in_max_yaw_rate,
                )
            else:
                v_target, w_target = 0.0, 0.0
                if not self._approach_depth_settle_ok():
                    self._entry_settle_count = 0

        elif self._state == DockState.ALIGN_ENTRY:
            if self._align_pose_usable_for_control():
                self._align_entry_holding_for_tag = False
                if self._entry_aligned():
                    self._align_settle_count += 1
                    v_target, w_target = 0.0, 0.0
                    if self._align_settle_count >= self._align_settle_cycles:
                        self._transition(
                            DockState.BACK_IN,
                            "aligned at virtual entry, back to dock center",
                        )
                else:
                    self._align_settle_count = 0
                    v_target = 0.0
                    w_target = _clamp(
                        self._stationary_steer_rate(
                            self._heading_error(), self._y_base
                        ),
                        -self._max_yaw_rate,
                        self._max_yaw_rate,
                    )
            else:
                self._align_entry_holding_for_tag = True
                v_target, w_target = 0.0, 0.0

        elif self._state == DockState.BACK_IN:
            if self._tag_ok_for_control():
                self._back_in_blind_active = False
                self._back_in_blind_start_time = None
                self._back_in_holding_for_tag = False
                heading = self._heading_error()
                self._back_in_last_heading_error = heading
                self._back_in_last_y_base = self._y_base
                if self._back_in_corridor_violation(heading):
                    self._back_in_violation_count += 1
                else:
                    self._back_in_violation_count = 0

                if (
                    self._back_in_grace_elapsed()
                    and self._back_in_violation_count >= self._back_in_violation_cycles
                ):
                    self._abort(
                        "CORRIDOR_VIOLATION_IN_BACK_IN", needs_reapproach=True
                    )
                elif self._require_charging_confirm and self._charging_active():
                    v_target, w_target = 0.0, 0.0
                    self._transition(
                        DockState.WAIT_CHARGE,
                        "charging detected, pausing",
                    )
                elif self._settled():
                    self._settle_count += 1
                    v_target, w_target = 0.0, 0.0
                    if self._settle_count >= self._settle_cycles:
                        if self._require_charging_confirm:
                            if self._charge_pause_on_pose_settle:
                                self._transition(
                                    DockState.WAIT_CHARGE,
                                    "pose settled, pausing for charge contact",
                                )
                        else:
                            self._transition(DockState.STOP, "pose settled")
                else:
                    self._settle_count = 0
                    speed = _clamp(
                        self._kx * self._back_in_approach_depth(),
                        self._min_back_in_speed,
                        self._back_in_speed,
                    )
                    v_target = -speed
                    w_target = _clamp(
                        self._backing_steer_rate(heading, self._y_base),
                        -self._back_in_max_yaw_rate,
                        self._back_in_max_yaw_rate,
                    )
            else:
                self._begin_back_in_blind()
                if (
                    self._back_in_blind_active
                    and self._back_in_blind_elapsed_sec() < self._back_in_blind_back_sec
                ):
                    v_target = -self._back_in_blind_speed
                    w_target = 0.0
                else:
                    self._back_in_blind_active = False
                    self._back_in_holding_for_tag = True
                    v_target, w_target = 0.0, 0.0

        elif self._state == DockState.WAIT_CHARGE:
            v_target, w_target = 0.0, 0.0
            if (
                self._abort_on_charge_pose_drift
                and not self._pose_acceptable_for_charge_wait()
            ):
                self._abort("CHARGE_POSE_LOST", needs_reapproach=True)
            elif (
                self._abort_on_charge_status_loss
                and self._charging_reported
                and not self._charging_status_fresh()
            ):
                self._abort("CHARGE_STATUS_LOST", needs_reapproach=True)
            elif self._charging_confirmed():
                self._transition(
                    DockState.STOP,
                    "wireless charging stable, docking complete",
                )

        elif self._state in (DockState.STOP, DockState.ABORT):
            v_target, w_target = 0.0, 0.0

        if self._state not in (DockState.ABORT, DockState.STOP, DockState.IDLE):
            if not self._motion_sensors_ok():
                self._publish_zero_velocity()
            elif self._state == DockState.WAIT_CHARGE:
                self._publish_zero_velocity()
            else:
                self._cmd_v, self._cmd_w = self._apply_limits(v_target, w_target)
                if self._state == DockState.SEARCH_SPIN:
                    self._update_search_spin_progress(self._cmd_w)
                self._publish_cmd_vel()
        elif self._state in (DockState.STOP, DockState.ABORT):
            self._publish_zero_velocity()

        self._publish_status()

    def _publish_cmd_vel(self) -> None:
        msg = Twist()
        msg.linear.x = float(self._cmd_v)
        msg.angular.z = float(self._cmd_w)
        self._cmd_pub.publish(msg)

    def _docking_stage_label(self) -> Optional[str]:
        if self._state in (
            DockState.GNSS_HEADING_ALIGN,
            DockState.GNSS_BACK_TO_ENTRY,
            DockState.GNSS_ENTRY_SETTLE,
        ):
            return "gnss_to_virtual_entry"
        if self._state in (DockState.VISION_SEARCH_TAG, DockState.SEARCH_SPIN):
            return "vision_acquire"
        if self._state == DockState.APPROACH_ENTRY:
            return "first_dock_approach"
        if self._state == DockState.ALIGN_ENTRY:
            return "vision_align"
        if self._state == DockState.BACK_IN:
            return "second_dock"
        return None

    def _publish_status(self) -> None:
        phase = phase_from_state(self._state.value)
        err_code = error_code_from_reason(self._abort_reason)
        approach_depth_m = round(float(self._approach_depth_m()), 4)
        gnss_err = self._gnss_errors()
        odom = self._odom_reading()
        payload = {
            "state": self._state.value,
            "phase": phase,
            "docking_stage": self._docking_stage_label(),
            "gnss_approach_enabled": bool(self._gnss_approach_enabled),
            "gnss_at_virtual_entry": bool(self._gnss_virtual_entry_reached),
            "success": self._state == DockState.STOP,
            "error_code": int(err_code),
            "abort_reason": self._abort_reason,
            "needs_reapproach": bool(self._needs_reapproach),
            "docking_time_sec": round(float(self._session_elapsed_sec()), 2)
            if self._docking_start_time is not None
            else None,
            "x_base": round(float(self._x_base), 4) if self._pose_valid else None,
            "y_base": round(float(self._y_base), 4) if self._pose_valid else None,
            "heading_error": round(float(self._heading_error()), 4)
            if self._pose_valid
            else None,
            "gnss_lat": round(float(self._gnss_lat), 8) if self._gnss_lat is not None else None,
            "gnss_lon": round(float(self._gnss_lon), 8) if self._gnss_lon is not None else None,
            "gnss_fix_status": int(self._gnss_status),
            "gnss_fix_ok": bool(self._gnss_fix_ok()),
            "dist_to_virtual_entry": round(float(gnss_err.dist_m), 4)
            if gnss_err
            else None,
            "gnss_bearing_rad": round(float(gnss_err.bearing_rad), 4)
            if gnss_err
            else None,
            "gnss_cross_track_m": round(float(gnss_err.cross_track_m), 4)
            if gnss_err
            else None,
            "gnss_along_track_m": round(float(gnss_err.along_track_m), 4)
            if gnss_err
            else None,
            "gnss_yaw_rad": round(float(gnss_err.yaw_rad), 4) if gnss_err else None,
            "gnss_desired_yaw_rad": round(float(gnss_err.desired_yaw_rad), 4)
            if gnss_err
            else None,
            "gnss_deyaw": round(float(gnss_err.deyaw), 4) if gnss_err else None,
            "gnss_leg_remaining_m": round(float(gnss_err.leg_remaining_m), 4)
            if gnss_err
            else None,
            "gnss_leg_locked": bool(self._gnss_leg_locked),
            "gnss_locked_stern_yaw": round(float(self._gnss_locked_stern_yaw), 4)
            if self._gnss_locked_stern_yaw is not None
            else None,
            "gnss_back_mode": self._gnss_back_mode,
            "gnss_back_correcting": bool(self._gnss_back_correcting),
            "gnss_near_entry": bool(
                gnss_err is not None
                and gnss_err.dist_m < self._gnss_back_heading_check_min_dist_m
            ),
            "gnss_pos_source": self._gnss_pos_source,
            "odom_x": round(float(self._odom_x), 4) if self._odom_x is not None else None,
            "odom_y": round(float(self._odom_y), 4) if self._odom_y is not None else None,
            "approach_depth_m": approach_depth_m,
            "entry_x_delta": round(
                float(approach_depth_m - self._entry_standoff_m), 4
            ),
            "pose_valid": bool(self._pose_valid),
            "tag_fresh": bool(self._fresh_tag_observation()),
            "tag_acquire_count": int(self._tag_acquire_count),
            "tag_acquire_target": int(self._tag_acquire_cycles),
            "tf_error": self._tf_error,
            "cmd_linear_x": round(float(self._cmd_v), 4),
            "cmd_angular_z": round(float(self._cmd_w), 4),
            "back_in_blind_active": bool(self._back_in_blind_active),
            "back_in_holding_for_tag": bool(self._back_in_holding_for_tag),
            "pose_settled": bool(self._settled()) if self._pose_valid else None,
            "motion_sensors_ok": bool(self._motion_sensors_ok()),
            "sensor_hold_active": bool(self._sensor_hold_active),
            "sensor_hold_reason": self._sensor_hold_reason,
        }
        payload.update(self._undock.status_payload(self._state.value, odom))
        if "undock_success" not in payload:
            payload["undock_success"] = False
        if "session_mode" not in payload:
            payload["session_mode"] = (
                "dock" if self._is_active_docking_state() else "idle"
            )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

