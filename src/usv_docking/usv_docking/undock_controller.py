"""Odom-based undock state machine (MVP). Separated from docking_controller."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

from usv_docking.undock import (
    heading_hold_omega,
    planar_distance,
    undock_forward_speed,
    undock_reached,
)

# ROS parameter defaults — declared by docking_controller, logic lives here.
UNDOCK_PARAM_DEFAULTS: dict[str, float | int | bool | str] = {
    "undock_topic": "/dock/undock",
    "undock_distance_m": 5.0,
    "undock_dist_tol_m": 0.15,
    "undock_speed": 0.25,
    "undock_min_speed": 0.05,
    "undock_creep_dist_m": 1.0,
    "undock_heading_hold": True,
    "undock_heading_kpsi": 0.35,
    "undock_omega_max": 0.15,
    "undock_settle_cycles": 5,
    "undock_timeout_sec": 60.0,
    "require_mission_idle_for_undock": False,
}


class UndockState(str, Enum):
    OUT = "DOCK_UNDOCK_OUT"
    SETTLE = "DOCK_UNDOCK_SETTLE"
    STOP = "DOCK_UNDOCK_STOP"


START_ALLOWED_FROM = frozenset(
    {
        "DOCK_IDLE",
        "DOCK_STOP",
        "DOCK_UNDOCK_STOP",
    }
)


@dataclass(frozen=True)
class UndockConfig:
    distance_m: float = 5.0
    dist_tol_m: float = 0.15
    speed: float = 0.25
    min_speed: float = 0.05
    creep_dist_m: float = 1.0
    heading_hold: bool = True
    heading_kpsi: float = 0.35
    omega_max: float = 0.15
    settle_cycles: int = 5
    timeout_sec: float = 60.0
    odom_timeout_sec: float = 1.5
    odom_startup_grace_sec: float = 2.0
    require_mission_idle: bool = False

    @classmethod
    def from_ros(cls, get_param) -> UndockConfig:
        g = get_param
        return cls(
            distance_m=float(g("undock_distance_m").value),
            dist_tol_m=float(g("undock_dist_tol_m").value),
            speed=float(g("undock_speed").value),
            min_speed=float(g("undock_min_speed").value),
            creep_dist_m=float(g("undock_creep_dist_m").value),
            heading_hold=bool(g("undock_heading_hold").value),
            heading_kpsi=float(g("undock_heading_kpsi").value),
            omega_max=float(g("undock_omega_max").value),
            settle_cycles=int(g("undock_settle_cycles").value),
            timeout_sec=float(g("undock_timeout_sec").value),
            odom_timeout_sec=float(g("odom_timeout_sec").value),
            odom_startup_grace_sec=float(g("odom_startup_grace_sec").value),
            require_mission_idle=bool(g("require_mission_idle_for_undock").value),
        )


@dataclass(frozen=True)
class OdomReading:
    have: bool
    x: Optional[float]
    y: Optional[float]
    yaw: Optional[float]
    age_sec: Optional[float]


@dataclass
class UndockStepResult:
    v_target: float = 0.0
    w_target: float = 0.0
    next_state: Optional[UndockState] = None
    transition_reason: Optional[str] = None
    abort_reason: Optional[str] = None


class UndockController:
    """Pure-Python undock session; docking_controller owns ROS I/O."""

    def __init__(self, config: UndockConfig) -> None:
        self._cfg = config
        self._start_x: Optional[float] = None
        self._start_y: Optional[float] = None
        self._hold_yaw: Optional[float] = None
        self._start_mono: Optional[float] = None
        self._settle_count = 0

    @property
    def config(self) -> UndockConfig:
        return self._cfg

    @staticmethod
    def is_active_state(state_value: str) -> bool:
        return state_value in {s.value for s in UndockState}

    @staticmethod
    def can_start_from(state_value: str) -> bool:
        return state_value in START_ALLOWED_FROM

    def begin(self, odom: OdomReading, start_mono: float) -> None:
        self._start_mono = start_mono
        self._settle_count = 0
        if odom.have and odom.x is not None and odom.y is not None:
            self._start_x = float(odom.x)
            self._start_y = float(odom.y)
            self._hold_yaw = odom.yaw
        else:
            self._start_x = None
            self._start_y = None
            self._hold_yaw = None

    def traveled_m(self, odom: OdomReading) -> float:
        if (
            self._start_x is None
            or self._start_y is None
            or odom.x is None
            or odom.y is None
        ):
            return 0.0
        return planar_distance(self._start_x, self._start_y, odom.x, odom.y)

    def elapsed_sec(self, now_mono: float) -> float:
        if self._start_mono is None:
            return 0.0
        return max(0.0, now_mono - self._start_mono)

    def check_timeout(self, now_mono: float) -> Optional[str]:
        if self._start_mono is None:
            return None
        if self.elapsed_sec(now_mono) > self._cfg.timeout_sec:
            return "UNDOCK_TIMEOUT"
        return None

    def odom_lost(self, odom: OdomReading, now_mono: float) -> bool:
        elapsed = self.elapsed_sec(now_mono)
        if elapsed < self._cfg.odom_startup_grace_sec:
            return False
        if not odom.have:
            return True
        if odom.age_sec is None:
            return True
        return odom.age_sec > self._cfg.odom_timeout_sec

    def step(self, state: UndockState, odom: OdomReading) -> UndockStepResult:
        if state == UndockState.OUT:
            if not odom.have or self._start_x is None:
                return UndockStepResult()
            traveled = self.traveled_m(odom)
            if undock_reached(traveled, self._cfg.distance_m, self._cfg.dist_tol_m):
                return UndockStepResult(
                    next_state=UndockState.SETTLE,
                    transition_reason=f"undock distance {traveled:.2f}m",
                )
            v_target = undock_forward_speed(
                traveled,
                self._cfg.distance_m,
                self._cfg.speed,
                self._cfg.min_speed,
                self._cfg.creep_dist_m,
            )
            w_target = 0.0
            if (
                self._cfg.heading_hold
                and odom.yaw is not None
                and self._hold_yaw is not None
            ):
                w_target = heading_hold_omega(
                    odom.yaw,
                    self._hold_yaw,
                    self._cfg.heading_kpsi,
                    self._cfg.omega_max,
                )
            return UndockStepResult(v_target=v_target, w_target=w_target)

        if state == UndockState.SETTLE:
            self._settle_count += 1
            if self._settle_count >= self._cfg.settle_cycles:
                return UndockStepResult(
                    next_state=UndockState.STOP,
                    transition_reason="undock complete",
                )
            return UndockStepResult()

        return UndockStepResult()

    def status_payload(self, state_value: str, odom: OdomReading) -> dict[str, Any]:
        if not (
            self.is_active_state(state_value)
            or state_value == UndockState.STOP.value
        ):
            return {}
        return {
            "undock_success": state_value == UndockState.STOP.value,
            "session_mode": "undock",
            "undock_traveled_m": round(self.traveled_m(odom), 4),
            "undock_target_m": round(self._cfg.distance_m, 4),
        }
