"""Odom-based undock: drive forward out of the dock channel (MVP)."""

from __future__ import annotations

import math


def planar_distance(
    x0: float, y0: float, x1: float, y1: float
) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def heading_hold_omega(
    current_yaw: float,
    target_yaw: float,
    kpsi: float,
    omega_max: float,
) -> float:
    err = wrap_yaw(target_yaw - current_yaw)
    w = kpsi * err
    return max(-omega_max, min(omega_max, w))


def undock_forward_speed(
    traveled_m: float,
    target_m: float,
    cruise_speed: float,
    min_speed: float,
    creep_m: float,
) -> float:
    """Slow near target distance; stop when traveled >= target."""
    remaining = max(0.0, target_m - traveled_m)
    if remaining <= 0.0:
        return 0.0
    speed = max(min_speed, min(cruise_speed, cruise_speed))
    if creep_m > 0.0 and remaining < creep_m:
        scale = max(0.15, remaining / creep_m)
        speed = min(speed, max(min_speed, cruise_speed * scale))
    return speed


def undock_reached(traveled_m: float, target_m: float, tol_m: float) -> bool:
    return traveled_m >= max(0.0, target_m - tol_m)
