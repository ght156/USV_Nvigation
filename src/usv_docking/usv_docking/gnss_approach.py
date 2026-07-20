"""GNSS leg error model: locked approach line + tight heading (phase-1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from usv_docking.gnss_geo import geodetic_delta_enu_m, wrap_yaw

GnssBackMode = Literal["cruise", "walk_fix", "correct", "hold"]


@dataclass(frozen=True)
class GnssWaypointErrors:
    dist_m: float
    bearing_rad: float
    cross_track_m: float
    along_track_m: float
    yaw_rad: float
    desired_yaw_rad: float
    deyaw: float
    leg_remaining_m: float = 0.0


def cross_track_to_gnss_segment(
    boat_lat: float,
    boat_lon: float,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> float:
    """Perpendicular distance (m) from boat to geodesic leg start→end."""
    ab_e, ab_n = geodetic_delta_enu_m(start_lat, start_lon, end_lat, end_lon)
    ap_e, ap_n = geodetic_delta_enu_m(start_lat, start_lon, boat_lat, boat_lon)
    ab_len_sq = ab_e * ab_e + ab_n * ab_n
    if ab_len_sq < 1e-8:
        return math.hypot(ap_e, ap_n)
    return abs(ap_e * ab_n - ap_n * ab_e) / math.sqrt(ab_len_sq)


def leg_remaining_m(
    boat_lat: float,
    boat_lon: float,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> float:
    """Distance remaining along leg to end (>0 before entry, <0 after overshoot)."""
    ab_e, ab_n = geodetic_delta_enu_m(start_lat, start_lon, end_lat, end_lon)
    ap_e, ap_n = geodetic_delta_enu_m(start_lat, start_lon, boat_lat, boat_lon)
    ab_len = math.hypot(ab_e, ab_n)
    if ab_len < 1e-6:
        return 0.0
    projection = (ap_e * ab_e + ap_n * ap_n) / ab_len
    return ab_len - projection


def stern_toward_yaw(bearing_rad: float) -> float:
    """Boat yaw so stern points along bearing_rad (boat → waypoint)."""
    return wrap_yaw(bearing_rad + math.pi)


def bearing_to_gnss_target(
    from_lat: float,
    from_lon: float,
    to_lat: float,
    to_lon: float,
) -> float:
    east, north = geodetic_delta_enu_m(from_lat, from_lon, to_lat, to_lon)
    return math.atan2(north, east)


def compute_leg_errors(
    boat_lat: float,
    boat_lon: float,
    boat_yaw: float,
    entry_lat: float,
    entry_lon: float,
    leg_start_lat: float,
    leg_start_lon: float,
    locked_stern_yaw: float,
) -> GnssWaypointErrors:
    """Errors using locked leg geometry (heading does NOT chase moving GPS bearing)."""
    east, north = geodetic_delta_enu_m(boat_lat, boat_lon, entry_lat, entry_lon)
    dist_m = math.hypot(east, north)
    bearing_rad = math.atan2(north, east)
    deyaw = wrap_yaw(locked_stern_yaw - boat_yaw)
    cross_track_m = cross_track_to_gnss_segment(
        boat_lat, boat_lon, leg_start_lat, leg_start_lon, entry_lat, entry_lon
    )
    remaining = leg_remaining_m(
        boat_lat, boat_lon, leg_start_lat, leg_start_lon, entry_lat, entry_lon
    )
    return GnssWaypointErrors(
        dist_m=dist_m,
        bearing_rad=bearing_rad,
        cross_track_m=cross_track_m,
        along_track_m=-remaining,
        yaw_rad=wrap_yaw(boat_yaw),
        desired_yaw_rad=locked_stern_yaw,
        deyaw=deyaw,
        leg_remaining_m=remaining,
    )


def compute_waypoint_errors(
    boat_lat: float,
    boat_lon: float,
    boat_yaw: float,
    entry_lat: float,
    entry_lon: float,
) -> GnssWaypointErrors:
    """Live bearing errors (used only before leg lock)."""
    east, north = geodetic_delta_enu_m(boat_lat, boat_lon, entry_lat, entry_lon)
    dist_m = math.hypot(east, north)
    bearing_rad = math.atan2(north, east)
    desired_yaw_rad = stern_toward_yaw(bearing_rad)
    deyaw = wrap_yaw(desired_yaw_rad - boat_yaw)
    return GnssWaypointErrors(
        dist_m=dist_m,
        bearing_rad=bearing_rad,
        cross_track_m=0.0,
        along_track_m=dist_m,
        yaw_rad=wrap_yaw(boat_yaw),
        desired_yaw_rad=desired_yaw_rad,
        deyaw=deyaw,
        leg_remaining_m=dist_m,
    )


def heading_aligned(errors: GnssWaypointErrors, tol: float) -> bool:
    return abs(errors.deyaw) < tol


def heading_resume_ok(deyaw: float, ok_tol: float) -> bool:
    return abs(deyaw) < ok_tol


def heading_needs_correction(deyaw: float, correct_tol: float) -> bool:
    return abs(deyaw) >= correct_tol


def heading_align_omega(deyaw: float, kpsi: float, omega_max: float) -> float:
    return max(-omega_max, min(omega_max, kpsi * deyaw))


def gnss_back_creep_speed(
    dist_m: float,
    leg_remaining_m: float,
    kx: float,
    v_min: float,
    v_max: float,
    creep_dist_m: float,
    overshoot_remaining_tol_m: float,
) -> float:
    if leg_remaining_m < -overshoot_remaining_tol_m:
        return 0.0
    speed = max(v_min, min(v_max, kx * dist_m))
    if creep_dist_m > 0.0 and dist_m < creep_dist_m:
        speed = min(speed, max(v_min, kx * dist_m))
    return speed


def gnss_back_motion_cmd(
    deyaw: float,
    speed: float,
    *,
    steer_deadband: float,
    ok_tol: float,
    correct_tol: float,
    steer_kpsi: float,
    walk_omega_max: float,
    align_kpsi: float,
    align_omega_max: float,
    walk_speed_scale: float = 0.75,
) -> tuple[float, float, GnssBackMode]:
    """Three-band backing: cruise | walk_fix | stop-correct."""
    del ok_tol  # reserved for hysteresis tuning; bands use deadband/correct_tol
    if speed <= 0.0:
        return 0.0, 0.0, "hold"

    ad = abs(deyaw)
    if ad >= correct_tol:
        return 0.0, heading_align_omega(deyaw, align_kpsi, align_omega_max), "correct"
    if ad >= steer_deadband:
        w = max(-walk_omega_max, min(walk_omega_max, steer_kpsi * deyaw))
        return -speed * walk_speed_scale, w, "walk_fix"
    return -speed, 0.0, "cruise"


def at_virtual_entry(
    errors: GnssWaypointErrors,
    dist_tol: float,
    yaw_tol: float,
    *,
    overshoot_remaining_tol_m: float = 0.25,
) -> bool:
    """Precise arrival: distance + locked heading, or controlled overshoot stop."""
    if errors.leg_remaining_m < -overshoot_remaining_tol_m and errors.dist_m < dist_tol * 2.5:
        return True
    if errors.dist_m >= dist_tol:
        return False
    return abs(errors.deyaw) < yaw_tol
