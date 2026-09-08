"""dock_enu frame transforms (Phase 2 skeleton).

All entry validation must use poses expressed in dock_enu:
  x: along channel axis (positive = inside dock)
  y: lateral offset from channel center
  yaw: heading relative to channel axis
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class EntryCorridor:
    x_min: float  # most negative ex allowed (farthest outside)
    x_max: float  # must be <= 0 (entry line at x=0)
    y_max: float
    yaw_max: float


@dataclass(frozen=True)
class BayDefinition:
    bay_id: str
    origin_map_x: float
    origin_map_y: float
    x_axis_yaw_map: float
    staging: Pose2D
    entry_point: Pose2D
    corridor: EntryCorridor
    standoff_m: float = 4.0


def wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


class DockEnuTransform:
    """Map frame ↔ dock_enu (2D)."""

    def __init__(self, bay: BayDefinition) -> None:
        self._bay = bay
        self._cos = math.cos(-bay.x_axis_yaw_map)
        self._sin = math.sin(-bay.x_axis_yaw_map)

    @property
    def bay(self) -> BayDefinition:
        return self._bay

    def map_to_dock(self, map_x: float, map_y: float, map_yaw: float) -> Pose2D:
        dx = map_x - self._bay.origin_map_x
        dy = map_y - self._bay.origin_map_y
        x = self._cos * dx - self._sin * dy
        y = self._sin * dx + self._cos * dy
        yaw = wrap_yaw(map_yaw - self._bay.x_axis_yaw_map)
        return Pose2D(x=x, y=y, yaw=yaw)

    def dock_to_map(self, pose: Pose2D) -> Pose2D:
        cos_f = math.cos(self._bay.x_axis_yaw_map)
        sin_f = math.sin(self._bay.x_axis_yaw_map)
        map_x = self._bay.origin_map_x + cos_f * pose.x - sin_f * pose.y
        map_y = self._bay.origin_map_y + sin_f * pose.x + cos_f * pose.y
        map_yaw = wrap_yaw(pose.yaw + self._bay.x_axis_yaw_map)
        return Pose2D(x=map_x, y=map_y, yaw=map_yaw)

    def default_staging_in_dock(self) -> Pose2D:
        return Pose2D(
            x=-abs(self._bay.standoff_m),
            y=0.0,
            yaw=math.pi,
        )


def load_bay_from_dict(bay_id: str, data: dict) -> BayDefinition:
    """Load one bay entry from dock_database.yaml structure."""
    origin = data.get("origin_map", {})
    staging = data.get("staging_dock_enu", {})
    entry = data.get("entry_point_dock_enu", {})
    corridor = data.get("entry_corridor", {})
    staging_pose = Pose2D(
        x=float(staging.get("x", -4.0)),
        y=float(staging.get("y", 0.0)),
        yaw=float(staging.get("yaw", math.pi)),
    )
    entry_default_x = staging_pose.x * 0.5 if staging_pose.x < 0 else -2.0
    return BayDefinition(
        bay_id=bay_id,
        origin_map_x=float(origin.get("x", 0.0)),
        origin_map_y=float(origin.get("y", 0.0)),
        x_axis_yaw_map=float(origin.get("x_axis_yaw", 0.0)),
        staging=staging_pose,
        entry_point=Pose2D(
            x=float(entry.get("x", entry_default_x)),
            y=float(entry.get("y", 0.0)),
            yaw=float(entry.get("yaw", staging_pose.yaw)),
        ),
        corridor=EntryCorridor(
            x_min=float(corridor.get("x_min", -6.0)),
            x_max=float(corridor.get("x_max", 0.0)),
            y_max=float(corridor.get("y_max", 1.0)),
            yaw_max=float(corridor.get("yaw_max", 0.15)),
        ),
        standoff_m=float(data.get("standoff_m", 4.0)),
    )
