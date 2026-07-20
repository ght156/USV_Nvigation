"""dock_enu frame for usv_docking GNSS approach (self-contained, no dock_mission)."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DockGeometryBay:
    bay_id: str
    origin_map_x: float
    origin_map_y: float
    x_axis_yaw_map: float
    virtual_entry: Pose2D


def wrap_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


class DockEnuTransform:
    """Map frame ↔ dock_enu (2D)."""

    def __init__(self, bay: DockGeometryBay) -> None:
        self._bay = bay
        self._cos = math.cos(-bay.x_axis_yaw_map)
        self._sin = math.sin(-bay.x_axis_yaw_map)

    @property
    def bay(self) -> DockGeometryBay:
        return self._bay

    def map_to_dock(self, map_x: float, map_y: float, map_yaw: float) -> Pose2D:
        dx = map_x - self._bay.origin_map_x
        dy = map_y - self._bay.origin_map_y
        x = self._cos * dx - self._sin * dy
        y = self._sin * dx + self._cos * dy
        yaw = wrap_yaw(map_yaw - self._bay.x_axis_yaw_map)
        return Pose2D(x=x, y=y, yaw=yaw)
