"""Dynamic staging pose replan (Phase 2 skeleton)."""

from __future__ import annotations

import math

from dock_mission.dock_enu import DockEnuTransform, Pose2D
from dock_mission.types import EntryAction


class StagingPlanner:
    """Project boat back to channel axis for replan staging goals."""

    def __init__(self, transform: DockEnuTransform) -> None:
        self._tf = transform

    def replan_staging_map(
        self,
        boat_map: Pose2D,
        action: EntryAction,
    ) -> Pose2D:
        boat_dock = self._tf.map_to_dock(boat_map.x, boat_map.y, boat_map.yaw)
        standoff = self._tf.bay.standoff_m
        corridor = self._tf.bay.corridor

        if action == EntryAction.BACKOFF:
            # Move further outside along -x
            target_dock = Pose2D(
                x=min(boat_dock.x - 3.0, -standoff),
                y=0.0,
                yaw=math.pi,
            )
        else:
            # REPLAN_STAGING: snap to channel center at standoff
            target_dock = Pose2D(
                x=max(corridor.x_min + 0.5, -standoff),
                y=0.0,
                yaw=math.pi,
            )

        return self._tf.dock_to_map(target_dock)
