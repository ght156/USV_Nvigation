"""Dock entry validation in dock_enu (Phase 2 skeleton)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from dock_mission.dock_enu import DockEnuTransform, Pose2D
from dock_mission.types import EntryAction, EntryReason


@dataclass
class TagObservation:
    x_base: float
    y_base: float
    heading_error: float
    valid: bool


@dataclass
class EntryValidationInput:
    boat_map: Pose2D
    rtk_fix: bool = True
    tag: Optional[TagObservation] = None
    tag_map_mismatch_m: Optional[float] = None


@dataclass
class EntryValidationResult:
    valid: bool
    action: EntryAction
    reason: EntryReason
    ex: float
    ey: float
    eyaw: float
    tag_visible: bool
    message: str


class DockEntryValidator:
    """Pure logic — no ROS dependencies."""

    def __init__(
        self,
        transform: DockEnuTransform,
        tag_mismatch_threshold_m: float = 0.8,
        require_tag_for_proceed: bool = False,
    ) -> None:
        self._tf = transform
        self._tag_mismatch_threshold_m = tag_mismatch_threshold_m
        self._require_tag = require_tag_for_proceed

    def validate(self, data: EntryValidationInput) -> EntryValidationResult:
        if not data.rtk_fix:
            return self._result(
                False,
                EntryAction.REJECT,
                EntryReason.RTK_NOT_FIX,
                0.0,
                0.0,
                0.0,
                False,
                "RTK not FIX",
            )

        dock_pose = self._tf.map_to_dock(
            data.boat_map.x, data.boat_map.y, data.boat_map.yaw
        )
        ex, ey, eyaw = dock_pose.x, dock_pose.y, dock_pose.yaw
        corridor = self._tf.bay.corridor
        tag_visible = bool(data.tag and data.tag.valid)

        if data.tag_map_mismatch_m is not None:
            if data.tag_map_mismatch_m > self._tag_mismatch_threshold_m:
                return self._result(
                    False,
                    EntryAction.REJECT,
                    EntryReason.FRAME_MISMATCH,
                    ex,
                    ey,
                    eyaw,
                    tag_visible,
                    f"tag/map mismatch {data.tag_map_mismatch_m:.2f}m",
                )

        if self._require_tag and not tag_visible:
            return self._result(
                False,
                EntryAction.REPLAN_STAGING,
                EntryReason.TAG_NOT_VISIBLE,
                ex,
                ey,
                eyaw,
                False,
                "tag not visible for proceed",
            )

        if ex >= corridor.x_max:
            return self._result(
                False,
                EntryAction.BACKOFF,
                EntryReason.EX_BEYOND_ENTRY,
                ex,
                ey,
                eyaw,
                tag_visible,
                "boat beyond entry line (ex >= 0)",
            )

        if ex <= corridor.x_min:
            return self._result(
                False,
                EntryAction.BACKOFF,
                EntryReason.EX_TOO_DEEP,
                ex,
                ey,
                eyaw,
                tag_visible,
                "boat too far outside feasible backoff zone",
            )

        if abs(ey) > corridor.y_max:
            return self._result(
                False,
                EntryAction.REPLAN_STAGING,
                EntryReason.EY_TOO_LARGE,
                ex,
                ey,
                eyaw,
                tag_visible,
                f"|ey|={abs(ey):.2f} > y_max",
            )

        if abs(eyaw) > corridor.yaw_max:
            return self._result(
                False,
                EntryAction.REPLAN_STAGING,
                EntryReason.EYAW_TOO_LARGE,
                ex,
                ey,
                eyaw,
                tag_visible,
                f"|eyaw|={abs(eyaw):.2f} > yaw_max",
            )

        staging = self._tf.bay.staging
        staging_err = math.hypot(ex - staging.x, ey - staging.y)
        _ = staging_err  # TODO: optional tighter staging sphere check

        return self._result(
            True,
            EntryAction.PROCEED,
            EntryReason.OK,
            ex,
            ey,
            eyaw,
            tag_visible,
            "entry corridor OK",
        )

    def _result(
        self,
        valid: bool,
        action: EntryAction,
        reason: EntryReason,
        ex: float,
        ey: float,
        eyaw: float,
        tag_visible: bool,
        message: str,
    ) -> EntryValidationResult:
        return EntryValidationResult(
            valid=valid,
            action=action,
            reason=reason,
            ex=ex,
            ey=ey,
            eyaw=eyaw,
            tag_visible=tag_visible,
            message=message,
        )
