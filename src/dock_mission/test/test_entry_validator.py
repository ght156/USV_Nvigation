"""Tests for DockEntryValidator pure logic."""

from __future__ import annotations

import math

import pytest

from dock_mission.dock_enu import Pose2D
from dock_mission.entry_validator import (
    DockEntryValidator,
    EntryValidationInput,
    TagObservation,
)
from dock_mission.types import EntryAction, EntryReason


def _valid_boat(dock_transform) -> Pose2D:
    """Boat pose inside entry corridor with acceptable heading."""
    return dock_transform.dock_to_map(Pose2D(x=-3.0, y=0.0, yaw=0.0))


def test_validate_ok_at_staging(dock_transform):
    validator = DockEntryValidator(dock_transform)
    boat = _valid_boat(dock_transform)
    result = validator.validate(EntryValidationInput(boat_map=boat))
    assert result.valid is True
    assert result.action == EntryAction.PROCEED
    assert result.reason == EntryReason.OK


def test_validate_rejects_no_rtk(dock_transform):
    validator = DockEntryValidator(dock_transform)
    boat = _valid_boat(dock_transform)
    result = validator.validate(
        EntryValidationInput(boat_map=boat, rtk_fix=False)
    )
    assert result.valid is False
    assert result.action == EntryAction.REJECT
    assert result.reason == EntryReason.RTK_NOT_FIX


def test_validate_beyond_entry_line(dock_transform):
    validator = DockEntryValidator(dock_transform)
    # Inside dock: ex >= 0 in dock_enu
    inside = dock_transform.dock_to_map(Pose2D(x=0.5, y=0.0, yaw=math.pi))
    result = validator.validate(EntryValidationInput(boat_map=inside))
    assert result.valid is False
    assert result.action == EntryAction.BACKOFF
    assert result.reason == EntryReason.EX_BEYOND_ENTRY


def test_validate_too_deep_outside(dock_transform):
    validator = DockEntryValidator(dock_transform)
    deep = dock_transform.dock_to_map(
        Pose2D(x=dock_transform.bay.corridor.x_min - 1.0, y=0.0, yaw=math.pi)
    )
    result = validator.validate(EntryValidationInput(boat_map=deep))
    assert result.valid is False
    assert result.action == EntryAction.BACKOFF
    assert result.reason == EntryReason.EX_TOO_DEEP


def test_validate_lateral_offset(dock_transform):
    validator = DockEntryValidator(dock_transform)
    lateral = dock_transform.dock_to_map(
        Pose2D(x=-3.0, y=2.0, yaw=math.pi)
    )
    result = validator.validate(EntryValidationInput(boat_map=lateral))
    assert result.valid is False
    assert result.action == EntryAction.REPLAN_STAGING
    assert result.reason == EntryReason.EY_TOO_LARGE


def test_validate_yaw_error(dock_transform):
    validator = DockEntryValidator(dock_transform)
    bad_yaw = dock_transform.dock_to_map(
        Pose2D(x=-3.0, y=0.0, yaw=math.pi + 0.5)
    )
    result = validator.validate(EntryValidationInput(boat_map=bad_yaw))
    assert result.valid is False
    assert result.action == EntryAction.REPLAN_STAGING
    assert result.reason == EntryReason.EYAW_TOO_LARGE


def test_validate_frame_mismatch(dock_transform):
    validator = DockEntryValidator(dock_transform, tag_mismatch_threshold_m=0.5)
    boat = _valid_boat(dock_transform)
    result = validator.validate(
        EntryValidationInput(
            boat_map=boat,
            tag_map_mismatch_m=1.2,
        )
    )
    assert result.valid is False
    assert result.reason == EntryReason.FRAME_MISMATCH


def test_validate_requires_tag_when_configured(dock_transform):
    validator = DockEntryValidator(
        dock_transform,
        require_tag_for_proceed=True,
    )
    boat = _valid_boat(dock_transform)
    result = validator.validate(EntryValidationInput(boat_map=boat))
    assert result.valid is False
    assert result.reason == EntryReason.TAG_NOT_VISIBLE

    tagged = validator.validate(
        EntryValidationInput(
            boat_map=boat,
            tag=TagObservation(x_base=0.0, y_base=0.0, heading_error=0.0, valid=True),
        )
    )
    assert tagged.valid is True
