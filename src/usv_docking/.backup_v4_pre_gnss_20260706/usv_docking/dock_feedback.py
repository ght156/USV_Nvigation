"""OpenNav-inspired docking feedback: semantic phases, error codes, recoverable aborts."""

from __future__ import annotations

from enum import IntEnum
from typing import Optional

DOCK_PHASE_INITIAL_PERCEPTION = "INITIAL_PERCEPTION"
DOCK_PHASE_SEARCH_SPIN = "SEARCH_SPIN"
DOCK_PHASE_APPROACH_ENTRY = "APPROACH_ENTRY"
DOCK_PHASE_CONTROLLING = "CONTROLLING"
DOCK_PHASE_WAIT_CHARGE = "WAIT_CHARGE"
DOCK_PHASE_DOCKED = "DOCKED"
DOCK_PHASE_FAILED = "FAILED"
DOCK_PHASE_IDLE = "IDLE"


class DockErrorCode(IntEnum):
    NONE = 0
    MISSION_NOT_IDLE = 1
    MISSION_EMERGENCY = 2
    WAIT_TAG_TIMEOUT = 10
    TAG_SEARCH_TIMEOUT = 11
    TAG_SEARCH_NO_TAG = 12
    TAG_TIMEOUT = 13
    TAG_LOST_IN_CORRIDOR = 14
    ODOM_LOST = 20
    DOCK_POSE_STREAM_LOST = 21
    CORRIDOR_VIOLATION_IN_BACK_IN = 31
    CORRIDOR_VIOLATION_IN_APPROACH_ENTRY = 32
    MAX_DOCKING_DURATION = 33
    CHARGE_CONFIRM_TIMEOUT = 34
    CHARGE_POSE_LOST = 35
    CHARGE_STATUS_LOST = 36
    APPROACH_ENTRY_TIMEOUT = 37
    ALIGN_ENTRY_TIMEOUT = 38
    IN_DOCK_TAG_SEARCH_TIMEOUT = 39
    UNKNOWN = 99


_ABORT_TO_ERROR: dict[str, DockErrorCode] = {
    "MISSION_NOT_IDLE": DockErrorCode.MISSION_NOT_IDLE,
    "MISSION_EMERGENCY": DockErrorCode.MISSION_EMERGENCY,
    "WAIT_TAG_TIMEOUT": DockErrorCode.WAIT_TAG_TIMEOUT,
    "TAG_SEARCH_TIMEOUT": DockErrorCode.TAG_SEARCH_TIMEOUT,
    "TAG_SEARCH_NO_TAG": DockErrorCode.TAG_SEARCH_NO_TAG,
    "TAG_TIMEOUT": DockErrorCode.TAG_TIMEOUT,
    "TAG_LOST_IN_CORRIDOR": DockErrorCode.TAG_LOST_IN_CORRIDOR,
    "ODOM_LOST": DockErrorCode.ODOM_LOST,
    "DOCK_POSE_STREAM_LOST": DockErrorCode.DOCK_POSE_STREAM_LOST,
    "CORRIDOR_VIOLATION_IN_BACK_IN": DockErrorCode.CORRIDOR_VIOLATION_IN_BACK_IN,
    "CORRIDOR_VIOLATION_IN_APPROACH_ENTRY": DockErrorCode.CORRIDOR_VIOLATION_IN_APPROACH_ENTRY,
    "MAX_DOCKING_DURATION": DockErrorCode.MAX_DOCKING_DURATION,
    "CHARGE_CONFIRM_TIMEOUT": DockErrorCode.CHARGE_CONFIRM_TIMEOUT,
    "CHARGE_POSE_LOST": DockErrorCode.CHARGE_POSE_LOST,
    "CHARGE_STATUS_LOST": DockErrorCode.CHARGE_STATUS_LOST,
    "APPROACH_ENTRY_TIMEOUT": DockErrorCode.APPROACH_ENTRY_TIMEOUT,
    "ALIGN_ENTRY_TIMEOUT": DockErrorCode.ALIGN_ENTRY_TIMEOUT,
    "IN_DOCK_TAG_SEARCH_TIMEOUT": DockErrorCode.IN_DOCK_TAG_SEARCH_TIMEOUT,
}

AUTO_RETRY_ABORT_REASONS = frozenset(
    {
        "TAG_SEARCH_NO_TAG",
        "TAG_SEARCH_TIMEOUT",
    }
)


def error_code_from_reason(reason: Optional[str]) -> int:
    if not reason:
        return int(DockErrorCode.NONE)
    return int(_ABORT_TO_ERROR.get(reason, DockErrorCode.UNKNOWN))


def phase_from_state(state_value: str) -> str:
    if state_value in ("DOCK_IDLE", "DOCK_PRECHECK"):
        return DOCK_PHASE_IDLE
    if state_value in ("DOCK_WAIT_TAG", "DOCK_SEARCH_SPIN"):
        return (
            DOCK_PHASE_SEARCH_SPIN
            if state_value == "DOCK_SEARCH_SPIN"
            else DOCK_PHASE_INITIAL_PERCEPTION
        )
    if state_value == "DOCK_APPROACH_ENTRY":
        return DOCK_PHASE_APPROACH_ENTRY
    if state_value == "DOCK_ALIGN_ENTRY":
        return DOCK_PHASE_APPROACH_ENTRY
    if state_value == "DOCK_BACK_IN":
        return DOCK_PHASE_CONTROLLING
    if state_value == "DOCK_WAIT_CHARGE":
        return DOCK_PHASE_WAIT_CHARGE
    if state_value == "DOCK_STOP":
        return DOCK_PHASE_DOCKED
    if state_value == "DOCK_ABORT":
        return DOCK_PHASE_FAILED
    return DOCK_PHASE_IDLE


def is_auto_retry_reason(reason: str) -> bool:
    return reason in AUTO_RETRY_ABORT_REASONS
