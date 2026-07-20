"""Upper-layer dock task API: command modes, status/event JSON (commercial single-dock)."""

from __future__ import annotations

import json
from enum import IntEnum
from typing import Any, Optional

from dock_mission.types import MissionState


class DockCommand(IntEnum):
    ONE_CLICK_DOCK = 1
    DOCK_ONLY = 2
    UNDOCK = 3
    CANCEL = 4


GCS_ACTION_TO_COMMAND = {
    "one_click_dock": DockCommand.ONE_CLICK_DOCK,
    "dock_only": DockCommand.DOCK_ONLY,
    "undock": DockCommand.UNDOCK,
    "cancel": DockCommand.CANCEL,
}


def run_state_from_mission(state: MissionState, *, dock_active: bool) -> str:
    if state == MissionState.SUCCEEDED:
        return "SUCCEEDED"
    if state == MissionState.FAILED:
        return "FAILED"
    if state == MissionState.CANCELLED:
        return "CANCELLED"
    if state in (MissionState.IDLE,) and not dock_active:
        return "IDLE"
    return "RUNNING"


def phase_from_mission(state: MissionState, mode: Optional[str]) -> str:
    if state in (MissionState.IDLE, MissionState.SUCCEEDED, MissionState.FAILED, MissionState.CANCELLED):
        return "IDLE"
    if mode == "undock":
        if state in (MissionState.UNDOCK_HANDOFF, MissionState.MONITOR_UNDOCK):
            return "UNDOCKING"
        if state == MissionState.COMPLETE_SETTLE:
            return "UNDOCKING"
        return "UNDOCKING"
    if state in (
        MissionState.ARMED,
        MissionState.NAV_TO_STAGING,
        MissionState.SETTLE,
        MissionState.ENTRY_VALIDATE,
    ):
        return "STAGING"
    if state in (MissionState.DOCK_HANDOFF, MissionState.MONITOR_DOCK, MissionState.COMPLETE_SETTLE):
        return "DOCKING"
    return "IDLE"


def build_task_status(
    *,
    state: MissionState,
    mode: Optional[str],
    dock_active: bool,
    retry_count: int,
    retry_max: int,
    needs_manual_takeover: bool,
    needs_reapproach: bool,
    mission_id: str,
    command_id: str,
    elapsed_sec: float,
    error_code: int = 0,
    error_message: Optional[str] = None,
    camera_ready: bool = True,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "run_state": run_state_from_mission(state, dock_active=dock_active),
        "phase": phase_from_mission(state, mode),
        "mode": mode,
        "dock_active": bool(dock_active),
        "retry_count": int(retry_count),
        "retry_max": int(retry_max),
        "needs_manual_takeover": bool(needs_manual_takeover),
        "needs_reapproach": bool(needs_reapproach),
        "camera_ready": bool(camera_ready),
        "error_code": int(error_code),
        "error_message": error_message,
        "mission_id": mission_id or None,
        "command_id": command_id or None,
        "elapsed_sec": round(float(elapsed_sec), 2),
    }


def build_task_event(
    event: str,
    *,
    mission_id: str = "",
    command_id: str = "",
    detail: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "event": event,
        "mission_id": mission_id or None,
        "command_id": command_id or None,
    }
    if detail:
        payload["detail"] = detail
    return payload


def dumps_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False)


def parse_gcs_command(raw: str) -> Optional[tuple[DockCommand, str, str]]:
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None
    action = str(data.get("action", "")).strip().lower()
    cmd = GCS_ACTION_TO_COMMAND.get(action)
    if cmd is None:
        return None
    mission_id = str(data.get("mission_id") or "")
    command_id = str(data.get("command_id") or "")
    return cmd, mission_id, command_id
