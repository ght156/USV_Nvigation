"""Upper-layer dock task API: command modes, status/event JSON (commercial single-dock)."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
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

# 与 mission_bridge.py 的 YAW_UNSPECIFIED=65536.0 对齐（度语义）：非法/缺失 dock_yaw 时
# 让导航按行进方向作为到点朝向。航向接口统一为度，超出 [-360, 360] 一律视为"不指定朝向"。
DOCK_YAW_UNSPECIFIED = 65536.0


@dataclass(frozen=True)
class GcsCommand:
    """/gcs_dock/command 解析结果。

    dock_lat/dock_lon/dock_yaw 为可选预泊点（靠泊点经纬度 + 船尾朝向船坞角度）。
    三者齐全且合法时，dock_mission 用它覆盖泊位库 gnss_staging 作为归港导航目标点；
    **dock_yaw 单位为度**（接口统一用度），dock_mission 按度透传给 mission_bridge
    （MissionWaypoint.yaw 同为度；Nav2 位姿时才转弧度）。仅给经纬度不带 yaw 时，
    yaw 取 DOCK_YAW_UNSPECIFIED（导航按行进方向）。dock_yaw 超范围（如 65536.0）同样
    被视为"不指定朝向"。
    """

    command: DockCommand
    mission_id: str
    command_id: str
    dock_lat: Optional[float] = None
    dock_lon: Optional[float] = None
    dock_yaw: Optional[float] = None

    @property
    def has_dock_point(self) -> bool:
        return self.dock_lat is not None and self.dock_lon is not None


def _opt_float(value: Any) -> Optional[float]:
    """把 JSON 里的字符串/数值转成有限 float；非法/缺失返回 None。"""
    if value is None or isinstance(value, bool):
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def is_valid_dock_point(lat: float, lon: float) -> bool:
    """预泊点经纬度有效性：非 (0,0)、落在 WGS84 合法范围。"""
    if lat == 0.0 and lon == 0.0:
        return False
    return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


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
    event_ack_ok: bool = True,
    event_ack_failures: int = 0,
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
        "event_ack_ok": bool(event_ack_ok),
        "event_ack_failures": int(event_ack_failures),
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


def parse_gcs_command(raw: str) -> Optional[GcsCommand]:
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
    return GcsCommand(
        command=cmd,
        mission_id=mission_id,
        command_id=command_id,
        dock_lat=_opt_float(data.get("dock_lat")),
        dock_lon=_opt_float(data.get("dock_lon")),
        dock_yaw=_opt_float(data.get("dock_yaw")),
    )
