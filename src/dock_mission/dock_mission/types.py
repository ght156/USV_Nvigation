"""Shared enums for dock_mission."""

from __future__ import annotations

from enum import Enum


class MissionState(str, Enum):
    IDLE = "DOCK_MISSION_IDLE"
    ARMED = "DOCK_ARMED"
    NAV_TO_STAGING = "NAV_TO_STAGING"
    SETTLE = "SETTLE"
    DOCK_HANDOFF = "DOCK_HANDOFF"
    MONITOR_DOCK = "MONITOR_DOCK"
    UNDOCK_HANDOFF = "UNDOCK_HANDOFF"
    MONITOR_UNDOCK = "MONITOR_UNDOCK"
    COMPLETE_SETTLE = "COMPLETE_SETTLE"
    SUCCEEDED = "DOCK_SUCCEEDED"
    FAILED = "DOCK_FAILED"
    CANCELLED = "DOCK_CANCELLED"


class SpeedAuthority(str, Enum):
    NAVIGATION = "NAVIGATION"
    STAGING_VERIFY = "STAGING_VERIFY"
    DOCKING = "DOCKING"
    # 任务正常收尾（成功/取消/关闭）后的让权信号：会话已结束，速度通道交还
    RELEASE = "RELEASE"
    # 真失败：任务以 FAILED 收场，需人工/上层介入
    FAILED = "FAILED"
