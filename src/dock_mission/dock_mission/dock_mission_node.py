"""Dock mission FSM: upper-layer service + GCS topic → staging → handoff → dock_task feedback."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from dock_mission.bay_loader import load_bay
from dock_mission.dock_task_api import (
    DOCK_YAW_UNSPECIFIED,
    DockCommand,
    GcsCommand,
    build_task_event,
    build_task_status,
    dumps_json,
    is_valid_dock_point,
    parse_gcs_command,
)
from dock_mission.gnss_staging import make_staging_waypoint
from dock_mission.mission_client import MissionBridgeClient
from dock_mission.nav2_goal_checker import Nav2GoalCheckerSwitch
from dock_mission.task_event import parse_task_event
from dock_mission.types import MissionState, SpeedAuthority

try:
    from m_common.srv import DockTaskCommand  # /dock_task/command（可能已移除）
except ImportError:  # pragma: no cover - before colcon build
    DockTaskCommand = None  # type: ignore[misc, assignment]

try:
    from m_common.srv import DockTaskEvent  # /dock_task/event 上报 service
except ImportError:  # pragma: no cover - before colcon build
    DockTaskEvent = None  # type: ignore[misc, assignment]


class DockMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("dock_mission_node")
        self._declare_parameters()
        self._load_parameters()

        self._state = MissionState.IDLE
        self._staging_retry = 0
        self._settle_elapsed = 0.0
        self._complete_settle_elapsed = 0.0
        self._pending_nav = False
        self._staging_send_inflight = False
        self._staging_send_mono = 0.0
        self._entry_validate_result: Optional[bool] = None
        self._entry_validate_inflight = False
        self._entry_validate_mono = 0.0
        self._mission_id = "dock_staging"
        self._command_id = ""
        self._staging_override: Optional[tuple[float, float, float]] = None
        self._mode: Optional[str] = None
        self._dock_active = False
        self._needs_manual_takeover = False
        self._needs_reapproach = False
        self._last_fail_reason = ""
        self._session_start_mono: Optional[float] = None
        self._camera_ready = True
        # 事件上报 ACK 跟踪（事件通道已从 topic 改为 service）
        self._event_ack_ok = True
        self._event_ack_failures = 0
        self._event_pending = 0
        self._event_pending_mono = 0.0
        self._event_pending_futures: list = []
        self._last_event = ""

        self._goal_switch = Nav2GoalCheckerSwitch(self)
        self._mission = MissionBridgeClient(self)

        self._authority_pub = self.create_publisher(String, "/dock/speed_authority", 10)
        self._legacy_status_pub = self.create_publisher(String, "/dock/mission_status", 10)
        self._task_status_pub = self.create_publisher(
            String, self._dock_task_status_topic, 10
        )
        # 事件观察流（可选，供日志/联调；无上层服务时也能看到事件）
        self._task_event_log_pub = None
        if self._dock_task_event_log_topic:
            self._task_event_log_pub = self.create_publisher(
                String, self._dock_task_event_log_topic, 10
            )
        self._dock_start_pub = self.create_publisher(Bool, "/dock/start", 10)
        self._dock_undock_pub = self.create_publisher(Bool, "/dock/undock", 10)
        self._dock_cancel_pub = self.create_publisher(Empty, "/dock/cancel", 10)

        self.create_subscription(String, "/task_event", self._task_event_cb, 10)
        self.create_subscription(String, "/dock/status", self._dock_status_cb, 10)
        home_topic = str(self.get_parameter("dock_home_topic").value)
        self.create_subscription(Bool, home_topic, self._home_cb, 10)
        self.create_subscription(String, self._gcs_dock_command_topic, self._gcs_command_cb, 10)

        self.create_service(Trigger, "/dock/mission/start", self._start_srv)
        self.create_service(Trigger, "/dock/mission/cancel", self._cancel_srv)
        if DockTaskCommand is not None:
            self.create_service(DockTaskCommand, "/dock_task/command", self._dock_task_command_srv)
        # 事件上报 client：船端(dock_mission) → 上层状态机(嵌软)，等待 ACK
        self._event_client = None
        if DockTaskEvent is not None:
            self._event_client = self.create_client(
                DockTaskEvent, self._dock_task_event_service
            )

        self._validate_cli = self.create_client(Trigger, "/dock/validate_entry")

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"dock_mission ready home_topic={home_topic} "
            f"task_status={self._dock_task_status_topic}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "bay_id": "bay2",
            "dock_database_path": "",
            "map_yaml_path": "",
            "use_gnss_staging": True,  # 实船默认 WGS84 预泊点；False=map_staging 反解(标定用)
            "staging_retry_max": 3,
            "settle_sec": 2.5,
            "complete_settle_sec": 2.0,
            "dock_home_topic": "/dock/home",
            "gcs_dock_command_topic": "/gcs_dock/command",
            "dock_task_status_topic": "/dock_task/status",
            "dock_task_event_service": "/dock_task/event",
            "dock_task_event_log_topic": "/dock_task/event_log",
            "event_ack_timeout": 3.0,
            "send_waypoints_service": "/mission_bridge/send_waypoints",
            "nav2_controller_node": "/controller_server",
            "goal_checker_selector_topic": "goal_checker_selector",
            "use_goal_checker_selector": True,
            "cruise_goal_checker_id": "general_goal_checker",
            "docking_goal_checker_id": "docking_goal_checker",
            "cruise_xy_goal_tolerance": 1.0,
            "cruise_yaw_goal_tolerance": 1.0,
            "docking_xy_goal_tolerance": 0.6,
            "docking_yaw_goal_tolerance": 0.15,
            "dock_mission_id": "dock_staging",
        }
        for name, value in defaults.items():
            if isinstance(value, bool):
                self.declare_parameter(name, value)
            elif isinstance(value, int):
                self.declare_parameter(name, value)
            elif isinstance(value, float):
                self.declare_parameter(name, value)
            else:
                self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        g = self.get_parameter
        self._bay_id = str(g("bay_id").value)
        self._dock_db_path = str(g("dock_database_path").value).strip()
        map_path = str(g("map_yaml_path").value).strip()
        if map_path:
            self._map_yaml = Path(map_path)
        else:
            try:
                from ament_index_python.packages import get_package_share_directory

                self._map_yaml = Path(get_package_share_directory("workspace_nav")) / "config" / "map_hk.yaml"
            except Exception:
                self._map_yaml = Path("")
        self._use_gnss = bool(g("use_gnss_staging").value)
        self._mission_id = str(g("dock_mission_id").value)
        self._gcs_dock_command_topic = str(g("gcs_dock_command_topic").value)
        self._dock_task_status_topic = str(g("dock_task_status_topic").value)
        self._dock_task_event_service = str(g("dock_task_event_service").value)
        self._dock_task_event_log_topic = str(g("dock_task_event_log_topic").value).strip()
        self._event_ack_timeout = float(g("event_ack_timeout").value)

    def _retry_max(self) -> int:
        return int(self.get_parameter("staging_retry_max").value)

    def _elapsed_sec(self) -> float:
        if self._session_start_mono is None:
            return 0.0
        return max(0.0, time.monotonic() - self._session_start_mono)

    def _set_authority(self, authority: SpeedAuthority) -> None:
        msg = String()
        msg.data = authority.value
        self._authority_pub.publish(msg)

    def _emit_event(self, event: str, detail: Optional[dict] = None) -> None:
        payload = build_task_event(
            event,
            mission_id=self._mission_id,
            command_id=self._command_id,
            detail=detail,
        )
        self._last_event = event
        # 可选观察流（本地日志/联调）：事件仍可见，即便上层状态机不在线
        if self._task_event_log_pub is not None:
            msg = String()
            msg.data = dumps_json(payload)
            self._task_event_log_pub.publish(msg)
        # 关键事件上报 service：请求上层（嵌软状态机）确认收到（ACK）
        self._report_event(payload)

    def _report_event(self, payload: dict) -> None:
        if self._event_client is None:
            self._record_event_ack(False, "event service unavailable (interface not built)")
            return
        req = DockTaskEvent.Request()
        req.event = str(payload.get("event") or "")
        req.mission_id = str(payload.get("mission_id") or "")
        req.command_id = str(payload.get("command_id") or "")
        req.detail = dumps_json(payload.get("detail") or {})
        req.stamp_sec = time.time()
        self._event_pending += 1
        self._event_pending_mono = time.monotonic()
        future = self._event_client.call_async(req)
        self._event_pending_futures.append(future)
        future.add_done_callback(self._on_event_ack)

    def _on_event_ack(self, future) -> None:
        # 仅当该 future 仍在待确认集合中才处理：watchdog 超时后已从集合移除，迟到的回执被忽略
        if future not in self._event_pending_futures:
            return
        self._event_pending_futures.remove(future)
        self._event_pending = max(0, self._event_pending - 1)
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._record_event_ack(False, f"event ack exception: {exc}")
            return
        if resp is not None and resp.success:
            self._record_event_ack(True, resp.message or resp.ack_id or "")
        else:
            self._record_event_ack(
                False, resp.message if resp is not None else "no response"
            )

    def _record_event_ack(self, ok: bool, note: str) -> None:
        self._event_ack_ok = bool(ok)
        if not ok:
            self._event_ack_failures += 1
        self.get_logger().info(
            f"event ack {'OK' if ok else 'FAILED'} {self._last_event or ''}: {note}"
        )

    def _publish_task_status(self) -> None:
        payload = build_task_status(
            state=self._state,
            mode=self._mode,
            dock_active=self._dock_active,
            retry_count=self._staging_retry,
            retry_max=self._retry_max(),
            needs_manual_takeover=self._needs_manual_takeover,
            needs_reapproach=self._needs_reapproach,
            mission_id=self._mission_id,
            command_id=self._command_id,
            elapsed_sec=self._elapsed_sec(),
            error_message=self._last_fail_reason or None,
            camera_ready=self._camera_ready,
            event_ack_ok=self._event_ack_ok,
            event_ack_failures=self._event_ack_failures,
        )
        msg = String()
        msg.data = dumps_json(payload)
        self._task_status_pub.publish(msg)

    def _publish_legacy_status(self) -> None:
        payload = {
            "state": self._state.value,
            "staging_retry": self._staging_retry,
            "staging_retry_max": self._retry_max(),
            "nav2_profile": self._goal_switch._active,
            "use_gnss_staging": self._use_gnss,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._legacy_status_pub.publish(msg)

    def _transition(self, new_state: MissionState, note: str = "") -> None:
        if new_state == self._state:
            return
        old = self._state
        self.get_logger().info(f"mission {old.value} → {new_state.value} {note}")
        if new_state == MissionState.SETTLE:
            self._settle_elapsed = 0.0
        if new_state == MissionState.COMPLETE_SETTLE:
            self._complete_settle_elapsed = 0.0
        if new_state == MissionState.ENTRY_VALIDATE:
            self._entry_validate_result = None
            self._entry_validate_inflight = False
        self._state = new_state

    def _release_docking_session(self, *, reason: str) -> None:
        self._dock_cancel_pub.publish(Empty())
        self._goal_switch.apply_cruise()
        self._set_authority(SpeedAuthority.FAILED)
        self._pending_nav = False
        self._staging_send_inflight = False
        self._staging_override = None
        self._dock_active = False
        self.get_logger().info(f"dock session released: {reason}")

    def _begin_session(
        self,
        command: DockCommand,
        *,
        mission_id: str = "",
        command_id: str = "",
        dock_point: Optional[tuple[float, float, float]] = None,
        require_camera: bool = False,
    ) -> tuple[bool, str]:
        if command == DockCommand.CANCEL:
            return self._cancel_active_session()

        if command == DockCommand.UNDOCK:
            if self._dock_active or self._state not in (
                MissionState.IDLE,
                MissionState.SUCCEEDED,
                MissionState.FAILED,
                MissionState.CANCELLED,
            ):
                return False, f"busy in {self._state.value}"

            self._staging_retry = 0
            self._needs_manual_takeover = False
            self._needs_reapproach = False
            self._last_fail_reason = ""
            self._session_start_mono = time.monotonic()
            self._dock_active = True
            self._staging_override = None
            self._mission_id = mission_id.strip() or str(
                self.get_parameter("dock_mission_id").value
            )
            self._command_id = command_id.strip()
            self._mode = "undock"
            self._transition(MissionState.ARMED, "undock requested")
            self._emit_event(
                "UNDOCK_TASK_ACCEPTED",
                detail={"mode": self._mode},
            )
            return True, "undock task accepted"

        if self._dock_active or self._state not in (
            MissionState.IDLE,
            MissionState.SUCCEEDED,
            MissionState.FAILED,
            MissionState.CANCELLED,
        ):
            return False, f"busy in {self._state.value}"

        self._staging_retry = 0
        self._needs_manual_takeover = False
        self._needs_reapproach = False
        self._last_fail_reason = ""
        self._session_start_mono = time.monotonic()
        self._dock_active = True
        self._staging_override = dock_point
        self._mission_id = mission_id.strip() or str(self.get_parameter("dock_mission_id").value)
        self._command_id = command_id.strip()

        if command == DockCommand.ONE_CLICK_DOCK:
            self._mode = "one_click"
        elif command == DockCommand.DOCK_ONLY:
            self._mode = "dock_only"
        else:
            return False, f"unsupported command {int(command)}"

        if require_camera:
            self._camera_ready = False
            self.get_logger().info("require_camera=true (camera handshake TBD)")
            self._camera_ready = True

        self._transition(MissionState.ARMED, f"{self._mode} requested")
        self._emit_event(
            "DOCK_TASK_ACCEPTED",
            detail={"mode": self._mode, "require_camera": require_camera},
        )
        return True, f"dock task accepted mode={self._mode}"

    def _cancel_active_session(self) -> tuple[bool, str]:
        if not self._dock_active and self._state == MissionState.IDLE:
            return True, "already idle"
        self._release_docking_session(reason="cancelled")
        self._transition(MissionState.CANCELLED, "cancelled")
        self._emit_event("DOCK_CANCELLED")
        self._transition(MissionState.IDLE, "cancel settle")
        return True, "cancelled"

    def _dock_task_command_srv(self, request, response):
        cmd = int(request.command)
        try:
            command = DockCommand(cmd)
        except ValueError:
            response.success = False
            response.message = f"invalid command {cmd}"
            return response
        ok, msg = self._begin_session(
            command,
            mission_id=str(request.mission_id),
            command_id=str(request.command_id),
            require_camera=bool(request.require_camera),
        )
        response.success = ok
        response.message = msg
        return response

    def _begin_dock_mission(self) -> tuple[bool, str]:
        return self._begin_session(DockCommand.ONE_CLICK_DOCK)

    def _start_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._begin_dock_mission()
        resp.success = ok
        resp.message = msg
        return resp

    def _home_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        ok, note = self._begin_dock_mission()
        if not ok:
            self.get_logger().warning(f"ignore /dock/home: {note}")

    def _gcs_command_cb(self, msg: String) -> None:
        parsed = parse_gcs_command(msg.data)
        if parsed is None:
            self.get_logger().warning(f"ignore gcs dock command: {msg.data[:120]}")
            return
        dock_point = self._resolve_dock_point(parsed)
        if parsed.has_dock_point and dock_point is None:
            self.get_logger().warning(
                f"gcs dock command ignored (invalid dock point): {msg.data[:120]}"
            )
            return
        ok, note = self._begin_session(
            parsed.command,
            mission_id=parsed.mission_id,
            command_id=parsed.command_id,
            dock_point=dock_point,
        )
        if not ok:
            self.get_logger().warning(f"gcs dock command rejected: {note}")

    def _resolve_dock_point(
        self, parsed: GcsCommand
    ) -> Optional[tuple[float, float, float]]:
        """从 GCS 命令里提取预泊点覆盖 (lat, lon, yaw_deg)。

        只在经纬度齐全且合法时返回覆盖；缺字段或非法返回 None（沿用泊位库）。
        dock_lat/dock_lon 为 WGS84 度；**dock_yaw 为度**（接口统一用度），直接透传。
        弧度转换只在 mission_bridge 生成 Nav2 位姿时做。dock_yaw 缺失或为超范围哨兵
        （如 65536.0）时填 DOCK_YAW_UNSPECIFIED，mission_bridge 判为"不指定朝向"，
        导航按行进方向取到点朝向。
        """
        if not parsed.has_dock_point:
            return None
        lat = parsed.dock_lat
        lon = parsed.dock_lon
        assert lat is not None and lon is not None
        if not is_valid_dock_point(lat, lon):
            return None
        # 缺失 yaw → 填哨兵（不指定朝向）；有 yaw → 按度直通。
        yaw = parsed.dock_yaw if parsed.dock_yaw is not None else DOCK_YAW_UNSPECIFIED
        return (lat, lon, yaw)

    def _cancel_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._cancel_active_session()
        resp.success = ok
        resp.message = msg
        return resp

    def _staging_waypoint(self) -> tuple[float, float, float]:
        """返回 (lat, lon, yaw_deg)；优先用 GCS 预泊点覆盖，否则用泊位库。"""
        if self._staging_override is not None:
            lat, lon, yaw = self._staging_override
            self.get_logger().info(
                f"using GCS pre-dock point ({lat:.7f}, {lon:.7f}) yaw_deg={yaw:.3f}"
            )
            return lat, lon, yaw
        bay = load_bay(self._bay_id, self._dock_db_path or None)
        return make_staging_waypoint(
            bay,
            use_gnss=self._use_gnss,
            map_yaml=None if self._use_gnss else self._map_yaml,
        )

    def _start_nav_to_staging(self) -> None:
        if self._staging_send_inflight:
            return
        self._goal_switch.apply_docking()
        self._set_authority(SpeedAuthority.NAVIGATION)
        if not self._mission.wait_ready(timeout_sec=3.0):
            self._on_staging_failed("send_waypoints service unavailable")
            return
        try:
            lat, lon, yaw = self._staging_waypoint()
        except Exception as exc:  # noqa: BLE001
            self._on_staging_failed(f"staging waypoint: {exc}")
            return
        # 异步发送：定时器回调里嵌套自旋等响应在单线程 executor 下必超时
        self._staging_send_inflight = True
        self._staging_send_mono = time.monotonic()
        self._mission.send_staging_async(
            latitude=lat,
            longitude=lon,
            yaw_deg=yaw,
            mission_id=self._mission_id,
            command_id=f"retry{self._staging_retry}",
            done_cb=self._on_staging_sent,
        )
        self.get_logger().info(
            f"Nav2 staging goal sent gnss=({lat:.7f}, {lon:.7f}) "
            f"yaw_deg={yaw:.2f} mission_id={self._mission_id!r}"
        )

    def _on_staging_sent(self, ok: bool, msg: str) -> None:
        if not self._staging_send_inflight:
            return  # 会话已取消/已判失败，忽略迟到响应
        self._staging_send_inflight = False
        if self._state != MissionState.NAV_TO_STAGING:
            return
        if not ok:
            self._on_staging_failed(f"send_waypoints failed: {msg}")
            return
        self._pending_nav = True
        self._emit_event("DOCK_STAGING_STARTED")

    def _task_event_cb(self, msg: String) -> None:
        if self._state != MissionState.NAV_TO_STAGING or not self._pending_nav:
            return
        parsed = parse_task_event(msg.data)
        if parsed is None:
            return
        event, detail = parsed
        mid = str(detail.get("task_id") or detail.get("mission_id") or "")
        if mid and mid != self._mission_id and "dock" not in mid:
            return
        if event == "TASK_COMPLETED":
            self._pending_nav = False
            self._transition(MissionState.SETTLE, "nav staging completed")
        elif event == "TASK_FAILED":
            self._pending_nav = False
            reason = str(detail.get("reason") or "nav task failed")
            self._on_staging_failed(reason)

    def _dock_status_cb(self, msg: String) -> None:
        if self._state not in (
            MissionState.MONITOR_DOCK,
            MissionState.MONITOR_UNDOCK,
        ):
            return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if self._state == MissionState.MONITOR_UNDOCK:
            if status.get("undock_success"):
                self._goal_switch.apply_cruise()
                self._set_authority(SpeedAuthority.FAILED)
                self._transition(
                    MissionState.COMPLETE_SETTLE, "undock distance reached"
                )
            elif status.get("state") == "DOCK_ABORT":
                reason = str(status.get("abort_reason") or "undock failed")
                self._finish_failed(reason)
            return

        if status.get("success"):
            self._goal_switch.apply_cruise()
            self._set_authority(SpeedAuthority.FAILED)
            self._transition(MissionState.COMPLETE_SETTLE, "dock pose success")
        elif status.get("needs_reapproach"):
            self._needs_reapproach = True
            self._on_staging_failed(status.get("abort_reason", "needs_reapproach"))

    def _finish_failed(self, reason: str) -> None:
        self._last_fail_reason = reason
        self._release_docking_session(reason=reason)
        if self._staging_retry >= self._retry_max():
            self._needs_manual_takeover = True
            self._emit_event(
                "MANUAL_TAKEOVER_REQUESTED",
                detail={"reason": reason, "retry_count": self._staging_retry},
            )
        self._transition(MissionState.FAILED, reason)
        self._emit_event(
            "DOCK_FAILED",
            detail={
                "reason": reason,
                "retry_count": self._staging_retry,
                "needs_manual_takeover": self._needs_manual_takeover,
                "needs_reapproach": self._needs_reapproach,
            },
        )

    def _on_staging_failed(self, reason: str) -> None:
        self._staging_retry += 1
        self._dock_cancel_pub.publish(Empty())
        self._goal_switch.apply_cruise()
        self._set_authority(SpeedAuthority.FAILED)
        self._pending_nav = False
        self._staging_send_inflight = False
        if self._staging_retry >= self._retry_max():
            self._finish_failed(reason)
            return
        self.get_logger().warning(
            f"staging retry {self._staging_retry}/{self._retry_max()}: {reason}"
        )
        if self._mode == "dock_only":
            self._transition(MissionState.ENTRY_VALIDATE, reason)
        else:
            self._transition(MissionState.NAV_TO_STAGING, reason)

    def _finish_succeeded(self) -> None:
        self._release_docking_session(reason="dock succeeded")
        self._transition(MissionState.SUCCEEDED, "dock complete")
        self._emit_event(
            "DOCK_SUCCEEDED",
            detail={"elapsed_sec": self._elapsed_sec(), "mode": self._mode},
        )
        self._mode = None
        self._transition(MissionState.IDLE, "session closed")

    def _finish_undock_succeeded(self) -> None:
        self._release_docking_session(reason="undock succeeded")
        self._transition(MissionState.SUCCEEDED, "undock complete")
        self._emit_event(
            "UNDOCK_SUCCEEDED",
            detail={"elapsed_sec": self._elapsed_sec(), "mode": self._mode},
        )
        self._mode = None
        self._transition(MissionState.IDLE, "session closed")

    def _request_entry_validation(self) -> None:
        """发起一次异步入口验收（结果在 _on_entry_validated 回写）。

        同步阻塞版（spin_until_future_complete）在单线程 executor 的定时器
        回调里收不到响应，必超时误判失败 —— 故改异步。
        """
        if not self._validate_cli.service_is_ready():
            self.get_logger().warning("validate_entry unavailable; proceed")
            self._entry_validate_result = True
            return
        self._entry_validate_inflight = True
        self._entry_validate_mono = time.monotonic()
        future = self._validate_cli.call_async(Trigger.Request())
        future.add_done_callback(self._on_entry_validated)

    def _on_entry_validated(self, future) -> None:
        if not self._entry_validate_inflight:
            return  # 已超时/已离开状态，忽略迟到响应
        self._entry_validate_inflight = False
        resp = future.result()
        self._entry_validate_result = bool(resp.success) if resp is not None else False

    def _tick(self) -> None:
        dt = 0.2

        # 事件上报 ACK 看门狗：超过阈值仍未收到上层回执 → 记一次失败并继续（不阻塞 FSM）
        if (
            self._event_pending
            and time.monotonic() - self._event_pending_mono > self._event_ack_timeout
        ):
            # 超时未收到 ACK：取消这些在途请求，并从待确认集合移除，避免其迟到回调重复上报
            for fut in list(self._event_pending_futures):
                if not fut.done():
                    try:
                        fut.cancel()
                    except Exception:  # noqa: BLE001
                        pass
            self._event_pending_futures.clear()
            self._event_pending = 0
            self._record_event_ack(False, "event ack timeout")

        if self._state == MissionState.ARMED:
            if self._mode == "undock":
                self._transition(MissionState.UNDOCK_HANDOFF, "undock skip nav")
            elif self._mode == "dock_only":
                self._transition(MissionState.ENTRY_VALIDATE, "dock_only skip nav")
            else:
                self._transition(MissionState.NAV_TO_STAGING, "start nav staging")
                self._start_nav_to_staging()

        elif self._state == MissionState.NAV_TO_STAGING:
            # send_waypoints 响应看门狗（异步发送，响应在 done_cb 里处理）
            if (
                self._staging_send_inflight
                and time.monotonic() - self._staging_send_mono > 12.0
            ):
                self._staging_send_inflight = False
                self._on_staging_failed("send_waypoints response timeout")

        elif self._state == MissionState.SETTLE:
            self._set_authority(SpeedAuthority.STAGING_VERIFY)
            self._settle_elapsed += dt
            settle_sec = float(self.get_parameter("settle_sec").value)
            if self._settle_elapsed >= settle_sec:
                self._transition(MissionState.ENTRY_VALIDATE)

        elif self._state == MissionState.ENTRY_VALIDATE:
            if self._entry_validate_result is not None:
                if self._entry_validate_result:
                    self._transition(MissionState.DOCK_HANDOFF, "entry OK")
                else:
                    self._on_staging_failed("entry validation failed")
            elif not self._entry_validate_inflight:
                self._request_entry_validation()
            elif time.monotonic() - self._entry_validate_mono > 3.0:
                # 响应超时按失败处理（与原同步版语义一致）
                self._entry_validate_inflight = False
                self._entry_validate_result = False

        elif self._state == MissionState.DOCK_HANDOFF:
            self._set_authority(SpeedAuthority.DOCKING)
            start = Bool()
            start.data = True
            self._dock_start_pub.publish(start)
            self._emit_event("DOCK_HANDOFF")
            self._transition(MissionState.MONITOR_DOCK, "handoff usv_docking")

        elif self._state == MissionState.UNDOCK_HANDOFF:
            self._set_authority(SpeedAuthority.DOCKING)
            undock = Bool()
            undock.data = True
            self._dock_undock_pub.publish(undock)
            self._emit_event("UNDOCK_HANDOFF")
            self._transition(MissionState.MONITOR_UNDOCK, "handoff usv_docking undock")

        elif self._state == MissionState.COMPLETE_SETTLE:
            self._complete_settle_elapsed += dt
            complete_sec = float(self.get_parameter("complete_settle_sec").value)
            if self._complete_settle_elapsed >= complete_sec:
                if self._mode == "undock":
                    self._finish_undock_succeeded()
                else:
                    self._finish_succeeded()

        self._publish_task_status()
        self._publish_legacy_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._release_docking_session(reason="shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
