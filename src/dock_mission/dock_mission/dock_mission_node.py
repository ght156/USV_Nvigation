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
    DockCommand,
    build_task_event,
    build_task_status,
    dumps_json,
    parse_gcs_command,
)
from dock_mission.gnss_staging import make_staging_pose
from dock_mission.mission_client import MissionBridgeClient
from dock_mission.nav2_goal_checker import Nav2GoalCheckerSwitch
from dock_mission.task_event import parse_task_event
from dock_mission.types import MissionState, SpeedAuthority

try:
    from m_common.srv import DockTaskCommand
except ImportError:  # pragma: no cover - before colcon build
    DockTaskCommand = None  # type: ignore[misc, assignment]


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
        self._mission_id = "dock_staging"
        self._command_id = ""
        self._mode: Optional[str] = None
        self._dock_active = False
        self._needs_manual_takeover = False
        self._needs_reapproach = False
        self._last_fail_reason = ""
        self._session_start_mono: Optional[float] = None
        self._camera_ready = True

        self._goal_switch = Nav2GoalCheckerSwitch(self)
        self._mission = MissionBridgeClient(self)

        self._authority_pub = self.create_publisher(String, "/dock/speed_authority", 10)
        self._legacy_status_pub = self.create_publisher(String, "/dock/mission_status", 10)
        self._task_status_pub = self.create_publisher(
            String, self._dock_task_status_topic, 10
        )
        self._task_event_pub = self.create_publisher(
            String, self._dock_task_event_topic, 10
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
            "use_gnss_staging": False,
            "staging_retry_max": 3,
            "settle_sec": 2.5,
            "complete_settle_sec": 2.0,
            "dock_home_topic": "/dock/home",
            "gcs_dock_command_topic": "/gcs_dock/command",
            "dock_task_status_topic": "/dock_task/status",
            "dock_task_event_topic": "/dock_task/event",
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
        self._dock_task_event_topic = str(g("dock_task_event_topic").value)

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
        msg = String()
        msg.data = dumps_json(payload)
        self._task_event_pub.publish(msg)

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
        self._state = new_state

    def _release_docking_session(self, *, reason: str) -> None:
        self._dock_cancel_pub.publish(Empty())
        self._goal_switch.apply_cruise()
        self._set_authority(SpeedAuthority.FAILED)
        self._pending_nav = False
        self._dock_active = False
        self.get_logger().info(f"dock session released: {reason}")

    def _begin_session(
        self,
        command: DockCommand,
        *,
        mission_id: str = "",
        command_id: str = "",
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
        command, mission_id, command_id = parsed
        ok, note = self._begin_session(
            command,
            mission_id=mission_id,
            command_id=command_id,
        )
        if not ok:
            self.get_logger().warning(f"gcs dock command rejected: {note}")

    def _cancel_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._cancel_active_session()
        resp.success = ok
        resp.message = msg
        return resp

    def _staging_pose(self):
        bay = load_bay(self._bay_id, self._dock_db_path or None)
        return make_staging_pose(
            bay,
            use_gnss=self._use_gnss,
            map_yaml=self._map_yaml if self._use_gnss else None,
        )

    def _start_nav_to_staging(self) -> None:
        self._goal_switch.apply_docking()
        self._set_authority(SpeedAuthority.NAVIGATION)
        if not self._mission.wait_ready(timeout_sec=3.0):
            self._on_staging_failed("send_waypoints service unavailable")
            return
        try:
            pose = self._staging_pose()
        except Exception as exc:  # noqa: BLE001
            self._on_staging_failed(f"staging pose: {exc}")
            return
        ok, msg = self._mission.send_staging(
            pose, mission_id=self._mission_id, command_id=f"retry{self._staging_retry}"
        )
        if not ok:
            self._on_staging_failed(f"send_waypoints failed: {msg}")
            return
        self._pending_nav = True
        self._emit_event("DOCK_STAGING_STARTED")
        self.get_logger().info(
            f"Nav2 staging goal sent map=({pose.pose.position.x:.2f}, "
            f"{pose.pose.position.y:.2f}) mission_id={self._mission_id!r}"
        )

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

    def _call_validate_entry(self) -> bool:
        if not self._validate_cli.service_is_ready():
            self.get_logger().warning("validate_entry unavailable; proceed")
            return True
        req = Trigger.Request()
        future = self._validate_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return False
        return bool(future.result().success)

    def _tick(self) -> None:
        dt = 0.2

        if self._state == MissionState.ARMED:
            if self._mode == "undock":
                self._transition(MissionState.UNDOCK_HANDOFF, "undock skip nav")
            elif self._mode == "dock_only":
                self._transition(MissionState.ENTRY_VALIDATE, "dock_only skip nav")
            else:
                self._transition(MissionState.NAV_TO_STAGING, "start nav staging")
                self._start_nav_to_staging()

        elif self._state == MissionState.SETTLE:
            self._set_authority(SpeedAuthority.STAGING_VERIFY)
            self._settle_elapsed += dt
            settle_sec = float(self.get_parameter("settle_sec").value)
            if self._settle_elapsed >= settle_sec:
                self._transition(MissionState.ENTRY_VALIDATE)

        elif self._state == MissionState.ENTRY_VALIDATE:
            if self._call_validate_entry():
                self._transition(MissionState.DOCK_HANDOFF, "entry OK")
            else:
                self._on_staging_failed("entry validation failed")

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
