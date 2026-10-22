"""Dock mission FSM: upper-layer service + GCS topic → staging → handoff → dock_task feedback."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, String
from std_srvs.srv import Trigger

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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
    from m_common.srv import DockTaskEvent  # /dock_task/event 上报 service
except ImportError:  # pragma: no cover - before colcon build
    DockTaskEvent = None  # type: ignore[misc, assignment]

try:
    from m_common.srv import SetChannelEnabled  # apriltag 检测通道开关 service
except ImportError:  # pragma: no cover - before colcon build
    SetChannelEnabled = None  # type: ignore[misc, assignment]

try:
    from m_common.srv import QueryInDock  # /dock/in_dock 出泊前"船是否在坞内"查询
except ImportError:  # pragma: no cover - before colcon build
    QueryInDock = None  # type: ignore[misc, assignment]

try:
    from usv_rs485_driver.srv import SetDO  # 船坞背光板（DO bit1），船端包
except ImportError:  # pragma: no cover - 主机/仿真无此包
    SetDO = None  # type: ignore[misc, assignment]

try:
    from usv_rs485_driver.action import ControlDO  # 只开灯不松闸（mask=2）
except ImportError:  # pragma: no cover - 主机/仿真无此包
    ControlDO = None  # type: ignore[misc, assignment]


class DockMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("dock_mission_node")
        self._declare_parameters()
        self._load_parameters()

        self._state = MissionState.IDLE
        self._staging_retry = 0
        self._settle_elapsed = 0.0
        self._settle_still_hold = 0.0
        self._complete_settle_elapsed = 0.0
        self._pending_nav = False
        self._staging_send_inflight = False
        self._staging_send_mono = 0.0
        # 预泊下发尝试序号：每次 _start_nav_to_staging +1，done_cb 据此丢弃
        # 上一轮（已超时/已重发）的迟到响应（2026-10-13 二审 P0-1）
        self._staging_attempt = 0
        self._mission_id = "dock_staging"
        self._command_id = ""
        self._staging_override: Optional[tuple[float, float, float]] = None
        self._mode: Optional[str] = None
        self._dock_active = False
        self._needs_manual_takeover = False
        self._needs_reapproach = False
        self._last_fail_reason = ""
        self._session_start_mono: Optional[float] = None
        # 相机握手未实现：camera_ready 恒 True 仅为 status schema 兼容（嵌软可能在读），
        # 勿当作相机就绪门
        self._camera_ready = True
        # SETTLE 实测停稳门的速度来源（/mavros/gps_input/local，与 docking_fsm 一致）
        self._odom_speed: Optional[float] = None
        self._odom_yaw_rate: Optional[float] = None
        self._odom_speed_stamp = None
        # /dock/in_dock 查询用的坞系位姿缓存（docking_pose_estimator 输出：
        # base_link 在 dock_est 系的平面位姿 + 来源分级）
        self._dock_pose_xy: Optional[tuple[float, float]] = None
        self._dock_pose_stamp = None
        self._dock_pose_source = ""
        # 估计器的“距最近视觉观测秒数”（Float32，inf=从未见过 Tag）。用于限制
        # ODOM_PREDICTION 的可用龄期，避免相机掉线后拿很旧的视觉判在坞。
        self._dock_measurement_age: Optional[float] = None
        self._dock_measurement_age_stamp = None
        # usv_docking 背光会话标志（/dock/status 缓存）：查询据此避免互抢
        self._fsm_backlight_active = False
        # 停稳门（2026-10-13 三审 P1）：按连续有效观测的实际时间累计，
        # 与发布频率解耦。_settle_hold_last_stamp=已处理的最新帧戳（判新帧）；
        # start/last/frames=当前连续停稳段的首帧戳/末帧戳/有效帧数
        self._settle_hold_last_stamp = None
        self._settle_still_start_stamp = None
        self._settle_still_last_stamp = None
        self._settle_still_frames = 0
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
        # 事件上报 client：船端(dock_mission) → 上层状态机(嵌软)，等待 ACK
        self._event_client = None
        if DockTaskEvent is not None:
            self._event_client = self.create_client(
                DockTaskEvent, self._dock_task_event_service
            )

        # apriltag 检测通道开关 client：进 SETTLE / 出泊交接时开启，
        # 归港/出泊成功收尾关闭（2026-10-13）
        self._apriltag_channel_client = None
        apriltag_srv = str(
            self.get_parameter("apriltag_channel_service").value
        ).strip()
        if SetChannelEnabled is not None and apriltag_srv:
            self._apriltag_channel_client = self.create_client(
                SetChannelEnabled, apriltag_srv
            )

        # 船坞背光板：点亮走 ControlDO action（start=true, mask=2，只开灯不松闸，
        # 2026-10-20）；关闭仍走 SetDO 全掩码写 0（查询结束恢复）。
        self._set_do_client = None
        set_do_srv = str(self.get_parameter("set_do_service").value).strip()
        if SetDO is not None and set_do_srv:
            self._set_do_client = self.create_client(SetDO, set_do_srv)

        self._ControlDO = ControlDO
        self._control_do_client = None
        if ControlDO is not None:
            self._control_do_client = ActionClient(
                self,
                ControlDO,
                str(self.get_parameter("control_do_action").value),
            )

        # SETTLE 停稳门的速度来源（与 docking_fsm 同话题同 QoS，RELIABLE）
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_speed_topic").value),
            self._odom_speed_cb,
            10,
        )

        # 二维码定位链输出（/dock/in_dock 查询的数据源）
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("dock_pose_topic").value),
            self._dock_pose_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("dock_pose_source_topic").value),
            self._dock_pose_source_cb,
            10,
        )
        # 视觉观测龄期：用于限制 ODOM_PREDICTION 的可用龄期
        self.create_subscription(
            Float32,
            str(self.get_parameter("dock_measurement_age_topic").value),
            self._dock_measurement_age_cb,
            10,
        )

        # 出泊前"船是否在坞内"查询（上层/嵌软 client → 本节点 server，
        # 2026-10-15）。handler 可能阻塞数秒等视觉锚定，放独立互斥回调组，
        # main() 用多线程 executor 保证不饿死默认组的定时器/订阅回调。
        in_dock_srv_name = str(self.get_parameter("in_dock_service").value).strip()
        if QueryInDock is not None and in_dock_srv_name:
            self.create_service(
                QueryInDock,
                in_dock_srv_name,
                self._in_dock_srv,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

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
            # 默认拒绝上层/GCS 下发的预泊点，始终使用 dock_database.yaml 的本地标定点。
            # 仅联调需要临时接受 dock_lat/dock_lon/dock_yaw 覆盖时才显式设为 True。
            "accept_gcs_staging_override": False,
            "staging_retry_max": 5,  # 2026-10-06 现场要求
            "settle_sec": 2.5,
            "complete_settle_sec": 2.0,
            # SETTLE 实测停稳门（2026-10-13 源码审查 P0）：原实现是纯延时交接，
            # 船还可能带着风/流残余速度就启动精靠泊。开启后要求速度源新鲜且
            # 实测速度 <= settle_max_speed 持续 settle_still_hold_sec 才交接；
            # 一直停不稳/速度源缺测超 settle_timeout_sec 走 staging 重试管线。
            # 阈值与 docking_fsm 的 approach_stop_speed/hold_sec 对齐。
            "settle_require_still": True,  # false 回退纯 settle_sec 延时
            "odom_speed_topic": "/mavros/gps_input/local",
            "settle_max_speed": 0.15,
            "settle_max_yaw_rate": 0.1,  # rad/s：原地偏航未稳也不算停稳（2026-10-13 二审 P1-4）
            "settle_still_hold_sec": 0.5,
            # 停稳门频率兼容（2026-10-13 三审 P1）：停稳时长按连续有效观测的
            # 实际时间累计；相邻帧间隔或无新帧沉默超 settle_max_frame_gap_sec
            # 才清零重计（默认 1s，2Hz 低频里程计可用），且至少
            # settle_min_still_frames 帧有效观测才放行，防单帧零速凑数
            "settle_max_frame_gap_sec": 1.0,
            "settle_min_still_frames": 3,
            "settle_timeout_sec": 15.0,
            "dock_home_topic": "/dock/home",
            "gcs_dock_command_topic": "/gcs_dock/command",
            "dock_task_status_topic": "/dock_task/status",
            "dock_task_event_service": "/dock_task/event",
            "dock_task_event_log_topic": "/dock_task/event_log",
            "event_ack_timeout": 3.0,
            # 到达预泊点（进入 SETTLE）异步开启后视相机二维码检测通道
            #（2026-10-13）：巡航段关闭检测省算力，快到/到达预泊点才打开。
            # 置空服务名 = 关闭本功能（仿真/无相机环境）
            "apriltag_channel_service": "/apriltag_node/set_channel_enabled",
            "apriltag_channel": "all",  # 通道名；"all"/空 = 全部通道
            # 出泊前"船是否在坞内"查询 service（2026-10-15）：上层（嵌软）出泊前
            # 调用，用二维码定位链 /docking/dock_pose（base_link 在 dock 系位姿，
            # 坞中心/充电位为原点，+x 朝坞内，坞外 x<0）+ /docking/pose_source
            # 做几何判定。置空服务名 = 不注册本 service。
            "in_dock_service": "/dock/in_dock",
            "dock_pose_topic": "/docking/dock_pose",
            "dock_pose_source_topic": "/docking/pose_source",
            "dock_measurement_age_topic": "/docking/measurement_age",
            # 在坞区域（dock 系，米）：x∈[x_min,x_max] 且 |y|<=y_max 判在坞内。
            # 停稳充电位 x≈0（容差 0.15/0.10，见 docking.yaml docked_*_tol），
            # 出泊完成 x≈-3（undock_complete_x）；半入坞也算在坞内（出泊可行）。
            "in_dock_x_min": -1.5,
            "in_dock_x_max": 1.0,
            "in_dock_y_max": 0.6,
            # 缓存位姿最大龄期：estimator 正常 20Hz 恒发，超此视为缺测
            "in_dock_pose_max_age_sec": 1.5,
            # 在坞判定对 ODOM_PREDICTION 的额外限制：相机掉线后锚点死推算会
            # 持续 70s，不能拿“10 秒前的最后一次视觉”判是否在坞。推算龄期
            # 超此值即视为不可用（返回 INVALID，等相机恢复/人工）；VISION 不受限。
            "in_dock_prediction_max_age_sec": 3.0,
            # wait_vision=true 且位姿无效时，临时打开二维码检测通道等视觉锚定的
            # 上限（首锚需 8 帧共识，秒级）
            "in_dock_vision_timeout_sec": 6.0,
            # 查询结束后是否恢复关闭由本查询打开的检测通道与背光
            #（会话进行中不动通道/背光）
            "in_dock_restore_channel": True,
            # 船坞背光板：点亮走 ControlDO action start=true + mask=2（只开灯
            # 不松闸，2026-10-20）；关闭走 SetDO service 写 0。等视觉锚定前
            # 点亮，否则黑天 Tag 不可见永远等不到锚。
            # 置空服务名/action 名或掩码 0 = 查询不控制背光（仿真/无背光船坞）
            "set_do_service": "set_do",
            # 点亮背光用 ControlDO action（mask=2 只开灯不松闸）；关闭用 SetDO 0
            "control_do_action": "control_do",
            "in_dock_backlight_mask": 2,
            "send_waypoints_service": "/mission_bridge/send_waypoints",
            "cancel_mission_service": "/mission_bridge/cancel_mission",
            "nav2_controller_node": "/controller_server",
            "goal_checker_selector_topic": "goal_checker_selector",
            "use_goal_checker_selector": True,
            "cruise_goal_checker_id": "general_goal_checker",
            "docking_goal_checker_id": "docking_goal_checker",
            # 2026-10-19/20：与 nav2_params_xingfei.yaml 同步——巡航 2.0（防到点转圈），
            # 归港预泊 1.0/0.4。运行时本节点会 set_parameters 覆盖 controller_server
            # 的同名 goal_checker 参数，所以这里必须与 yaml 保持一致，否则改 yaml 不生效。
            "cruise_xy_goal_tolerance": 2.0,
            "cruise_yaw_goal_tolerance": 1.0,
            "docking_xy_goal_tolerance": 1.0,
            "docking_yaw_goal_tolerance": 0.4,
            "dock_mission_id": "dock_staging",
        }
        for name, value in defaults.items():
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

                self._map_yaml = Path(get_package_share_directory("workspace_nav")) / "config" / "map.yaml"
            except Exception:
                self._map_yaml = Path("")
        self._use_gnss = bool(g("use_gnss_staging").value)
        self._accept_gcs_staging_override = bool(
            g("accept_gcs_staging_override").value
        )
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
            odom_speed=self._odom_speed,
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
            self._settle_still_hold = 0.0
            self._settle_hold_last_stamp = None
            self._settle_still_start_stamp = None
            self._settle_still_last_stamp = None
            self._settle_still_frames = 0
            # 到达预泊点：开启后视相机二维码检测，给交接精靠泊预热视觉
            self._set_apriltag_channels(True, reason="到达预泊点")
        if new_state == MissionState.UNDOCK_HANDOFF:
            # 出泊交接：同样开启检测（出泊后常紧接归港，且坞内段可用视觉辅助）
            self._set_apriltag_channels(True, reason="出泊交接")
        if new_state == MissionState.DOCK_HANDOFF:
            # 交接精靠泊：开启检测。dock_only 直连不走 SETTLE（没有预热点），必须在这里开，
            # 否则 usv_docking 进 ACQUIRE_TAG 时通道还是关的 → 一直 ACQUIRE_TIMEOUT
            # （2026-10-13 现场踩到）；nav 路径 SETTLE 已开过，这里幂等重申，
            # 预泊/交接失败重试回 DOCK_HANDOFF 时也能补开。
            self._set_apriltag_channels(True, reason="交接精靠泊")
        if new_state == MissionState.COMPLETE_SETTLE:
            self._complete_settle_elapsed = 0.0
        self._state = new_state

    def _set_apriltag_channels(self, enabled: bool, *, reason: str) -> None:
        """异步开关 apriltag 检测通道（2026-10-13）。

        开启等效：ros2 service call /apriltag_node/set_channel_enabled
        m_common/srv/SetChannelEnabled "{channel: 'all', enabled: true}"。
        开：进入 SETTLE（到达预泊点）/ DOCK_HANDOFF（交接精靠泊，dock_only 直连
        只经过这里）/ UNDOCK_HANDOFF（出泊交接）；
        关：归港/出泊成功收尾。服务不在线只告警不阻塞 FSM——主流程不依赖
        本次调用的回执，节点随后上线时由下一次触发点补发。"""
        if self._apriltag_channel_client is None:
            return
        if not self._apriltag_channel_client.service_is_ready():
            self.get_logger().warn(
                "apriltag 通道开关服务暂不在线，仍发起异步调用（结果见后续日志）"
            )
        req = SetChannelEnabled.Request()
        req.channel = str(self.get_parameter("apriltag_channel").value)
        req.enabled = enabled
        fut = self._apriltag_channel_client.call_async(req)
        fut.add_done_callback(
            lambda f, e=enabled, r=reason: self._apriltag_channel_done(f, e, r)
        )

    def _apriltag_channel_done(self, fut, enabled: bool, reason: str) -> None:
        action = "开启" if enabled else "关闭"
        try:
            resp = fut.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"apriltag 通道{action}调用失败({reason}): {exc}")
            return
        if resp.success:
            self.get_logger().info(
                f"apriltag 检测通道已{action}({reason}): {resp.message}"
            )
        else:
            self.get_logger().warn(f"apriltag 通道{action}被拒({reason}): {resp.message}")

    # ── /dock/in_dock：出泊前"船是否在坞内"查询（2026-10-15）──

    def _dock_pose_cb(self, msg: PoseStamped) -> None:
        self._dock_pose_xy = (msg.pose.position.x, msg.pose.position.y)
        self._dock_pose_stamp = self.get_clock().now()

    def _dock_pose_source_cb(self, msg: String) -> None:
        self._dock_pose_source = msg.data

    def _dock_measurement_age_cb(self, msg: Float32) -> None:
        age = float(msg.data)
        # NaN 不采信；inf 是估计器“从未见过 Tag”的合法表达
        if math.isnan(age):
            return
        self._dock_measurement_age = age
        self._dock_measurement_age_stamp = self.get_clock().now()

    def _measurement_age_usable(self) -> Optional[float]:
        """返回新鲜的 measurement_age；缺测/过期返回 None。"""
        if self._dock_measurement_age_stamp is None:
            return None
        age = (
            self.get_clock().now() - self._dock_measurement_age_stamp
        ).nanoseconds * 1e-9
        if age > float(self.get_parameter("in_dock_pose_max_age_sec").value):
            return None
        return self._dock_measurement_age

    def _estimator_alive(self) -> bool:
        """估计器是否在发数据（即使 source=INVALID 也会 20Hz 发布）。"""
        if self._dock_pose_stamp is None:
            return False
        age = (self.get_clock().now() - self._dock_pose_stamp).nanoseconds * 1e-9
        return age <= float(self.get_parameter("in_dock_pose_max_age_sec").value)

    def _in_dock_pose_usable(self) -> bool:
        """坞系位姿是否可用于在坞判定：来源有效（VISION/ODOM_PREDICTION）且新鲜。

        ODOM_PREDICTION 额外受 in_dock_prediction_max_age_sec 限制：相机掉线后
        锚点死推算可持续 70s，不能拿很旧的视觉观测判“是否在坞”。"""
        if self._dock_pose_xy is None or self._dock_pose_stamp is None:
            return False
        if self._dock_pose_source not in ("VISION", "ODOM_PREDICTION"):
            return False
        age = (self.get_clock().now() - self._dock_pose_stamp).nanoseconds * 1e-9
        if age > float(self.get_parameter("in_dock_pose_max_age_sec").value):
            return False
        if self._dock_pose_source == "ODOM_PREDICTION":
            measurement_age = self._measurement_age_usable()
            if measurement_age is not None and math.isfinite(measurement_age):
                if measurement_age > float(
                    self.get_parameter("in_dock_prediction_max_age_sec").value
                ):
                    return False
        return True

    def _set_dock_backlight(self, on: bool, *, reason: str) -> bool:
        """点亮/关闭船坞背光（异步 fire-and-forget）。返回是否实际发起了写。

        点亮：ControlDO action start=true + mask=2（只写 bit1 背光，不驱动夹爪）。
        关闭：SetDO service 写 0（16 位全掩码；查询只在无会话时动 DO，夹爪本应 0）。
        """
        mask = int(self.get_parameter("in_dock_backlight_mask").value) & 0xFFFF
        if mask == 0:
            return False
        if on:
            if self._control_do_client is None or self._ControlDO is None:
                return False
            if not self._control_do_client.server_is_ready():
                self.get_logger().warn(
                    "control_do action server 不在线，无法点背光"
                    "（需 usv_rs485_driver 最新版本）"
                )
                return False
            goal = self._ControlDO.Goal()
            if not hasattr(goal, "mask"):
                self.get_logger().warn(
                    "当前 usv_rs485_driver ControlDO 无 mask 字段（旧版本），"
                    "无法只开灯不松闸"
                )
                return False
            goal.start = True
            goal.mask = mask
            future = self._control_do_client.send_goal_async(goal)
            future.add_done_callback(
                lambda fut, m=mask: self._on_backlight_goal(fut, m)
            )
            self.get_logger().info(
                f"船坞背光点亮({reason}): ControlDO mask=0x{mask:04X}"
                "（只开灯，不驱动夹爪）"
            )
            return True

        if self._set_do_client is None:
            return False
        if not self._set_do_client.service_is_ready():
            self.get_logger().warn(
                "set_do 服务暂不在线，仍发起异步调用（关背光，结果见驱动日志）"
            )
        req = SetDO.Request()
        req.digital_output = 0
        self._set_do_client.call_async(req)
        self.get_logger().info(f"船坞背光关闭({reason}): DO 0x0000")
        return True

    def _on_backlight_goal(self, future, mask: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"ControlDO 点背光 0x{mask:04X} 发送失败: {exc}")
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(
                f"ControlDO 点背光 0x{mask:04X} 被拒（驱动可能有在途 goal）"
            )

    def _await_vision_pose(self) -> tuple[bool, bool]:
        """临时打开二维码检测通道 + 点亮船坞背光，等视觉锚定（阻塞，秒级）。

        返回 (opened_channel, lit_backlight)，查完由调用方按
        in_dock_restore_channel 恢复关闭。会话进行中不动通道/背光——
        归港/出泊流程自己管理。"""
        if self._dock_active or self._apriltag_channel_client is None:
            return False, False
        # 黑天下 Tag 靠船坞背光板照亮：先点灯再开检测。
        # usv_docking 背光会话活跃（归港失败待重试等）时不碰 DO：
        # 查询收尾的 0x0000 全掩码写会把 FSM 为重试保持的灯误关
        lit_backlight = (
            False
            if self._fsm_backlight_active
            else self._set_dock_backlight(True, reason="在坞查询")
        )
        self._set_apriltag_channels(True, reason="在坞查询")
        deadline = time.monotonic() + float(
            self.get_parameter("in_dock_vision_timeout_sec").value
        )
        while time.monotonic() < deadline:
            if self._in_dock_pose_usable():
                return True, lit_backlight
            time.sleep(0.1)
        self.get_logger().warning("在坞查询：等视觉锚定超时，位姿仍无效")
        return True, lit_backlight

    def _in_dock_srv(self, req, resp):
        """出泊前查询：二维码定位判 base_link 是否在坞内（可能阻塞等视觉锚定）。

        只返回既有的四种来源（VISION / ODOM_PREDICTION / INVALID / NONE），
        不额外上报相机故障：相机故障由上层状态机通过 /sensor_health 自己判。
        无有效位姿时 in_dock=false、pose_valid=false；上层结合自己的相机健康
        决定“坞外”还是“故障”。
        """
        opened_channel = False
        lit_backlight = False
        try:
            if not self._in_dock_pose_usable() and bool(req.wait_vision):
                opened_channel, lit_backlight = self._await_vision_pose()
        finally:
            if not self._dock_active and bool(
                self.get_parameter("in_dock_restore_channel").value
            ):
                if lit_backlight:
                    self._set_dock_backlight(False, reason="在坞查询收尾")
                if opened_channel:
                    self._set_apriltag_channels(False, reason="在坞查询收尾")

        resp.pose_valid = self._in_dock_pose_usable()
        resp.pose_source = self._dock_pose_source or "NONE"
        if resp.pose_valid:
            x, y = self._dock_pose_xy
            x_min = float(self.get_parameter("in_dock_x_min").value)
            x_max = float(self.get_parameter("in_dock_x_max").value)
            y_max = float(self.get_parameter("in_dock_y_max").value)
            resp.dock_x = x
            resp.dock_y = y
            resp.in_dock = x_min <= x <= x_max and abs(y) <= y_max
            resp.message = (
                f"dock系({x:.2f},{y:.2f}) 区域x∈[{x_min},{x_max}] |y|<={y_max} "
                f"→ {'在坞内' if resp.in_dock else '不在坞内'}"
            )
        else:
            resp.in_dock = False
            resp.dock_x = 0.0
            resp.dock_y = 0.0
            measurement_age = self._measurement_age_usable()
            channel_ready = (
                self._apriltag_channel_client is not None
                and self._apriltag_channel_client.service_is_ready()
            )
            if resp.pose_source in ("VISION", "ODOM_PREDICTION"):
                resp.message = (
                    f"坞系位姿过期/不可用(pose_source={resp.pose_source})，"
                    "in_dock 不可信"
                )
            elif (
                bool(req.wait_vision)
                and opened_channel
                and channel_ready
                and self._estimator_alive()
                and measurement_age is not None
                and math.isinf(measurement_age)
            ):
                # 检测节点正常、确实开了检测，但从未见到 Tag → 按坞外处理。
                # 相机是否掉线由上层状态机用 /sensor_health 自己判。
                resp.message = (
                    "检测节点正常、已开启检测但未检测到 Tag；判定不在坞内"
                    "（相机故障由上层状态机判定）"
                )
            elif bool(req.wait_vision) and not (
                channel_ready and self._estimator_alive()
            ):
                resp.message = (
                    "检测节点异常/未就绪（apriltag 服务或估计器不在线），"
                    "按失败处理，in_dock 不可信"
                )
            elif not bool(req.wait_vision):
                resp.message = (
                    "未尝试开启检测（wait_vision=false），in_dock 不可信；"
                    "出泊/派航线前请用 wait_vision=true"
                )
            else:
                resp.message = (
                    f"无有效坞系位姿(pose_source={resp.pose_source})，"
                    "in_dock 不可信"
                )
        self.get_logger().info(f"in_dock 查询: {resp.message}")
        return resp

    def _release_docking_session(
        self,
        *,
        reason: str,
        authority: SpeedAuthority = SpeedAuthority.RELEASE,
        end_session: bool = True,
    ) -> None:
        """释放归港会话占用的下游资源（取消 docking、切回巡航容差、发速度让权）。

        authority：RELEASE=正常收尾（成功/取消/关闭）让权；FAILED=任务失败收场。
        end_session=False 用于预泊失败重试：会话仍存活（_dock_active 保持 True），
        且 GCS 预泊点覆盖（_staging_override）必须保留——清掉会让重试退回泊位库
        默认点，与"同点重试"语义不符。

        撤销 Nav2 goal 只在 end_session=True 且 staging 导航可能还在跑时发：
        重试路径（end_session=False）不发——重试的 send_waypoints 会在
        mission_bridge 内抢占替换旧 goal（for_replace=True），显式 cancel 若晚于
        新 goal 到达会把新 goal 一起撤掉。
        """
        if end_session and (
            self._pending_nav
            or self._staging_send_inflight
            or self._state == MissionState.NAV_TO_STAGING
        ):
            self._cancel_staging_nav(reason)
        self._dock_cancel_pub.publish(Empty())
        self._goal_switch.apply_cruise()
        self._set_authority(authority)
        self._pending_nav = False
        self._staging_send_inflight = False
        if end_session:
            self._staging_override = None
            self._dock_active = False
        self.get_logger().info(f"dock session released: {reason}")

    def _cancel_staging_nav(self, reason: str) -> None:
        """让 mission_bridge 撤销已下发的 Nav2 FollowWaypoints goal。

        取消失败只记日志，不阻塞主流程（桥侧对 IDLE 幂等，重复取消安全）。
        """

        def _done(ok: bool, msg: str) -> None:
            log = self.get_logger().info if ok else self.get_logger().warning
            log(f"cancel_mission [{reason}]: {'OK' if ok else 'FAILED'} {msg}")

        self._mission.cancel_async(done_cb=_done)

    def _begin_session(
        self,
        command: DockCommand,
        *,
        mission_id: str = "",
        command_id: str = "",
        dock_point: Optional[tuple[float, float, float]] = None,
    ) -> tuple[bool, str]:
        if command == DockCommand.CANCEL:
            return self._cancel_active_session()

        if not self._accepting_new_task():
            return False, f"busy in {self._state.value}"

        if command == DockCommand.UNDOCK:
            mode = "undock"
            dock_point = None  # 出泊不导航，忽略预泊点覆盖
        elif command == DockCommand.ONE_CLICK_DOCK:
            mode = "one_click"
        elif command == DockCommand.DOCK_ONLY:
            mode = "dock_only"
        else:
            return False, f"unsupported command {int(command)}"

        self._arm_session(
            mission_id=mission_id,
            command_id=command_id,
            mode=mode,
            dock_point=dock_point,
        )
        self._transition(MissionState.ARMED, f"{mode} requested")
        detail = {"mode": mode}
        if mode != "undock":
            self._emit_event("DOCK_TASK_ACCEPTED", detail=detail)
            return True, f"dock task accepted mode={mode}"
        self._emit_event("UNDOCK_TASK_ACCEPTED", detail=detail)
        return True, "undock task accepted"

    def _accepting_new_task(self) -> bool:
        return not self._dock_active and self._state in (
            MissionState.IDLE,
            MissionState.SUCCEEDED,
            MissionState.FAILED,
            MissionState.CANCELLED,
        )

    def _arm_session(
        self,
        *,
        mission_id: str,
        command_id: str,
        mode: str,
        dock_point: Optional[tuple[float, float, float]] = None,
    ) -> None:
        """新会话公共初始化：清上一轮重试/失败残留，开计时，登记 id 与模式。"""
        self._staging_retry = 0
        self._needs_manual_takeover = False
        self._needs_reapproach = False
        self._last_fail_reason = ""
        self._session_start_mono = time.monotonic()
        self._dock_active = True
        self._staging_override = dock_point
        self._mission_id = mission_id.strip() or str(
            self.get_parameter("dock_mission_id").value
        )
        self._command_id = command_id.strip()
        self._mode = mode

    def _cancel_active_session(self) -> tuple[bool, str]:
        if not self._dock_active and self._state == MissionState.IDLE:
            return True, "already idle"
        self._release_docking_session(reason="cancelled")
        self._transition(MissionState.CANCELLED, "cancelled")
        self._emit_event("DOCK_CANCELLED")
        self._transition(MissionState.IDLE, "cancel settle")
        return True, "cancelled"

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
        if (
            self._accept_gcs_staging_override
            and parsed.has_dock_point
            and dock_point is None
        ):
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
        """从 GCS 命令里提取可选的预泊点覆盖 (lat, lon, yaw_deg)。

        默认配置 `accept_gcs_staging_override=false`，完全不接受状态机/GCS
        下发的预泊点，始终回退 dock_database.yaml 的本地标定点。合法、非法
        或 65536 哨兵均不会影响归港指令受理。

        只在经纬度齐全且合法时返回覆盖；缺字段（或 null/非数/65536 哨兵）返回 None
        （沿用泊位库）。65536 哨兵在 parse_gcs_command 已归一化为 None，因此
        `has_dock_point` 为 false，不会走“非法丢弃”分支。
        字段齐全但非法（(0,0)/真正越界，如 lat=91）同样返回 None，由调用方据此
        丢弃整条指令。
        dock_lat/dock_lon 为 WGS84 度；**dock_yaw 为度**（接口统一用度），直接透传。
        弧度转换只在 mission_bridge 生成 Nav2 位姿时做。dock_yaw 缺失或为超范围哨兵
        （如 65536.0）时填 DOCK_YAW_UNSPECIFIED，mission_bridge 判为"不指定朝向"，
        导航按行进方向取到点朝向。
        dock_yaw 是**预泊点处船艏航向**（ENU/map 系，CCW，0°=东）：船尾朝坞 ⇒
        该值为船艏朝坞外的航向（详见 dock_task_api.GcsCommand docstring）。
        """
        if not self._accept_gcs_staging_override:
            return None
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
        """返回 (lat, lon, yaw_deg)；

        默认只用本地泊位库；只有显式开启 `accept_gcs_staging_override` 且收到
        合法覆盖时，才使用 GCS 预泊点。
        """
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
        self._staging_attempt += 1
        attempt = self._staging_attempt
        self._staging_send_inflight = True
        self._staging_send_mono = time.monotonic()
        self._mission.send_staging_async(
            latitude=lat,
            longitude=lon,
            yaw_deg=yaw,
            mission_id=self._mission_id,
            command_id=f"retry{self._staging_retry}",
            done_cb=lambda ok, msg, a=attempt: self._on_staging_sent(ok, msg, a),
        )
        self.get_logger().info(
            f"Nav2 staging goal sent gnss=({lat:.7f}, {lon:.7f}) "
            f"yaw_deg={yaw:.2f} mission_id={self._mission_id!r} attempt={attempt}"
        )

    def _on_staging_sent(self, ok: bool, msg: str, attempt: int) -> None:
        if attempt != self._staging_attempt or not self._staging_send_inflight:
            # 上一轮（已超时/已重发）或已取消会话的迟到响应：不消费。但若 bridge
            # 实际受理了该次下发（ok），此前 release 发出的 cancel 可能先于 send
            # 到达 bridge 而落空，这里补一次取消，确保 Nav2 goal 被撤销。
            # 注意：重发后旧响应到达时 state 是 NAV_TO_STAGING 且新一轮 inflight，
            # bridge 侧旧 goal 已被新 send 抢占替换（for_replace），无需补 cancel。
            if ok and self._state != MissionState.NAV_TO_STAGING:
                self._cancel_staging_nav("late staging response")
            return
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
        # 严格关联（2026-10-13 二审 P1-7）：mission_bridge 会回传 task_id=
        # 我们下发的 mission_id + command_id=retryN；带 id 但不匹配的一律丢弃
        #（旧的子串放行会让任何含 "dock" 的别家任务事件污染本会话）。
        # 空 mid 的 legacy 事件保留兼容。
        if mid and mid != self._mission_id:
            return
        cid = str(detail.get("command_id") or "")
        if cid and cid != f"retry{self._staging_retry}":
            # 上一轮重试的残留事件（bridge 侧旧 goal 的迟报）：不属于本次 attempt
            self.get_logger().warning(
                f"忽略残留任务事件 {event}（command_id={cid!r}，"
                f"当前 retry{self._staging_retry}）"
            )
            return
        if event == "TASK_COMPLETED":
            self._pending_nav = False
            self._transition(MissionState.SETTLE, "nav staging completed")
        elif event == "TASK_FAILED":
            self._pending_nav = False
            reason = str(detail.get("reason") or "nav task failed")
            self._on_staging_failed(reason)

    def _dock_status_cb(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # 不论本侧状态都缓存 usv_docking 的背光会话标志：在坞查询据此
        # 避免与 FSM 的背光管理互抢（FSM 会话期间查询不写 DO）
        self._fsm_backlight_active = bool(status.get("backlight_session_active"))
        if self._state not in (
            MissionState.MONITOR_DOCK,
            MissionState.MONITOR_UNDOCK,
        ):
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
        elif status.get("needs_manual_takeover"):
            # FSM 判定禁止自动重试（2026-10-09 夹爪半抓住保护：爪子已扣住船体，
            # 驶出/重试会拉拽船坞）。FAILED 的状态 JSON 里 needs_reapproach 被
            # 强制 true（防 MONITOR_DOCK 挂死的历史契约），所以必须抢在
            # needs_reapproach 分支之前拦截：直接终局失败，等人工接管。
            self._needs_manual_takeover = True
            self._finish_failed(
                str(status.get("abort_reason") or "manual takeover")
            )
        elif status.get("needs_reapproach"):
            self._needs_reapproach = True
            self._on_staging_failed(status.get("abort_reason", "needs_reapproach"))

    def _finish_failed(self, reason: str) -> None:
        self._last_fail_reason = reason
        self._release_docking_session(reason=reason, authority=SpeedAuthority.FAILED)
        if self._needs_manual_takeover or self._staging_retry >= self._retry_max():
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
        # 重试中会话不结束：end_session=False 保留 _dock_active 与
        # _staging_override（GCS 预泊点覆盖沿用至下一次重试）
        self._release_docking_session(
            reason=f"staging retry: {reason}",
            authority=SpeedAuthority.FAILED,
            end_session=False,
        )
        if self._staging_retry >= self._retry_max():
            self._finish_failed(reason)
            return
        self.get_logger().warning(
            f"staging retry {self._staging_retry}/{self._retry_max()}: {reason}"
        )
        if self._mode == "dock_only":
            self._transition(MissionState.DOCK_HANDOFF, reason)
        else:
            self._transition(MissionState.NAV_TO_STAGING, reason)

    def _finish_succeeded(self, *, undock: bool = False) -> None:
        event = "UNDOCK_SUCCEEDED" if undock else "DOCK_SUCCEEDED"
        kind = "undock" if undock else "dock"
        self._release_docking_session(reason=f"{kind} succeeded")
        self._transition(MissionState.SUCCEEDED, f"{kind} complete")
        # 归港/出泊成功：关闭二维码检测通道，省算力（2026-10-13）
        self._set_apriltag_channels(False, reason=f"{kind} 成功收尾")
        self._emit_event(
            event,
            detail={"elapsed_sec": self._elapsed_sec(), "mode": self._mode},
        )
        self._mode = None
        self._transition(MissionState.IDLE, "session closed")

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
                self._transition(MissionState.DOCK_HANDOFF, "dock_only skip nav")
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
            elif not self._pending_nav and not self._staging_send_inflight:
                # 预泊失败重试（TASK_FAILED / SETTLE_TIMEOUT / needs_reapproach）
                # 转回本状态后必须真正重新下发 Nav2 目标——此前只有 ARMED 首发
                # 调一次 _start_nav_to_staging，重试会永远挂在本状态
                #（2026-10-13 二审 P0-1）
                self._start_nav_to_staging()

        elif self._state in (MissionState.SETTLE, MissionState.COMPLETE_SETTLE):
            self._tick_settle(dt)

        elif self._state in (MissionState.DOCK_HANDOFF, MissionState.UNDOCK_HANDOFF):
            self._tick_handoff(undock=self._state == MissionState.UNDOCK_HANDOFF)

        self._publish_task_status()
        self._publish_legacy_status()

    def _odom_speed_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        if not (
            math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)
        ):
            self.get_logger().warning(
                "丢弃非有限 Odometry 速度帧（NaN/inf）", throttle_duration_sec=2.0
            )
            return
        self._odom_speed = math.hypot(vx, vy)
        self._odom_yaw_rate = wz
        self._odom_speed_stamp = self.get_clock().now()

    def _odom_speed_fresh(self) -> bool:
        """速度源新鲜度：>1s 无帧视为缺测（与 docking safety 断流阈值同量级）。"""
        if self._odom_speed_stamp is None:
            return False
        age = (self.get_clock().now() - self._odom_speed_stamp).nanoseconds * 1e-9
        return age <= 1.0

    def _tick_settle(self, dt: float) -> None:
        """SETTLE / COMPLETE_SETTLE 停稳等待。

        SETTLE（预泊后交接前）：settle_require_still=true 时要求实测速度
        （odom_speed_topic）<= settle_max_speed 且 |角速度| <=
        settle_max_yaw_rate，按连续有效观测的实际时间累计满
        settle_still_hold_sec 才交接精靠泊——纯延时交接会让船带着风/流残余
        速度进 APPROACH（2026-10-13 源码审查 P0）。settle_sec 仍是最小延时
        下限。累计与发布频率解耦（2026-10-13 三审 P1）：新帧刷新停稳条件，
        短暂无新帧不清零；相邻帧间隔或无新帧沉默超 settle_max_frame_gap_sec
        才重置，且至少 settle_min_still_frames 帧有效观测，防单帧零速误放行
        （二审 P1-4）。一直停不稳/速度源缺测超 settle_timeout_sec ->
        _on_staging_failed 走 staging 重试管线。settle_require_still=false
        回退纯延时（标定/仿真）。

        COMPLETE_SETTLE（靠泊/出泊完成后收尾）：保持纯延时——坞内成功时船已被
        夹爪抓住，出泊成功时船已驶离，无需速度门。"""
        if self._state == MissionState.SETTLE:
            self._set_authority(SpeedAuthority.STAGING_VERIFY)
            self._settle_elapsed += dt
            if not bool(self.get_parameter("settle_require_still").value):
                if self._settle_elapsed >= float(self.get_parameter("settle_sec").value):
                    self._transition(MissionState.DOCK_HANDOFF, "settle done (timer only)")
                return
            speed = self._odom_speed
            yaw_rate = self._odom_yaw_rate
            max_gap = float(self.get_parameter("settle_max_frame_gap_sec").value)
            min_frames = int(self.get_parameter("settle_min_still_frames").value)
            now = self.get_clock().now()
            # 停稳时长按连续有效观测的实际时间累计（2026-10-13 三审 P1）：
            # 不再要求每 tick 都有新帧（那会把归港绑死在 ≥5Hz 的发布频率上，
            # 2Hz 低频里程计永远 SETTLE_TIMEOUT）。新帧刷新停稳条件；短暂无
            # 新帧冻结累计不清零；帧间隔/沉默超 max_gap 才重置；至少
            # min_frames 帧有效观测，防单帧零速凑数（2026-10-13 二审 P1-4）
            new_frame = (
                self._odom_speed_stamp is not None
                and (
                    self._settle_hold_last_stamp is None
                    or self._odom_speed_stamp > self._settle_hold_last_stamp
                )
            )
            if new_frame:
                stamp = self._odom_speed_stamp
                self._settle_hold_last_stamp = stamp
                still = (
                    speed is not None
                    and speed <= float(self.get_parameter("settle_max_speed").value)
                    and yaw_rate is not None
                    and abs(yaw_rate)
                    <= float(self.get_parameter("settle_max_yaw_rate").value)
                )
                if still:
                    gap = (
                        (stamp - self._settle_still_last_stamp).nanoseconds * 1e-9
                        if self._settle_still_last_stamp is not None
                        else None
                    )
                    if gap is None or gap > max_gap:
                        # 首帧或断档后：从本帧起重新累计一段连续停稳
                        self._settle_still_start_stamp = stamp
                        self._settle_still_frames = 1
                    else:
                        self._settle_still_frames += 1
                    self._settle_still_last_stamp = stamp
                else:
                    # 实测在动：立即清零重计
                    self._settle_still_start_stamp = None
                    self._settle_still_last_stamp = None
                    self._settle_still_frames = 0
            elif (
                self._settle_still_last_stamp is not None
                and (now - self._settle_still_last_stamp).nanoseconds * 1e-9
                > max_gap
            ):
                # 速度源沉默超允许帧间隔：旧"停稳"不可信，清零重计
                self._settle_still_start_stamp = None
                self._settle_still_last_stamp = None
                self._settle_still_frames = 0

            self._settle_still_hold = (
                (
                    self._settle_still_last_stamp - self._settle_still_start_stamp
                ).nanoseconds
                * 1e-9
                if self._settle_still_start_stamp is not None
                else 0.0
            )
            if (
                self._settle_still_start_stamp is not None
                and self._settle_still_hold
                >= float(self.get_parameter("settle_still_hold_sec").value)
                and self._settle_still_frames >= min_frames
                and self._settle_elapsed
                >= float(self.get_parameter("settle_sec").value)
            ):
                self._transition(
                    MissionState.DOCK_HANDOFF,
                    f"settle done (speed={speed:.3f} m/s)",
                )
                return
            if self._settle_elapsed > float(
                self.get_parameter("settle_timeout_sec").value
            ):
                note = (
                    f"speed={speed:.3f} m/s"
                    if speed is not None and self._odom_speed_fresh()
                    else "odom speed source missing/stale"
                )
                self._on_staging_failed(f"SETTLE_TIMEOUT: {note}")
            return
        self._complete_settle_elapsed += dt
        if self._complete_settle_elapsed >= float(
            self.get_parameter("complete_settle_sec").value
        ):
            self._finish_succeeded(undock=self._mode == "undock")

    def _tick_handoff(self, *, undock: bool) -> None:
        """DOCK_HANDOFF / UNDOCK_HANDOFF：把控制权交给 usv_docking 并转 MONITOR_*。"""
        self._set_authority(SpeedAuthority.DOCKING)
        pub = self._dock_undock_pub if undock else self._dock_start_pub
        pub.publish(Bool(data=True))
        self._emit_event("UNDOCK_HANDOFF" if undock else "DOCK_HANDOFF")
        self._transition(
            MissionState.MONITOR_UNDOCK if undock else MissionState.MONITOR_DOCK,
            "handoff usv_docking undock" if undock else "handoff usv_docking",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockMissionNode()
    # 多线程 executor：/dock/in_dock 查询 handler 在独立回调组里阻塞等视觉锚定
    # （秒级），不能饿死默认互斥组的定时器/订阅回调
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._release_docking_session(reason="shutdown")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
