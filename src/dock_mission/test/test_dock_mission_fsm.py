"""Tests for DockMissionNode FSM transitions with mocked dependencies."""

from __future__ import annotations

import json
import time
from unittest.mock import MagicMock, patch

import pytest
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from dock_mission.dock_mission_node import DockMissionNode
from dock_mission.types import MissionState


@pytest.fixture
def dock_node(ros_context, sample_dock_database, sample_map_yaml):
    with patch("dock_mission.dock_mission_node.Nav2GoalCheckerSwitch") as mock_switch_cls, patch(
        "dock_mission.dock_mission_node.MissionBridgeClient"
    ) as mock_mission_cls:
        mock_switch = MagicMock()
        mock_switch._active = None
        mock_switch_cls.return_value = mock_switch

        mock_mission = MagicMock()
        mock_mission.wait_ready.return_value = True
        mock_mission_cls.return_value = mock_mission

        node = DockMissionNode()
        node._dock_db_path = str(sample_dock_database)
        node._map_yaml = sample_map_yaml
        node._use_gnss = False
        node._mission_id = "dock_staging"
        node._bay_id = "bay2"
        node.set_parameters([Parameter("settle_sec", value=0.4)])
        node._goal_switch = mock_switch
        node._mission = mock_mission

        yield node, mock_switch, mock_mission
        node.destroy_node()


def test_begin_dock_mission_from_idle(dock_node):
    node, _, _ = dock_node
    assert node._state == MissionState.IDLE
    ok, msg = node._begin_dock_mission()
    assert ok is True
    assert node._state == MissionState.ARMED
    assert "dock task accepted" in msg


def test_begin_dock_mission_rejects_when_busy(dock_node):
    node, _, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    ok, msg = node._begin_dock_mission()
    assert ok is False
    assert "busy" in msg


def test_tick_armed_starts_nav_staging(dock_node):
    node, mock_switch, mock_mission = dock_node
    node._state = MissionState.ARMED

    with patch.object(node, "_staging_waypoint") as mock_wp:
        mock_wp.return_value = (31.48618175, 120.36797148, 0.0)
        node._tick()

    assert node._state == MissionState.NAV_TO_STAGING
    mock_switch.apply_docking.assert_called_once()
    mock_mission.send_staging_async.assert_called_once()
    kwargs = mock_mission.send_staging_async.call_args.kwargs
    assert kwargs["latitude"] == 31.48618175
    assert kwargs["longitude"] == 120.36797148
    assert kwargs["mission_id"] == "dock_staging"
    # 服务回包（done_cb）后才算预泊任务已受理
    kwargs["done_cb"](True, "ok")
    assert node._pending_nav is True


def test_task_event_completed_transitions_to_settle(dock_node):
    node, _, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    msg = String()
    msg.data = json.dumps(
        {"event": "TASK_COMPLETED", "detail": {"mission_id": "dock_staging"}}
    )
    node._task_event_cb(msg)

    assert node._state == MissionState.SETTLE
    assert node._pending_nav is False


def test_settle_entry_enables_apriltag_channel(dock_node):
    """到达预泊点（进入 SETTLE）自动开启后视相机二维码检测通道
    （2026-10-13）：等效 ros2 service call /apriltag_node/set_channel_enabled
    m_common/srv/SetChannelEnabled "{channel: 'all', enabled: true}"。"""
    node, _, _ = dock_node
    client = MagicMock()
    client.service_is_ready.return_value = True
    node._apriltag_channel_client = client

    node._transition(MissionState.SETTLE, "test")

    client.call_async.assert_called_once()
    req = client.call_async.call_args.args[0]
    assert req.channel == "all"
    assert req.enabled is True


def test_undock_handoff_enables_apriltag_channel(dock_node):
    """出泊交接（UNDOCK_HANDOFF）同样开启二维码检测通道（2026-10-13）。"""
    node, _, _ = dock_node
    client = MagicMock()
    client.service_is_ready.return_value = True
    node._apriltag_channel_client = client

    node._transition(MissionState.UNDOCK_HANDOFF, "test")

    client.call_async.assert_called_once()
    req = client.call_async.call_args.args[0]
    assert req.channel == "all"
    assert req.enabled is True


def test_success_disables_apriltag_channel(dock_node):
    """归港/出泊成功收尾都关闭二维码检测通道（省算力，2026-10-13）。"""
    node, _, _ = dock_node
    client = MagicMock()
    client.service_is_ready.return_value = True
    node._apriltag_channel_client = client

    node._state = MissionState.MONITOR_DOCK
    node._mode = "one_click"
    node._finish_succeeded(undock=False)
    req = client.call_async.call_args.args[0]
    assert req.channel == "all"
    assert req.enabled is False

    client.reset_mock()
    node._state = MissionState.MONITOR_UNDOCK
    node._mode = "undock"
    node._finish_succeeded(undock=True)
    client.call_async.assert_called_once()
    req = client.call_async.call_args.args[0]
    assert req.enabled is False


def test_task_event_failed_retries_staging(dock_node):
    node, mock_switch, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_FAILED",
            "detail": {"mission_id": "dock_staging", "reason": "planner failed"},
        }
    )
    node._task_event_cb(msg)

    assert node._state == MissionState.NAV_TO_STAGING
    assert node._staging_retry == 1
    mock_switch.apply_cruise.assert_called()


def _feed_speed(node, speed: float, yaw_rate: float = 0.0) -> None:
    """喂一帧"新鲜"实测速度（帧戳=now）。仅适用于不需要停稳时长累计的
    用例（超时/纯延时）；停稳门用例请用 _SpeedFeeder 控制帧间隔。"""
    node._odom_speed = speed
    node._odom_yaw_rate = yaw_rate
    node._odom_speed_stamp = node.get_clock().now()


class _SpeedFeeder:
    """按虚拟时间轴喂速度帧：帧戳严格递增、间隔可控（模拟不同发布频率）。

    停稳门按连续有效观测的实际时间累计（2026-10-13 三审 P1），不能再用
    "每 tick 一帧、帧戳≈now"的喂法——那样帧间隔≈0，永远凑不够停稳时长。
    帧戳可以略超前真实时钟（停稳门只对"落后超 max_gap 的沉默"清零）。
    """

    def __init__(self, node) -> None:
        self._node = node
        self._t = node.get_clock().now()

    def feed(
        self, speed: float = 0.0, yaw_rate: float = 0.0, period: float = 0.2
    ) -> None:
        self._t = self._t + Duration(seconds=period)
        self._node._odom_speed = speed
        self._node._odom_yaw_rate = yaw_rate
        self._node._odom_speed_stamp = self._t


def test_settle_advances_to_handoff(dock_node):
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0
    published = []
    node._dock_start_pub.publish = lambda m: published.append(m.data)

    # 实测停稳门（settle_require_still 默认 true）：停稳时长按连续有效观测
    # 的实际时间累计（2026-10-13 三审 P1）——模拟 5Hz 速度源（帧间隔 0.2s），
    # 4 帧覆盖 0.6s > settle_still_hold_sec(0.5)，且过 settle_sec(0.4) 下限
    feeder = _SpeedFeeder(node)
    for _ in range(4):
        feeder.feed(0.0)
        node._tick()
    assert node._state == MissionState.DOCK_HANDOFF

    node._tick()
    assert node._state == MissionState.MONITOR_DOCK
    assert published == [True]


def test_settle_waits_for_measured_still(dock_node):
    """残余速度超 settle_max_speed：过 settle_sec 也不交接；速度回落后
    重新累计停稳时长才放行。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    # 0.3 m/s 残余速度：5 tick（1.0s）已过 settle_sec(0.4) 仍留 SETTLE
    feeder = _SpeedFeeder(node)
    for _ in range(5):
        feeder.feed(0.3)
        node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_hold == 0.0

    # 停稳后重新累计：4 帧 × 0.2s 覆盖 0.6s 窗口，达标才交接
    for _ in range(4):
        feeder.feed(0.0)
        node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


def test_settle_timeout_goes_to_staging_retry(dock_node):
    """速度源缺测（一帧都没收到）且超 settle_timeout_sec：走 staging
    重试管线（retry+1 回 NAV_TO_STAGING），不是干等也不是盲目交接。"""
    node, _, _ = dock_node
    node.set_parameters([Parameter("settle_timeout_sec", value=0.5)])
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._mode = "one_click"

    for _ in range(4):  # 0.8s > 0.5s 超时；_odom_speed_stamp 恒 None
        node._tick()
    assert node._staging_retry == 1
    assert node._state == MissionState.NAV_TO_STAGING


def test_settle_timeout_exhausts_to_failed(dock_node):
    """一直停不稳且重试耗尽：终局 FAILED + 请求人工接管。"""
    node, _, _ = dock_node
    node.set_parameters(
        [
            Parameter("settle_timeout_sec", value=0.5),
            Parameter("staging_retry_max", value=1),
        ]
    )
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._mode = "one_click"

    for _ in range(4):
        _feed_speed(node, 0.3)  # 新鲜但一直停不稳
        node._tick()
    assert node._state == MissionState.FAILED
    assert node._needs_manual_takeover is True


def test_settle_timer_only_when_still_gate_disabled(dock_node):
    """settle_require_still=false 回退纯 settle_sec 延时（标定/仿真用），
    无速度源也照常交接。"""
    node, _, _ = dock_node
    node.set_parameters([Parameter("settle_require_still", value=False)])
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0

    node._tick()
    node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


def test_settle_single_frame_cannot_fill_hold(dock_node):
    """停稳门漏检窗口回归（2026-10-13 二审 P1-4 / 三审 P1）：只收到一帧零速
    后速度源沉默，不得凑满持续停稳时长——单帧窗口为 0 且帧数 <
    settle_min_still_frames(3)。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    _feed_speed(node, 0.0)  # 只有一帧
    for _ in range(5):  # 1.0s：已过 settle_sec(0.4)，但无新帧
        node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_hold == 0.0  # 单帧观测窗口为 0
    assert node._settle_still_frames <= 1


def test_settle_yaw_rate_blocks_handoff(dock_node):
    """原地偏航未稳（|角速度| 超 settle_max_yaw_rate）不算停稳，不交接。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    for _ in range(5):
        _feed_speed(node, 0.0, yaw_rate=0.5)
        node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_hold == 0.0


# ══════════════ 停稳门低频/抖动里程计兼容（2026-10-13 三审 P1）══════════════


@pytest.mark.parametrize(
    "period,n_frames",
    [(0.5, 3), (0.25, 3), (0.2, 4), (0.1, 6)],  # 2Hz / 4Hz / 5Hz / 10Hz
)
def test_settle_low_rate_odom_passes(dock_node, period, n_frames):
    """低频里程计回归（三审 P1）：旧实现要求每 0.2s tick 都有新帧，2/4Hz
    速度源即使船始终零速也永远 SETTLE_TIMEOUT。新实现按连续有效观测的
    实际时间累计，2Hz~10Hz 都必须能通过 SETTLE。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    feeder = _SpeedFeeder(node)
    # 达标帧：2Hz/4Hz 第 3 帧（min_frames=3 且窗口>=0.5s）、5Hz 第 4 帧、
    # 10Hz 第 6 帧；此前各 tick 必须留 SETTLE（窗口/帧数未凑齐不放行）
    for _ in range(n_frames - 1):
        feeder.feed(0.0, period=period)
        node._tick()
        assert node._state == MissionState.SETTLE
    feeder.feed(0.0, period=period)
    node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


def test_settle_brief_silence_does_not_clear(dock_node):
    """短暂无新帧（帧间隔抖动，< settle_max_frame_gap_sec）不清零累计：
    3 帧 0.2s 间隔 → 空 2 tick → 再来 1 帧，停稳窗口应延续累计并放行。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    feeder = _SpeedFeeder(node)
    for _ in range(3):  # 覆盖 0.4s，未满 0.5s
        feeder.feed(0.0)
        node._tick()
    assert node._state == MissionState.SETTLE
    hold_before = node._settle_still_hold
    assert hold_before == pytest.approx(0.4)

    node._tick()  # 无新帧（沉默 < max_gap=1s）：冻结而非清零
    node._tick()
    assert node._settle_still_hold == pytest.approx(hold_before)
    assert node._state == MissionState.SETTLE

    feeder.feed(0.0)  # 第 4 帧：窗口 0.6s 达标
    node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


def test_settle_frame_gap_beyond_max_restarts(dock_node):
    """相邻帧间隔超 settle_max_frame_gap_sec（断档）：从断档后首帧重新
    累计，之前的停稳窗口不得带入。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    feeder = _SpeedFeeder(node)
    for _ in range(2):  # 窗口 0.2s
        feeder.feed(0.0)
        node._tick()
    feeder.feed(0.0, period=1.5)  # 断档 1.5s > max_gap(1s)
    node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_frames == 1  # 重新起段
    assert node._settle_still_hold == 0.0

    for _ in range(3):  # 断档后连续 4 帧 × 0.2s：覆盖 0.6s 放行
        feeder.feed(0.0)
        node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


def test_settle_silence_beyond_max_gap_clears(dock_node):
    """速度源沉默超 settle_max_frame_gap_sec：旧"停稳"不可信，清零重计，
    不得用断流前的缓存停稳时长放行（防旧数据）。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    feeder = _SpeedFeeder(node)
    for _ in range(3):  # 窗口 0.4s（未达 0.5s，避免提前交接）
        feeder.feed(0.0)
        node._tick()
    assert node._settle_still_hold == pytest.approx(0.4)

    # 模拟断流：末帧戳回拨到 2s 前，下一 tick 无新帧 → 沉默超 max_gap(1s)
    node._settle_still_last_stamp = node.get_clock().now() - Duration(seconds=2.0)
    node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_hold == 0.0
    assert node._settle_still_frames == 0


def test_settle_min_frames_blocks_two_frame_pass(dock_node):
    """防少帧误放行：2 帧即便覆盖窗口 >= settle_still_hold_sec 也不放行，
    第 3 帧到达才交接（settle_min_still_frames 默认 3）。"""
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    node._settle_still_hold = 0.0

    feeder = _SpeedFeeder(node)
    for _ in range(2):  # 0.6s 间隔 × 2 帧：窗口 0.6s 达标但仅 2 帧
        feeder.feed(0.0, period=0.6)
        node._tick()
    assert node._state == MissionState.SETTLE
    assert node._settle_still_hold == pytest.approx(0.6)
    assert node._settle_still_frames == 2

    feeder.feed(0.0, period=0.6)
    node._tick()
    assert node._state == MissionState.DOCK_HANDOFF


# ══════════════ 预泊重试真正重发（2026-10-13 二审 P0-1）══════════════


def test_staging_retry_actually_resends_nav_goal(dock_node):
    """TASK_FAILED 后下一 tick 必须真正重新调用 send_staging_async——
    此前重试只转状态不重发，会永远挂在 NAV_TO_STAGING。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True
    node._mode = "one_click"

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_FAILED",
            "detail": {"mission_id": "dock_staging", "reason": "planner failed"},
        }
    )
    node._task_event_cb(msg)
    assert node._staging_retry == 1
    mock_mission.send_staging_async.assert_not_called()  # 失败当拍不发

    with patch.object(node, "_staging_waypoint") as mock_wp:
        mock_wp.return_value = (31.48618175, 120.36797148, 0.0)
        node._tick()
    mock_mission.send_staging_async.assert_called_once()
    assert mock_mission.send_staging_async.call_args.kwargs["command_id"] == "retry1"


def test_staging_retry_exhausted_no_resend(dock_node):
    """重试耗尽转 FAILED：不再重发。"""
    node, _, mock_mission = dock_node
    node.set_parameters([Parameter("staging_retry_max", value=1)])
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True
    node._mode = "one_click"

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_FAILED",
            "detail": {"mission_id": "dock_staging", "reason": "planner failed"},
        }
    )
    node._task_event_cb(msg)
    assert node._state == MissionState.FAILED

    node._tick()
    mock_mission.send_staging_async.assert_not_called()


def test_late_staging_response_does_not_corrupt_retry(dock_node):
    """上一轮 attempt 的迟到 ok 响应不得污染新一轮发送（attempt 关联）。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.ARMED
    node._mode = "one_click"

    with patch.object(node, "_staging_waypoint") as mock_wp:
        mock_wp.return_value = (31.48618175, 120.36797148, 0.0)
        node._tick()  # ARMED -> NAV_TO_STAGING + attempt1 发出
        assert mock_mission.send_staging_async.call_count == 1
        cb1 = mock_mission.send_staging_async.call_args.kwargs["done_cb"]
        # attempt1 响应看门狗超时 -> 失败重试
        node._staging_send_mono = time.monotonic() - 13.0
        node._tick()
        assert node._staging_retry == 1
        node._tick()  # 重发 attempt2
        cb2 = mock_mission.send_staging_async.call_args.kwargs["done_cb"]

    assert mock_mission.send_staging_async.call_count == 2
    assert cb1 is not cb2
    # attempt1 的迟到 ok：不置 _pending_nav、不动 attempt2 的 inflight
    cb1(True, "ok")
    assert node._pending_nav is False
    assert node._staging_send_inflight is True
    # attempt2 的正常响应才生效
    cb2(True, "ok")
    assert node._pending_nav is True
    assert node._staging_send_inflight is False


# ══════════════ 任务事件严格关联（2026-10-13 二审 P1-7）══════════════


def test_task_event_foreign_dock_mission_rejected(dock_node):
    """含 "dock" 字样的其他 mission_id 事件不得污染本会话。"""
    node, _, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    msg = String()
    msg.data = json.dumps(
        {"event": "TASK_COMPLETED", "detail": {"task_id": "dock_other"}}
    )
    node._task_event_cb(msg)
    assert node._state == MissionState.NAV_TO_STAGING
    assert node._pending_nav is True


def test_task_event_stale_retry_rejected(dock_node):
    """上一轮 retry 的残留事件（command_id 不匹配）不推进状态；
    当前 attempt 的事件正常受理。"""
    node, _, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True
    node._staging_retry = 1  # 当前 attempt 是 retry1

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_COMPLETED",
            "detail": {"task_id": "dock_staging", "command_id": "retry0"},
        }
    )
    node._task_event_cb(msg)
    assert node._state == MissionState.NAV_TO_STAGING
    assert node._pending_nav is True

    msg.data = json.dumps(
        {
            "event": "TASK_COMPLETED",
            "detail": {"task_id": "dock_staging", "command_id": "retry1"},
        }
    )
    node._task_event_cb(msg)
    assert node._state == MissionState.SETTLE


def test_cancel_transitions_to_idle(dock_node):
    node, mock_switch, _ = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    resp = node._cancel_srv(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    # 取消语义：CANCELLED（瞬态，伴随 DOCK_CANCELLED 事件）→ 立即回 IDLE
    assert node._state == MissionState.IDLE
    assert node._dock_active is False
    mock_switch.apply_cruise.assert_called()


def test_cancel_during_nav_cancels_bridge_goal(dock_node):
    """NAV_TO_STAGING 阶段取消：必须让 mission_bridge 撤销 Nav2 goal。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    resp = node._cancel_srv(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    mock_mission.cancel_async.assert_called_once()


def test_staging_retry_does_not_cancel_bridge_goal(dock_node):
    """预泊重试（end_session=False）不发取消：新 goal 由 bridge 抢占替换旧 goal。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_FAILED",
            "detail": {"mission_id": "dock_staging", "reason": "planner failed"},
        }
    )
    node._task_event_cb(msg)

    assert node._state == MissionState.NAV_TO_STAGING
    assert node._staging_retry == 1
    mock_mission.cancel_async.assert_not_called()


def test_staging_retry_exhausted_cancels_bridge_goal(dock_node):
    """重试耗尽判失败：会话结束，必须撤销 Nav2 goal。"""
    node, _, mock_mission = dock_node
    node.set_parameters([Parameter("staging_retry_max", value=1)])
    node._state = MissionState.NAV_TO_STAGING
    node._pending_nav = True

    msg = String()
    msg.data = json.dumps(
        {
            "event": "TASK_FAILED",
            "detail": {"mission_id": "dock_staging", "reason": "planner failed"},
        }
    )
    node._task_event_cb(msg)

    assert node._state == MissionState.FAILED
    mock_mission.cancel_async.assert_called_once()


def test_dock_monitor_failure_does_not_cancel_bridge_goal(dock_node):
    """对接/出泊阶段失败没有 Nav2 goal：不发取消。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.MONITOR_UNDOCK

    msg = String()
    msg.data = json.dumps({"state": "DOCK_ABORT", "abort_reason": "align lost"})
    node._dock_status_cb(msg)

    assert node._state == MissionState.FAILED
    mock_mission.cancel_async.assert_not_called()


def test_dock_status_success(dock_node):
    node, mock_switch, _ = dock_node
    node._state = MissionState.MONITOR_DOCK
    node.set_parameters([Parameter("complete_settle_sec", value=0.4)])

    msg = String()
    msg.data = json.dumps({"success": True})
    node._dock_status_cb(msg)

    # success → COMPLETE_SETTLE（等 complete_settle_sec）→ SUCCEEDED（瞬态）→ IDLE
    assert node._state == MissionState.COMPLETE_SETTLE
    mock_switch.apply_cruise.assert_called()

    for _ in range(3):
        node._tick()
    assert node._state == MissionState.IDLE
    assert node._dock_active is False


def test_home_cb_arms_mission(dock_node):
    node, _, _ = dock_node
    msg = Bool()
    msg.data = True
    node._home_cb(msg)
    assert node._state == MissionState.ARMED


def test_dock_status_manual_takeover_no_retry(dock_node):
    """FSM 报 needs_manual_takeover（夹爪半抓住保护）：直接 FAILED 终局，
    不走 staging retry。2026-10-09 实船回归：FAILED 的状态 JSON 里
    needs_reapproach 被契约强制 true，dock_mission 只认 needs_reapproach
    导致误判重试，把扣在坞里的船开了出去。"""
    node, _, mock_mission = dock_node
    node._state = MissionState.MONITOR_DOCK

    msg = String()
    msg.data = json.dumps(
        {
            "state": "DOCK_ABORT",
            "needs_reapproach": True,  # FAILED 契约强制 true，必须被拦截
            "needs_manual_takeover": True,
            "abort_reason": "FINAL_PUSH_TIMEOUT",
        }
    )
    node._dock_status_cb(msg)

    assert node._state == MissionState.FAILED
    assert node._needs_manual_takeover is True
    assert node._staging_retry == 0  # 未走重试
    mock_mission.cancel_async.assert_not_called()


def test_dock_status_reapproach_still_retries(dock_node):
    """普通 needs_reapproach（无 manual_takeover）：仍走 staging retry。"""
    node, _, _ = dock_node
    node._state = MissionState.MONITOR_DOCK
    node._mode = "dock_only"

    msg = String()
    msg.data = json.dumps(
        {"needs_reapproach": True, "abort_reason": "ALIGN_TIMEOUT"}
    )
    node._dock_status_cb(msg)

    assert node._staging_retry == 1
    assert node._state == MissionState.DOCK_HANDOFF
    assert node._needs_manual_takeover is False


# ══════════════ /gcs_dock/command 预泊点哨兵兼容（2026-10-20）══════════════
def test_gcs_command_sentinel_dock_point_falls_back_to_database(dock_node):
    """Decision 无泊位时 lat/lon/yaw 全发 65536：按“未指定”回退泊位库，不丢指令。"""
    node, _, _ = dock_node
    msg = String()
    msg.data = json.dumps(
        {
            "action": "one_click_dock",
            "mission_id": "m001",
            "command_id": "c1",
            "dock_lat": 65536.0,
            "dock_lon": 65536.0,
            "dock_yaw": 65536.0,
        }
    )
    node._gcs_command_cb(msg)

    assert node._state == MissionState.ARMED
    assert node._mode == "one_click"
    assert node._staging_override is None
    assert node._mission_id == "m001"
    assert node._command_id == "c1"


def test_gcs_command_valid_dock_point_is_ignored_by_default(dock_node):
    node, _, _ = dock_node
    msg = String()
    msg.data = json.dumps(
        {
            "action": "one_click_dock",
            "mission_id": "m001",
            "command_id": "c1",
            "dock_lat": 30.5,
            "dock_lon": 120.4,
        }
    )
    node._gcs_command_cb(msg)

    assert node._state == MissionState.ARMED
    assert node._staging_override is None


def test_gcs_command_invalid_dock_point_is_ignored_by_default(dock_node):
    """默认不接受覆盖时，非法 dock point 也不阻塞归港指令。"""
    node, _, _ = dock_node
    msg = String()
    msg.data = json.dumps(
        {
            "action": "one_click_dock",
            "mission_id": "m001",
            "command_id": "c1",
            "dock_lat": 95.0,
            "dock_lon": 120.0,
        }
    )
    node._gcs_command_cb(msg)

    assert node._state == MissionState.ARMED
    assert node._mode == "one_click"
    assert node._staging_override is None


def test_gcs_command_valid_dock_point_can_override_when_enabled(dock_node):
    """联调开关显式开启后，保留旧的合法预泊点覆盖能力。"""
    node, _, _ = dock_node
    node._accept_gcs_staging_override = True
    msg = String()
    msg.data = json.dumps(
        {
            "action": "one_click_dock",
            "mission_id": "m001",
            "command_id": "c1",
            "dock_lat": 30.5,
            "dock_lon": 120.4,
        }
    )
    node._gcs_command_cb(msg)

    assert node._state == MissionState.ARMED
    assert node._staging_override == (30.5, 120.4, 65536.0)


def test_gcs_command_truly_invalid_dock_point_is_dropped_when_enabled(dock_node):
    """显式开启覆盖时，非 65536 的真越界仍丢整条指令。"""
    node, _, _ = dock_node
    node._accept_gcs_staging_override = True
    msg = String()
    msg.data = json.dumps(
        {
            "action": "one_click_dock",
            "mission_id": "m001",
            "command_id": "c1",
            "dock_lat": 95.0,
            "dock_lon": 120.0,
        }
    )
    node._gcs_command_cb(msg)

    assert node._state == MissionState.IDLE
    assert node._mode is None
    assert node._staging_override is None
