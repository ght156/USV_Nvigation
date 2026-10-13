"""Tests for DockMissionNode FSM transitions with mocked dependencies."""

from __future__ import annotations

import json
from unittest.mock import MagicMock, patch

import pytest
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


def test_settle_advances_to_handoff(dock_node):
    node, _, _ = dock_node
    node._state = MissionState.SETTLE
    node._settle_elapsed = 0.0
    published = []
    node._dock_start_pub.publish = lambda m: published.append(m.data)

    node._tick()
    node._tick()
    assert node._state == MissionState.DOCK_HANDOFF

    node._tick()
    assert node._state == MissionState.MONITOR_DOCK
    assert published == [True]


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
