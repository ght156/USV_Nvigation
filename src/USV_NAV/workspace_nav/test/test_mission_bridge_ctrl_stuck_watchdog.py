from types import SimpleNamespace
from unittest.mock import MagicMock

from m_common.action import NavigateTask

from workspace_nav import mission_bridge
from workspace_nav.mission_bridge import MissionBridgeNode, MissionState


def _make_node(monkeypatch, *, now: float) -> MissionBridgeNode:
    node = object.__new__(MissionBridgeNode)
    node._ctrl_stuck_timeout_sec = 30.0
    node._sm_lock = mission_bridge.threading.RLock()
    node._state = MissionState.RUNNING
    node._ctrl_stuck_since = None
    node._ctrl_stuck_last_seen_mono = 0.0
    node._mission_token = 0
    node._cancel_active_nav2_goal = MagicMock()
    node._on_nav_fatal = MagicMock()
    logger = MagicMock()
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    monkeypatch.setattr(mission_bridge.time, "monotonic", lambda: now)
    return node


def _nav_status(*, stuck: bool) -> SimpleNamespace:
    return SimpleNamespace(
        task_state="RUNNING",
        last_error="CTRL_STUCK" if stuck else "",
        controller_status="STUCK" if stuck else "OK",
    )


def test_ctrl_stuck_watchdog_resets_when_recovered(monkeypatch):
    node = _make_node(monkeypatch, now=100.0)
    node._cb_nav_status_watchdog(_nav_status(stuck=True))
    assert node._ctrl_stuck_since == 100.0

    monkeypatch.setattr(mission_bridge.time, "monotonic", lambda: 115.0)
    node._cb_nav_status_watchdog(_nav_status(stuck=False))
    assert node._ctrl_stuck_since is None
    node._on_nav_fatal.assert_not_called()


def test_ctrl_stuck_watchdog_fails_after_30_seconds(monkeypatch):
    node = _make_node(monkeypatch, now=130.0)
    node._ctrl_stuck_since = 100.0
    node._ctrl_stuck_last_seen_mono = 130.0
    node._mission_token = 7

    node._tick_ctrl_stuck_watchdog()

    assert node._mission_token == 8
    node._cancel_active_nav2_goal.assert_called_once()
    node._on_nav_fatal.assert_called_once()
    kwargs = node._on_nav_fatal.call_args.kwargs
    assert kwargs["error_code"] == "CTRL_STUCK"
    assert kwargs["nt_result_code"] == NavigateTask.Result.RESULT_CONTROLLER_FAILED


def test_ctrl_stuck_watchdog_does_not_fail_before_timeout(monkeypatch):
    node = _make_node(monkeypatch, now=129.9)
    node._ctrl_stuck_since = 100.0
    node._ctrl_stuck_last_seen_mono = 129.9

    node._tick_ctrl_stuck_watchdog()

    node._cancel_active_nav2_goal.assert_not_called()
    node._on_nav_fatal.assert_not_called()
