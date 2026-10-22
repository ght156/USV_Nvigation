"""Tests for the /dock/in_dock 出泊前在坞查询 handler (QueryInDock)."""

from __future__ import annotations

import math
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from dock_mission.dock_mission_node import DockMissionNode


@pytest.fixture
def in_dock_node(ros_context):
    with patch("dock_mission.dock_mission_node.Nav2GoalCheckerSwitch"), patch(
        "dock_mission.dock_mission_node.MissionBridgeClient"
    ):
        node = DockMissionNode()
    node.set_parameters([Parameter("in_dock_vision_timeout_sec", value=0.3)])
    yield node
    node.destroy_node()


def _set_pose(node, x, y, source="VISION"):
    node._dock_pose_xy = (x, y)
    node._dock_pose_stamp = node.get_clock().now()
    node._dock_pose_source = source


def _set_measurement_age(node, age: float):
    node._dock_measurement_age = age
    node._dock_measurement_age_stamp = node.get_clock().now()


def _query(node, wait_vision=False):
    req = SimpleNamespace(wait_vision=wait_vision)
    return node._in_dock_srv(req, SimpleNamespace())


def test_in_dock_when_at_charging_spot(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.0, 0.0)
    resp = _query(node)
    assert resp.pose_valid is True
    assert resp.in_dock is True
    assert resp.pose_source == "VISION"
    assert resp.dock_x == pytest.approx(0.0)


def test_not_in_dock_when_outside(in_dock_node):
    node = in_dock_node
    _set_pose(node, -3.5, 0.1)  # 出泊完成位置附近
    resp = _query(node)
    assert resp.pose_valid is True
    assert resp.in_dock is False


def test_not_in_dock_when_lateral_offset_too_big(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.0, 1.2)
    resp = _query(node)
    assert resp.pose_valid is True
    assert resp.in_dock is False


def test_odom_prediction_source_accepted(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.1, -0.1, source="ODOM_PREDICTION")
    resp = _query(node)
    assert resp.pose_valid is True
    assert resp.in_dock is True
    assert resp.pose_source == "ODOM_PREDICTION"


def test_invalid_pose_not_trusted_without_wait_vision(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.0, 0.0, source="INVALID")
    t0 = time.monotonic()
    resp = _query(node, wait_vision=False)
    assert time.monotonic() - t0 < 0.2  # 立即作答，不开通道等待
    assert resp.pose_valid is False
    assert resp.in_dock is False
    assert "不可信" in resp.message


def test_no_pose_data_reports_none(in_dock_node):
    node = in_dock_node
    resp = _query(node, wait_vision=False)
    assert resp.pose_valid is False
    assert resp.in_dock is False
    assert resp.pose_source == "NONE"


def test_stale_pose_rejected(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.0, 0.0)
    node._dock_pose_stamp = node.get_clock().now() - Duration(seconds=10.0)
    resp = _query(node, wait_vision=False)
    assert resp.pose_valid is False
    assert resp.in_dock is False


def test_wait_vision_times_out_and_stays_invalid(in_dock_node):
    node = in_dock_node
    _set_pose(node, 0.0, 0.0, source="INVALID")
    t0 = time.monotonic()
    resp = _query(node, wait_vision=True)
    # 无真实 apriltag 服务/estimator，等满超时（0.3s）后仍无效
    assert time.monotonic() - t0 >= 0.3
    assert resp.pose_valid is False
    assert resp.in_dock is False


def test_no_channel_toggle_during_active_session(in_dock_node):
    node = in_dock_node
    node._dock_active = True
    _set_pose(node, 0.0, 0.0, source="INVALID")
    calls = []
    node._set_apriltag_channels = lambda enabled, *, reason: calls.append(enabled)
    t0 = time.monotonic()
    resp = _query(node, wait_vision=True)
    assert time.monotonic() - t0 < 0.2  # 会话进行中不等待
    assert calls == []  # 不动检测通道
    assert resp.pose_valid is False


def test_no_backlight_toggle_when_fsm_backlight_active(in_dock_node):
    """usv_docking 背光会话活跃（归港失败待重试等）时查询不碰 DO：
    查询收尾的 0x0000 全掩码写会误关 FSM 为重试保持的背光。"""
    node = in_dock_node
    node._fsm_backlight_active = True
    calls = []
    node._set_dock_backlight = lambda on, *, reason: calls.append(on) or True
    _set_pose(node, 0.0, 0.0, source="INVALID")
    resp = _query(node, wait_vision=True)
    assert calls == []  # 背光不归查询管
    assert resp.pose_valid is False


def test_odom_prediction_too_old_not_used_for_in_dock(in_dock_node):
    """相机掉线后很久的推算位姿不能判在坞（in_dock_prediction_max_age_sec）。"""
    node = in_dock_node
    _set_pose(node, 0.0, 0.0, source="ODOM_PREDICTION")
    _set_measurement_age(node, 10.0)  # > 默认 3s
    resp = _query(node, wait_vision=False)
    assert resp.pose_valid is False
    assert resp.in_dock is False
    assert resp.pose_source == "ODOM_PREDICTION"
    assert "过期" in resp.message


def test_backlight_on_uses_control_do_mask2(in_dock_node):
    """在坞查询点灯走 ControlDO start=true, mask=2（只开灯，不驱动夹爪）。"""
    node = in_dock_node
    sent = []

    class _Goal:
        start = False
        mask = 0

    class _Handle:
        accepted = True

    class _Future:
        def add_done_callback(self, _cb):
            pass

        def result(self):
            return _Handle()

    class _Client:
        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            sent.append(goal)
            return _Future()

    node._ControlDO = SimpleNamespace(Goal=_Goal)
    node._control_do_client = _Client()
    assert node._set_dock_backlight(True, reason="test") is True
    assert sent and sent[-1].start is True and sent[-1].mask == 2


def test_no_tag_with_node_normal_reports_outside(in_dock_node):
    """检测节点正常、已开检测但没 Tag -> in_dock=false，message 说明坞外；
    相机故障由上层状态机判定。"""
    node = in_dock_node
    _set_pose(node, 0.0, 0.0, source="INVALID")
    _set_measurement_age(node, math.inf)
    node._apriltag_channel_client = SimpleNamespace(
        service_is_ready=lambda: True
    )
    node._set_apriltag_channels = lambda enabled, *, reason: None
    node._set_dock_backlight = lambda on, *, reason: False
    resp = _query(node, wait_vision=True)
    assert resp.in_dock is False
    assert resp.pose_valid is False
    assert resp.pose_source == "INVALID"
    assert "未检测到 Tag" in resp.message
    assert "不在坞" in resp.message


def test_no_tag_with_node_abnormal_reports_failure(in_dock_node):
    """检测节点异常/不在线 -> message 明确按失败处理，不能当坞外。"""
    node = in_dock_node
    _set_pose(node, 0.0, 0.0, source="INVALID")
    _set_measurement_age(node, math.inf)
    node._apriltag_channel_client = SimpleNamespace(
        service_is_ready=lambda: False
    )
    node._set_apriltag_channels = lambda enabled, *, reason: None
    node._set_dock_backlight = lambda on, *, reason: False
    resp = _query(node, wait_vision=True)
    assert resp.in_dock is False
    assert resp.pose_valid is False
    assert "异常" in resp.message
    assert "不可信" in resp.message
