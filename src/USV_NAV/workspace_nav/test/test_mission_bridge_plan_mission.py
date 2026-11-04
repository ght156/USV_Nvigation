#!/usr/bin/env python3

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from m_common.msg import MissionWaypoint
from m_common.srv import PlanMission
from nav_msgs.msg import Path

from workspace_nav.mission_bridge import MissionBridgeNode, MissionState, YAW_UNSPECIFIED


class _ImmediateFuture:
    def __init__(self, value):
        self._value = value

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._value


class _PendingFuture:
    def add_done_callback(self, _callback):
        pass


def _pose(x, y):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def _request():
    request = PlanMission.Request()
    request.waypoints = [
        MissionWaypoint(latitude=31.0, longitude=120.0, yaw=YAW_UNSPECIFIED, seq=1),
        MissionWaypoint(latitude=31.0001, longitude=120.0001, yaw=90.0, seq=2),
    ]
    return request


def _node(state=MissionState.IDLE):
    node = object.__new__(MissionBridgeNode)
    node._sm_lock = threading.Lock()
    node._state = state
    node._datum_lat = 31.0
    node._datum_lon = 120.0
    node._datum_easting = 0.0
    node._datum_northing = 0.0
    node._projection = "enu"
    node._datum_source = "map_yaml"
    node._map_yaml_resolved = "map.yaml"
    node._ref_key = "ref_gnss_10"
    node._map_ox = 0.0
    node._map_oy = 0.0
    node._map_origin_yaw = 0.0
    node.get_parameter = MagicMock(return_value=SimpleNamespace(value=0.1))
    node.get_logger = MagicMock(return_value=SimpleNamespace(
        info=MagicMock(), warning=MagicMock(), error=MagicMock()))
    node.create_pose_msg = lambda x, y, _z, _yaw: _pose(x, y)
    node._path_client = MagicMock()
    node._path_client.wait_for_server.return_value = True
    return node


def _finish_plan(node, status=GoalStatus.STATUS_SUCCEEDED, poses=None):
    path = Path()
    path.poses = [_pose(0.0, 0.0), _pose(3.0, 4.0)] if poses is None else poses
    raw = SimpleNamespace(status=status, result=SimpleNamespace(path=path))
    goal_handle = SimpleNamespace(
        accepted=True,
        get_result_async=MagicMock(return_value=_ImmediateFuture(raw)),
        cancel_goal_async=MagicMock(),
    )
    node._path_client.send_goal_async.return_value = _ImmediateFuture(goal_handle)
    return goal_handle


def test_plan_mission_returns_full_path_without_side_effects(monkeypatch):
    node = _node()
    _finish_plan(node)
    write_json = MagicMock()
    monkeypatch.setattr("workspace_nav.mission_bridge.atomic_write_json", write_json)

    response = MissionBridgeNode._cb_plan_mission(
        node, _request(), PlanMission.Response())

    assert response.success is True
    assert len(response.path.poses) == 2
    sent_goal = node._path_client.send_goal_async.call_args.args[0]
    assert len(sent_goal.goals) == 2
    assert sent_goal.goals[0].pose.position.x == pytest.approx(0.0)
    assert sent_goal.goals[1].pose.position.x > sent_goal.goals[0].pose.position.x
    assert sent_goal.goals[1].pose.position.y > sent_goal.goals[0].pose.position.y
    assert sent_goal.use_start is False
    assert node._state == MissionState.IDLE
    write_json.assert_not_called()


@pytest.mark.parametrize("state", [
    MissionState.WAITING_SYSTEM,
    MissionState.RUNNING,
    MissionState.PAUSED,
])
def test_plan_mission_rejects_when_not_idle(state):
    node = _node(state)

    response = MissionBridgeNode._cb_plan_mission(
        node, _request(), PlanMission.Response())

    assert response.success is False
    assert state.value in response.message
    node._path_client.wait_for_server.assert_not_called()


def test_plan_mission_returns_planner_failure():
    node = _node()
    _finish_plan(node, status=GoalStatus.STATUS_ABORTED)

    response = MissionBridgeNode._cb_plan_mission(
        node, _request(), PlanMission.Response())

    assert response.success is False
    assert "failed" in response.message


def test_plan_mission_times_out_without_pending_goal():
    node = _node()
    node._path_client.send_goal_async.return_value = _PendingFuture()

    response = MissionBridgeNode._cb_plan_mission(
        node, _request(), PlanMission.Response())

    assert response.success is False
    assert "timed out" in response.message
    assert node._state == MissionState.IDLE
