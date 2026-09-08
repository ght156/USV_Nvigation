"""Tests for Nav2GoalCheckerSwitch with mocked parameter client."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest
from rclpy.parameter import Parameter

from dock_mission.nav2_goal_checker import Nav2GoalCheckerSwitch


class _FakeFuture:
    def __init__(self, results=None):
        self._results = results or []

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        resp = MagicMock()
        resp.results = self._results
        return resp


@pytest.fixture
def goal_switch(mock_ros_node):
    with patch(
        "dock_mission.nav2_goal_checker._AsyncParametersClient"
    ) as mock_client_cls:
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = True
        mock_client.set_parameters.return_value = _FakeFuture()
        mock_client_cls.return_value = mock_client
        switch = Nav2GoalCheckerSwitch(mock_ros_node)
        switch._param_client = mock_client
        yield switch, mock_client


def test_apply_docking_sets_active_and_publishes(goal_switch, mock_ros_node):
    switch, param_client = goal_switch
    published = []

    def _capture(msg):
        published.append(msg.data)

    switch._selector_pub.publish = _capture

    switch.apply_docking()
    assert switch._active == "docking"
    assert "docking_goal_checker" in published

    params = param_client.set_parameters.call_args[0][0]
    names = {p.name for p in params}
    assert "general_goal_checker.xy_goal_tolerance" in names

    all_names = set()
    for call in param_client.set_parameters.call_args_list:
        for p in call[0][0]:
            all_names.add(p.name)
    assert "docking_goal_checker.xy_goal_tolerance" in all_names


def test_apply_cruise_idempotent(goal_switch):
    switch, param_client = goal_switch
    switch._selector_pub.publish = MagicMock()

    switch.apply_cruise()
    first_calls = param_client.set_parameters.call_count
    switch.apply_cruise()
    assert switch._active == "cruise"
    assert param_client.set_parameters.call_count == first_calls


def test_apply_docking_idempotent(goal_switch):
    switch, param_client = goal_switch
    switch._selector_pub.publish = MagicMock()

    switch.apply_docking()
    first_calls = param_client.set_parameters.call_count
    switch.apply_docking()
    assert switch._active == "docking"
    assert param_client.set_parameters.call_count == first_calls


def test_selector_disabled_skips_publish(mock_ros_node):
    mock_ros_node.set_parameters([
        Parameter("use_goal_checker_selector", value=False),
    ])
    with patch(
        "dock_mission.nav2_goal_checker._AsyncParametersClient"
    ) as mock_client_cls:
        mock_client = MagicMock()
        mock_client.wait_for_service.return_value = False
        mock_client_cls.return_value = mock_client
        switch = Nav2GoalCheckerSwitch(mock_ros_node)
        published = []
        switch._selector_pub.publish = lambda msg: published.append(msg.data)
        switch.apply_docking()
        assert published == []
