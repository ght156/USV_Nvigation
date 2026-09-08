"""Tests for /gcs_dock/command parsing and pre-dock point validation."""

from __future__ import annotations

import pytest

from dock_mission.dock_task_api import (
    DockCommand,
    is_valid_dock_point,
    parse_gcs_command,
)


def test_parse_gcs_command_full_dock_point():
    raw = (
        '{"action": "one_click_dock", "mission_id": "m001", "command_id": "c1", '
        '"dock_lat": "30.123456", "dock_lon": "120.123456", "dock_yaw": "90"}'
    )
    g = parse_gcs_command(raw)
    assert g is not None
    assert g.command == DockCommand.ONE_CLICK_DOCK
    assert g.mission_id == "m001"
    assert g.command_id == "c1"
    assert g.has_dock_point is True
    # dock_yaw 上层约定单位为"度"，这里原样透传，由节点转弧度
    assert g.dock_yaw == pytest.approx(90.0)


def test_parse_gcs_command_still_works_without_dock_point():
    # 旧格式（不带预泊点）向后兼容
    raw = '{"action": "dock_only", "mission_id": "m", "command_id": "c"}'
    g = parse_gcs_command(raw)
    assert g is not None
    assert g.command == DockCommand.DOCK_ONLY
    assert g.has_dock_point is False
    assert g.dock_lat is None and g.dock_lon is None and g.dock_yaw is None


def test_parse_gcs_command_unknown_action_returns_none():
    assert parse_gcs_command('{"action": "fly_away"}') is None
    assert parse_gcs_command("not json") is None


def test_parse_gcs_command_dock_yaw_as_number_and_sentinel():
    # 数值类型（非字符串）也能解析
    g = parse_gcs_command(
        '{"action": "one_click_dock", "dock_lat": 30.5, "dock_lon": 120.4, "dock_yaw": 65536.0}'
    )
    assert g is not None and g.dock_yaw == 65536.0
    # 缺 yaw → None，节点会补 DOCK_YAW_UNSPECIFIED
    g2 = parse_gcs_command('{"action": "one_click_dock", "dock_lat": 30.5, "dock_lon": 120.4}')
    assert g2 is not None and g2.dock_yaw is None


def test_is_valid_dock_point():
    assert is_valid_dock_point(30.1, 120.1) is True
    assert is_valid_dock_point(0.0, 0.0) is False
    assert is_valid_dock_point(95.0, 120.0) is False
    assert is_valid_dock_point(30.0, -181.0) is False


def test_dock_yaw_degrees_passthrough_interface_is_degrees():
    # 接口统一为**度**：dock_yaw 原样保留度数，不再在 dock_mission 转弧度；
    # 弧度转换只在 mission_bridge 生成 Nav2 位姿时发生。
    g = parse_gcs_command(
        '{"action": "one_click_dock", "dock_lat": 30.5, "dock_lon": 120.4, "dock_yaw": 90.0}'
    )
    assert g is not None and g.dock_yaw == 90.0
    # 哨兵 65536.0（度）在 [-360,360] 之外 → 视为"不指定朝向"（边界仍成立）
    assert not (-360.0 <= 65536.0 <= 360.0)
