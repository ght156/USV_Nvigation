#!/usr/bin/env python3

from pathlib import Path

import pytest
from ament_index_python.packages import get_package_prefix


PKG = Path(__file__).resolve().parents[1]


def test_package_is_native_cmake_with_mlogger():
    cmake = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (PKG / "package.xml").read_text(encoding="utf-8")
    assert "find_package(rclcpp REQUIRED)" in cmake
    assert "find_package(m_common REQUIRED)" in cmake
    assert "m_common::mlogger" in cmake
    assert "<build_type>ament_cmake</build_type>" in package_xml
    assert "<exec_depend>rclpy</exec_depend>" not in package_xml
    assert not (PKG / "setup.py").exists()


def test_manifest_keeps_launch_dependencies():
    package_xml = (PKG / "package.xml").read_text(encoding="utf-8")
    for dependency in ("ament_index_python", "python3-yaml", "launch", "launch_ros"):
        assert f"<exec_depend>{dependency}</exec_depend>" in package_xml


def test_dock_mission_node_is_cpp_elf():
    executable = (
        Path(get_package_prefix("dock_mission"))
        / "lib"
        / "dock_mission"
        / "dock_mission_node"
    )
    assert executable.is_file(), f"missing C++ executable: {executable}"
    with executable.open("rb") as stream:
        assert stream.read(4) == b"\x7fELF"


def test_native_node_uses_ros_clock_timer():
    source = (PKG / "src" / "dock_mission_node.cpp").read_text(encoding="utf-8")
    assert "create_wall_timer" not in source
    assert "rclcpp::create_timer" in source


@pytest.mark.parametrize("header", [
    "dock_task_api.hpp",
    "bay_loader.hpp",
    "gnss_staging.hpp",
    "task_event.hpp",
    "mission_logic.hpp",
])
def test_core_behavior_headers_exist(header):
    assert (PKG / "include" / "dock_mission" / header).is_file()
