#!/usr/bin/env python3

from pathlib import Path

import pytest
from ament_index_python.packages import get_package_prefix


PKG = Path(__file__).resolve().parents[1]
NODES = [
    "docking_pose_estimator",
    "docking_fsm",
    "docking_motion_controller",
    "docking_safety",
]


def test_package_is_native_cmake_with_mlogger_and_components():
    cmake = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")
    package_xml = (PKG / "package.xml").read_text(encoding="utf-8")
    assert "find_package(rclcpp REQUIRED)" in cmake
    assert "find_package(rclcpp_components REQUIRED)" in cmake
    assert "find_package(m_common REQUIRED)" in cmake
    assert "m_common::mlogger" in cmake
    assert "rclcpp_components_register_nodes" in cmake
    assert "<build_type>ament_cmake</build_type>" in package_xml
    assert "<depend>rclpy</depend>" not in package_xml
    assert not (PKG / "setup.py").exists()


def test_manifest_keeps_native_and_launch_dependencies():
    package_xml = (PKG / "package.xml").read_text(encoding="utf-8")
    for dependency in (
        "rclcpp_components", "rclcpp_action", "tf2", "usv_rs485_driver",
        "ament_index_python", "launch", "launch_ros",
    ):
        assert f"<depend>{dependency}</depend>" in package_xml or (
            f"<exec_depend>{dependency}</exec_depend>" in package_xml
        ), dependency


@pytest.mark.parametrize("node_name", NODES)
def test_docking_node_is_cpp_elf(node_name):
    executable = (
        Path(get_package_prefix("usv_docking"))
        / "lib"
        / "usv_docking"
        / node_name
    )
    assert executable.is_file(), f"missing C++ executable: {executable}"
    with executable.open("rb") as stream:
        assert stream.read(4) == b"\x7fELF"


def test_core_nodes_share_container_and_estimator_stays_independent():
    launch = (PKG / "launch" / "docking.launch.py").read_text(encoding="utf-8")
    assert "component_container_isolated" in launch
    assert "docking_core_container" in launch
    for plugin in (
        "usv_docking::DockingFsm",
        "usv_docking::DockingMotionController",
        "usv_docking::DockingSafety",
    ):
        assert plugin in launch
    assert "LoadComposableNodes" in launch
    estimator = launch.split("docking_core_container", 1)[0]
    assert "executable=\"docking_pose_estimator\"" in launch
    assert "plugin=\"usv_docking::DockingPoseEstimator\"" not in launch


def workspace_script(name):
    for root in (PKG.parents[1], PKG.parents[2]):
        candidate = root / "scripts" / name
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(name)


def test_stack_stop_patterns_include_core_container():
    script = workspace_script("start_usv_nav.sh").read_text(encoding="utf-8")
    assert '"docking_core_container"' in script


def test_gui_exposes_cone_switch_as_boolean():
    script = workspace_script("dock_tune_gui.py").read_text(encoding="utf-8")
    assert '"approach_cone_enabled"' in script
    assert '"bool"' in script
    assert "PARAMETER_BOOL" in script


def test_native_nodes_use_ros_clock_timers():
    for node in NODES:
        source = (PKG / "src" / f"{node}.cpp").read_text(encoding="utf-8")
        assert "create_wall_timer" not in source
        assert "rclcpp::create_timer" in source


def test_native_nodes_register_per_node_mlogger_identities():
    for node in NODES:
        source = (PKG / "src" / f"{node}.cpp").read_text(encoding="utf-8")
        assert "MLOGGER_NODE_REGISTER" in source
        assert '"docking"' in source
