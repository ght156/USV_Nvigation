#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_prefix
import pytest


@pytest.mark.parametrize("node_name", [
    "waypoint_transform",
    "waypoint_with_state",
    "nav_status_aggregator",
    "zone_manager",
    "zone_monitor",
    "livox_cloud_filter",
    "map_server_guard",
])
def test_navigation_node_is_cpp_elf(node_name):
    executable = Path(get_package_prefix("workspace_nav")) / "lib" / "workspace_nav" / node_name
    assert executable.is_file(), f"missing C++ executable: {executable}"
    with executable.open("rb") as stream:
        assert stream.read(4) == b"\x7fELF", f"node is still a Python script: {executable}"
