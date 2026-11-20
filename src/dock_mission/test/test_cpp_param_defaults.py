#!/usr/bin/env python3

from pathlib import Path
import re

import yaml


PKG = Path(__file__).resolve().parents[1]


def test_cpp_and_yaml_cover_same_mission_parameters():
    source = (PKG / "src" / "dock_mission_node.cpp").read_text(encoding="utf-8")
    declared = set(re.findall(r'declare_parameter\("([^"]+)"', source))
    config = yaml.safe_load((PKG / "config" / "dock_mission.yaml").read_text(encoding="utf-8"))
    configured = set(config["dock_mission_node"]["ros__parameters"])
    assert declared == configured, {
        "only_cpp": sorted(declared - configured),
        "only_yaml": sorted(configured - declared),
    }
