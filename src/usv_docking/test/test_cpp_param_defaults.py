#!/usr/bin/env python3

import ast
from pathlib import Path
import re

import pytest
import yaml


PKG = Path(__file__).resolve().parents[1]
NODES = [
    "docking_pose_estimator",
    "docking_fsm",
    "docking_motion_controller",
    "docking_safety",
]
DECLARE_RE = re.compile(r'declare_parameter\(\s*"([^"]+)"\s*,\s*([^;]+)\);')
INTENTIONAL_EXEMPTIONS = {("docking_motion_controller", "test_only")}
KNOWN_DEAD = {"docked_yaw_tol_deg", "docked_hold_sec", "reacquire_search_yaw_deg"}


def _literal(text):
    text = text.strip()
    if text == "true":
        return True
    if text == "false":
        return False
    return ast.literal_eval(text)


def _defaults(node):
    text = (PKG / "src" / f"{node}.cpp").read_text(encoding="utf-8")
    return {name: _literal(value) for name, value in DECLARE_RE.findall(text)}


def _config():
    return yaml.safe_load((PKG / "config" / "docking.yaml").read_text(encoding="utf-8"))


@pytest.mark.parametrize("node", NODES)
def test_cpp_defaults_match_yaml(node):
    defaults = _defaults(node)
    expected = _config()[node]["ros__parameters"]
    mismatches = []
    for key, value in expected.items():
        if (node, key) in INTENTIONAL_EXEMPTIONS:
            continue
        actual = defaults.get(key)
        if isinstance(value, float):
            ok = isinstance(actual, float) and actual == pytest.approx(value)
        else:
            ok = actual == value
        if not ok:
            mismatches.append(f"{key}: C++={actual!r} yaml={value!r}")
    assert not mismatches, "\n".join(mismatches)


@pytest.mark.parametrize("node", NODES)
def test_yaml_and_cpp_declare_same_configuration_surface(node):
    defaults = set(_defaults(node))
    expected = set(_config()[node]["ros__parameters"])
    assert defaults == expected, {
        "only_cpp": sorted(defaults - expected),
        "only_yaml": sorted(expected - defaults),
    }


def test_only_intentional_default_exemption_remains():
    defaults = _defaults("docking_motion_controller")
    expected = _config()["docking_motion_controller"]["ros__parameters"]
    assert defaults["test_only"] is True
    assert expected["test_only"] is False


def test_no_new_dead_parameters():
    offenders = []
    declared_dead = set()
    for node in NODES:
        source_path = PKG / "src" / f"{node}.cpp"
        header_path = PKG / "include" / "usv_docking" / f"{node}.hpp"
        text = source_path.read_text(encoding="utf-8")
        if header_path.is_file():
            text += header_path.read_text(encoding="utf-8")
        for name, _ in DECLARE_RE.findall(source_path.read_text(encoding="utf-8")):
            if text.count(f'"{name}"') < 2:
                declared_dead.add(name)
                if name not in KNOWN_DEAD:
                    offenders.append(f"{node}: {name}")
    assert not offenders, offenders
    assert declared_dead == KNOWN_DEAD
