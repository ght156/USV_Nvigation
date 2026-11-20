#!/usr/bin/env python3
"""C++ 节点的跨节点 YAML 参数一致性检查，无需 DDS。"""

from pathlib import Path

import yaml


PKG = Path(__file__).resolve().parents[1]
MUST_MATCH = [
    ("staging_x", ["docking_fsm", "docking_motion_controller"]),
    ("final_target_x", ["docking_fsm", "docking_motion_controller"]),
    ("back_in_gate2_y", ["docking_fsm", "docking_motion_controller"]),
    ("back_in_gate2_yaw_deg", ["docking_fsm", "docking_motion_controller"]),
    ("docked_x_tol", ["docking_fsm", "docking_motion_controller"]),
    ("docked_y_tol", ["docking_fsm", "docking_motion_controller"]),
    ("blind_exit_enabled", ["docking_fsm", "docking_motion_controller"]),
    ("max_docking_duration_sec", ["docking_fsm", "docking_safety"]),
]
PAIRED_TOPIC_PARAMS = {
    "dock_pose_topic",
    "tag_visible_topic",
    "pose_source_topic",
    "measurement_age_topic",
    "reset_anchor_topic",
    "state_topic",
    "target_mode_topic",
    "safety_stop_topic",
    "abort_request_topic",
}
ABORT_EXIT_MARGIN_SEC = 5.0


def _config():
    return yaml.safe_load((PKG / "config" / "docking.yaml").read_text(encoding="utf-8"))


def test_pub_sub_topic_params_are_consistent():
    cfg = _config()
    for name in sorted(PAIRED_TOPIC_PARAMS):
        values = {
            module: section["ros__parameters"][name]
            for module, section in cfg.items()
            if name in section["ros__parameters"]
        }
        assert len(values) >= 2
        assert len(set(map(repr, values.values()))) == 1, (name, values)


def test_must_match_params_are_consistent():
    cfg = _config()
    for key, modules in MUST_MATCH:
        values = {}
        for module in modules:
            section = cfg[module]["ros__parameters"]
            assert key in section, f"{module} 的 yaml 段缺 {key}"
            values[module] = section[key]
        assert len(set(map(repr, values.values()))) == 1, (key, values)


def test_duplicated_param_names_are_classified():
    cfg = _config()
    seen = {}
    for module, section in cfg.items():
        for key in section["ros__parameters"]:
            seen.setdefault(key, set()).add(module)
    duplicated = {key: modules for key, modules in seen.items() if len(modules) > 1}
    classified = PAIRED_TOPIC_PARAMS | {key for key, _ in MUST_MATCH} | {"violation_cycles"}
    unclassified = {
        key: sorted(modules)
        for key, modules in duplicated.items()
        if key not in classified
    }
    assert not unclassified, unclassified


def test_time_window_inequalities():
    cfg = _config()

    def value(module, key):
        return cfg[module]["ros__parameters"][key]

    assert value("docking_pose_estimator", "anchor_prediction_timeout_sec") >= (
        value("docking_safety", "abort_exit_odom_timeout") + ABORT_EXIT_MARGIN_SEC
    )
    assert value("docking_safety", "align_odom_prediction_timeout") > value(
        "docking_fsm", "align_tag_loss_grace_sec"
    )


def test_safety_rate_reduction_preserves_debounce_windows():
    cfg = _config()
    fsm = cfg["docking_fsm"]["ros__parameters"]
    safety = cfg["docking_safety"]["ros__parameters"]
    assert fsm["state_rate"] == 10.0
    assert fsm["violation_cycles"] == 10
    assert safety["check_rate"] == 10.0
    assert safety["violation_cycles"] == 5
    assert fsm["violation_cycles"] / fsm["state_rate"] == 1.0
    assert safety["violation_cycles"] / safety["check_rate"] == 0.5
