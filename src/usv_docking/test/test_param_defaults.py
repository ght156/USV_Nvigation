#!/usr/bin/env python3
"""防默认值漂移：四节点代码 declare_parameter 默认值须与
config/docking.yaml 逐项一致（脱离 yaml 运行时，代码默认值即配置；
历史上 approach_y_tol 默认 0.5 vs yaml 0.15 会造成 APPROACH<->ALIGN
慢性循环）。

另两项约束：
  - yaml 必须是**完整**配置面：代码声明的每个参数都要能在 yaml 里改
    （2026-10-07 之前控制器有 5 个参数只存在于代码，现场改 yaml 改不到）；
  - 豁免项（安全联锁类的刻意不一致）必须在"确实不一致"时才成立，
    一旦被对齐就应删除豁免，防豁免清单变成垃圾桶。
"""

from pathlib import Path

import pytest
import rclpy
import yaml

from usv_docking.docking_fsm import DockingFsm
from usv_docking.docking_motion_controller import DockingMotionController
from usv_docking.docking_pose_estimator import DockingPoseEstimator
from usv_docking.docking_safety import DockingSafety

NODES = [
    ("docking_pose_estimator", DockingPoseEstimator),
    ("docking_fsm", DockingFsm),
    ("docking_motion_controller", DockingMotionController),
    ("docking_safety", DockingSafety),
]

# 刻意与 yaml 不一致的代码默认值，仅限"更安全方向"的联锁，逐项说明理由。
INTENTIONAL_EXEMPTIONS = {
    ("docking_motion_controller", "test_only"): (
        "安全联锁：不带参数文件裸跑时只发 /docking/cmd_vel_test，不驱动实船；"
        "yaml/launch 显式给 false 才接管 /cmd_vel_nav"
    ),
}

# rclpy 自动声明的内置参数（由 launch 传参，不属于 yaml 配置面）
BUILTIN_PARAMS = {"use_sim_time"}


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _yaml_config():
    yaml_path = Path(__file__).resolve().parents[1] / "config" / "docking.yaml"
    return yaml.safe_load(yaml_path.read_text())


def test_code_defaults_match_yaml(ros_context):
    cfg = _yaml_config()
    for name, cls in NODES:
        node = cls()
        try:
            mismatches = []
            for key, expected in cfg[name]["ros__parameters"].items():
                if (name, key) in INTENTIONAL_EXEMPTIONS:
                    continue
                actual = node.get_parameter(key).value
                if isinstance(expected, float):
                    ok = isinstance(actual, float) and actual == pytest.approx(
                        expected
                    )
                else:
                    ok = actual == expected
                if not ok:
                    mismatches.append(
                        f"{name}.{key}: code={actual!r} yaml={expected!r}"
                    )
            assert not mismatches, "\n".join(mismatches)
        finally:
            node.destroy_node()


def test_yaml_covers_all_declared_params(ros_context):
    """yaml 是完整配置面：代码声明的参数若不在 yaml 里，现场就调不了。"""
    cfg = _yaml_config()
    for name, cls in NODES:
        node = cls()
        try:
            declared = set(node.get_parameters_by_prefix("").keys())
            declared -= BUILTIN_PARAMS
            missing = sorted(declared - set(cfg[name]["ros__parameters"]))
            assert not missing, (
                f"{name} 有 {len(missing)} 个参数只在代码里、yaml 改不到："
                f"{missing}"
            )
        finally:
            node.destroy_node()


def test_exemptions_are_not_stale(ros_context):
    """豁免项只在确实与 yaml 不一致时成立；一旦对齐就该删掉豁免。"""
    cfg = _yaml_config()
    by_name = dict(NODES)
    for (name, key), reason in INTENTIONAL_EXEMPTIONS.items():
        node = by_name[name]()
        try:
            actual = node.get_parameter(key).value
        finally:
            node.destroy_node()
        expected = cfg[name]["ros__parameters"][key]
        assert actual != expected, (
            f"{name}.{key} 的代码默认值已与 yaml 一致（={expected!r}），"
            f"请删除 INTENTIONAL_EXEMPTIONS 中的该豁免项。原豁免理由：{reason}"
        )
