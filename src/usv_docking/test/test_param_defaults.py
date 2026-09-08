#!/usr/bin/env python3
"""防默认值漂移：四节点代码 declare_parameter 默认值须与
config/docking.yaml 逐项一致（脱离 yaml 运行时，代码默认值即配置；
历史上 approach_y_tol 默认 0.5 vs yaml 0.15 会造成 APPROACH<->ALIGN
慢性循环）。"""

from pathlib import Path

import pytest
import rclpy
import yaml

from usv_docking.docking_fsm import DockingFsm
from usv_docking.docking_motion_controller import DockingMotionController
from usv_docking.docking_pose_estimator import DockingPoseEstimator
from usv_docking.docking_safety import DockingSafety


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_code_defaults_match_yaml(ros_context):
    yaml_path = Path(__file__).resolve().parents[1] / "config" / "docking.yaml"
    cfg = yaml.safe_load(yaml_path.read_text())
    for name, cls in [
        ("docking_pose_estimator", DockingPoseEstimator),
        ("docking_fsm", DockingFsm),
        ("docking_motion_controller", DockingMotionController),
        ("docking_safety", DockingSafety),
    ]:
        node = cls()
        try:
            mismatches = []
            for key, expected in cfg[name]["ros__parameters"].items():
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
