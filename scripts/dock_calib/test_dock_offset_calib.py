#!/usr/bin/env python3
"""dock_offset_calib.py 的纯数学/YAML 自测（不需要 ROS）。

运行：
    cd scripts/dock_calib
    python3 -m unittest -v test_dock_offset_calib
或：
    python3 test_dock_offset_calib.py
"""

from __future__ import annotations

import math
import unittest

import numpy as np
import yaml

import dock_offset_calib as d


class MathTests(unittest.TestCase):
    def test_rpy_roundtrip(self) -> None:
        rng = np.random.default_rng(42)
        for _ in range(300):
            roll, pitch, yaw = rng.uniform(-math.pi, math.pi, 3)
            T = d.transform_from_xyz_rpy(0.3, -1.2, 2.0, roll, pitch, yaw)
            x, y, z, r, p, yw = d.transform_to_xyz_rpy(T, degrees=False)
            self.assertAlmostEqual(x, 0.3)
            self.assertAlmostEqual(y, -1.2)
            self.assertAlmostEqual(z, 2.0)
            self.assertTrue(
                np.allclose(
                    d.rotation_matrix_from_rpy(r, p, yw),
                    T[:3, :3],
                    atol=1e-9,
                )
            )

    def test_inverse(self) -> None:
        rng = np.random.default_rng(43)
        for _ in range(100):
            T = d.transform_from_xyz_rpy(
                *rng.normal(0.0, 1.0, 3),
                *rng.uniform(-math.pi, math.pi, 3),
            )
            self.assertTrue(np.allclose(T @ d.invert_transform(T), np.eye(4), atol=1e-9))

    def test_synthetic_calibration(self) -> None:
        true_offsets = {
            0: d.transform_from_xyz_rpy(
                -0.666, 0.724, -0.649,
                math.radians(-0.19), math.radians(-0.25), math.radians(-0.26),
            ),
            1: d.transform_from_xyz_rpy(
                -0.688, -0.540, -0.649,
                math.radians(0.21), math.radians(0.34), math.radians(0.15),
            ),
        }
        target = np.eye(4)
        rng = np.random.default_rng(44)
        samples = {}
        for tid, T_tag_dock in true_offsets.items():
            T_base_tag = d.invert_transform(T_tag_dock) @ target
            seq = []
            for _ in range(80):
                noise = d.transform_from_xyz_rpy(
                    *rng.normal(0.0, 0.002, 3),
                    *rng.normal(0.0, math.radians(0.05), 3),
                )
                seq.append(noise @ T_base_tag)
            samples[tid] = seq

        results = d.calibrate_from_samples(samples, target, min_samples=20)
        for tid, expected in true_offsets.items():
            t_err, r_err = d.transform_error(expected, results[tid].transform)
            self.assertLess(t_err, 0.01, f"tag_{tid} 平移误差 {t_err}")
            self.assertLess(r_err, 0.2, f"tag_{tid} 旋转误差 {r_err}")

    def test_offsets_roundtrip(self) -> None:
        off = {
            "dock_offset_x": 1.2,
            "dock_offset_y": -0.3,
            "dock_offset_z": 0.4,
            "dock_offset_roll": 1.5,
            "dock_offset_pitch": -2.0,
            "dock_offset_yaw": 3.0,
        }
        T = d.offsets_to_transform(off)
        back = d.transform_to_offsets(T)
        for key in d.OFFSET_KEYS:
            self.assertAlmostEqual(off[key], back[key], places=5)


class ConfigUpdateTests(unittest.TestCase):
    SAMPLE = """apriltag_node:
  ros__parameters:
    tag_ids: [1, 0]
    tag_1: # tag id 1
      dock_offset_x: -0.688 #单位m
      dock_offset_y: -0.540
      dock_offset_z: -0.649
      dock_offset_roll: 0.21
      dock_offset_pitch: 0.34
      dock_offset_yaw: 0.15
    tag_0: # tag id 0
      dock_offset_x: -0.666
      dock_offset_y: 0.724
      dock_offset_z: -0.649
      dock_offset_roll: -0.19
      dock_offset_pitch: -0.25
      dock_offset_yaw: -0.26
"""

    def test_update_preserves_comments_and_values(self) -> None:
        new_offsets = {
            0: {k: 2.0 for k in d.OFFSET_KEYS},
            1: {k: 1.0 for k in d.OFFSET_KEYS},
        }
        updated, changes = d.update_config_text(self.SAMPLE, new_offsets)
        self.assertIn("#单位m", updated)
        self.assertIn("tag_1: # tag id 1", updated)
        self.assertEqual(len(changes), 12)

        parsed = yaml.safe_load(updated)["apriltag_node"]["ros__parameters"]
        for tid, expected in ((0, 2.0), (1, 1.0)):
            for key in d.OFFSET_KEYS:
                self.assertAlmostEqual(float(parsed[f"tag_{tid}"][key]), expected)

    def test_update_rejects_missing_keys(self) -> None:
        broken = self.SAMPLE.replace("      dock_offset_yaw: 0.15\n", "")
        with self.assertRaises(d.ConfigError):
            d.update_config_text(
                broken,
                {0: {k: 2.0 for k in d.OFFSET_KEYS}, 1: {k: 1.0 for k in d.OFFSET_KEYS}},
            )


class SelfTestTests(unittest.TestCase):
    def test_builtin_self_test(self) -> None:
        self.assertEqual(d.run_self_test(), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
