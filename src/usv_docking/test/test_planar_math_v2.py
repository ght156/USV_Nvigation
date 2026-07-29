#!/usr/bin/env python3
"""docking_pose_estimator_v2 平面数学单元测试（不需要 ROS 节点/仿真）。"""

import math

from usv_docking.docking_pose_estimator_v2 import (
    compose_2d,
    inverse_2d,
    quat_to_planar_yaw,
    wrap_angle,
    yaw_to_quat,
)


def test_wrap_angle_basic():
    assert abs(wrap_angle(0.0)) < 1e-9
    assert abs(wrap_angle(math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap_angle(3.0 * math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap_angle(-3.0 * math.pi) - (-math.pi)) < 1e-9
    assert abs(wrap_angle(0.3) - 0.3) < 1e-9


def test_compose_identity():
    x, y, yaw = compose_2d(1.0, 2.0, 0.5, 0.0, 0.0, 0.0)
    assert abs(x - 1.0) < 1e-9
    assert abs(y - 2.0) < 1e-9
    assert abs(yaw - 0.5) < 1e-9


def test_compose_rotation():
    # A 系旋转 90°，B 在 A 系 (1,0) -> 世界 (0,1)
    x, y, yaw = compose_2d(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0)
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9


def test_compose_inverse_roundtrip():
    a = (1.5, -2.0, 0.7)
    b = (-0.5, 1.0, -1.2)
    ab = compose_2d(*a, *b)
    a_inv = inverse_2d(*a)
    b2 = compose_2d(*a_inv, *ab)
    assert abs(b2[0] - b[0]) < 1e-9
    assert abs(b2[1] - b[1]) < 1e-9
    assert abs(wrap_angle(b2[2] - b[2])) < 1e-9


def test_inverse_self_roundtrip():
    p = (2.0, 3.0, -0.9)
    ident = compose_2d(*p, *inverse_2d(*p))
    assert abs(ident[0]) < 1e-9
    assert abs(ident[1]) < 1e-9
    assert abs(wrap_angle(ident[2])) < 1e-9


def test_quat_planar_yaw_pure_yaw():
    for yaw in (-2.9, -1.0, 0.0, 0.5, 3.0):
        qx, qy, qz, qw = yaw_to_quat(yaw)
        got = quat_to_planar_yaw(qx, qy, qz, qw)
        assert abs(wrap_angle(got - yaw)) < 1e-6


def test_quat_planar_yaw_with_roll_pitch():
    """dock_frame 带 roll/pitch 时，用 X 轴投影取平面艏向仍应接近真值。"""
    yaw = 2.3
    roll, pitch = 0.12, -0.08
    # ZYX 顺序构造四元数
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    got = quat_to_planar_yaw(qx, qy, qz, qw)
    assert abs(wrap_angle(got - yaw)) < 0.02  # 小倾角下误差 ~1°
