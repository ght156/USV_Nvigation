#!/usr/bin/env python3
"""Recompute detection_cfg_sim.yml dock_offset from dock_2022/model.sdf geometry.

The apriltag node applies:
  T_cam_P = camera2camera_link * T_det * camera_tag2ros_ * dock_offset

dock_offset must be expressed in the frame AFTER camera_tag2ros_, not raw dock-model deltas.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

PI = math.pi
BAY2_P = (1.5, 9.0, 0.25)
PLACARD_Y = {0: 6.0, 43: 12.0}

# Same constants as apriltag_node.h
C2CL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
A2R = (R.from_euler("y", [-PI / 2]) * R.from_euler("x", [PI / 2])).as_matrix()


def T_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def chain(*transforms: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    for t in transforms:
        out = out @ t
    return out


def placard_tag_in_dock(placard_y: float) -> np.ndarray:
    return chain(
        T_xyz_rpy(5.75, placard_y, 1.5, 0, 0, PI / 2),
        T_xyz_rpy(0, -0.2, 0.25, 0, 0, PI),
        T_xyz_rpy(0, 0, 1, 0, 0, 0),
        T_xyz_rpy(0, 0.07, 0, 0, 0, 0),
        T_xyz_rpy(0, 0, 0.02, PI / 2, 0, PI),
    )


def compute_dock_offset(tag_id: int) -> dict[str, float]:
    placard_y = PLACARD_Y[tag_id]
    T_dock_tag = placard_tag_in_dock(placard_y)
    T_dock_P = T_xyz_rpy(*BAY2_P, 0, 0, 0)

    T_c2cl = np.eye(4)
    T_c2cl[:3, :3] = C2CL
    T_a2r = np.eye(4)
    T_a2r[:3, :3] = A2R

    # Ideal detection at ship spawn: dock @ world (-4,9,0,yaw=pi), camera_rear @ (-0.25,0,0.35,yaw=pi)
    T_world_dock = T_xyz_rpy(-4, 9, 0, 0, 0, PI)
    T_world_cam = T_xyz_rpy(-0.25, 0, 0.35, 0, 0, PI)
    T_world_tag = T_world_dock @ T_dock_tag
    T_cam_tag = np.linalg.inv(T_world_cam) @ T_world_tag
    T_cam_P = np.linalg.inv(T_world_cam) @ T_world_dock @ T_dock_P

    T_det = np.linalg.inv(T_c2cl) @ T_cam_tag
    T_yaml = np.linalg.inv(T_a2r) @ np.linalg.inv(T_det) @ np.linalg.inv(T_c2cl) @ T_cam_P

    t = T_yaml[:3, 3]
    rpy = R.from_matrix(T_yaml[:3, :3]).as_euler("xyz")
    return {
        "dock_offset_x": round(float(t[0]), 4),
        "dock_offset_y": round(float(t[1]), 4),
        "dock_offset_z": round(float(t[2]), 4),
        "dock_offset_roll": round(float(np.degrees(rpy[0])), 2),
        "dock_offset_pitch": round(float(np.degrees(rpy[1])), 2),
        "dock_offset_yaw": round(float(np.degrees(rpy[2])), 2),
    }


def main() -> None:
    print("# bay2 center P =", BAY2_P)
    print("# apriltag_node: T = c2cl * T_det * a2r * dock_offset\n")
    for tag_id in (0, 43):
        off = compute_dock_offset(tag_id)
        print(f"tag_{tag_id}:")
        for k, v in off.items():
            print(f"  {k}: {v}")
        print()


if __name__ == "__main__":
    main()
