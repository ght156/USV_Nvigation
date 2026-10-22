# -*- coding: utf-8 -*-
"""看 nav2 体素层到底标了什么：/local_costmap/voxel_grid 统计 + 代价图致命格（只读）。"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.msg import VoxelGrid
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Node("codex_ghost14")
    vg, cm, clouds, ob, bl = [], [], [], [], None

    def on_vg(msg: VoxelGrid) -> None:
        vg.append((time.time(), msg))

    def on_cm(msg: OccupancyGrid) -> None:
        cm.append((time.time(), msg))

    def on_cloud(msg: PointCloud2) -> None:
        clouds.append(msg)

    def on_tf(msg: TFMessage) -> None:
        nonlocal bl
        for t in msg.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                ob.append((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9,
                           t.transform.translation.x, t.transform.translation.y))
            elif t.header.frame_id == "base_link" and t.child_frame_id == "livox_frame":
                bl = (t.transform.translation.x, t.transform.translation.y,
                      t.transform.translation.z)

    node.create_subscription(VoxelGrid, "/local_costmap/voxel_grid", on_vg, 10)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_cm, 10)
    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, BEST)
    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    node.create_subscription(TFMessage, "/tf_static", on_tf, LATCHED)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    print("是否有 /local_costmap/voxel_grid:", "有" if vg else "无（没发布）")
    if vg:
        m = vg[-1][1]
        sz = m.size_x * m.size_y * m.size_z
        nz = [i for i, v in enumerate(m.data) if v > 0]
        print(f"体素网格 {m.size_x}x{m.size_y}x{m.size_z}"
              f"@({m.resolutions.x:.3f},{m.resolutions.y:.3f},{m.resolutions.z:.3f}) "
              f"origin=({m.origin.x:+.2f},{m.origin.y:+.2f},"
              f"{m.origin.z:+.2f})  非零体素={len(nz)} / {sz}")
        if nz:
            zs = {}
            for i in nz:
                z = i // (m.size_x * m.size_y)
                zs[z] = zs.get(z, 0) + 1
            print("  各 z 层非零体素数:", dict(sorted(zs.items())))
            xs = [(i % m.size_x) for i in nz]
            ys = [((i // m.size_x) % m.size_y) for i in nz]
            print(f"  x 索引范围 [{min(xs)},{max(xs)}] / {m.size_x}，"
                  f"y 索引范围 [{min(ys)},{max(ys)}] / {m.size_y}")
    if cm:
        g = cm[-1][1]
        lethal = sum(1 for v in g.data if v >= 99)
        pos = sum(1 for v in g.data if 0 < v < 99)
        print(f"代价图 {g.info.width}x{g.info.height}@0.1 origin="
              f"({g.info.origin.position.x:+.2f},{g.info.origin.position.y:+.2f})  "
              f"致命格={lethal} 膨胀格={pos}  age={time.time() - cm[-1][0]:+.2f}s")
    print(f"点云帧数={len(clouds)}  最新 frame="
          f"{clouds[-1].header.frame_id if clouds else '-'}  点数="
          f"{clouds[-1].width * clouds[-1].height if clouds else 0}")
    print(f"base_link→livox_frame={bl}   odom→base_link 最新="
          f"{tuple(round(v, 2) for v in ob[-1]) if ob else None}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
