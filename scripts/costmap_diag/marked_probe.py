#!/usr/bin/env python3
"""采样 /local_costmap/voxel_marked_cloud：统计真正参与打标的点的 z/r 分布。"""

import math
import time

import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def pct(v, q):
    return v[min(len(v) - 1, int(q * len(v)))]


def main():
    rclpy.init()
    n = rclpy.create_node("marked_probe")
    got = {}
    n.create_subscription(
        PointCloud2,
        "/local_costmap/voxel_marked_cloud",
        lambda m: got.__setitem__("m", m),
        10,
    )
    t0 = time.time()
    while "m" not in got and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.5)
    m = got.get("m")
    if m is None:
        print("NO voxel_marked_cloud")
        return
    pts = [
        p
        for p in point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
    ]
    if not pts:
        print("marked cloud EMPTY（当前没有任何标记障碍的点）")
        return
    n_ = len(pts)
    zs = sorted(p[2] for p in pts)
    rs = sorted(math.hypot(p[0], p[1]) for p in pts)
    print(f"frame={m.header.frame_id} points={n_}")
    print(
        f"z: min={zs[0]:.2f} p5={pct(zs, 0.05):.2f} p50={pct(zs, 0.5):.2f} "
        f"p95={pct(zs, 0.95):.2f} max={zs[-1]:.2f}"
    )
    print(
        f"r: p5={pct(rs, 0.05):.2f} p50={pct(rs, 0.5):.2f} "
        f"p95={pct(rs, 0.95):.2f} max={rs[-1]:.1f}"
    )
    below0 = sum(1 for z in zs if z < 0.0)
    below_015 = sum(1 for z in zs if z < -0.15)
    near3 = sum(1 for r in rs if r < 3.0)
    print(
        f"z<0: {below0} ({100.0 * below0 / n_:.1f}%)  "
        f"z<-0.15: {below_015}  r<3m: {near3} ({100.0 * near3 / n_:.1f}%)"
    )


if __name__ == "__main__":
    main()
