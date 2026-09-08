#!/usr/bin/env python3
"""连续采样 costmap（默认 40s）：逐条消息统计 lethal 格数/最近障碍距离/更新时刻，看假障碍的间歇性。"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0


def main():
    rclpy.init()
    n = rclpy.create_node("ghost_watch")
    latest = {"cm": None}

    def cb(m):
        latest["cm"] = m

    n.create_subscription(
        OccupancyGrid,
        "/local_costmap/costmap",
        cb,
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    t0 = time.time()
    prev_header = None
    while time.time() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=1.0)
        m = latest["cm"]
        if m is None:
            continue
        if latest.get("printed") == id(m):
            continue
        latest["printed"] = id(m)
        data = m.data
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        cx, cy = ox + w * res / 2.0, oy + h * res / 2.0
        lethal = 0
        best = 1e9
        for iy in range(h):
            row = iy * w
            for ix in range(w):
                if data[row + ix] >= 100:
                    lethal += 1
                    d = math.hypot(
                        (ix + 0.5) * res + ox - cx,
                        (iy + 0.5) * res + oy - cy,
                    )
                    if d < best:
                        best = d
        nearest = f"{best:.1f}" if lethal else " - "
        print(f"[{time.time() - t0:5.1f}s] lethal={lethal:4d} 最近={nearest}", flush=True)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
