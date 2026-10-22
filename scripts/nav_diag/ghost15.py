# -*- coding: utf-8 -*-
"""列出 /tf 上 odom→base_link（和 map→odom）的所有不同取值族，判断是否有重复发布者（只读）。"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    rclpy.init()
    node = Node("codex_ghost15")
    fam: dict = {}
    stamps: dict = {}

    def on_tf(msg: TFMessage) -> None:
        for t in msg.transforms:
            key = f"{t.header.frame_id}->{t.child_frame_id}"
            if key not in ("odom->base_link", "map->odom"):
                continue
            tr, q = t.transform.translation, t.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            v = (round(tr.x, 2), round(tr.y, 2), round(yaw, 1))
            fam.setdefault(key, {})
            fam[key][v] = fam[key].get(v, 0) + 1
            st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            lo, hi = stamps.get(key, (st, st))
            stamps[key] = (min(lo, st), max(hi, st))

    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()
    for key in ("odom->base_link", "map->odom"):
        d = fam.get(key, {})
        lo, hi = stamps.get(key, (0, 0))
        print(f"\n{key}: 不同取值 {len(d)} 种，总条数 {sum(d.values())}，"
              f"stamp 范围 age [{now - hi:+.2f}s, {now - lo:+.2f}s]")
        for v, n in sorted(d.items(), key=lambda kv: -kv[1])[:8]:
            print(f"    (x={v[0]:+.2f}, y={v[1]:+.2f}, yaw={v[2]:+.1f}°)  ×{n}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
