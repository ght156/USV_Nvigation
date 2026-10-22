#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看 /tf 与 /tf_static 上到底有哪些边、base_link→livox_frame 的四元数分量（只读）。"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)
VOL = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST)


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Node("codex_ghost9")
    seen = {"static": {}, "tf": {}}

    def mk(tag):
        def cb(msg: TFMessage) -> None:
            for t in msg.transforms:
                key = f"{t.header.frame_id} -> {t.child_frame_id}"
                tr, q = t.transform.translation, t.transform.rotation
                seen[tag][key] = (round(tr.x, 4), round(tr.y, 4), round(tr.z, 4),
                                  round(q.x, 4), round(q.y, 4), round(q.z, 4),
                                  round(q.w, 4))
        return cb

    node.create_subscription(TFMessage, "/tf_static", mk("static"), LATCHED)
    node.create_subscription(TFMessage, "/tf", mk("tf"), VOL)
    print("/tf_static 发布者:", [(i.node_name) for i in
                                node.get_publishers_info_by_topic("/tf_static")])
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    for tag in ("static", "tf"):
        print(f"\n=== /{tag} 收到的边（{len(seen[tag])} 条）===")
        for k, v in sorted(seen[tag].items()):
            print(f"  {k:34s} xyz=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f}) "
                  f"quat(x,y,z,w)=({v[3]:+.3f},{v[4]:+.3f},{v[5]:+.3f},{v[6]:+.3f})")
    bl = seen["static"].get("base_link -> livox_frame") or seen["tf"].get("base_link -> livox_frame")
    print(f"\nbase_link→livox_frame = {bl}")
    if bl:
        import math
        roll = math.degrees(math.atan2(2 * (bl[6] * bl[3] + bl[4] * bl[5]),
                                       1 - 2 * (bl[3] ** 2 + bl[4] ** 2)))
        print(f"→ 等价 roll ≈ {roll:+.1f}°（±180 就是倒装补偿在起作用）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
