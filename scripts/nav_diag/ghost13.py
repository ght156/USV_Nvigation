# -*- coding: utf-8 -*-
"""时间戳对齐探针：点云 / TF / 代价图 各自的 stamp 与年龄，以及 /tf 发布者（只读）。"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, Imu
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


def stamp_of(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Node("codex_ghost13")
    rec = {"cloud": [], "imu": [], "cm": [], "ob": [], "mo": []}

    def on_cloud(msg: PointCloud2) -> None:
        rec["cloud"].append((stamp_of(msg.header), time.time(), msg.header.frame_id))

    def on_imu(msg: Imu) -> None:
        rec["imu"].append((stamp_of(msg.header), time.time()))

    def on_cm(msg: OccupancyGrid) -> None:
        rec["cm"].append((stamp_of(msg.header), time.time(), msg.info.origin.position.x,
                          msg.info.origin.position.y))

    def on_tf(msg: TFMessage) -> None:
        for t in msg.transforms:
            key = f"{t.header.frame_id} -> {t.child_frame_id}"
            if key == "odom -> base_link":
                rec["ob"].append((stamp_of(t.header), time.time(),
                                  t.transform.translation.x, t.transform.translation.y))
            elif key == "map -> odom":
                rec["mo"].append((stamp_of(t.header), time.time(),
                                  t.transform.translation.x, t.transform.translation.y))

    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, BEST)
    node.create_subscription(Imu, "/livox/imu", on_imu, BEST)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_cm, 10)
    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    node.create_subscription(TFMessage, "/tf_static", on_tf, LATCHED)

    print("/tf 发布者:", [i.node_name for i in node.get_publishers_info_by_topic("/tf")])
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()

    def summary(key, label, fmt):
        rows = rec[key]
        if not rows:
            print(f"{label}: 无数据")
            return None
        rows.sort()
        last = rows[-1]
        age_stamp = now - last[0]
        age_recv = now - last[1]
        print(f"{label}: n={len(rows)}  最新 stamp_age={age_stamp:+.3f}s  "
              f"接收_age={age_recv:+.3f}s   " + fmt(last))
        return last

    print()
    summary("cloud", "/livox/lidar ", lambda r: f"frame={r[2]}")
    summary("imu", "/livox/imu   ", lambda r: "")
    summary("ob", "odom→base_link", lambda r: f"x={r[2]:+.2f} y={r[3]:+.2f}")
    summary("mo", "map→odom     ", lambda r: f"x={r[2]:.2f} y={r[3]:.2f}")
    summary("cm", "/local_costmap/costmap", lambda r: f"origin=({r[2]:+.2f},{r[3]:+.2f})")

    if rec["cloud"] and rec["ob"]:
        c = max(r[0] for r in rec["cloud"])
        o = max(r[0] for r in rec["ob"])
        print(f"\n点云 stamp 与 TF stamp 之差 = {c - o:+.3f} s（正=点云更新）")
        print(f"点云最新 stamp 对应墙钟 {time.strftime('%H:%M:%S', time.localtime(c))}；"
              f"本机现在 {time.strftime('%H:%M:%S', time.localtime(now))}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
