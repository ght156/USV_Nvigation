# -*- coding: utf-8 -*-
"""代价图/点云 vs 静态地图岸线：量距离分布，判断有没有整体错位或镜像（只读）。"""
from __future__ import annotations

import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

BEST = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Node("codex_ghost16")
    st, static, cm, clouds = {}, {}, [], []

    def on_tf(msg: TFMessage) -> None:
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key in (("map", "odom"), ("odom", "base_link"), ("base_link", "livox_frame")):
                tr = t.transform.translation
                st[key] = (tr.x, tr.y, tr.z, yaw_of(t.transform.rotation))

    def on_map(msg: OccupancyGrid) -> None:
        static.update({"w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
                       "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
                       "data": tuple(msg.data)})

    def on_cm(msg: OccupancyGrid) -> None:
        cm.append(msg)

    def on_cloud(msg: PointCloud2) -> None:
        step, n = msg.point_step, msg.width * msg.height
        clouds.append([struct.unpack_from("<fff", msg.data, i * step) for i in range(n)])

    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    node.create_subscription(TFMessage, "/tf_static", on_tf, LATCHED)
    node.create_subscription(OccupancyGrid, "/map", on_map, LATCHED)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_cm, 10)
    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, BEST)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    need = [("map", "odom"), ("odom", "base_link"), ("base_link", "livox_frame")]
    if any(k not in st for k in need) or not static or not cm or not clouds:
        print("数据不全")
        return
    sw, sh, sres = static["w"], static["h"], static["res"]
    sox, soy = static["ox"], static["oy"]

    # 静态地图障碍点集合（降采样成 0.5 m 网格，加快最近邻查询）
    occ = set()
    for gy in range(0, sh, 2):
        for gx in range(0, sw, 2):
            if static["data"][gy * sw + gx] >= 65:
                occ.add((round(sox + (gx + 0.5) * sres, 1), round(soy + (gy + 0.5) * sres, 1)))
    occ_l = sorted(occ)

    def nearest(p):
        best = 1e9
        for q in occ_l:
            d = (q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2
            if d < best:
                best = d
        return math.sqrt(best)

    (mx, my, _, myaw) = st[("map", "odom")]
    (bx, by, _, byaw) = st[("odom", "base_link")]
    (lx, ly, _, _) = st[("base_link", "livox_frame")]
    c, s = math.cos(myaw), math.sin(myaw)

    def odom2map(px, py):
        return (mx + c * px - s * py, my + s * px + c * py)

    g = cm[-1]
    lethal = [(g.info.origin.position.x + ((i % g.info.width) + 0.5) * g.info.resolution,
               g.info.origin.position.y + ((i // g.info.width) + 0.5) * g.info.resolution)
              for i, v in enumerate(g.data) if v >= 99]
    print(f"代价图致命格={len(lethal)}")
    if lethal:
        ds = sorted(nearest(odom2map(x, y)) for x, y in lethal[::max(1, len(lethal) // 400)])
        n = len(ds)
        print(f"  致命格到静态岸线距离：中位={ds[n//2]:.2f} m  "
              f"25%={ds[n//4]:.2f}  75%={ds[3*n//4]:.2f}  max={ds[-1]:.2f}")
        for th in (0.3, 0.5, 1.0, 2.0, 5.0):
            print(f"    ≤{th} m: {100 * sum(1 for d in ds if d <= th) / n:.0f}%")

    # 点云（经 TF 到 map）到岸线距离
    cl = []
    for pts in clouds:
        for (x, y, z) in pts:
            if math.sqrt(x * x + y * y + z * z) < 3.0:
                continue
            px = bx + math.cos(byaw) * (lx + x) - math.sin(byaw) * (-y)
            py = by + math.sin(byaw) * (lx + x) + math.cos(byaw) * (-y)
            cl.append(odom2map(px, py))
    if cl:
        step = max(1, len(cl) // 500)
        ds = sorted(nearest(p) for p in cl[::step])
        n = len(ds)
        print(f"点云(≥3m，{len(cl)} 点，抽样 {n})到静态岸线距离：中位={ds[n//2]:.2f} m  "
              f"25%={ds[n//4]:.2f}  75%={ds[3*n//4]:.2f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
