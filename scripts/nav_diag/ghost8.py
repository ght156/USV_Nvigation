#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""船体系左/右判定：静态岸线在左还是右？代价图标记在左还是右？点云镜像假设对照（只读）。

三个统计都以「船体系」为准（+x 前、+y 左，符合 ROS 右手系）：
  1) 静态地图的岸线格：左/右计数  —— 真值基准
  2) 局部代价图致命格：左/右计数  —— nav2 标出来的
  3) 点云落点：真实 vs 镜像(y→-y) 各自落在静态岸线上的比例 —— 哪个假设更自洽
用法：python3 /tmp/ghost8.py [秒数]
"""
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
    node = Node("codex_ghost8")
    st = {"mo": None, "ob": None, "bl": None}
    local, static, clouds = {}, {}, []

    def on_tf(msg: TFMessage) -> None:
        for t in msg.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            key = (t.header.frame_id, t.child_frame_id)
            if key == ("map", "odom"):
                st["mo"] = (tr.x, tr.y, yaw_of(q))
            elif key == ("odom", "base_link"):
                st["ob"] = (tr.x, tr.y, yaw_of(q))
            elif key == ("base_link", "livox_frame"):
                st["bl"] = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)

    def mk(store):
        def cb(msg: OccupancyGrid) -> None:
            store.update({"w": msg.info.width, "h": msg.info.height,
                          "res": msg.info.resolution,
                          "ox": msg.info.origin.position.x,
                          "oy": msg.info.origin.position.y,
                          "data": tuple(msg.data)})
        return cb

    def on_cloud(msg: PointCloud2) -> None:
        step, n = msg.point_step, msg.width * msg.height
        clouds.append([struct.unpack_from("<fff", msg.data, i * step) for i in range(n)])

    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    # /tf_static 是 latched（TRANSIENT_LOCAL），晚加入必须用同种 durability 才收得到
    node.create_subscription(TFMessage, "/tf_static", on_tf, LATCHED)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", mk(local), 10)
    node.create_subscription(OccupancyGrid, "/map", mk(static), LATCHED)
    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, BEST)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not local or not static or not st["mo"] or not st["ob"]:
        print("数据不全", bool(local), bool(static), st)
        return
    print(f"base_link→livox_frame: {st['bl']}")
    mx, my, myaw = st["mo"]
    bx, by, byaw = st["ob"]
    lx_o, ly_o = st["bl"][0], st["bl"][1] if st["bl"] else (1.6, 0.0)
    c, s = math.cos(byaw), math.sin(byaw)
    lidar_odom = (bx + c * lx_o - s * ly_o, by + s * lx_o + c * ly_o)
    lidar_map = (mx + math.cos(myaw) * lidar_odom[0] - math.sin(myaw) * lidar_odom[1],
                 my + math.sin(myaw) * lidar_odom[0] + math.cos(myaw) * lidar_odom[1])
    print(f"船(map)=({mx + math.cos(myaw)*bx - math.sin(myaw)*by:+.2f},"
          f"{my + math.sin(myaw)*bx + math.cos(myaw)*by:+.2f})  "
          f"yaw_map={math.degrees(myaw + byaw):+.1f}°")

    mc, ms = math.cos(myaw), math.sin(myaw)

    def odom_to_boat(px: float, py: float):
        dx, dy = px - bx, py - by
        return (c * dx + s * dy, -s * dx + c * dy)     # R(-byaw)

    # 1) 静态岸线：窗口内、船体系左/右
    w, h, res, ox, oy = (local["w"], local["h"], local["res"],
                         local["ox"], local["oy"])
    sw, sh, sres = static["w"], static["h"], static["res"]
    sox, soy = static["ox"], static["oy"]

    def sval(px: float, py: float):
        gx = int((px - sox) / sres)
        gy = int((py - soy) / sres)
        if 0 <= gx < sw and 0 <= gy < sh:
            return static["data"][gy * sw + gx]
        return -1

    land = {"左": 0, "右": 0, "前": 0}
    for gy in range(0, h, 2):
        for gx in range(0, w, 2):
            px = ox + (gx + 0.5) * res
            py = oy + (gy + 0.5) * res
            sx = mx + mc * px - ms * py
            sy = my + ms * px + mc * py
            if sval(sx, sy) < 65:
                continue
            fx, fy = odom_to_boat(px, py)      # 船体系
            d = math.hypot(fx, fy)
            if d > 12:
                continue
            if abs(fy) < 1.0:
                land["前"] += 1
            elif fy > 0:
                land["左"] += 1
            else:
                land["右"] += 1

    # 2) 代价图致命格：船体系左/右
    cm_side = {"左": 0, "右": 0}
    infl_side = {"左": 0, "右": 0}
    for gy in range(h):
        for gx in range(w):
            v = local["data"][gy * w + gx]
            if v <= 0:
                continue
            px = ox + (gx + 0.5) * res
            py = oy + (gy + 0.5) * res
            fx, fy = odom_to_boat(px, py)
            if math.hypot(fx, fy) > 12:
                continue
            key = "左" if fy > 0 else "右"
            if v >= 99:
                cm_side[key] += 1
            else:
                infl_side[key] += 1

    print(f"\n静态岸线格（|x|<12m，按船体系分侧）：左={land['左']}  右={land['右']}  "
          f"正前/正后带(|y|<1m)={land['前']}")
    print(f"代价图致命格：左={cm_side['左']}  右={cm_side['右']}")
    print(f"代价图膨胀格：左={infl_side['左']}  右={infl_side['右']}")
    if land["左"] + land["右"]:
        print(f"→ 岸线重心偏 {'左' if land['左'] > land['右'] else '右'} 侧；"
              f"代价图重心偏 {'左' if cm_side['左'] > cm_side['右'] else '右'} 侧")

    # 3) 点云：真实 vs 镜像(y→-y) 落在岸线上的比例
    on_real = on_mirror = tot = 0
    for pts in clouds:
        for (x, y, z) in pts:
            if math.sqrt(x * x + y * y + z * z) < 0.5:
                continue
            for mirror in (False, True):
                yy = -y if mirror else y
                bx_ = bx + c * (lx_o + x) - s * yy
                by_ = by + s * (lx_o + x) + c * yy
                px_ = mx + mc * bx_ - ms * by_
                py_ = my + ms * bx_ + mc * by_
                hit = sval(px_, py_) >= 65
                if mirror:
                    on_mirror += hit
                else:
                    on_real += hit
            tot += 1
    print(f"\n点云(≥0.5m，{len(clouds)} 帧/{tot} 点)落到静态岸线上的比例：")
    print(f"  真实     : {on_real}/{tot} = {100*on_real/max(tot,1):.1f}%")
    print(f"  镜像 y→-y: {on_mirror}/{tot} = {100*on_mirror/max(tot,1):.1f}%")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
