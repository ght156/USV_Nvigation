# -*- coding: utf-8 -*-
"""点云 vs 局部代价图：逐帧取 TF（船在动也准），按代价图真实参数复算，只读。

参数（取自运行中的 /local_costmap/local_costmap）：
  obstacle_min_range=2.0, max=20.0（3D 距离）；obs 高度 [-0.25, 1.5]；
  voxel layer 高度 [0.0, 1.0]，z_res=0.1，origin_z=-0.2，mark_threshold=1（z 体素列需 ≥2 层）；
  footprint [[2.72,0.95],[2.72,-0.95],[-0.4,-0.95],[-0.4,0.95]]，footprint_clearing_enabled=true。
输出：命中/期望未标/多标 计数、最近邻距离、最佳整体平移、船体系 ASCII 叠加。
用法：python3 /tmp/ghost12.py [秒数]
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

BEST = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

MIN_R, MAX_R = 2.0, 20.0
MIN_Z, MAX_Z = 0.0, 1.0
Z_RES, ORIGIN_Z = 0.1, -0.2


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rclpy.init()
    node = Node("codex_ghost12")
    ob, bl, grids, clouds = [], None, [], []

    def on_tf(msg: TFMessage) -> None:
        nonlocal bl
        for t in msg.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                st = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                ob.append((st, tr.x, tr.y, tr.z, yaw_of(q)))
            elif t.header.frame_id == "base_link" and t.child_frame_id == "livox_frame":
                bl = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)

    def on_grid(msg: OccupancyGrid) -> None:
        grids.append((time.time(), msg.info.width, msg.info.height, msg.info.resolution,
                      msg.info.origin.position.x, msg.info.origin.position.y,
                      tuple(msg.data)))

    def on_cloud(msg: PointCloud2) -> None:
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        step, n = msg.point_step, msg.width * msg.height
        clouds.append((st, [struct.unpack_from("<fff", msg.data, i * step) for i in range(n)]))

    node.create_subscription(TFMessage, "/tf", on_tf, BEST)
    node.create_subscription(TFMessage, "/tf_static", on_tf, LATCHED)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_grid, 10)
    node.create_subscription(PointCloud2, "/livox/lidar", on_cloud, BEST)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not ob or bl is None or not grids or not clouds:
        print("数据不全:", len(ob), bl, len(grids), len(clouds))
        return
    ob.sort()
    print(f"base_link→livox_frame = {bl}")
    if abs(bl[3]) > 0.5:
        print("  （含 roll≈180°，倒装补偿在 TF 里 ✓）")

    def pose_at(stamp):
        best = ob[0]
        for r in ob:
            if r[0] <= stamp + 1e-6:
                best = r
            else:
                break
        return best

    (_, w, h, res, ox, oy, gdata) = grids[-1]
    lethal = {i for i, v in enumerate(gdata) if v >= 99}
    print(f"代价图 {w}x{h}@{res:.2f} origin=({ox:+.2f},{oy:+.2f})  致命格={len(lethal)}")

    def cell_of(x, y):
        gx, gy = int((x - ox) / res), int((y - oy) / res)
        return gy * w + gx if 0 <= gx < w and 0 <= gy < h else None

    # —— 逐帧把点云折到 odom（用该帧 stamp 的 TF），再套代价图过滤 ——
    exp_cells: dict[int, set] = {}
    pts_frame = []          # (x_odom, y_odom, z_odom) 供画图
    cnt = {"total": 0, "near": 0, "far": 0, "low": 0, "high": 0, "foot": 0, "used": 0,
           "out": 0}
    for st, pts in clouds:
        _, bx, by, bz, byaw = pose_at(st)
        c, s = math.cos(byaw), math.sin(byaw)
        for (x, y, z) in pts:
            cnt["total"] += 1
            r3 = math.sqrt(x * x + y * y + z * z)
            if r3 < MIN_R:
                cnt["near"] += 1
                continue
            if r3 > MAX_R:
                cnt["far"] += 1
                continue
            # livox_frame 里 roll=π 已由 TF 承担：点先翻 y,z，再加杆臂
            py_, pz_ = -y, -z
            gx = bx + c * (bl[0] + x) - s * py_
            gy = by + s * (bl[0] + x) + c * py_
            gz = bz + bl[2] + pz_
            if gz < MIN_Z:
                cnt["low"] += 1
                continue
            if gz > MAX_Z:
                cnt["high"] += 1
                continue
            fx = c * (gx - bx) + s * (gy - by)      # base_link 系
            fy = -s * (gx - bx) + c * (gy - by)
            if (-0.4 - 0.05 <= fx <= 2.72 + 0.05) and abs(fy) <= 0.95 + 0.05:
                cnt["foot"] += 1
                continue
            cell = cell_of(gx, gy)
            if cell is None:
                cnt["out"] += 1
                continue
            cnt["used"] += 1
            pts_frame.append((fx, fy))
            exp_cells.setdefault(cell, set()).add(int((gz - ORIGIN_Z) / Z_RES))

    exp = {c for c, zs in exp_cells.items() if len(zs) > 1}
    # z 直方图（odom，0.1m 一档）与「每列 z 层数」分布
    zhist: dict[int, int] = {}
    for st, pts in clouds:
        _, bx, by, bz, byaw = pose_at(st)
        c, s = math.cos(byaw), math.sin(byaw)
        for (x, y, z) in pts:
            r3 = math.sqrt(x * x + y * y + z * z)
            if r3 < MIN_R or r3 > MAX_R:
                continue
            gz = bz + bl[2] + (-z)
            zhist[int(math.floor(gz / 0.1))] = zhist.get(int(math.floor(gz / 0.1)), 0) + 1
    print("幸存点云的 odom z 直方图（0.1 m 一档，仅 2~20 m 距离内）：")
    for k in sorted(zhist):
        bar = "#" * min(60, zhist[k] // 300)
        print(f"  z∈[{k*0.1:+.1f},{k*0.1+0.1:+.1f})  {zhist[k]:7d}  {bar}")
    layers = [len(zs) for zs in exp_cells.values()]
    dist: dict[int, int] = {}
    for v in layers:
        dist[v] = dist.get(v, 0) + 1
    print("每列（x,y 格）被占的 z 层数分布：", dict(sorted(dist.items())))

    print(f"点云过滤（{len(clouds)} 帧）：总数 {cnt['total']} | 盲区<2m {cnt['near']} | "
          f">20m {cnt['far']} | z<0 {cnt['low']} | z>1 {cnt['high']} | footprint 内 {cnt['foot']} | "
          f"出窗 {cnt['out']} | 用上 {cnt['used']}")
    print(f"期望标记格={len(exp)}（z 体素列 ≥2 层）")
    print(f"  ✔ 期望且已标 = {len(exp & lethal)}")
    print(f"  ✘ 期望但没标 = {len(exp - lethal)}")
    print(f"  ? 标了但点云不支持 = {len(lethal - exp)}")

    def xy_of(cells):
        return [((c % w) * res + ox + res / 2, (c // w) * res + oy + res / 2) for c in cells]

    ex_xy = xy_of(exp & lethal) or xy_of(exp)
    le_xy = xy_of(lethal - exp)
    if ex_xy and le_xy:
        # 多标格到最近「期望格」的距离
        ds = []
        for (x, y) in le_xy:
            ds.append(min(math.hypot(x - a, y - b) for (a, b) in ex_xy))
        ds.sort()
        n = len(ds)
        print(f"\n多标格到最近期望格距离：中位={ds[n//2]:.2f} m  "
              f"10%={ds[n//10]:.2f}  90%={ds[int(n*0.9)]:.2f}  max={ds[-1]:.2f}")
        best = None
        for dxi in range(-30, 31):
            for dyi in range(-30, 31):
                dx, dy = dxi * 0.2, dyi * 0.2
                keys = {(round(x + dx, 1), round(y + dy, 1)) for (x, y) in xy_of(exp)}
                hit = sum(1 for (x, y) in le_xy if (round(x, 1), round(y, 1)) in keys)
                if best is None or hit > best[0]:
                    best = (hit, dx, dy)
        base = sum(1 for (x, y) in le_xy
                   if (round(x, 1), round(y, 1)) in {(round(a, 1), round(b, 1)) for a, b in xy_of(exp)})
        print(f"整体平移搜索：不平移命中 {base}/{len(le_xy)}；最佳平移 ({best[1]:+.1f},{best[2]:+.1f}) m "
              f"命中 {best[0]}/{len(le_xy)}")

    # 船体系 ASCII
    print("\n船体系 ASCII（0.5m/格，上=船头，右=右舷）：# 代价图致命格  o 点云期望标记  *=重合  B 船")
    half, cell = 10.0, 0.5
    W = int(2 * half / cell)
    grid = [[" "] * W for _ in range(W)]
    _, bx, by, _, byaw = ob[-1]
    c, s = math.cos(-byaw), math.sin(-byaw)
    for (fx, fy) in pts_frame:
        gx, gy = int((fx + half) / cell), int((-fy + half) / cell)
        if 0 <= gx < W and 0 <= gy < W:
            grid[gy][gx] = "o"
    for cid in lethal:
        px = ox + ((cid % w) + 0.5) * res
        py = oy + ((cid // w) + 0.5) * res
        fx = c * (px - bx) - s * (py - by)
        fy = s * (px - bx) + c * (py - by)
        gx, gy = int((fx + half) / cell), int((-fy + half) / cell)
        if 0 <= gx < W and 0 <= gy < W:
            grid[gy][gx] = "*" if grid[gy][gx] == "o" else "#"
    grid[W // 2][W // 2] = "B"
    for row in grid[::-1]:
        print("  " + "".join(row))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
