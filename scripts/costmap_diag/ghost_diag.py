#!/usr/bin/env python3
"""水面假障碍诊断（开发机本地，DOMAIN 5 直连船端图）。

1) 原始 /livox/lidar 点云变换到 odom 后，过当前滤波链
   （r∈[2,20], z_odom∈[-0.25,1.5], voxel cap z≤1.0），统计幸存点 = 可能打成假障碍的点；
2) /local_costmap/costmap lethal 格的距离环/扇区分布（相对窗口中心≈船位）。
"""

import math
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

TL = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# 当前船上滤波链（与 nav2_params_real_mavros.yaml 一致）
R_MIN, R_MAX = 2.0, 20.0
Z_OBS_MIN, Z_OBS_MAX = -0.25, 1.5
Z_VOXEL_CAP = 1.0


def quat_to_mat(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def pct(v, q):
    return v[min(len(v) - 1, int(q * len(v)))]


def main():
    rclpy.init()
    n = rclpy.create_node("ghost_diag")
    got = {}
    n.create_subscription(
        OccupancyGrid, "/local_costmap/costmap", lambda m: got.__setitem__("cm", m), TL
    )
    n.create_subscription(
        PointCloud2, "/livox/lidar", lambda m: got.__setitem__("pc", m), 10
    )
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf, n)

    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        if "pc" in got and "cm" in got:
            break

    pc = got.get("pc")
    tf = None
    if pc is not None:
        # TF 树可能尚未就绪，重试等待静态变换
        for _ in range(30):
            try:
                tf = tfbuf.lookup_transform("odom", pc.header.frame_id, rclpy.time.Time())
                break
            except Exception:
                rclpy.spin_once(n, timeout_sec=0.3)

    if pc is not None and tf is None:
        print(f"TF odom→{pc.header.frame_id} 不可用，跳过点云分析")

    if pc is not None and tf is not None:
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_mat((q.x, q.y, q.z, q.w))
        surv_z, surv_r = [], []
        band = {"[-0.25,-0.1)": 0, "[-0.1,0)": 0, "[0,0.2)": 0,
                "[0.2,0.5)": 0, "[0.5,1.0]": 0}
        total = 0
        for p in point_cloud2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True):
            total += 1
            x = R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t.x
            y = R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t.y
            z = R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t.z
            r = math.hypot(x, y)
            if not (R_MIN <= r <= R_MAX and Z_OBS_MIN <= z <= Z_VOXEL_CAP):
                continue
            surv_z.append(z)
            surv_r.append(r)
            if z < -0.1:
                band["[-0.25,-0.1)"] += 1
            elif z < 0.0:
                band["[-0.1,0)"] += 1
            elif z < 0.2:
                band["[0,0.2)"] += 1
            elif z < 0.5:
                band["[0.2,0.5)"] += 1
            else:
                band["[0.5,1.0]"] += 1
        n_s = len(surv_z)
        print(f"[点云] 原始 {total} 点 → 过滤后幸存 {n_s} 点 (odom 系, r∈[{R_MIN},{R_MAX}], z∈[{Z_OBS_MIN},{Z_VOXEL_CAP}])")
        if n_s:
            print(f"  z 分布: p5={pct(surv_z, 0.05):.2f} p50={pct(surv_z, 0.5):.2f} p95={pct(surv_z, 0.95):.2f}")
            print(f"  r 分布: p5={pct(surv_r, 0.05):.1f} p50={pct(surv_r, 0.5):.1f} p95={pct(surv_r, 0.95):.1f}")
            print("  z 分带: " + "  ".join(f"{k}={v}" for k, v in band.items()))
            waterish = band["[-0.25,-0.1)"] + band["[-0.1,0)"]
            print(f"  → 疑似水面带 (z<0): {waterish} ({100.0 * waterish / n_s:.1f}%)")

    cm = got.get("cm")
    if cm is not None:
        data = cm.data
        w, h, res = cm.info.width, cm.info.height, cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        cx, cy = ox + w * res / 2.0, oy + h * res / 2.0
        rings = {"0-2m": 0, "2-4m": 0, "4-6m": 0, "6-10m": 0, "10m+": 0}
        sectors = set()
        for iy in range(h):
            row = iy * w
            for ix in range(w):
                if data[row + ix] >= 100:
                    dx = (ix + 0.5) * res + ox - cx
                    dy = (iy + 0.5) * res + oy - cy
                    d = math.hypot(dx, dy)
                    sectors.add(int((math.atan2(dy, dx) + math.pi) / (math.pi / 4)))
                    if d < 2:
                        rings["0-2m"] += 1
                    elif d < 4:
                        rings["2-4m"] += 1
                    elif d < 6:
                        rings["4-6m"] += 1
                    elif d < 10:
                        rings["6-10m"] += 1
                    else:
                        rings["10m+"] += 1
        print(f"[costmap] lethal 距离环: " + "  ".join(f"{k}={v}" for k, v in rings.items()))
        print(f"[costmap] lethal 散布扇区数(共8): {len(sectors)}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
