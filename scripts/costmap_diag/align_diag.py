#!/usr/bin/env python3
"""对齐与低河岸诊断（在船上跑）：

1) local costmap lethal 格 → map 系，与 global costmap（静态地图）lethal 的最近距离分布 → 对齐偏差；
2) 原始点云变换到 odom 后按 z 分带统计（低河岸点是否被 min_obstacle_height=-0.25 切掉）；
3) GNSS fix 状态。
"""

import math
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2
from sensor_msgs_py import point_cloud2

TL = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

GPS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# 与船上一致的滤波链
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


def apply_tf(R, t, p):
    return (
        R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2] + t.x,
        R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2] + t.y,
        R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2] + t.z,
    )


def pct(v, q):
    return v[min(len(v) - 1, int(q * len(v)))]


def lethal_cells(m):
    """返回 [(x, y), ...]（消息自身 frame 坐标）"""
    data = m.data
    w, res = m.info.width, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    cells = []
    for iy in range(m.info.height):
        row = iy * w
        for ix in range(w):
            if data[row + ix] >= 100:
                cells.append(((ix + 0.5) * res + ox, (iy + 0.5) * res + oy))
    return cells


def main():
    rclpy.init()
    n = rclpy.create_node("align_diag")
    got = {}
    n.create_subscription(
        OccupancyGrid, "/local_costmap/costmap", lambda m: got.__setitem__("lcm", m), TL
    )
    n.create_subscription(
        OccupancyGrid, "/global_costmap/costmap", lambda m: got.__setitem__("gcm", m), TL
    )
    n.create_subscription(
        NavSatFix, "/mavros/gps_input/raw/fix", lambda m: got.__setitem__("fix", m), GPS
    )
    n.create_subscription(
        PointCloud2, "/livox/lidar", lambda m: got.__setitem__("pc", m), 10
    )
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf, n)

    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        if all(k in got for k in ("lcm", "gcm", "fix", "pc")):
            break

    # ---- 1) local vs global 对齐 ----
    lcm, gcm = got.get("lcm"), got.get("gcm")
    if lcm is not None and gcm is not None:
        tf_map_odom = None
        for _ in range(20):
            try:
                tf_map_odom = tfbuf.lookup_transform(
                    "map", lcm.header.frame_id, rclpy.time.Time()  # latest
                )
                break
            except Exception:
                rclpy.spin_once(n, timeout_sec=0.3)
        if tf_map_odom is None:
            print("[对齐] map→local 系 TF 不可用，跳过对齐计算")
        else:
            tr = tf_map_odom.transform
            R = quat_to_mat(
                (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
            )
            t = tr.translation
            # local lethal (odom 系) → map 系
            local_map = [
                apply_tf(R, t, (x, y, 0.0)) for (x, y) in lethal_cells(lcm)
            ]
            g_cells = lethal_cells(gcm)
            print(f"[对齐] local lethal={len(local_map)}  global lethal={len(g_cells)}")
            if local_map and g_cells:
                # 对局部每个 lethal 格找最近的 global lethal，记录距离与偏移向量
                dists = []
                vx_sum = vy_sum = 0.0
                for lm_x, lm_y, _ in local_map:
                    best = 1e9
                    bvx = bvy = 0.0
                    for gx, gy in g_cells:
                        d = math.hypot(lm_x - gx, lm_y - gy)
                        if d < best:
                            best = d
                            bvx, bvy = gx - lm_x, gy - lm_y
                    dists.append(best)
                    vx_sum += bvx
                    vy_sum += bvy
                dists.sort()
                print(
                    f"[对齐] local障碍→最近全局地图障碍距离: "
                    f"p50={pct(dists, 0.5):.2f}m p90={pct(dists, 0.9):.2f}m max={dists[-1]:.2f}m"
                )
                far = sum(1 for d in dists if d > 1.0)
                print(f"[对齐] 距全局地图障碍 >1m 的局部障碍格: {far}/{len(dists)}")
                n_d = len(dists)
                if n_d:
                    mvx, mvy = vx_sum / n_d, vy_sum / n_d
                    print(
                        f"[对齐] 平均偏移向量（地图障碍 − 实测障碍，map系）: "
                        f"dx={mvx:+.2f} dy={mvy:+.2f} |v|={math.hypot(mvx, mvy):.2f}m "
                        f"方向={math.degrees(math.atan2(mvy, mvx)):+.0f}°"
                    )
    else:
        print("[对齐] local/global costmap 缺失")

    # ---- 2) 低河岸点云 z 分带 ----
    pc = got.get("pc")
    if pc is not None:
        tf = None
        for _ in range(20):
            try:
                tf = tfbuf.lookup_transform("odom", pc.header.frame_id, rclpy.time.Time())
                break
            except Exception:
                rclpy.spin_once(n, timeout_sec=0.3)
        if tf is not None:
            R2 = quat_to_mat(
                (tf.transform.rotation.x, tf.transform.rotation.y,
                 tf.transform.rotation.z, tf.transform.rotation.w)
            )
            t2 = tf.transform.translation
            band = {
                "z<-0.25(被min切)": 0,
                "[-0.25,0)幸存": 0,
                "[0,0.3)幸存": 0,
                "[0.3,1.0]幸存": 0,
                "z>1.0(被cap切)": 0,
            }
            near_low = 0  # 2~8m 且 z∈[-0.25,0.3) 的点（典型低河岸/水面）
            # 距离必须以传感器位置为原点（Nav2 的 obstacle_min_range 语义）
            sx, sy = t2.x, t2.y
            for p in point_cloud2.read_points(pc, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = apply_tf(R2, t2, p)
                r = math.hypot(x - sx, y - sy)
                if not (R_MIN <= r <= R_MAX):
                    continue
                if z < Z_OBS_MIN:
                    band["z<-0.25(被min切)"] += 1
                    if r < 8.0:
                        near_low += 1
                elif z <= Z_VOXEL_CAP:
                    if z < 0.0:
                        band["[-0.25,0)幸存"] += 1
                    elif z < 0.3:
                        band["[0,0.3)幸存"] += 1
                    else:
                        band["[0.3,1.0]幸存"] += 1
                else:
                    band["z>1.0(被cap切)"] += 1
            print("[低河岸] 2~20m 点按 odom z 分带: " + "  ".join(f"{k}={v}" for k, v in band.items()))
            print(f"[低河岸] 2~8m 内被 min=-0.25 切掉的低点: {near_low}")
        else:
            print("[点云] TF 不可用")

    fix = got.get("fix")
    if fix is not None:
        names = {-1: "NO_FIX", 0: "FIX", 1: "SBAS", 2: "GBAS/RTK"}
        print(f"[GNSS] status={fix.status.status} ({names.get(fix.status.status, '?')})")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
