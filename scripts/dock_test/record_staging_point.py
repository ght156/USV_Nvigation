#!/usr/bin/env python3
"""预泊点记录：船在坞内停稳时运行，按当前坞位反算预泊点写入泊位库。

原理：dock_pose = 船(base_link)在坞系的位姿；船在坞内时 T_map_dock =
T_map_base ∘ inv(T_dock_base)。预泊点 = 坞原点沿坞轴向外（-x 方向）
standoff 米处，航向 = 坞轴朝外方向（船艏朝坞外）。

用法: python3 record_staging_point.py [bay_id] [standoff_m]
"""
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import yaml

BAY = sys.argv[1] if len(sys.argv) > 1 else "bay2"
STANDOFF = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
DB = Path("/home/hkzl/usv-main/src/USV_NAV/dock_mission/config/dock_database.yaml")
MAP_YAML = Path("/home/hkzl/usv-main/src/USV_NAV/workspace_nav/config/map.yaml")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node("record_staging")
    got = {"p": None}
    n.create_subscription(PoseStamped, "/docking/dock_pose",
                          lambda m: got.__setitem__("p", m), 10)
    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf, n)
    t0 = time.time()
    while time.time() - t0 < 8 and got["p"] is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    if got["p"] is None:
        print("FAIL: /docking/dock_pose 无数据（归港栈没跑？）")
        return 1
    p = got["p"].pose

    bx = by = byaw = None
    t_tf = time.time()
    while time.time() - t_tf < 15.0:
        try:
            tf = tfbuf.lookup_transform("map", "base_link", rclpy.time.Time(),
                                        timeout=rclpy.duration.Duration(seconds=1.0))
            bx = tf.transform.translation.x
            by = tf.transform.translation.y
            byaw = yaw_of(tf.transform.rotation)
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.5)
    if bx is None:
        print("FAIL: map→base_link TF 15s 未就绪（定位栈没起？）")
        return 1

    # 坞系下船的位姿
    dx, dy = p.position.x, p.position.y
    dyaw = yaw_of(p.orientation)  # 船艏在坞系 yaw（停稳时≈pi，船尾朝坞内）

    # T_map_dock: 坞原点在 map 系的位置与坞轴朝向
    # 船在坞系的位姿 = T_dock_base；T_map_dock = T_map_base ∘ inv(T_dock_base)
    # 坞轴 map 朝向 = byaw - dyaw
    dock_yaw = byaw - dyaw
    # 坞原点 = 船位置 - R(dock_yaw)·(dx, dy)
    ox = bx - (math.cos(dock_yaw) * dx - math.sin(dock_yaw) * dy)
    oy = by - (math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy)

    # 预泊点：坞原点沿坞轴向外（-x）standoff 米，船艏朝外
    out_yaw = dock_yaw + math.pi  # 坞轴朝外方向（坞系+x朝坞内）
    sx = ox + math.cos(dock_yaw + math.pi) * STANDOFF
    sy = oy + math.sin(dock_yaw + math.pi) * STANDOFF
    # 归一化航向
    out_yaw = (out_yaw + math.pi) % (2 * math.pi) - math.pi
    yaw_deg = math.degrees(out_yaw)

    print(f"船 map 位姿: ({bx:.3f}, {by:.3f}) yaw={math.degrees(byaw):.1f}°")
    print(f"船在坞系: x={dx:.3f} y={dy:.3f} yaw={math.degrees(dyaw):.1f}°")
    print(f"坞原点 map: ({ox:.3f}, {oy:.3f}) 坞轴朝外 {math.degrees(out_yaw):.1f}°")
    print(f"预泊点 map: ({sx:.3f}, {sy:.3f}) yaw={yaw_deg:.1f}° (standoff {STANDOFF}m)")

    # map → GNSS（用 dock_mission 的 gnss_staging 模块的反函数：先算 GNSS）
    cfg = yaml.safe_load(MAP_YAML.read_text())
    ref = cfg.get("ref_gnss_10") or cfg.get("ref_gnss")
    ref_lon, ref_lat = float(ref[0]), float(ref[1])
    # map 系 x=东 y=北（ENU），米 → 度
    lat = ref_lat + sy / 111320.0
    lon = ref_lon + sx / (111320.0 * math.cos(math.radians(ref_lat)))
    print(f"预泊点 GNSS: lat={lat:.8f} lon={lon:.8f}")

    # 备份并写库
    import shutil
    bak = DB.with_suffix(f".yaml.bak_{time.strftime('%Y%m%d_%H%M%S')}")
    shutil.copy(DB, bak)
    db = yaml.safe_load(DB.read_text())
    bay = db["bays"][BAY]
    bay["name"] = f"xingfei（{time.strftime('%Y-%m-%d')} 坞内实测重标）"
    bay["gnss_staging"]["latitude"] = round(lat, 8)
    bay["gnss_staging"]["longitude"] = round(lon, 8)
    bay["gnss_staging"]["yaw_deg"] = round(yaw_deg, 1)
    bay["map_staging"]["x"] = round(sx, 3)
    bay["map_staging"]["y"] = round(sy, 3)
    bay["map_staging"]["yaw"] = round(out_yaw, 4)
    DB.write_text(yaml.safe_dump(db, allow_unicode=True, sort_keys=False))
    print(f"已写入 {DB}（备份 {bak.name}）")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
