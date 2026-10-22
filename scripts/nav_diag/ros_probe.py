#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""船端只读 ROS 探针：列话题 / 量 /livox/lidar 与 /usv_map_pose 的时间戳-墙钟差。

只订阅、不发布任何指令。用法（船端，环境已 source）：
    python3 /tmp/ros_probe.py list                       # 列话题
    python3 /tmp/ros_probe.py svc                        # 列服务
    python3 /tmp/ros_probe.py lidar 6                    # 量 6 秒 /livox/lidar
    python3 /tmp/ros_probe.py pose 5                     # 量 5 秒 /usv_map_pose
    python3 /tmp/ros_probe.py gngga 6                    # 量 6 秒 /gi320/gngga（NMEA 时间字段/定位质量）
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)


def cmd_list(node) -> None:
    pairs = node.get_topic_names_and_types()
    for name, types in sorted(pairs):
        n = node.count_publishers(name)
        print(f"{name:52s} pubs={n:2d}  {','.join(types)}")


def cmd_svc(node) -> None:
    for name, types in sorted(node.get_service_names_and_types()):
        print(f"{name:56s} {','.join(types)}")


def cmd_measure(node, topic: str, seconds: float) -> None:
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path

    samples: list[tuple[float, float, str]] = []

    def mk_cb():
        def cb(msg):
            st = msg.header.stamp
            samples.append((time.time(), st.sec + st.nanosec * 1e-9,
                            getattr(msg.header, "frame_id", "")))
        return cb

    types = dict(node.get_topic_names_and_types()).get(topic, [])
    print(f"订阅 {topic}  类型={types}  发布者={node.count_publishers(topic)}")
    if not types:
        print("话题不存在（当前没有发布者）。")
        return
    t = types[0]
    if t.endswith("PointCloud2"):
        node.create_subscription(PointCloud2, topic, mk_cb(), BEST)
    elif t.endswith("PoseStamped"):
        node.create_subscription(PoseStamped, topic, mk_cb(), 10)
    elif t.endswith("OccupancyGrid"):
        node.create_subscription(OccupancyGrid, topic, mk_cb(), 10)
    elif t.endswith("Path"):
        node.create_subscription(Path, topic, mk_cb(), 10)
    else:
        print(f"不支持的类型 {t}")
        return

    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    n = len(samples)
    print(f"收到 {n} 条 / {seconds:.0f} s  ({n / seconds:.2f} Hz)")
    if not n:
        return
    ages = [recv - stamp for recv, stamp, _ in samples]
    ages_sorted = sorted(ages)
    med = ages_sorted[n // 2]
    print(f"age = 收到时墙钟 - header.stamp:  med={med:+.4f} s  "
          f"min={ages_sorted[0]:+.4f}  max={ages_sorted[-1]:+.4f}  "
          f"spread={(ages_sorted[-1] - ages_sorted[0]) * 1e3:.1f} ms")
    print("判读: age≈+0.10 s 是 10 Hz 帧首口径（正常）；age≈-0.1~-0.5 s = 时间戳超前主机；"
          "age≈+几十秒/1e9 = 时间戳根本没有绝对 UTC。")
    print(f"frame_id={samples[-1][2]}  首条 stamp={samples[0][1]:.3f} ({time.strftime('%F %T', time.localtime(samples[0][1]))})")


def cmd_gngga(node, seconds: float) -> None:
    """看 GI320 的 GNGGA：报文时间字段（含小数秒）+ 定位质量 + 与墙钟差。"""
    from bw_gi320_driver.msg import Gi320Gngga

    rows: list[tuple[float, float, int, int, int, float]] = []

    def cb(msg):
        st = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        rows.append((time.time(), st, int(msg.fix_type), int(msg.satellite_num),
                     int(msg.stamp.nanosec), float(msg.hdop)))

    print(f"订阅 /gi320/gngga  发布者={node.count_publishers('/gi320/gngga')}")
    node.create_subscription(Gi320Gngga, "/gi320/gngga", cb, 10)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    n = len(rows)
    print(f"收到 {n} 条 / {seconds:.0f} s  ({n / seconds:.1f} Hz)")
    if not n:
        return
    ages = sorted(recv - st for recv, st, *_ in rows)
    print(f"age(GGA stamp - 墙钟): med={ages[n // 2]:+.4f} s  "
          f"min={ages[0]:+.4f}  max={ages[-1]:+.4f}")
    nanos = sorted(r[4] % 1_000_000_000 for r in rows)
    print(f"NMEA 小数秒(ns): 首={rows[0][4] % 1_000_000_000}  "
          f"中位={nanos[n // 2]}  唯一值={sorted(set(nanos))[:8]}")
    print(f"fix_type={rows[-1][2]} (4=RTK固定 5=RTK浮点 2=伪距差分 1=单点)  "
          f"sat={rows[-1][3]}  hdop={rows[-1][5]:.2f}")
    print(f"最新 lat={rows[-1][0]:.0f} 样本; stamp={rows[-1][1]:.3f} "
          f"({time.strftime('%F %T', time.localtime(rows[-1][1]))})")


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        return
    mode = sys.argv[1]
    rclpy.init()
    node = Node("codex_probe")
    if mode == "list":
        cmd_list(node)
    elif mode == "svc":
        cmd_svc(node)
    elif mode == "lidar":
        cmd_measure(node, "/livox/lidar", float(sys.argv[2]) if len(sys.argv) > 2 else 6.0)
    elif mode == "pose":
        cmd_measure(node, "/usv_map_pose", float(sys.argv[2]) if len(sys.argv) > 2 else 5.0)
    elif mode == "gngga":
        cmd_gngga(node, float(sys.argv[2]) if len(sys.argv) > 2 else 6.0)
    elif mode == "topic":
        cmd_measure(node, sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 5.0)
    elif mode == "latched":
        # 订阅 latched（TRANSIENT_LOCAL）话题：如 map_server 的 /map
        from nav_msgs.msg import OccupancyGrid
        from rclpy.duration import Duration

        got = []
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topic = sys.argv[2] if len(sys.argv) > 2 else "/map"
        secs = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
        print(f"{topic} 发布者={node.count_publishers(topic)}（TRANSIENT_LOCAL 订阅）")
        if node.count_publishers(topic):
            for info in node.get_publishers_info_by_topic(topic):
                print(f"  pub QoS: {info.qos_profile.reliability.name}/{info.qos_profile.durability.name}")
        node.create_subscription(OccupancyGrid, topic, lambda m: got.append(m), q)
        t0 = time.time()
        while time.time() - t0 < secs and not got:
            rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            m = got[0]
            print(f"收到 {len(got)} 条：{m.info.width}x{m.info.height} res={m.info.resolution} "
                  f"frame={m.header.frame_id} stamp={m.header.stamp.sec}")
        else:
            print("没收到（话题没有 latched 数据 / 没发布者）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
