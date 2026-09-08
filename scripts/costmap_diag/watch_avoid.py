#!/usr/bin/env python3
"""避障观察采样（开发机本地跑，ROS_DOMAIN_ID=5 直连船端图）。

输出一行紧凑状态：导航目标、cmd_vel、costmap 占用、最近障碍距离。
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

WINDOW_SEC = 3.0


def main():
    rclpy.init()
    n = rclpy.create_node("avoid_probe")
    got = {}

    def mk(topic, tp, key, qos=None):
        n.create_subscription(
            tp, topic, lambda m: got.__setitem__(key, m), qos or 10
        )

    tl = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    mk("/local_costmap/costmap", OccupancyGrid, "cm", tl)
    mk("/cmd_vel_nav", Twist, "cmd")
    mk("/navigate_to_pose/_action/status", GoalStatusArray, "nav_status")
    mk("/follow_waypoints/_action/status", GoalStatusArray, "wp_status")

    t0 = time.time()
    while time.time() - t0 < WINDOW_SEC:
        rclpy.spin_once(n, timeout_sec=0.2)

    out = []
    cm = got.get("cm")
    if cm is not None:
        data = cm.data
        w, h, res = cm.info.width, cm.info.height, cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        # 滚动窗口以船为中心，中心≈船位
        rx, ry = ox + w * res / 2.0, oy + h * res / 2.0
        lethal = 0
        best = 1e9
        for iy in range(h):
            row = iy * w
            for ix in range(w):
                if data[row + ix] >= 100:
                    lethal += 1
                    d = math.hypot(
                        (ix + 0.5) * res + ox - rx,
                        (iy + 0.5) * res + oy - ry,
                    )
                    if d < best:
                        best = d
        nonzero = sum(1 for v in data if v > 0)
        nearest = f"{best:.1f}m" if lethal else "无"
        out.append(
            f"lethal={lethal} inflated={nonzero - lethal} 最近障碍={nearest}"
        )
    else:
        out.append("costmap=N/A")

    cmd = got.get("cmd")
    if cmd is not None:
        out.append(f"cmd_vel_nav v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}")
    else:
        out.append("cmd_vel_nav=静默")

    for key, label in (("nav_status", "nav_to_pose"), ("wp_status", "follow_wps")):
        st = got.get(key)
        active = st is not None and any(
            s.status in (0, 1) for s in st.status_list
        )
        out.append(f"{label}={'ACTIVE' if active else 'idle'}")

    print(" | ".join(out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
