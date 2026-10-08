#!/usr/bin/env python3
"""归港状态快照：dock_pose/tag_visible/pose_source/state 各读一次。"""
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

rclpy.init()
n = Node("dock_snap")
out = {}
def mk(topic, cls, key, fn):
    def cb(m):
        out[key] = fn(m)
    n.create_subscription(cls, topic, cb, 10)

import math
mk("/docking/dock_pose", PoseStamped, "pose", lambda m: (m.pose.position.x, m.pose.position.y,
   math.degrees(2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w))))
mk("/docking/tag_visible", Bool, "vis", lambda m: m.data)
mk("/docking/pose_source", String, "src", lambda m: m.data)
mk("/docking/state", String, "state", lambda m: m.data)
mk("/docking/safety_stop", Bool, "safety", lambda m: m.data)

t0 = time.time()
while time.time() - t0 < 8 and len(out) < 5:
    rclpy.spin_once(n, timeout_sec=0.3)
for k in ["state", "src", "vis", "safety", "pose"]:
    print(f"{k}: {out.get(k, '未收到')}")
n.destroy_node()
rclpy.shutdown()
