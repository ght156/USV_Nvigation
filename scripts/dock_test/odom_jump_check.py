#!/usr/bin/env python3
"""检查 odom->base_link TF 是否有跳变（定位原点重置）。船上运行: python3 /tmp/odom_jump_check.py [起始epoch] [结束epoch]"""
import sqlite3, glob, math, sys
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

bag = "/home/jetson/usv_bags/dock_20260912_082401_auto_boat1"
con = sqlite3.connect(f"file:{glob.glob(bag + '/*.db3')[0]}?mode=ro", uri=True)
tid = {n: i for i, n in con.execute("select id, name from topics")}

t0 = float(sys.argv[1]) * 1e9 if len(sys.argv) > 1 else 0
t1 = float(sys.argv[2]) * 1e9 if len(sys.argv) > 2 else 9e18

seq = []
for ts, d in con.execute("select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp", (tid["/tf"], t0, t1)):
    msg = deserialize_message(bytes(d), TFMessage)
    for t in msg.transforms:
        if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            seq.append((ts/1e9, t.transform.translation.x, t.transform.translation.y, yaw))

print(f"odom->base_link 共 {len(seq)} 帧")
jumps = 0
for (pa, xa, ya, yawa), (pb, xb, yb, yawb) in zip(seq, seq[1:]):
    dt = pb - pa
    d = math.hypot(xb-xa, yb-ya)
    dyaw = abs(math.degrees((yawb-yawa+math.pi)%(2*math.pi)-math.pi))
    # 船物理速度上限 ~1.5m/s；dt 内位移超 3*dt+0.3m 即跳变
    if d > 3*dt + 0.3 or dyaw > 30:
        jumps += 1
        print(f"  t={pa:.1f} 跳变: 位移 {d:.2f}m/{dt:.2f}s, 艏向变 {dyaw:.1f}°  ({xa:.2f},{ya:.2f}) -> ({xb:.2f},{yb:.2f})")
print(f"跳变共 {jumps} 次")
# 全程位置范围
X = np.array([[x, y] for _, x, y, _ in seq])
print(f"位置范围 x [{X[:,0].min():.2f},{X[:,0].max():.2f}] y [{X[:,1].min():.2f},{X[:,1].max():.2f}]")
