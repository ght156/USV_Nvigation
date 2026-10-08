#!/usr/bin/env python3
"""运动测试 bag 分析：cmd_vel_nav 指令段 × odom->base_link 真值。船上运行:
python3 /tmp/motion_bag_analyze.py <bag目录>"""
import sqlite3, glob, math, sys
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

bag = sys.argv[1]
con = sqlite3.connect("file:" + glob.glob(bag + "/*.db3")[0] + "?mode=ro", uri=True)
tid = {n: i for i, n in con.execute("select id, name from topics")}

tf = []
for ts, d in con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid["/tf"],)):
    msg = deserialize_message(bytes(d), TFMessage)
    for t in msg.transforms:
        if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            tf.append((ts/1e9, t.transform.translation.x, t.transform.translation.y, yaw))
tf = np.array(tf)
print(f"TF {len(tf)} 帧")

cv = []
for ts, d in con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid["/cmd_vel_nav"],)):
    m = deserialize_message(bytes(d), Twist)
    cv.append((ts/1e9, m.linear.x, m.angular.z))
cv = np.array(cv)

# 按指令值切段
segs, cur = [], None
for t, vx, wz in cv:
    key = (round(float(vx), 2), round(float(wz), 2))
    if cur is None or cur[2] != key:
        if cur:
            segs.append(cur)
        cur = [t, t, key]
    else:
        cur[1] = t
if cur:
    segs.append(cur)

print("== 指令段 × 实际运动（去头尾瞬态）==")
for a, b, (vx, wz) in segs:
    dur = b - a
    if dur < 2.5:
        continue
    m = (tf[:, 0] >= a + 1.0) & (tf[:, 0] <= b - 0.5)
    if m.sum() < 5:
        continue
    seg = tf[m]
    dx, dy = seg[-1, 1] - seg[0, 1], seg[-1, 2] - seg[0, 2]
    dist = math.hypot(dx, dy)
    dd = seg[-1, 0] - seg[0, 0]
    dyaw = math.degrees((seg[-1, 3] - seg[0, 3] + math.pi) % (2*math.pi) - math.pi)
    yawm = float(seg[:, 3].mean())
    rel = math.degrees((math.atan2(dy, dx) - yawm + math.pi) % (2*math.pi) - math.pi)
    if abs(vx) > 0.01:
        direction = "前进" if abs(rel) < 90 else "倒退"
        expect = "前进" if vx > 0 else "倒退"
        flag = "  ⚠️方向反了!" if direction != expect else ""
        print(f"vx={vx:+.2f} {dur:4.1f}s: {dist:5.2f}m -> {dist/dd:.3f} m/s ({direction}, 相对艏向{rel:+.0f}°) 达成{abs(dist/dd/vx)*100:.0f}%{flag}")
    elif abs(wz) > 0.01:
        cmd_deg = math.degrees(wz * dd)
        flag = "  ⚠️方向反了!" if (dyaw > 0) != (cmd_deg > 0) else ""
        print(f"wz={wz:+.2f} {dur:4.1f}s: 转过{dyaw:+6.1f}° -> {math.radians(abs(dyaw)/dd):.3f} rad/s (指令{cmd_deg:+.0f}°) 达成{abs(dyaw/cmd_deg)*100:.0f}%{flag}")
    else:
        print(f"零速  {dur:4.1f}s: 漂移{dist:.2f}m -> {dist/dd:.3f} m/s")
