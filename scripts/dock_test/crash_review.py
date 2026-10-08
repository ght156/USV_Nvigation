#!/usr/bin/env python3
"""碰撞复盘：第二次归港窗口内 状态/控制量/估计位姿/原始视觉 对照。
在船上运行: python3 /tmp/crash_review.py"""
import sqlite3, glob, math
import numpy as np
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

bag = sorted(glob.glob("/home/jetson/usv_bags/dock_*auto_boat1*"))[0]
bag = bag.replace(".record.log", "")
con = sqlite3.connect(f"file:{glob.glob(bag + '/*.db3')[0]}?mode=ro", uri=True)
tid = {n: i for i, n in con.execute("select id, name from topics")}

TRIG = 1789204531.0          # 第二次 dock_only 触发时刻
W0, W1 = (TRIG - 15) * 1e9, (TRIG + 110) * 1e9

def yawq(qz, qw):
    return math.degrees(2 * math.atan2(qz, qw))

def load(topic):
    if topic not in tid:
        return []
    return con.execute(
        "select timestamp, data from messages where topic_id=? and timestamp between ? and ? order by timestamp",
        (tid[topic], W0, W1)).fetchall()

# 状态切换
print("== FSM 状态切换 ==")
prev = None
for ts, d in load("/docking/state"):
    v = deserialize_message(bytes(d), String).data
    if v != prev:
        print(f"  t={ts/1e9-TRIG:6.1f}s  {v}")
        prev = v

print("\n== 时间线（2s 间隔）：ex/ey/eyaw | cmd_vel | 原始视觉tag距离/方位 | GPS速度 ==")
dp = load("/docking/dock_pose")
cv = load("/cmd_vel_nav")
od = load("/mavros/gps_input/local")
# 原始视觉：camera->dock_frame
vis = []
for ts, d in load("/tf"):
    msg = deserialize_message(bytes(d), TFMessage)
    for t in msg.transforms:
        if t.header.frame_id == "camera_back_video_link" and t.child_frame_id == "dock_frame":
            tr = t.transform.translation
            rng = math.hypot(tr.x, tr.y)
            az = math.degrees(math.atan2(tr.y, tr.x))
            vis.append((ts / 1e9, rng, az))

def nearest(rows, t, parse):
    best, bd = None, 3.0
    for ts, d in rows:
        dt = abs(ts / 1e9 - t)
        if dt < bd:
            best, bd = parse(d), dt
    return best

t = -15.0
while t <= 110:
    te = TRIG + t
    p = nearest(dp, te, lambda d: deserialize_message(bytes(d), PoseStamped))
    c = nearest(cv, te, lambda d: deserialize_message(bytes(d), Twist))
    o = nearest(od, te, lambda d: deserialize_message(bytes(d), Odometry))
    vv = [v for v in vis if abs(v[0] - te) < 1.0]
    line = f"t={t:6.1f}s"
    if p:
        y = yawq(p.pose.orientation.z, p.pose.orientation.w)
        eyaw = (y - 180 if y > 0 else y + 180)
        line += f" | 估计 ex={p.pose.position.x:6.2f} ey={p.pose.position.y:5.2f} eyaw={eyaw:7.1f}°"
    if c:
        line += f" | cmd vx={c.linear.x:+.2f} wz={c.angular.z:+.2f}"
    else:
        line += " | cmd 无"
    if o:
        tw = o.twist.twist
        line += f" | gps v=({tw.linear.x:+.2f},{tw.linear.y:+.2f})"
    if vv:
        r = np.mean([x[1] for x in vv]); a = np.mean([x[2] for x in vv])
        line += f" | 视觉 距离={r:4.1f}m 方位={a:+5.1f}°"
    else:
        line += " | 视觉 无"
    print(line)
    t += 2.0
