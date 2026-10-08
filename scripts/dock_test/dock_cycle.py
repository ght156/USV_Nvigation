#!/usr/bin/env python3
"""出港→再归港全循环：触发 undock → IDLE 后等视觉位姿进门 → 自动 dock_only → 盯到 DOCKED。
|ey|>1.0 持续 ~1s 自动取消。超时安全退出。"""
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

GATE_X = (-6.5, -1.2)   # 再归港触发门（视觉 dock 系）
GATE_Y = 0.8
EY_ABORT = 1.0

rclpy.init()
n = Node("dock_cycle")
st = {"state": None, "ex": None, "ey": None, "yaw": None, "src": None, "safety": None}
n.create_subscription(String, "/docking/state", lambda m: st.update(state=m.data), 10)
n.create_subscription(String, "/docking/pose_source", lambda m: st.update(src=m.data), 10)
n.create_subscription(Bool, "/docking/safety_stop", lambda m: st.update(safety=m.data), 10)
def pc(m):
    st.update(ex=m.pose.position.x, ey=m.pose.position.y,
              yaw=math.degrees(2*math.atan2(m.pose.orientation.z, m.pose.orientation.w)))
n.create_subscription(PoseStamped, "/docking/dock_pose", pc, 10)
pub = n.create_publisher(String, "/gcs_dock/command", 10)

def send(action):
    m = String(); m.data = '{"action":"%s"}' % action
    pub.publish(m)
    print(f"{time.strftime('%H:%M:%S')} >>> 触发 {action}", flush=True)

def spin_wait(sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.1)

def snap():
    if st["ex"] is None:
        return ""
    yaw = st["yaw"]; eyaw = (yaw - 180 if yaw > 0 else yaw + 180)
    return f"{st['state']:<13} ex={st['ex']:6.2f} ey={st['ey']:+5.2f} eyaw={eyaw:+6.1f}° {st['src']}"

spin_wait(2)
print(f"{time.strftime('%H:%M:%S')} 起手 {snap()}", flush=True)

# ── 阶段1：出港 ──
send("undock")
t0 = time.time()
while time.time() - t0 < 90:
    spin_wait(1.0)
    print(f"{time.strftime('%H:%M:%S')} 出港中 {snap()}", flush=True)
    if st["state"] == "IDLE" and time.time() - t0 > 8:
        break
print(f"{time.strftime('%H:%M:%S')} 出港结束 {snap()}", flush=True)

# ── 阶段2：等进触发门（最多 45s），连续 2 次采样在门内才触发 ──
hit = 0
t0 = time.time()
triggered = False
while time.time() - t0 < 45:
    spin_wait(0.5)
    print(f"{time.strftime('%H:%M:%S')} 等门 {snap()}", flush=True)
    if st["ex"] is None or st["src"] != "VISION":
        hit = 0
        continue
    if GATE_X[0] < st["ex"] < GATE_X[1] and abs(st["ey"]) < GATE_Y:
        hit += 1
        if hit >= 2:
            triggered = True
            break
    else:
        hit = 0
if not triggered:
    print("‼️ 45s 内没进触发门（可能漂出走廊），放弃再归港，请人工就位", flush=True)
    n.destroy_node(); rclpy.shutdown()
    raise SystemExit(1)

# ── 阶段3：再归港 ──
send("dock_only")
over = 0
t0 = time.time()
while time.time() - t0 < 240:
    spin_wait(0.25)
    print(f"{time.strftime('%H:%M:%S')} 归港 {snap()} safety={st['safety']}", flush=True)
    if st["state"] == "IDLE" and time.time() - t0 > 10:
        print("=== 归港会话结束 ===", flush=True)
        break
    if st["ey"] is not None and abs(st["ey"]) > EY_ABORT and st["state"] != "IDLE":
        over += 1
        if over >= 4:
            print(f"‼️ |ey|={st['ey']:.2f} 超限，自动取消！", flush=True)
            send("cancel")
            break
    else:
        over = 0
n.destroy_node(); rclpy.shutdown()
