#!/usr/bin/env python3
"""归港触发+全程监视：触发 dock_only，4Hz 盯 ey/状态；|ey|>1.0 持续 0.5s 自动取消（人肉安全兜底）。"""
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

EY_ABORT = 1.0
ABORT_NEED = 5          # 连续 5 拍（约 1s）超限才取消
WATCH_SEC = 300

rclpy.init()
n = Node("dock_watch")
st = {"state": None, "ex": None, "ey": None, "yaw": None, "src": None, "safety": None}
n.create_subscription(String, "/docking/state", lambda m: st.update(state=m.data), 10)
n.create_subscription(String, "/docking/pose_source", lambda m: st.update(src=m.data), 10)
n.create_subscription(Bool, "/docking/safety_stop", lambda m: st.update(safety=m.data), 10)
def pc(m):
    st.update(ex=m.pose.position.x, ey=m.pose.position.y,
              yaw=math.degrees(2*math.atan2(m.pose.orientation.z, m.pose.orientation.w)))
n.create_subscription(PoseStamped, "/docking/dock_pose", pc, 10)

pub = n.create_publisher(String, "/gcs_dock/command", 10)
cancel_pub = n.create_publisher(String, "/gcs_dock/command", 10)

# 等数据就绪
t0 = time.time()
while time.time() - t0 < 8 and st["ex"] is None:
    rclpy.spin_once(n, timeout_sec=0.3)

cmd = String(); cmd.data = '{"action":"dock_only"}'
pub.publish(cmd)
print(f"{time.strftime('%H:%M:%S')} 已触发 dock_only", flush=True)

over = 0
tstart = time.time()
try:
    while time.time() - tstart < WATCH_SEC:
        rclpy.spin_once(n, timeout_sec=0.2)
        if st["ex"] is None:
            continue
        yaw = st["yaw"]; eyaw = (yaw - 180 if yaw > 0 else yaw + 180)
        print(f"{time.strftime('%H:%M:%S')} {st['state']:<14} ex={st['ex']:6.2f} ey={st['ey']:+5.2f} eyaw={eyaw:+6.1f}° {st['src']} safety={st['safety']}", flush=True)
        if st["state"] in ("IDLE",) and time.time() - tstart > 10:
            print("=== 会话结束（IDLE）===", flush=True)
            break
        if st["safety"]:
            print("⚠️ safety_stop=true！", flush=True)
        if abs(st["ey"]) > EY_ABORT and st["state"] not in ("IDLE",):
            over += 1
            if over >= ABORT_NEED:
                print(f"‼️ |ey|={st['ey']:.2f} 超 {EY_ABORT}m 持续，自动取消归港！", flush=True)
                c = String(); c.data = '{"action":"cancel"}'
                cancel_pub.publish(c)
                break
        else:
            over = 0
        time.sleep(0.25)
finally:
    n.destroy_node(); rclpy.shutdown()
