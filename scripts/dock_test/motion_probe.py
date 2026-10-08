#!/usr/bin/env python3
"""运动能力开环测试：按阶梯序列发 /cmd_vel_nav，同步记录 GPS 实测速度到 CSV。
用法: python3 motion_probe.py [输出csv路径]
安全：单段 ≤8s、幅值 ≤0.45m/s / 0.35rad/s，全程自动归零收尾；Ctrl-C 立即归零退出。
"""
import csv, math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/motion_probe.csv"

# (时长s, vx, wz, 标签)
SEQ = [
    (8, 0.0, 0.0, "drift"),
    (8, 0.20, 0.0, "fwd0.2"),
    (3, 0.0, 0.0, "settle"),
    (8, 0.45, 0.0, "fwd0.45"),
    (3, 0.0, 0.0, "settle"),
    (8, -0.20, 0.0, "rev0.2"),
    (3, 0.0, 0.0, "settle"),
    (8, -0.45, 0.0, "rev0.45"),
    (3, 0.0, 0.0, "settle"),
    (6, 0.0, 0.20, "yaw+0.2"),
    (3, 0.0, 0.0, "settle"),
    (6, 0.0, -0.35, "yaw-0.35"),
    (2, 0.0, 0.0, "stop"),
]

rclpy.init()
node = rclpy.create_node("motion_probe")
pub = node.create_publisher(Twist, "/cmd_vel_nav", 10)
latest = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "yaw": 0.0, "ox": 0.0, "oy": 0.0}

def odom_cb(m):
    latest["vx"] = m.twist.twist.linear.x
    latest["vy"] = m.twist.twist.linear.y
    latest["wz"] = m.twist.twist.angular.z
    q = m.pose.pose.orientation
    latest["yaw"] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

node.create_subscription(Odometry, "/mavros/gps_input/local", odom_cb, 10)
tfbuf = Buffer(); TransformListener(tfbuf, node)
def tf_poll():
    try:
        t = tfbuf.lookup_transform("odom", "base_link", rclpy.time.Time())
        latest["ox"] = t.transform.translation.x
        latest["oy"] = t.transform.translation.y
    except Exception:
        pass

f = open(OUT, "w", newline="")
w = csv.writer(f)
w.writerow(["t", "phase", "cmd_vx", "cmd_wz", "gps_vx", "gps_vy", "gps_wz", "yaw", "ox", "oy"])
t0 = time.time()
try:
    for dur, vx, wz, tag in SEQ:
        print(f"[{time.time()-t0:5.1f}s] {tag}: cmd vx={vx} wz={wz}", flush=True)
        tend = time.time() + dur
        while time.time() < tend:
            m = Twist(); m.linear.x = vx; m.angular.z = wz
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.02); tf_poll()
            w.writerow([f"{time.time()-t0:.2f}", tag, vx, wz,
                        f"{latest['vx']:.3f}", f"{latest['vy']:.3f}", f"{latest['wz']:.3f}", f"{latest['yaw']:.4f}", f"{latest['ox']:.4f}", f"{latest['oy']:.4f}"])
            time.sleep(0.08)
finally:
    z = Twist()
    for _ in range(15):
        pub.publish(z); time.sleep(0.1)
    f.close()
    print(f"完成，已归零。数据: {OUT}")
node.destroy_node(); rclpy.shutdown()
