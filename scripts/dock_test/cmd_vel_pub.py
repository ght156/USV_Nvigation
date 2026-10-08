#!/usr/bin/env python3
"""持续发 cmd_vel 到 /cmd_vel_nav，T 秒后自动归零（速度桥有 1s 看门狗，命令必须连续）。
用法: cmd_vel_pub.py <linear_x> <angular_z> <秒>
"""
import sys, time
import rclpy
from geometry_msgs.msg import Twist

vx, wz, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
rclpy.init()
node = rclpy.create_node("cmd_vel_pub")
pub = node.create_publisher(Twist, "/cmd_vel_nav", 10)
msg = Twist()
msg.linear.x = vx
msg.angular.z = wz
t0 = time.time()
while time.time() - t0 < dur:
    msg.linear.x = vx
    msg.angular.z = wz
    pub.publish(msg)
    time.sleep(0.1)
msg.linear.x = 0.0
msg.angular.z = 0.0
for _ in range(15):  # 归零多发 1.5s，确保看门狗吃到停船命令
    pub.publish(msg)
    time.sleep(0.1)
print(f"done: vx={vx} wz={wz} dur={dur}s + zeroed")
node.destroy_node()
rclpy.shutdown()
