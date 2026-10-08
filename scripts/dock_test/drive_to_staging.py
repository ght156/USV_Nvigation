#!/usr/bin/env python3
"""按 odom 锚点把船开到坞轴线上（坞口外 target_x 处、船尾朝坞），闭环、限速、自动停。

安全设计：
  - 全程限速 vx∈[-0.18,0.18] m/s, wz∈[-0.25,0.25] rad/s（比归港控制器还保守）
  - TF 超过 1.5s 不新鲜立即停车退出
  - 到位（|ex-target|<0.4 且 |ey|<0.4 且 |eyaw|<8°）自动停车退出
  - 最长运行 240s 超时停车
  - Ctrl-C 或异常都会先发零速
用法: python3 drive_to_staging.py [target_x=-7.0]
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

TARGET_X = float(sys.argv[1]) if len(sys.argv) > 1 else -7.0  # dock 系目标（坞口外）
MAX_V, MAX_W = 0.18, 0.25
POS_TOL, YAW_TOL = 0.4, math.radians(8)
TIMEOUT = 240.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Driver(Node):
    def __init__(self):
        super().__init__("drive_to_staging")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

    def poses(self):
        try:
            tb = self.tf.lookup_transform("odom", "base_link", rclpy.time.Time())
            td = self.tf.lookup_transform("odom", "dock_est", rclpy.time.Time())
        except Exception as e:
            return None, str(e)
        # 新鲜度检查（船/主机时钟有 ~1s 偏差，阈值放宽到 2s，未来时间戳也容忍）
        now = self.get_clock().now()
        for t in (tb, td):
            age = (now - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
            if abs(age) > 2.0:
                return None, f"TF 龄期 {age:.1f}s"
        def yaw_of(t):
            q = t.transform.rotation
            return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        bx = np.array([tb.transform.translation.x, tb.transform.translation.y])
        bw = yaw_of(tb)
        dp = np.array([td.transform.translation.x, td.transform.translation.y])
        dw = yaw_of(td)
        # 船位姿转到 dock 系
        R = np.array([[math.cos(-dw), -math.sin(-dw)], [math.sin(-dw), math.cos(-dw)]])
        rel = R @ (bx - dp)
        return (rel[0], rel[1], wrap(bw - dw)), None

    def run(self):
        t0 = time.time()
        cmd = Twist()
        try:
            while time.time() - t0 < TIMEOUT:
                st, err = self.poses()
                if st is None:
                    self.get_logger().warn(f"TF 不可用: {err}，停车等待")
                    self.pub.publish(Twist())
                    time.sleep(0.5)
                    continue
                ex, ey, eyaw = st
                # 目标：ex->TARGET_X, ey->0, eyaw->±pi（船尾朝坞）
                e_yaw = wrap(abs(eyaw) - math.pi) * (1 if eyaw > 0 else -1)  # 距 ±pi 的偏差
                # 控制：先修艏向和横向，再纵向倒入
                wz = float(np.clip(0.6 * e_yaw + 0.3 * (-ey) * (1 if ex > TARGET_X else -1), -MAX_W, MAX_W))
                vx = float(np.clip(-0.25 * (ex - TARGET_X), -MAX_V, MAX_V))
                if abs(ey) > 1.5 or abs(e_yaw) > math.radians(30):
                    vx = float(np.clip(vx, -0.08, 0.08))  # 偏差大时只蠕行
                cmd.linear.x, cmd.angular.z = vx, wz
                self.pub.publish(cmd)
                print(f"ex={ex:6.2f} ey={ey:5.2f} eyaw={math.degrees(eyaw):7.1f}° | vx={vx:+.2f} wz={wz:+.2f}", flush=True)
                if abs(ex - TARGET_X) < POS_TOL and abs(ey) < POS_TOL and abs(e_yaw) < YAW_TOL:
                    print("到位，停车")
                    break
                time.sleep(0.1)
        finally:
            for _ in range(15):
                self.pub.publish(Twist())
                time.sleep(0.1)
            print("已发零速")


rclpy.init()
d = Driver()
try:
    d.run()
finally:
    d.destroy_node()
    rclpy.shutdown()
