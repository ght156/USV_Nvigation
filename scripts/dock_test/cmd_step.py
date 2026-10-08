#!/usr/bin/env python3
"""转向阶跃试验：在 /cmd_vel_nav 上发方波角速度，用于测控制链路各段延迟。

注意：这是**直接驱船**的测试脚本（绕开归港 FSM），仅在确认
  - 飞控 GUIDED + armed
  - 归港 FSM 处于 IDLE（控制器静默）
  - 开阔水域、有人持遥控器
时才运行。发完自动发零速；速度桥另有 1s 看门狗兜底。
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

AMP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.12   # rad/s
HALF = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0   # 每段时长 s
CYCLES = int(sys.argv[3]) if len(sys.argv) > 3 else 5


class Step(Node):
    def __init__(self):
        super().__init__("cmd_step")
        self.pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

    def send(self, w, dur):
        msg = Twist()
        msg.angular.z = w
        t_end = time.time() + dur
        while time.time() < t_end:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)                      # ~50Hz，远高于桥的看门狗


def main():
    rclpy.init()
    n = Step()
    ts = lambda: time.strftime("%H:%M:%S")        # noqa: E731
    try:
        print(f"[{ts()}] 基线：零速 2s", flush=True)
        n.send(0.0, 2.0)
        for i in range(CYCLES):
            for s in (1, -1):
                print(f"[{ts()}] 第{i + 1}周期 w={AMP * s:+.3f} 持续{HALF}s", flush=True)
                n.send(AMP * s, HALF)
        print(f"[{ts()}] 收尾：零速 2s", flush=True)
        n.send(0.0, 2.0)
        print("DONE", flush=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
