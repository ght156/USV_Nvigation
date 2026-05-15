#!/usr/bin/env python3
# 向 /mavros/manual_control/send 发送 mavros_msgs/ManualControl（WASD），用于 ArduPilot + MAVROS 试机。

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import ManualControl


class MavrosKeyboardTeleop(Node):

    def __init__(self):
        super().__init__('mavros_keyboard_teleop')

        self.declare_parameter('topic', '/mavros/manual_control/send')
        self.declare_parameter('stick', 350.0)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.stick = float(self.get_parameter('stick').get_parameter_value().double_value)

        self.pub = self.create_publisher(ManualControl, topic, 10)
        self.z = 0.0
        self.r = 0.0
        self.timer = self.create_timer(0.05, self._send)

        self.get_logger().info(
            f'Publishing ManualControl to {topic} at 20 Hz. '
            f'W/A/S/D stick={self.stick:.0f}, Q to quit. Focus this terminal.'
        )

    def _send(self):
        m = ManualControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.x = 0.0
        m.y = 0.0
        m.z = float(self.z)
        m.r = float(self.r)
        m.buttons = 0
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosKeyboardTeleop()
    old = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                c = sys.stdin.read(1)
                s = node.stick
                if c == 'w':
                    node.z, node.r = s, 0.0
                elif c == 's':
                    node.z, node.r = 0.0, 0.0
                elif c == 'a':
                    node.z, node.r = 0.0, -s
                elif c == 'd':
                    node.z, node.r = 0.0, s
                elif c in ('q', '\x03'):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
