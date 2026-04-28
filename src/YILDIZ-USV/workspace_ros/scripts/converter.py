#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that converts /cmd_vel_nav Twist commands into left and right thruster Float64 outputs
#  for the RoboBoat. It scales linear and angular velocities and publishes corresponding thrust
#  values to the thruster topics for motion control.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class Nav2ThrusterController(Node):
    def __init__(self):
        super().__init__('converter')

        self.left_thruster_topic = '/roboboat/thrusters/left/thrust'
        self.right_thruster_topic = '/roboboat/thrusters/right/thrust'

        self.left_thruster_pub = self.create_publisher(Float64, self.left_thruster_topic, 20)
        self.right_thruster_pub = self.create_publisher(Float64, self.right_thruster_topic, 20)

        self.linear_scale = 1.75
        self.angular_scale = 20.0

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            20
        )

        self.stop_motors()

    def cmd_vel_callback(self, msg):
        self.get_logger().info(
            f"Received cmd_vel: linear_x = {msg.linear.x}, angular_z = {msg.angular.z}"
        )

        left_thrust = self.linear_scale * msg.linear.x - self.angular_scale * msg.angular.z
        right_thrust = self.linear_scale * msg.linear.x + self.angular_scale * msg.angular.z

        self.left_thruster_pub.publish(Float64(data=left_thrust))
        self.right_thruster_pub.publish(Float64(data=right_thrust))

        self.get_logger().info(
            f"Left thrust = {left_thrust:.6f}, Right thrust = {right_thrust:.6f}"
        )

    def stop_motors(self):
        self.left_thruster_pub.publish(Float64(data=0.0))
        self.right_thruster_pub.publish(Float64(data=0.0))
        self.get_logger().info("Motors stopped")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ThrusterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()