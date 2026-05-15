#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that provides manual keyboard control of the RoboBoat using the WASD keys to produce
#  per-thruster Float64 setpoints. It initializes thrusters to a known safe state on startup and
#  logs control actions during operation.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pynput.keyboard import Listener

STARTUP_MSG = "Manual control of the usv via the WASD keys has been successfully initiated."

class ThrusterControl(Node):

    def __init__(self):
        super().__init__('manual_control')
        
        self.left_thruster_topic = '/usv/thrusters/left/thrust'
        self.right_thruster_topic = '/usv/thrusters/right/thrust'
        
        self.left_thruster_pub = self.create_publisher(Float64, self.left_thruster_topic, 10)
        self.right_thruster_pub = self.create_publisher(Float64, self.right_thruster_topic, 10)
        
        self.stop_thrusters()

        self.listener = Listener(on_press=self.on_press)
        self.listener.start()

        if not getattr(self, '_startup_message_printed', False):
            print(STARTUP_MSG)
            self._startup_message_printed = True

    def start_thrusters(self):
        left_thruster_power = 10.0
        right_thruster_power = 10.0

        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))

        self.get_logger().info("Thrusters activated.")

    def stop_thrusters(self):
        left_thruster_power = 0.0  
        right_thruster_power = 0.0  
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Thrusters deactivated.")

    def turn_left(self):
        left_thruster_power = -10.0  
        right_thruster_power = 10.0
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Turning left.")

    def turn_right(self):
        left_thruster_power = 10.0 
        right_thruster_power = -10.0
        
        self.left_thruster_pub.publish(Float64(data=left_thruster_power))
        self.right_thruster_pub.publish(Float64(data=right_thruster_power))
        
        self.get_logger().info("Turning right.")

    def on_press(self, key):
        """Keyboard callback mapping keys to control actions."""
        try:
            if key.char == 'w':  
                self.start_thrusters()
            elif key.char == 's':  
                self.stop_thrusters()
            elif key.char == 'a': 
                self.turn_left()
            elif key.char == 'd':  
                self.turn_right()
        except AttributeError:
            pass 

    def stop_listener(self):
        try:
            if self.listener and self.listener.running:
                self.listener.stop()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_listener()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()