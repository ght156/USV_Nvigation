#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that republishes GPS (NavSatFix) data with an assigned covariance matrix for localization
#  consistency. It adjusts the covariance values to provide stable input for sensor fusion nodes
#  such as robot_localization.
# ----------------------------------------------------------------------------------------------- #

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GpsCovarianceRepub(Node):
    def __init__(self):
        super().__init__('gps_covariance_repub')
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.subscription = self.create_subscription(
            NavSatFix,
            '/roboboat/sensors/gps/navsat',
            self.gps_callback,
            10
        )
        self.publisher = self.create_publisher(
            NavSatFix,
            '/gps/fixed_cov',
            10
        )

    def gps_callback(self, msg):
        msg.header.frame_id = 'gps_link'
        hdop = 3.0
        var = (hdop * 1.5) ** 2
        msg.position_covariance = [
            var, 0.0, 0.0,
            0.0, var, 0.0,
            0.0, 0.0, var * 4.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsCovarianceRepub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()