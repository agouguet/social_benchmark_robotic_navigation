#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LaserMinReader(Node):
    def __init__(self):
        super().__init__('laser_min_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/env_0/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg: LaserScan):
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0.01]
        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Minimum distance: {min_distance:.2f} m')
        else:
            self.get_logger().warn('No valid laser data!')

def main(args=None):
    rclpy.init(args=args)
    node = LaserMinReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()