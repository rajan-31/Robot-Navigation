#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

def main(args=None):
    # first line
    rclpy.init(args=args)

    node = Node('py_test')  # not need to be name of the file
    node.get_logger().info('Hello ROS2')

    # pauses node here, and keeps it alive
    rclpy.spin(node)

    # last line
    rclpy.shutdown()

if __name__ == '__main__':
    main()