#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msg.msg import LaserScan


class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/command', 10)




def main(args=None):
    rclpy.init(args=args)
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
   