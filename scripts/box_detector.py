#!/usr/bin/env python3
"""Classify front obstacles using dual-height rays.

  /scan      (low)  — sees short boxes and tall walls
  /scan_high (high) — sees walls / racks only

  box ahead  = low blocked, high clear
  wall ahead = both blocked
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.low = None
        self.high = None
        self.window = 20
        self.block_threshold = 2.5  # meters
        self.subscriber =  self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        def image_callback(self, msg):
            self.latest_image = msg
            
        def detect_color(self):


        def publish_command(self):



def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()