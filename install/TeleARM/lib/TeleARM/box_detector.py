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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.low = None
        self.high = None
        self.window = 20
        self.block_threshold = 2.5  # meters

        self.create_subscription(LaserScan, '/scan', self.low_cb, 10)
        self.create_subscription(LaserScan, '/scan_high', self.high_cb, 10)
        self.box_pub = self.create_publisher(Bool, '/box_ahead', 10)
        self.kind_pub = self.create_publisher(String, '/front_obstacle_type', 10)
        self.dist_pub = self.create_publisher(Float32, '/box_distance', 10)
        self.create_timer(0.1, self.tick)

    def low_cb(self, msg):
        self.low = msg

    def high_cb(self, msg):
        self.high = msg

    def front_min(self, scan):
        if scan is None:
            return float('inf')
        n = len(scan.ranges)
        c = n // 2
        chunk = scan.ranges[max(0, c - self.window):c + self.window]
        vals = [r for r in chunk if math.isfinite(r) and r > 0.05]
        return min(vals) if vals else float('inf')

    def tick(self):
        low_d = self.front_min(self.low)
        high_d = self.front_min(self.high)
        low_hit = low_d < self.block_threshold
        high_hit = high_d < self.block_threshold

        if low_hit and not high_hit:
            kind = 'box'
            is_box = True
        elif low_hit and high_hit:
            kind = 'wall'
            is_box = False
        else:
            kind = 'clear'
            is_box = False

        self.box_pub.publish(Bool(data=is_box))
        self.kind_pub.publish(String(data=kind))
        if is_box:
            self.dist_pub.publish(Float32(data=float(low_d)))


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
