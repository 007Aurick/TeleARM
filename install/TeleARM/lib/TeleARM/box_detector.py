#!/usr/bin/env python3
"""Detect colored warehouse boxes from the RGB camera.

Publishes /detected_box_color as: orange | blue | green | red | none
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


COLORS = {
    'orange': (255, 128, 13),
    'blue': (26, 102, 255),
    'green': (38, 217, 64),
    'red': (255, 38, 38),
}


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.color_pub = self.create_publisher(String, '/detected_box_color', 10)
        self.latest_image = None
        self.max_distance = 80.0
        self.timer = self.create_timer(0.1, self.publish_command)

    def image_callback(self, msg):
        self.latest_image = msg

    def detect_color(self):
        if self.latest_image is None:
            return 'none'

        msg = self.latest_image
        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return 'none'

        img = np.frombuffer(msg.data, dtype=np.uint8)
        if img.size != h * w * 3:
            return 'none'
        img = img.reshape((h, w, 3))

        # Center ROI
        y0, y1 = h // 2 - h // 8, h // 2 + h // 8
        x0, x1 = w // 2 - w // 8, w // 2 + w // 8
        roi = img[y0:y1, x0:x1]
        if roi.size == 0:
            return 'none'

        mean_rgb = roi.reshape(-1, 3).mean(axis=0)

        best_name = 'none'
        best_dist = float('inf')
        for name, target in COLORS.items():
            dist = math.sqrt(
                (mean_rgb[0] - target[0]) ** 2
                + (mean_rgb[1] - target[1]) ** 2
                + (mean_rgb[2] - target[2]) ** 2
            )
            if dist < best_dist:
                best_dist = dist
                best_name = name

        if best_dist > self.max_distance:
            return 'none'
        return best_name

    def publish_command(self):
        msg = String()
        msg.data = self.detect_color()
        self.color_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
