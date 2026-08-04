#!/usr/bin/env python3
"""Detect colored warehouse boxes from the RGB camera.

Publishes:
  /detected_box_color  (String)  orange|blue|green|red|none
  /box_offset          (Float32) -1 left ... +1 right (color blob centroid)
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


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
        self.offset_pub = self.create_publisher(Float32, '/box_offset', 10)
        self.latest_image = None
        self.max_distance = 90.0
        self.timer = self.create_timer(0.1, self.publish_command)

    def image_callback(self, msg):
        self.latest_image = msg

    def detect(self):
        """Return (color_name, lateral_offset)."""
        if self.latest_image is None:
            return 'none', 0.0

        msg = self.latest_image
        h, w = msg.height, msg.width
        if h == 0 or w == 0:
            return 'none', 0.0

        img = np.frombuffer(msg.data, dtype=np.uint8)
        if img.size != h * w * 3:
            return 'none', 0.0
        img = img.reshape((h, w, 3)).astype(np.float32)

        # Lower-center band — floor boxes ahead of the camera
        y0, y1 = int(h * 0.35), int(h * 0.95)
        x0, x1 = int(w * 0.1), int(w * 0.9)
        roi = img[y0:y1, x0:x1]
        if roi.size == 0:
            return 'none', 0.0

        best_name = 'none'
        best_score = 0
        best_offset = 0.0
        thresh = self.max_distance

        for name, target in COLORS.items():
            diff = roi - np.array(target, dtype=np.float32)
            dist = np.sqrt(np.sum(diff * diff, axis=2))
            mask = dist < thresh
            count = int(mask.sum())
            if count < 30:
                continue
            if count > best_score:
                best_score = count
                best_name = name
                ys, xs = np.where(mask)
                # centroid x in full image coords → [-1, 1]
                cx = float(xs.mean()) + x0
                best_offset = (cx / w) * 2.0 - 1.0

        return best_name, best_offset

    def publish_command(self):
        color, offset = self.detect()
        cmsg = String()
        cmsg.data = color
        self.color_pub.publish(cmsg)
        omsg = Float32()
        omsg.data = float(offset) if color != 'none' else 0.0
        self.offset_pub.publish(omsg)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
