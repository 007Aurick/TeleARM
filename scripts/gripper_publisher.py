#!/usr/bin/env python3
"""Open/close forklift fingers.

Joint origins at ±0.16, finger thickness 0.14 → gap ≈ 0.18 + 2*pos.
Open 0.72 → ~1.62 m gap (obvious). Close 0.32 → ~0.82 m (grips 0.90 m box).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )
        self.create_subscription(String, '/gripper_command', self.command_callback, 10)
        self.command = 'open'
        self.create_timer(0.05, self.publish_command)
        self.close_pos = 0.32
        self.open_pos = 0.72

    def command_callback(self, msg):
        self.command = msg.data

    def publish_command(self):
        pos = self.close_pos if self.command == 'close' else self.open_pos
        msg = Float64MultiArray()
        msg.data = [pos, pos]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
