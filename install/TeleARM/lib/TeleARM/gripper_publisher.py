#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GripperPublisher(Node):
    """Open/close the underbody gripper by commanding joint positions."""

    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10,
        )
        self.timer = self.create_timer(0.05, self.publish_command)
        self.ticks = 0
        self.open_pos = 0.35
        self.closed_pos = 0.0
        self.period_ticks = 80  # ~4s open/close cycle at 20 Hz

    def publish_command(self):
        # Smooth 0→1→0 triangle so fingers open then close.
        phase = (self.ticks % self.period_ticks) / float(self.period_ticks)
        if phase < 0.5:
            t = phase * 2.0
        else:
            t = (1.0 - phase) * 2.0
        pos = self.closed_pos + t * (self.open_pos - self.closed_pos)

        msg = Float64MultiArray()
        msg.data = [pos, pos]
        self.publisher.publish(msg)
        self.ticks += 1


def main(args=None):
    rclpy.init(args=args)
    node = GripperPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
