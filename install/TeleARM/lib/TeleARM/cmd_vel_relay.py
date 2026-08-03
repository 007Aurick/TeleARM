#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    """Convert Nav2 Twist output to TwistStamped for diff_drive_controller."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10,
        )
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10,
        )

    def callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'body'
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()