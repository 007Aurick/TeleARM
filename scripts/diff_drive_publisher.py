#!/usr/bin/env python3
"""Wander: drive straight when clear. Turn in place if blocked. No reverse spam."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class DiffDrivePublisher(Node):
    def __init__(self):
        super().__init__('diff_drive_publisher')
        latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            TwistStamped, '/diff_drive_base_controller/cmd_vel', 10
        )
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/enable_wander', self.wander_callback, latch)

        self.wander_enabled = False
        self.was_enabled = False
        self.latest_scan = None
        self.create_timer(0.1, self.publish_command)

        self.ticks = 0
        self.state = 'forward'
        self.turn_direction = 1

        # Only stop when something is actually close
        self.stop_distance = 2.2
        self.clear_distance = 3.0
        self.forward_speed = 0.90
        self.turn_speed = 0.55
        self.min_turn_ticks = 24
        self.max_turn_ticks = 60

    def scan_callback(self, msg):
        self.latest_scan = msg

    def wander_callback(self, msg):
        self.wander_enabled = msg.data

    def _finite(self, ranges):
        return [r for r in ranges if r > 0.08 and math.isfinite(r)]

    def get_front_distance(self):
        if self.latest_scan is None or not self.latest_scan.ranges:
            return float('inf')
        n = len(self.latest_scan.ranges)
        center = n // 2
        window = max(n // 14, 3)
        vals = self._finite(self.latest_scan.ranges[center - window:center + window])
        return min(vals) if vals else float('inf')

    def get_side_preference(self):
        if self.latest_scan is None:
            return 1
        n = len(self.latest_scan.ranges)
        right = self._finite(self.latest_scan.ranges[n // 8: 3 * n // 8])
        left = self._finite(self.latest_scan.ranges[5 * n // 8: 7 * n // 8])
        left_d = min(left) if left else float('inf')
        right_d = min(right) if right else float('inf')
        return 1 if left_d >= right_d else -1

    def _publish(self, lin, ang):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body'
        cmd.twist.linear.x = float(lin)
        cmd.twist.angular.z = float(ang)
        self.publisher.publish(cmd)

    def publish_command(self):
        if not self.wander_enabled:
            if self.was_enabled:
                self._publish(0.0, 0.0)
                self.state = 'forward'
                self.ticks = 0
            self.was_enabled = False
            return
        self.was_enabled = True

        front = self.get_front_distance()

        if self.state == 'forward':
            if front > self.stop_distance:
                self._publish(self.forward_speed, 0.0)
            else:
                self.state = 'turn'
                self.ticks = 0
                self.turn_direction = self.get_side_preference()
                self._publish(0.0, self.turn_direction * self.turn_speed)

        elif self.state == 'turn':
            self._publish(0.0, self.turn_direction * self.turn_speed)
            self.ticks += 1
            if self.ticks >= self.min_turn_ticks and (
                front > self.clear_distance or self.ticks >= self.max_turn_ticks
            ):
                self.state = 'forward'
                self.ticks = 0


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePublisher()
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
