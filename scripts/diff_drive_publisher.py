#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class DiffDrivePublisher(Node):
    def __init__(self):
        super().__init__('diff_drive_publisher')
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10,
        )
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.wander_sub = self.create_subscription(
            Bool, '/enable_wander', self.wander_callback, 10
        )
        self.wander_enabled = True
        self.latest_scan = None
        self.timer = self.create_timer(0.05, self.publish_command)

        self.ticks = 0
        self.state = 'forward'
        self.turn_direction = 1

        # Stop earlier; only resume after a real clear gap (less twitch)
        self.stop_distance = 1.6
        self.clear_distance = 2.4

        self.forward_speed = 0.55
        self.reverse_speed = -0.45
        self.turn_speed = 1.1          # snappier turn (was 0.5 — felt like a seizure)

        self.backup_ticks = 16         # ~0.8s reverse
        self.min_turn_ticks = 36       # ~1.8s committed turn before re-check
        self.max_turn_ticks = 120      # ~6s cap

    def scan_callback(self, msg):
        self.latest_scan = msg

    def wander_callback(self, msg):
        self.wander_enabled = msg.data

    def _finite(self, ranges):
        return [r for r in ranges if r > 0.05 and math.isfinite(r)]

    def get_front_distance(self):
        if self.latest_scan is None or not self.latest_scan.ranges:
            return float('inf')
        n = len(self.latest_scan.ranges)
        center = n // 2
        window = max(n // 18, 3)  # ~±10–20 deg — front cone only
        vals = self._finite(
            self.latest_scan.ranges[center - window:center + window]
        )
        return min(vals) if vals else float('inf')

    def get_side_preference(self):
        """Pick turn direction: toward the more open side."""
        if self.latest_scan is None:
            return 1
        n = len(self.latest_scan.ranges)
        # left = +90-ish, right = -90-ish for 360 scan
        right = self._finite(self.latest_scan.ranges[n // 8: 3 * n // 8])
        left = self._finite(self.latest_scan.ranges[5 * n // 8: 7 * n // 8])
        left_d = min(left) if left else float('inf')
        right_d = min(right) if right else float('inf')
        return 1 if left_d >= right_d else -1

    def publish_command(self):
        if not self.wander_enabled:
            return

        front = self.get_front_distance()
        lin, ang = 0.0, 0.0

        if self.state == 'forward':
            if front > self.stop_distance:
                lin, ang = self.forward_speed, 0.0
            else:
                self.state = 'reverse'
                self.ticks = 0
                self.turn_direction = self.get_side_preference()

        elif self.state == 'reverse':
            lin, ang = self.reverse_speed, 0.0
            self.ticks += 1
            if self.ticks >= self.backup_ticks:
                self.state = 'turn'
                self.ticks = 0

        elif self.state == 'turn':
            # Pure spin — commit for min_turn_ticks so it doesn't micro-twitch
            lin, ang = 0.0, self.turn_direction * self.turn_speed
            self.ticks += 1
            # Only care about FRONT being open (sides near a wall are fine)
            if self.ticks >= self.min_turn_ticks and (
                front > self.clear_distance or self.ticks >= self.max_turn_ticks
            ):
                self.state = 'forward'
                self.ticks = 0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body'
        cmd.twist.linear.x = lin
        cmd.twist.angular.z = ang
        self.publisher.publish(cmd)


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
