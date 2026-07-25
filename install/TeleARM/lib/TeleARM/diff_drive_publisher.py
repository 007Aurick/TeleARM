#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


class DiffDrivePublisher(Node):
    def __init__(self):
        super().__init__('diff_drive_publisher')
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10,
        )
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.latest_scan = None
        self.timer = self.create_timer(0.05, self.publish_command)
        self.ticks = 0
        self.state = 'forward'
        self.obstacle_threshold = 1.5   # meters
        self.forward_speed = 0.1
        self.reverse_speed = -0.2
        self.turn_speed = 0.5
        self.backup_ticks = 20          # ~1.0s at 0.05s timer
        self.turn_ticks = 63   

        

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_front_distance(self):
        # No scan yet → treat as "unknown / far" (or blocked if you prefer)
        if self.latest_scan is None:
            return float('inf')

        n = len(self.latest_scan.ranges)#360 lidar scans 1 per degree
        center = n // 2# center index of the scan
        window = 15# window of indices to consider around the center (15 degrees on each side)
        front_ranges = self.latest_scan.ranges[center - window:center + window]#takes a slice of the ranges array from center - window to center + window, which gives us a total of 30 values (15 on each side of the center)
        front_ranges = [
            r for r in front_ranges
            if r > 0.0 and r != float('inf')#only append the range values that are greater than 0.0 and not equal to infinity to the front_ranges list
        ]
        return min(front_ranges) if front_ranges else float('inf')#only the minimum value of the front_ranges list is returned as it'll be closer to the object.


    def publish_command(self):
        lin, ang = 0.0, 0.0
        front_distance = self.get_front_distance()

        if self.state == 'forward':
            if front_distance > self.obstacle_threshold:
                lin,ang = self.forward_speed, 0.0
            else:
                self.state = 'reverse'
                self.ticks = 0
        elif self.state == 'reverse':
            lin,ang = self.reverse_speed, 0.0
            self.ticks += 1#1 tick per 0.05s timer callback, so 20 ticks = 1 second of backing up
            if self.ticks >= self.backup_ticks:
                self.state = 'turn'#once we exceed the backup_ticks, we transition to the 'turn' state and reset the tick counter to 0
                self.ticks = 0
        elif self.state == 'turn':
            lin,ang = 0.0, self.turn_speed
            self.ticks += 1 #63 ticks = 3.15 seconds of turning at 0.05s timer callback
            if self.ticks >= self.turn_ticks:
                self.state = 'forward'#once we exceed the turn_ticks, we transition back to the 'forward' state and reset the tick counter to 0
                self.ticks = 0
           

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist.linear.x = lin
        command.twist.angular.z = ang
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
