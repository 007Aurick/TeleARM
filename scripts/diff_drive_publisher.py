#!/usr/bin/env python3

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
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.wander_sub = self.create_subscription(
            Bool, '/enable_wander', self.wander_callback, 10
        )
        self.wander_enabled = True
        self.latest_scan = None
        self.timer = self.create_timer(0.05, self.publish_command)
        self.ticks = 0
        self.state = 'forward'
        self.obstacle_threshold = 2.0   # meters, front stopping distance
        self.side_obstacle_threshold = 1.0   # meters, only react to genuinely close flank contact
        self.forward_speed = 0.5
        self.reverse_speed = -0.5
        self.turn_speed = 0.5
        self.backup_ticks = 20          # ~1.0s at 0.05s timer
        self.min_turn_ticks = 10        # ~0.5s minimum turn before re-checking sensors
        self.max_turn_ticks = 400       # ~20s failsafe cap if it can never see clear
        self.turn_direction = 0

        

    def scan_callback(self, msg):
        self.latest_scan = msg

    def wander_callback(self, msg):
        self.wander_enabled = msg.data

    def get_sides(self, ranges):
        n = len(ranges)
        half_width = n // 6  # +/- 60 degrees around each side center, meets the front cone with no gap
        right_region = ranges[n//4-half_width:n//4+half_width]  # centered on -90 degrees: right
        left_region = ranges[3*n//4-half_width:3*n//4+half_width]  # centered on +90 degrees: left
        left_distance = [r for r in left_region if r > 0.0 and r != float('inf')]
        right_distance = [r for r in right_region if r > 0.0 and r != float('inf')]

        left_clear = min(left_distance) if left_distance else float('inf')
        right_clear = min(right_distance) if right_distance else float('inf')

        return left_clear, right_clear

    def get_front_distance(self):
        # No scan yet → treat as "unknown / far" (or blocked if you prefer)
        if self.latest_scan is None:
            return float('inf')

        n = len(self.latest_scan.ranges)
        center = n // 2# center index of the scan
        window = n // 12# +/- 30 degrees around center, meets the side cones with no gap
        front_ranges = self.latest_scan.ranges[center - window:center + window]#takes a slice of the ranges array from center - window to center + window, which gives us a total of 30 values (15 on each side of the center)
        front_ranges = [
            r for r in front_ranges
            if r > 0.0 and r != float('inf')#only append the range values that are greater than 0.0 and not equal to infinity to the front_ranges list
        ]
        return min(front_ranges) if front_ranges else float('inf')#only the minimum value of the front_ranges list is returned as it'll be closer to the object.


    def publish_command(self):
        if not self.wander_enabled:
            return

        lin, ang = 0.0, 0.0
        front_distance = self.get_front_distance()
        left_distance, right_distance = self.get_sides(self.latest_scan.ranges) if self.latest_scan else (float('inf'), float('inf'))
        
        if self.state == 'forward':
            if front_distance > self.obstacle_threshold and left_distance > self.side_obstacle_threshold and right_distance > self.side_obstacle_threshold:
                lin,ang = self.forward_speed, 0.0
            else:
                self.state = 'reverse'
                self.ticks = 0
                if left_distance > right_distance:
                    self.turn_direction = 1  # turn left
                else:
                    self.turn_direction = -1  # turn right
        elif self.state == 'reverse':
            lin,ang = self.reverse_speed, 0.0
            self.ticks += 1#1 tick per 0.05s timer callback, so 20 ticks = 1 second of backing up
            if self.ticks >= self.backup_ticks:
                self.state = 'turn'#once we exceed the backup_ticks, we transition to the 'turn' state and reset the tick counter to 0
                self.ticks = 0
        elif self.state == 'turn':
            lin,ang = 0.0, self.turn_direction * self.turn_speed
            self.ticks += 1
            is_clear = (front_distance > self.obstacle_threshold
                        and left_distance > self.side_obstacle_threshold
                        and right_distance > self.side_obstacle_threshold)
            # Turn until sensors actually agree it's clear, not for a fixed guessed angle.
            # min_turn_ticks avoids a same-tick no-op turn; max_turn_ticks is a failsafe against endless spinning.
            if self.ticks >= self.min_turn_ticks and (is_clear or self.ticks >= self.max_turn_ticks):
                self.state = 'forward'
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
