#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan


class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.timer = self.create_timer(0.05, self.publish_command)
        self.wall_threshold = 2.0
        self.open_pos = 0.35
        self.close_pos = 0.0
        self.window = 15


    def get_front_distance(self):
        if self.latest_scan is None:
            return float('inf')
        
        n = len(self.latest_scan.ranges)
        center = n // 2

        front_ranges = self.latest_scan.ranges[center - self.window: center + self.window]
        front_ranges = [r for r in front_ranges if r > 0.0 and r != float('inf')]
        return min(front_ranges) if front_ranges else float('inf')
    
    def scan_callback(self, msg):
        self.latest_scan = msg

    def publish_command(self):
        front_distance = self.get_front_distance()
        if front_distance < self.wall_threshold:
            pos = self.close_pos
        else:
            pos = self.open_pos
            
        msg = Float64MultiArray()
        msg.data = [pos, pos]
        self.publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
   