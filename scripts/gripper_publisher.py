#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String



class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.color_subscriber = self.create_subscription(String, '/detected_box_color', self.color_callback, 10)
        self.latest_color = None
        self.timer = self.create_timer(0.05, self.publish_command)
        self.close_pos = 0.0
        self.open_pos = 0.35
    
    def color_callback(self, msg):
        self.latest_color = msg.data
    
    def publish_command(self):
        if self.latest_color != 'none' and self.latest_color is not None:
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
   