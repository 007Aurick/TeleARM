#!/usr/bin/env python3
"""Classify front obstacles using dual-height rays.

  /scan      (low)  — sees short boxes and tall walls
  /scan_high (high) — sees walls / racks only

  box ahead  = low blocked, high clear
  wall ahead = both blocked
"""



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.subscriber =  self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.color = self.create_publisher(String, '/detected_box_color', 10)
        self.latest_image = None
        self.timer = self.create_timer(0.1, self.publish_command)
        
    
        

    def image_callback(self, msg):
        self.latest_image = msg
            
    def detect_color(self):
        if self.latest_image is None:
            return None
        orange_color = (255,123,13)
        blue_color = (26,102,255)
        green_color = (38,217,64)
        red_color = (255,38,38)


    def publish_command(self):
    



    msg = String()
    msg.data = self.detect_color()
    self.color.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()