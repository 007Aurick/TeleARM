import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.color_subscriber = self.create_subscription(String, '/detected_box_color',self.color_callback, 10)
        self.gripper_publisher = self.create_publisher(String, '/gripper_command', 10)
        self.carried_box = False
        self.state = 'searching'

    def color_callback(self, msg):
        self.carried_box = msg.data
    
    def publish_command(self):


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()