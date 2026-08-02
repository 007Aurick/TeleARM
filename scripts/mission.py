import rclpy
from rclpy.node import Node

class Mission(Node):
    def __init__(self):
        super().__init__('mission')



def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()