#!/usr/bin/env python3
"""Mission: detect color -> grip -> Nav2 to matching drop pad -> release."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.color_subscriber = self.create_subscription(
            String, '/detected_box_color', self.color_callback, 10
        )
        self.gripper_publisher = self.create_publisher(String, '/gripper_command', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.latest_color = None
        self.carried = None
        self.state = 'searching'
        self.goal_sent = False

        self.pads = {
            'orange': (-14.0, 9.0),
            'blue': (14.0, 9.0),
            'green': (-14.0, -9.0),
            'red': (14.0, -9.0),
        }

        self.timer = self.create_timer(0.1, self.tick)

    def color_callback(self, msg):
        self.latest_color = msg.data

    def send_gripper(self, command):
        msg = String()
        msg.data = command
        self.gripper_publisher.publish(msg)

    def send_to_pad(self):
        if self.carried not in self.pads:
            self.get_logger().error(f'No pad for color: {self.carried}')
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 navigate_to_pose not available yet')
            return

        x, y = self.pads[self.carried]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Delivering {self.carried} to pad ({x}, {y})')
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self.goal_sent = False
            self.state = 'searching'
            self.carried = None
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        self.get_logger().info('Arrived at drop pad')
        self.state = 'dropping'
        self.goal_sent = False

    def tick(self):
        if self.state == 'searching':
            self.send_gripper('open')
            if self.latest_color in ('orange', 'blue', 'green', 'red'):
                self.carried = self.latest_color
                self.send_gripper('close')
                self.state = 'delivering'
                self.goal_sent = False
                self.send_to_pad()

        elif self.state == 'delivering':
            self.send_gripper('close')
            # Retry goal if Nav2 wasn't up the first time
            if not self.goal_sent:
                self.send_to_pad()

        elif self.state == 'dropping':
            self.send_gripper('open')
            self.carried = None
            self.latest_color = None
            self.goal_sent = False
            self.state = 'searching'


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
