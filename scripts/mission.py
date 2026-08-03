#!/usr/bin/env python3
"""Mission: search -> approach/align box -> grip -> Nav2 to pad -> drop.

Also mutes /diff_drive_publisher while controlling the base, and keeps the
picked Gazebo box glued under the robot while delivering.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose, TwistStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import GetEntityState, GetModelList, SetEntityState
    HAS_GAZEBO_MSGS = True
except ImportError:
    HAS_GAZEBO_MSGS = False


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.cb_group = ReentrantCallbackGroup()
        self.color_sub = self.create_subscription(
            String, '/detected_box_color', self.color_callback, 10
        )
        self.offset_sub = self.create_subscription(
            Float32, '/box_offset', self.offset_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_base_controller/cmd_vel', 10
        )
        self.wander_enable_pub = self.create_publisher(Bool, '/enable_wander', 10)
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group
        )

        self.latest_color = 'none'
        self.box_offset = 0.0
        self.latest_scan = None
        self.carried = None
        self.carried_model = None
        self.state = 'searching'
        self.goal_sent = False
        self.grip_ticks = 0
        self.lost_ticks = 0

        self.pads = {
            'orange': (-14.0, 9.0),
            'blue': (14.0, 9.0),
            'green': (-14.0, -9.0),
            'red': (14.0, -9.0),
        }

        # Approach tuning
        self.align_tol = 0.18          # |offset| under this = centered enough
        self.grip_distance = 1.6       # meters on /scan to close forks
        self.approach_speed = 0.35
        self.turn_gain = 0.9
        self.search_turn = 0.35
        self.grip_hold_ticks = 15      # ~1.5s closed before Nav2

        if HAS_GAZEBO_MSGS:
            self.get_models_cli = self.create_client(
                GetModelList, '/get_model_list', callback_group=self.cb_group
            )
            self.get_state_cli = self.create_client(
                GetEntityState, '/get_entity_state', callback_group=self.cb_group
            )
            self.set_state_cli = self.create_client(
                SetEntityState, '/set_entity_state', callback_group=self.cb_group
            )
        else:
            self.get_logger().warn('gazebo_msgs not available — box will not be carried in sim')

        self.timer = self.create_timer(0.1, self.tick, callback_group=self.cb_group)
        self.get_logger().info('Mission ready (search → approach → grip → deliver)')

    def color_callback(self, msg):
        self.latest_color = msg.data

    def offset_callback(self, msg):
        self.box_offset = msg.data

    def scan_callback(self, msg):
        self.latest_scan = msg

    def send_gripper(self, command):
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def set_wander(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.wander_enable_pub.publish(msg)

    def drive(self, linear_x: float, angular_z: float):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_drive(self):
        self.drive(0.0, 0.0)

    def front_distance(self):
        if self.latest_scan is None or not self.latest_scan.ranges:
            return float('inf')
        n = len(self.latest_scan.ranges)
        center = n // 2
        window = max(n // 12, 1)
        vals = [
            r for r in self.latest_scan.ranges[center - window:center + window]
            if r > 0.05 and math.isfinite(r)
        ]
        return min(vals) if vals else float('inf')

    def send_to_pad(self):
        if self.carried not in self.pads:
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 not ready')
            return

        x, y = self.pads[self.carried]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Nav2 → {self.carried} pad ({x}, {y})')
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self.goal_sent = False
            return
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, _future):
        self.get_logger().info('Arrived at drop pad — dropping')
        self.state = 'dropping'
        self.goal_sent = False

    def pick_nearest_box_model(self):
        """Find Gazebo model box_<color>_* closest to the robot."""
        if not HAS_GAZEBO_MSGS or self.carried is None:
            return None
        if not self.get_models_cli.wait_for_service(timeout_sec=0.5):
            return None
        if not self.get_state_cli.wait_for_service(timeout_sec=0.5):
            return None

        models = self.get_models_cli.call(GetModelList.Request())
        if models is None:
            return None

        prefix = f'box_{self.carried}_'
        robot = self.get_state_cli.call(
            GetEntityState.Request(name='robot', reference_frame='world')
        )
        if robot is None or not robot.success:
            return None

        rx, ry = robot.state.pose.position.x, robot.state.pose.position.y
        best_name, best_d = None, float('inf')
        for name in models.model_names:
            if not name.startswith(prefix):
                continue
            st = self.get_state_cli.call(
                GetEntityState.Request(name=name, reference_frame='world')
            )
            if st is None or not st.success:
                continue
            dx = st.state.pose.position.x - rx
            dy = st.state.pose.position.y - ry
            d = math.hypot(dx, dy)
            if d < best_d:
                best_d = d
                best_name = name
        return best_name

    def glue_box_to_robot(self):
        """Keep carried box under the chassis while delivering."""
        if not HAS_GAZEBO_MSGS or not self.carried_model:
            return
        if not self.get_state_cli.service_is_ready():
            return
        if not self.set_state_cli.service_is_ready():
            return

        robot = self.get_state_cli.call(
            GetEntityState.Request(name='robot', reference_frame='world')
        )
        if robot is None or not robot.success:
            return

        # Place box slightly in front / under body (matches underbody gripper)
        yaw = self._yaw_from_quat(robot.state.pose.orientation)
        ox = 1.6 * math.cos(yaw)
        oy = 1.6 * math.sin(yaw)

        state = EntityState()
        state.name = self.carried_model
        state.pose = Pose()
        state.pose.position.x = robot.state.pose.position.x + ox
        state.pose.position.y = robot.state.pose.position.y + oy
        state.pose.position.z = 0.28
        state.pose.orientation = robot.state.pose.orientation
        state.reference_frame = 'world'
        self.set_state_cli.call(SetEntityState.Request(state=state))

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def tick(self):
        if self.state == 'searching':
            self.set_wander(True)
            self.send_gripper('open')
            self.carried_model = None
            if self.latest_color in self.pads:
                self.carried = self.latest_color
                self.lost_ticks = 0
                self.set_wander(False)
                self.stop_drive()
                self.state = 'approaching'
                self.get_logger().info(f'Saw {self.carried} — approaching')

        elif self.state == 'approaching':
            self.set_wander(False)
            self.send_gripper('open')

            if self.latest_color != self.carried:
                self.lost_ticks += 1
                if self.lost_ticks > 20:
                    self.get_logger().warn('Lost box — searching again')
                    self.carried = None
                    self.state = 'searching'
                    self.set_wander(True)
                else:
                    # gentle turn to reacquire
                    self.drive(0.0, self.search_turn)
                return
            self.lost_ticks = 0

            offset = self.box_offset
            front = self.front_distance()
            ang = -self.turn_gain * offset
            ang = max(-0.6, min(0.6, ang))

            if abs(offset) > self.align_tol:
                # Align first
                self.drive(0.1, ang)
            elif front > self.grip_distance:
                # Centered — drive in
                self.drive(self.approach_speed, ang * 0.5)
            else:
                # Close enough — grip
                self.stop_drive()
                self.send_gripper('close')
                self.grip_ticks = 0
                self.state = 'gripping'
                self.get_logger().info(
                    f'In range ({front:.2f} m) — gripping {self.carried}'
                )

        elif self.state == 'gripping':
            self.set_wander(False)
            self.stop_drive()
            self.send_gripper('close')
            self.grip_ticks += 1
            if self.grip_ticks == 5:
                self.carried_model = self.pick_nearest_box_model()
                if self.carried_model:
                    self.get_logger().info(f'Carrying model {self.carried_model}')
                else:
                    self.get_logger().warn('No Gazebo box model found to attach')
            if self.grip_ticks >= self.grip_hold_ticks:
                self.state = 'delivering'
                self.goal_sent = False
                self.send_to_pad()

        elif self.state == 'delivering':
            self.set_wander(False)
            self.send_gripper('close')
            self.glue_box_to_robot()
            if not self.goal_sent:
                self.send_to_pad()

        elif self.state == 'dropping':
            self.set_wander(False)
            self.stop_drive()
            self.send_gripper('open')
            self.carried = None
            self.carried_model = None
            self.latest_color = 'none'
            self.goal_sent = False
            self.state = 'searching'
            self.set_wander(True)
            self.get_logger().info('Dropped — searching for next box')


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    # Multi-threaded so Gazebo service calls from the timer don't deadlock
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
