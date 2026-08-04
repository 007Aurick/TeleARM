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
    from gazebo_msgs.msg import EntityState, ModelStates
    from gazebo_msgs.srv import SetEntityState
    HAS_GAZEBO_MSGS = True
except ImportError:
    HAS_GAZEBO_MSGS = False
    ModelStates = None


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
        self.last_offset = 0.0
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

        # Lidar sits ~2.1 m forward; forks ~2.7 m. Close only when face is near forks.
        self.align_tol = 0.28
        self.grip_distance = 0.45
        self.approach_speed = 0.32
        self.turn_gain = 0.9
        self.search_turn = 0.45
        self.grip_hold_ticks = 25
        self.carry_forward = 2.70
        self.lost_limit = 60          # 6s sticky track before giving up
        self.model_states = None

        if HAS_GAZEBO_MSGS:
            self.create_subscription(
                ModelStates, '/gazebo/model_states', self.model_states_cb, 10
            )
            # gazebo_ros_state exposes both; prefer namespaced service
            self.set_state_cli = self.create_client(
                SetEntityState, '/gazebo/set_entity_state', callback_group=self.cb_group
            )
        else:
            self.set_state_cli = None
            self.get_logger().warn('gazebo_msgs not available — box will not be carried in sim')

        self.timer = self.create_timer(0.1, self.tick, callback_group=self.cb_group)
        self.get_logger().info('Mission ready (search → approach → grip → deliver)')

    def color_callback(self, msg):
        self.latest_color = msg.data

    def offset_callback(self, msg):
        self.box_offset = msg.data
        if self.latest_color != 'none':
            self.last_offset = msg.data

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

    def model_states_cb(self, msg):
        self.model_states = msg

    def robot_world_pose(self):
        if self.model_states is None:
            return None
        try:
            i = self.model_states.name.index('robot')
        except ValueError:
            return None
        p = self.model_states.pose[i]
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return p.position.x, p.position.y, math.atan2(siny, cosy), q

    def pick_nearest_box_model(self):
        """Nearest Gazebo box_<color>_* to the robot in world frame."""
        if self.carried is None or self.model_states is None:
            return None
        pose = self.robot_world_pose()
        if pose is None:
            return None
        rx, ry, _, _ = pose
        prefix = f'box_{self.carried}_'
        best_name, best_d = None, float('inf')
        for name, p in zip(self.model_states.name, self.model_states.pose):
            if not name.startswith(prefix):
                continue
            d = math.hypot(p.position.x - rx, p.position.y - ry)
            if d < best_d:
                best_d = d
                best_name = name
        return best_name

    def glue_box_to_robot(self):
        """Keep carried box between the forks (Gazebo world frame)."""
        if not HAS_GAZEBO_MSGS or not self.carried_model or self.set_state_cli is None:
            return
        if not self.set_state_cli.service_is_ready():
            return
        pose = self.robot_world_pose()
        if pose is None:
            return
        rx, ry, yaw, q = pose
        ox = self.carry_forward * math.cos(yaw)
        oy = self.carry_forward * math.sin(yaw)

        state = EntityState()
        state.name = self.carried_model
        state.pose = Pose()
        state.pose.position.x = rx + ox
        state.pose.position.y = ry + oy
        state.pose.position.z = 0.30
        state.pose.orientation = q
        state.reference_frame = 'world'
        self.set_state_cli.call_async(SetEntityState.Request(state=state))

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

            seeing = self.latest_color == self.carried
            if seeing:
                self.lost_ticks = 0
                offset = self.box_offset
            else:
                self.lost_ticks += 1
                front_probe = self.front_distance()
                # Keep committing if lidar still sees something close ahead
                if self.lost_ticks > self.lost_limit and front_probe > 2.5:
                    self.get_logger().warn('Lost box — searching again')
                    self.carried = None
                    self.state = 'searching'
                    self.set_wander(True)
                    return
                offset = self.last_offset

            front = self.front_distance()
            ang = -self.turn_gain * offset
            ang = max(-0.7, min(0.7, ang))

            if abs(offset) > self.align_tol and front > self.grip_distance + 0.3:
                self.drive(0.12, ang)
            elif front > self.grip_distance:
                self.drive(self.approach_speed, ang * 0.4)
            else:
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
