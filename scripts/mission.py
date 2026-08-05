#!/usr/bin/env python3
"""Wander → pick colored box → deliver to matching pad → drop → repeat."""

import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

try:
    from gazebo_msgs.msg import EntityState, ModelStates
    from gazebo_msgs.srv import SetEntityState
    HAS_GAZEBO = True
except ImportError:
    HAS_GAZEBO = False
    ModelStates = None


COLORS = ('orange', 'blue', 'green', 'red')


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.cb = ReentrantCallbackGroup()
        latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/diff_drive_base_controller/cmd_vel', 10
        )
        self.gripper_pub = self.create_publisher(String, '/gripper_command', 10)
        self.wander_pub = self.create_publisher(Bool, '/enable_wander', latch)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.scan = None
        self.models = None
        self.state = 'searching'
        self.color = None
        self.target = None
        self.holding = False
        self.grip_t = 0
        self.lost_t = 0
        self.avoid_t = 0
        self.avoid_dir = 1
        self.stuck_t = 0
        self.last_goal_d = None
        self.turn_sign = 0
        self.hold_tick = 0
        self.drop_t = 0
        self.drop_turn = 0
        self.drop_xy = None
        self._wander = None
        self.drop_counts = {c: 0 for c in COLORS}
        self.box_half = 0.45

        self.pads = {
            'orange': (-22.0, 15.0),
            'blue': (22.0, 15.0),
            'green': (-22.0, -15.0),
            'red': (22.0, -15.0),
        }

        # Tuned for ~0.90 m boxes + scaled forklift (S=1.35, forks ~x=3.17)
        self.grip_fwd = 3.70
        self.lat_ok = 0.28
        self.detect_range = 9.0
        self.fov = 1.0
        self.pad_ok = 3.2
        self.carry_fwd = 3.70
        self.box_spacing = 1.40

        self.set_state = None
        if HAS_GAZEBO:
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
            self.set_state = self.create_client(
                SetEntityState, '/gazebo/set_entity_state', callback_group=self.cb
            )
        else:
            self.get_logger().error('gazebo_msgs missing')

        self.create_timer(0.1, self.tick, callback_group=self.cb)
        self.enable_wander(False)
        self.get_logger().info('Mission: find box → pick → color pad → drop')

    def on_scan(self, msg):
        self.scan = msg

    def on_models(self, msg):
        self.models = msg

    def enable_wander(self, on):
        if self._wander is on:
            return
        self._wander = on
        msg = Bool()
        msg.data = on
        self.wander_pub.publish(msg)

    def gripper(self, cmd):
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

    def drive(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'body'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd_pub.publish(m)

    def stop(self):
        self.drive(0.0, 0.0)

    @staticmethod
    def wrap(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def yaw_of(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def robot_pose(self):
        if self.models is None:
            return None
        try:
            i = self.models.name.index('robot')
        except ValueError:
            return None
        p = self.models.pose[i]
        return p.position.x, p.position.y, self.yaw_of(p.orientation), p.orientation

    def ranges(self, lo, hi, min_r=0.08):
        if self.scan is None or not self.scan.ranges:
            return float('inf')
        n = len(self.scan.ranges)
        a, b = int(n * lo), max(int(n * lo) + 1, int(n * hi))
        vals = [
            r for r in self.scan.ranges[a:b]
            if r > min_r and math.isfinite(r)
        ]
        return min(vals) if vals else float('inf')

    def front(self):
        return self.ranges(0.42, 0.58)

    def front_obstacle(self, carrying=False):
        """Wall/rack ahead. When carrying, ignore near hits (= the box in forks)."""
        if carrying:
            return self.ranges(0.38, 0.62, min_r=1.55)
        return self.ranges(0.38, 0.62, min_r=0.12)

    def open_side(self):
        left = self.ranges(0.55, 0.85, min_r=0.3)
        right = self.ranges(0.15, 0.45, min_r=0.3)
        return 1 if left >= right else -1

    def near_pad(self, color, bx, by):
        """Skip boxes already stacked on their color pad."""
        px, py = self.pads[color]
        return math.hypot(bx - px, by - py) < 7.0

    def pad_drop_xy(self, color):
        """Slot ON the colored pad (marker overlap OK). Grid so later boxes don't kick earlier ones."""
        px, py = self.pads[color]
        n = self.drop_counts[color]
        # 4x4 pad; 0.90 m boxes with black border — pack in a tight on-pad grid
        slots = (
            (0.0, 0.0),
            (1.15, 0.0),
            (-1.15, 0.0),
            (0.0, 1.15),
            (0.0, -1.15),
            (1.15, 1.15),
            (-1.15, 1.15),
            (1.15, -1.15),
            (-1.15, -1.15),
        )
        ox, oy = slots[n % len(slots)]
        return px + ox, py + oy

    def set_box_pose(self, name, x, y, z, yaw=0.0):
        if self.set_state is None or not self.set_state.service_is_ready():
            return False
        st = EntityState()
        st.name = name
        st.pose = Pose()
        st.pose.position.x = float(x)
        st.pose.position.y = float(y)
        st.pose.position.z = float(z)
        st.pose.orientation.z = math.sin(yaw * 0.5)
        st.pose.orientation.w = math.cos(yaw * 0.5)
        # Kill leftover carry velocity so the box plants on the pad
        st.twist.linear.x = 0.0
        st.twist.linear.y = 0.0
        st.twist.linear.z = 0.0
        st.twist.angular.x = 0.0
        st.twist.angular.y = 0.0
        st.twist.angular.z = 0.0
        st.reference_frame = 'world'
        self.set_state.call_async(SetEntityState.Request(state=st))
        return True

    def best_box(self, color=None, name=None, any_bearing=False):
        rp = self.robot_pose()
        if rp is None or self.models is None:
            return None
        rx, ry, yaw, _ = rp
        best = None
        for n, p in zip(self.models.name, self.models.pose):
            c = next((col for col in COLORS if n.startswith(f'box_{col}_')), None)
            if c is None:
                continue
            if color and c != color:
                continue
            if name and n != name:
                continue
            # Don't re-pick boxes already delivered to their pad
            if name is None and self.near_pad(c, p.position.x, p.position.y):
                continue
            dx, dy = p.position.x - rx, p.position.y - ry
            dist = math.hypot(dx, dy)
            if dist < 0.15 or dist > self.detect_range:
                continue
            brg = self.wrap(math.atan2(dy, dx) - yaw)
            if name is None and not any_bearing and abs(brg) > self.fov:
                continue
            fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
            lat = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if best is None or dist < best['dist']:
                best = {
                    'name': n, 'color': c, 'dist': dist, 'bearing': brg,
                    'fwd': fwd, 'lat': lat,
                }
        return best

    def in_fork_pocket(self, b):
        return b is not None and (3.30 < b['fwd'] < 4.20) and (abs(b['lat']) < 0.50)

    def hold_box(self):
        """Keep the already-forked box seated in the pocket (no far snap)."""
        if not self.holding or not self.target or self.set_state is None:
            return
        if not self.set_state.service_is_ready():
            return
        self.hold_tick += 1
        if self.hold_tick % 3 != 0:
            return
        rp = self.robot_pose()
        if rp is None:
            return
        rx, ry, yaw, q = rp
        st = EntityState()
        st.name = self.target
        st.pose = Pose()
        st.pose.position.x = rx + self.carry_fwd * math.cos(yaw)
        st.pose.position.y = ry + self.carry_fwd * math.sin(yaw)
        st.pose.position.z = self.box_half
        st.pose.orientation = q
        st.reference_frame = 'world'
        self.set_state.call_async(SetEntityState.Request(state=st))

    def go_to(self, tx, ty, speed):
        """Face goal then drive straight. Reverse only if about to hit something."""
        rp = self.robot_pose()
        if rp is None:
            self.stop()
            return float('inf')
        rx, ry, yaw, _ = rp
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)
        brg = self.wrap(math.atan2(dy, dx) - yaw)
        clear = self.front_obstacle(carrying=self.holding)

        if self.avoid_t > 0:
            self.avoid_t -= 1
            self.drive(0.0, self.avoid_dir * 0.60)
            return dist

        if clear < 1.6:
            if self.avoid_t == 0:
                self.avoid_dir = self.open_side()
                self.avoid_t = 25
                self.turn_sign = 0
            self.drive(0.0, self.avoid_dir * 0.60)
            return dist

        if abs(brg) > 0.45:
            if self.turn_sign == 0:
                self.turn_sign = 1 if brg > 0.0 else -1
            self.drive(0.0, self.turn_sign * 0.50)
            return dist
        self.turn_sign = 0

        self.drive(speed, 0.0)
        return dist

    def tick(self):
        if self.state == 'searching':
            self.holding = False
            self.enable_wander(True)
            self.gripper('open')
            self.target = None
            b = self.best_box()
            if b is None:
                return
            if self.front() + 1.2 < b['fwd']:
                return
            self.color = b['color']
            self.target = b['name']
            self.lost_t = 0
            self.enable_wander(False)
            self.stop()
            self.state = 'approaching'
            self.get_logger().info(f"Target {b['color']} ({b['name']}) {b['dist']:.1f} m")

        elif self.state == 'approaching':
            self.enable_wander(False)
            self.gripper('open')
            b = self.best_box(color=self.color, name=self.target, any_bearing=True)
            if b is None:
                b = self.best_box(color=self.color, any_bearing=True)
                if b:
                    self.target = b['name']
            if b is None:
                self.lost_t += 1
                if self.lost_t > 60:
                    self.get_logger().warn('Lost box')
                    self.color = None
                    self.state = 'searching'
                else:
                    self.drive(0.0, 0.45)
                return
            self.lost_t = 0

            if self.front() + 1.0 < b['fwd'] and self.front() < 2.0:
                self.drive(0.0, 0.55 if b['lat'] >= 0 else -0.55)
                return

            if self.in_fork_pocket(b):
                self.stop()
                self.gripper('close')
                self.grip_t = 0
                self.state = 'gripping'
                self.get_logger().info(
                    f"In forks — clamping {b['color']} (fwd={b['fwd']:.2f})"
                )
                return

            if abs(b['bearing']) > 0.35 and b['fwd'] > self.grip_fwd + 0.25:
                self.drive(0.0, 0.50 if b['bearing'] > 0 else -0.50)
            elif b['fwd'] > self.grip_fwd:
                self.drive(0.40, 0.0)
            elif abs(b['lat']) > self.lat_ok:
                self.drive(0.0, 0.45 if b['lat'] > 0 else -0.45)
            else:
                self.drive(0.15, 0.0)

        elif self.state == 'gripping':
            self.enable_wander(False)
            self.stop()
            self.gripper('close')
            self.grip_t += 1
            if self.grip_t >= 8:
                self.holding = True
                self.hold_box()
            if self.grip_t >= 18:
                b = self.best_box(color=self.color, name=self.target, any_bearing=True)
                if not self.in_fork_pocket(b):
                    self.get_logger().warn('Pick failed — retry')
                    self.holding = False
                    self.gripper('open')
                    self.color = None
                    self.target = None
                    self.state = 'searching'
                    return
                self.avoid_t = 0
                self.stuck_t = 0
                self.turn_sign = 0
                self.last_goal_d = None
                self.drop_xy = None  # pick next free on-pad slot
                self.state = 'delivering'
                px, py = self.pads[self.color]
                self.get_logger().info(f'Got it — going to {self.color} pad ({px:.0f},{py:.0f})')

        elif self.state == 'delivering':
            self.enable_wander(False)
            self.gripper('close')
            self.hold_box()
            # Drive to the exact on-pad slot we'll plant the box on
            if self.drop_xy is None and self.color:
                self.drop_xy = self.pad_drop_xy(self.color)
            dx, dy = self.drop_xy if self.drop_xy else self.pads[self.color]
            d = self.go_to(dx, dy, 0.80)
            if d < self.pad_ok:
                self.stop()
                self.drop_t = 0
                self.drop_turn = 0
                self.state = 'dropping'
                self.get_logger().info(
                    f'At pad slot ({dx:.1f},{dy:.1f}) — dropping'
                )

        elif self.state == 'dropping':
            # Plant box ON the pad under the claws → open → reverse clear → turn → wander
            self.enable_wander(False)
            self.holding = False
            self.drop_t += 1

            if self.drop_xy is None and self.color:
                self.drop_xy = self.pad_drop_xy(self.color)

            if self.drop_t == 1 and self.target and self.drop_xy:
                self.drop_counts[self.color] += 1
                self.set_box_pose(
                    self.target, self.drop_xy[0], self.drop_xy[1], self.box_half
                )
                self.get_logger().info(
                    f'Placed {self.color} on pad at ({self.drop_xy[0]:.1f},{self.drop_xy[1]:.1f}) '
                    f'slot #{self.drop_counts[self.color]}'
                )

            self.gripper('open')

            if self.drop_t <= 18:
                # Hold planted pose while claws open so physics doesn't fling it
                self.stop()
                if self.target and self.drop_xy:
                    self.set_box_pose(
                        self.target, self.drop_xy[0], self.drop_xy[1], self.box_half
                    )
            elif self.drop_t <= 48:
                # Reverse; keep pinning box for a bit so forks don't kick it off the pad
                self.drive(-0.55, 0.0)
                if self.drop_t <= 28 and self.target and self.drop_xy:
                    self.set_box_pose(
                        self.target, self.drop_xy[0], self.drop_xy[1], self.box_half
                    )
            elif self.drop_t <= 78:
                if self.drop_turn == 0:
                    self.drop_turn = self.open_side()
                self.drive(0.0, self.drop_turn * 0.65)
            else:
                dropped = self.color
                self.color = None
                self.target = None
                self.drop_xy = None
                self.last_goal_d = None
                self.avoid_t = 0
                self.turn_sign = 0
                self.drop_turn = 0
                self.state = 'searching'
                self.enable_wander(True)
                self.get_logger().info(f'Dropped {dropped} — wandering for next')


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.holding = False
            node.enable_wander(False)
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
