#!/usr/bin/env python3
"""Optional debug publisher — mission now does its own detection.

Kept so Foxglove/RViz still show /detected_box_color if this node is run.
Mission also publishes the same topic; prefer only one running.
"""

import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32, String


COLORS = ('orange', 'blue', 'green', 'red')


class SimBoxDetector(Node):
    def __init__(self):
        super().__init__('sim_box_detector')
        self.color_pub = self.create_publisher(String, '/detected_box_color', 10)
        self.offset_pub = self.create_publisher(Float32, '/box_offset', 10)
        self.models = None
        self.create_subscription(ModelStates, '/gazebo/model_states', self.models_cb, 10)
        self.detect_range = 7.0
        self.fov_half_angle = 0.9
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'sim_box_detector (debug). Mission has its own detector — stop this if both run.'
        )

    def models_cb(self, msg):
        self.models = msg

    @staticmethod
    def yaw_from_quat(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def tick(self):
        if self.models is None:
            self.publish('none', 0.0)
            return
        try:
            ri = self.models.name.index('robot')
        except ValueError:
            self.publish('none', 0.0)
            return
        rp = self.models.pose[ri]
        rx, ry = rp.position.x, rp.position.y
        yaw = self.yaw_from_quat(rp.orientation)
        best = None
        for name, pose in zip(self.models.name, self.models.pose):
            color = None
            for c in COLORS:
                if name.startswith(f'box_{c}_'):
                    color = c
                    break
            if color is None:
                continue
            dx = pose.position.x - rx
            dy = pose.position.y - ry
            dist = math.hypot(dx, dy)
            if dist < 0.2 or dist > self.detect_range:
                continue
            bearing = math.atan2(dy, dx) - yaw
            while bearing > math.pi:
                bearing -= 2.0 * math.pi
            while bearing < -math.pi:
                bearing += 2.0 * math.pi
            if abs(bearing) > self.fov_half_angle:
                continue
            if best is None or dist < best[0]:
                best = (dist, color, bearing)
        if best is None:
            self.publish('none', 0.0)
            return
        _, color, bearing = best
        self.publish(color, max(-1.0, min(1.0, bearing / self.fov_half_angle)))

    def publish(self, color, offset):
        c = String()
        c.data = color
        self.color_pub.publish(c)
        o = Float32()
        o.data = float(offset)
        self.offset_pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = SimBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
