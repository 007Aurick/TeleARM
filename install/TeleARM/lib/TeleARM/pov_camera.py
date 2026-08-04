#!/usr/bin/env python3
"""First-person /camera/image_raw for Foxglove when Gazebo RGB is dead.

Uses TF (odom→body) + fixed warehouse box poses — no Gazebo service calls
(those hang under load on this setup).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener


# From worlds/storage_warehouse.world (x, y, z) + color
BOXES = [
    (-3.0, 4.5, 0.28, 'orange'),
    (10.0, 2.0, 0.28, 'orange'),
    (-12.0, -7.0, 0.28, 'orange'),
    (3.0, -7.0, 0.28, 'blue'),
    (-10.0, -2.0, 0.28, 'blue'),
    (0.0, 7.0, 0.28, 'green'),
    (8.0, 8.0, 0.28, 'green'),
    (4.0, 5.5, 0.28, 'green'),
    (12.0, -5.0, 0.28, 'red'),
    (-8.0, 8.0, 0.28, 'red'),
]

BOX_RGB = {
    'orange': (255, 128, 13),
    'blue': (26, 102, 255),
    'green': (38, 217, 64),
    'red': (255, 38, 38),
}


class PovCamera(Node):
    def __init__(self):
        super().__init__('pov_camera')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.width = 320
        self.height = 240
        self.fov = 1.3962634
        self.cam_height = 0.45
        self.frame_parent = 'odom'
        self.frame_robot = 'body'

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('POV publishing /camera/image_raw from TF + world boxes')

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_parent, self.frame_robot, rclpy.time.Time()
            )
        except Exception:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        return x, y, yaw

    def project(self, x, y, z, fx, fy):
        if x <= 0.15:
            return None
        u = self.width * 0.5 - (y / x) * fx
        v = self.height * 0.5 - (z / x) * fy
        return u, v

    def tick(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[: self.height // 2, :] = (190, 200, 210)
        img[self.height // 2 :, :] = (90, 90, 90)

        fx = (self.width * 0.5) / math.tan(self.fov * 0.5)
        fy = fx

        pose = self.robot_pose()
        if pose is not None:
            rx, ry, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            drawn = []
            for bx, by, bz, color in BOXES:
                dx = bx - rx
                dy = by - ry
                local_x = c * dx + s * dy
                local_y = -s * dx + c * dy
                local_z = bz - self.cam_height
                if 0.5 < local_x < 20.0:
                    drawn.append((local_x, local_y, local_z, color))
            drawn.sort(key=lambda t: -t[0])
            for lx, ly, lz, color in drawn:
                size = max(10, int(90.0 / lx))
                pt = self.project(lx, ly, lz, fx, fy)
                if pt is None:
                    continue
                u_mid, v_mid = int(pt[0]), int(pt[1])
                u0 = max(0, u_mid - size)
                u1 = min(self.width - 1, u_mid + size)
                v0 = max(0, v_mid - size)
                v1 = min(self.height - 1, v_mid + size)
                if u1 > u0 and v1 > v0:
                    img[v0:v1, u0:u1] = BOX_RGB[color]

        # Always draw crosshair so Foxglove shows *something*
        cx, cy = self.width // 2, self.height // 2
        img[cy - 8:cy + 9, cx] = (0, 255, 0)
        img[cy, cx - 8:cx + 9] = (0, 255, 0)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PovCamera()
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
