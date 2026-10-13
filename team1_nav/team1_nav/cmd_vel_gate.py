#!/usr/bin/env python3
"""
cmd_vel safety gate: the ONLY node that publishes <ns>/cmd_vel.

  Nav2 (controller -> velocity_smoother, behaviours) --> <ns>/cmd_vel_gate_in --> [gate] --> <ns>/cmd_vel

Forward motion is vetoed (rotation and reversing are still allowed so Nav2 recoveries can work) if
an obstacle is inside a box in front of the robot, using:
  * LIDAR (<ns>/scan), and
  * camera floor-obstacle points (<ns>/camera_obstacles/points) kept in a short odometry-based
    memory. The camera cannot see the floor closer than ~0.28 m (it is mounted ~9 cm high with no
    tilt), so remembering recently seen points covers that blind zone as the robot approaches.

If commands stop arriving (Nav2 idle / goal finished) a single zero Twist is sent so the robot stays
in place. Nothing is published before the first command, so the robot never moves on its own.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CmdVelGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_gate')
        defaults = {
            'input_topic': 'cmd_vel_gate_in',
            'output_topic': 'cmd_vel',
            'camera_points_topic': 'camera_obstacles/points',
            'scan_topic': 'scan',
            'odom_topic': 'odom',
            'base_frame': 'base_footprint',
            'robot_front': 0.08,      # m, front edge of waffle_pi footprint from base_footprint
            'half_width': 0.17,       # m, half body width + margin
            'stop_distance': 0.10,    # m, minimum free space required ahead
            'lookahead_time': 0.5,    # s, box grows with speed
            'memory_time': 4.0,       # s, camera point memory
            'cmd_timeout': 0.5,       # s
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {n: self.get_parameter(n).value for n in defaults}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, self.p['output_topic'], 10)
        self.create_subscription(Twist, self.p['input_topic'], self._on_cmd, 10)
        self.create_subscription(PointCloud2, self.p['camera_points_topic'], self._on_cam, 10)
        self.create_subscription(LaserScan, self.p['scan_topic'], self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 10)
        self.create_timer(0.1, self._watchdog)

        self.pose = None                      # (x, y, yaw) in odom
        self.memory = []                      # list of (t, Nx2 array in odom frame)
        self.scan_pts = np.zeros((0, 2))
        self.scan_tf = None
        self.last_cmd = None
        self.stopped = True

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _on_cam(self, msg):
        if self.pose is None:
            return
        pts = np.array([[q[0], q[1]] for q in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)],
                       dtype=float).reshape(-1, 2)
        if len(pts) == 0:
            return
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        world = np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])
        self.memory.append((time.monotonic(), world))

    def _on_scan(self, msg):
        if self.scan_tf is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.p['base_frame'], msg.header.frame_id, Time())
                t = tf.transform.translation
                self.scan_tf = (t.x, t.y, yaw_from_quat(tf.transform.rotation))
            except TransformException:
                return
        r = np.asarray(msg.ranges, dtype=float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        r, a = r[ok], a[ok]
        tx, ty, tth = self.scan_tf
        self.scan_pts = np.column_stack([tx + r * np.cos(a + tth), ty + r * np.sin(a + tth)])

    def _blocked(self, v):
        reach = self.p['robot_front'] + self.p['stop_distance'] + v * self.p['lookahead_time']
        hw = self.p['half_width']
        clouds = [self.scan_pts]
        now = time.monotonic()
        self.memory = [(t, pts) for t, pts in self.memory if now - t < self.p['memory_time']]
        if self.pose is not None and self.memory:
            x, y, th = self.pose
            c, s = math.cos(th), math.sin(th)
            world = np.vstack([pts for _, pts in self.memory])
            dx, dy = world[:, 0] - x, world[:, 1] - y
            clouds.append(np.column_stack([c * dx + s * dy, -s * dx + c * dy]))
        for pts in clouds:
            if len(pts) and np.any((pts[:, 0] > 0.0) & (pts[:, 0] < reach) & (np.abs(pts[:, 1]) < hw)):
                return True
        return False

    def _on_cmd(self, msg):
        out = Twist()
        out.linear.x, out.angular.z = msg.linear.x, msg.angular.z
        if out.linear.x > 0.0 and self._blocked(out.linear.x):
            out.linear.x = 0.0
            self.get_logger().warn('Obstacle directly ahead: forward motion vetoed', throttle_duration_sec=2.0)
        self.pub.publish(out)
        self.last_cmd = time.monotonic()
        self.stopped = False

    def _watchdog(self):
        if self.last_cmd is not None and not self.stopped and time.monotonic() - self.last_cmd > self.p['cmd_timeout']:
            self.pub.publish(Twist())
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGate()
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
