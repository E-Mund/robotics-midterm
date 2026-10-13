#!/usr/bin/env python3
"""
Mission manager: sequences /<ns>/goal1 -> /<ns>/goal2 using the Nav2 NavigateToPose action.

Required behaviours
  * no goals                -> robot stays where it is (nothing is ever sent to Nav2)
  * only goal1              -> go to goal1, then wait for goal2
  * only goal2              -> store it, do NOT move until goal1 arrives
  * goal1 and goal2         -> goal1 first, then goal2

Extra robustness (useful for the "one more challenge"):
  * A NEW goal1 at any time pre-empts the current motion and restarts the sequence at goal1.
  * A NEW goal2 while driving to goal2 re-targets; while driving to goal1 it is queued.
  * Re-publishing an identical point (e.g. `ros2 topic pub -r 1 ...`) is ignored, so a
    finished mission is not restarted by a latched/repeating publisher.
  * Nav2 failures are retried (costmaps are cleared before each retry).
  * Current mission state is published on <ns>/mission_state (std_msgs/String).

Coordinate frames
  Goals are geometry_msgs/Point in the Gazebo world frame. The Gazebo diff-drive plugin used by
  the TurtleBot3 model reports odometry in world coordinates (odometry_source = world), so by default
  `goal_frame` = odom and the world->odom offset is zero. If odom starts at the spawn pose instead,
  set world_origin_x / _y / _yaw to the spawn pose. Goals are transformed into the Nav2 global frame
  (map, published by slam_toolbox) via TF just before they are sent.

Sources: Nav2 NavigateToPose action API (https://docs.nav2.org), rclpy action client tutorial
(https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).
"""
import math
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class State(Enum):
    IDLE = 'IDLE'              # no goal1 yet -> stay still
    NAV_GOAL1 = 'NAV_GOAL1'
    WAIT_GOAL2 = 'WAIT_GOAL2'  # at goal1, waiting for goal2
    NAV_GOAL2 = 'NAV_GOAL2'
    DONE = 'DONE'              # both reached, waiting for a new goal1
    FAILED = 'FAILED'          # retries exhausted, waiting for new goals


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        defaults = {
            'goal1_topic': 'goal1',            # relative -> /<namespace>/goal1
            'goal2_topic': 'goal2',
            'odom_topic': 'odom',
            'state_topic': 'mission_state',
            'navigate_action': 'navigate_to_pose',
            'goal_frame': 'odom',
            'global_frame': 'map',
            'robot_frame': 'base_footprint',
            'world_origin_x': 0.0,
            'world_origin_y': 0.0,
            'world_origin_yaw': 0.0,
            'same_goal_tolerance': 0.05,
            'max_retries': 10,                 # < 0 means retry forever
            'retry_delay': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.goal_frame = p('goal_frame')
        self.global_frame = p('global_frame')
        self.robot_frame = p('robot_frame')
        self.origin = (p('world_origin_x'), p('world_origin_y'), p('world_origin_yaw'))
        self.same_tol = p('same_goal_tolerance')
        self.max_retries = p('max_retries')
        self.retry_delay = p('retry_delay')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, p('navigate_action'))
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap'),
        ]

        self.create_subscription(Point, p('goal1_topic'), self._on_goal1, 10)
        self.create_subscription(Point, p('goal2_topic'), self._on_goal2, 10)
        self.create_subscription(Odometry, p('odom_topic'), self._on_odom, 10)
        self.state_pub = self.create_publisher(String, p('state_topic'), 10)

        # mission data
        self.goal1 = None
        self.goal2 = None
        self.last_goal1 = None
        self.last_goal2 = None
        self.goal1_new = False   # unprocessed goal1
        self.goal2_new = False   # goal2 not yet consumed by a mission
        self.state = State.IDLE
        self.active = None       # 'goal1' | 'goal2' | None
        self.goal_handle = None
        self.token = 0           # invalidates callbacks of stale Nav2 goals
        self.retries = 0
        self.retry_at = None
        self.odom = None
        self.dist_remaining = float('nan')

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info(
            f'Mission manager ready. Listening on {self.resolve_topic_name(p("goal1_topic"))} and '
            f'{self.resolve_topic_name(p("goal2_topic"))}. Robot holds position until goal1 arrives.')

    # ------------------------------------------------------------------ inputs
    def _same(self, a, b):
        return b is not None and math.hypot(a.x - b.x, a.y - b.y) < self.same_tol

    def _on_goal1(self, msg):
        if self._same(msg, self.last_goal1):
            return
        self.last_goal1 = msg
        self.goal1 = msg
        self.goal1_new = True
        self.get_logger().info(f'goal1 received: ({msg.x:.2f}, {msg.y:.2f})')

    def _on_goal2(self, msg):
        if self._same(msg, self.last_goal2):
            return
        self.last_goal2 = msg
        self.goal2 = msg
        self.goal2_new = True
        extra = '' if self.state != State.IDLE or self.goal1 else ' (waiting for goal1 before moving)'
        self.get_logger().info(f'goal2 received: ({msg.x:.2f}, {msg.y:.2f}){extra}')

    def _on_odom(self, msg):
        self.odom = msg

    # ------------------------------------------------------------ state machine
    def _tick(self):
        now = self.get_clock().now()

        if self.goal1_new:                       # new goal1 always (re)starts the sequence
            self.goal1_new = False
            self._start('goal1')
            return

        if self.goal2_new and (
                self.state in (State.WAIT_GOAL2, State.NAV_GOAL2) or
                (self.state == State.FAILED and self.active == 'goal2')):
            self._start('goal2')
            return

        if self.retry_at is not None and now >= self.retry_at:
            self.retry_at = None
            self._send(self.active)

    def _start(self, target):
        self._cancel_active()
        self.active = target
        self.retries = 0
        self.retry_at = None
        if target == 'goal1':
            self.state = State.NAV_GOAL1
        else:
            self.state = State.NAV_GOAL2
            self.goal2_new = False               # goal2 consumed by this mission
        self._send(target)

    def _cancel_active(self):
        self.token += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _send(self, target):
        if target is None:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().info('Waiting for Nav2 navigate_to_pose server...',
                                   throttle_duration_sec=5.0)
            self.retry_at = self.get_clock().now() + Duration(seconds=1.0)
            return
        point = self.goal1 if target == 'goal1' else self.goal2
        goal = NavigateToPose.Goal()
        goal.pose = self._make_goal_pose(point)
        self.token += 1
        tok = self.token
        self.dist_remaining = float('nan')
        self.get_logger().info(
            f'Navigating to {target}: world ({point.x:.2f}, {point.y:.2f}) -> '
            f'{goal.pose.header.frame_id} ({goal.pose.pose.position.x:.2f}, '
            f'{goal.pose.pose.position.y:.2f}) [attempt {self.retries + 1}]')
        fut = self.nav_client.send_goal_async(goal, feedback_callback=lambda fb, t=tok: self._on_feedback(fb, t))
        fut.add_done_callback(lambda f, t=tok: self._on_accepted(f, t))

    def _on_accepted(self, future, tok):
        handle = future.result()
        if tok != self.token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            self._on_failure()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f, t=tok: self._on_result(f, t))

    def _on_feedback(self, fb, tok):
        if tok == self.token:
            self.dist_remaining = fb.feedback.distance_remaining

    def _on_result(self, future, tok):
        if tok != self.token:
            return                                # result of a pre-empted goal
        self.goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_reached()
        else:
            self.get_logger().warn(f'Navigation to {self.active} ended with status {status}')
            self._on_failure()

    def _on_reached(self):
        if self.active == 'goal1':
            self.get_logger().info('Reached goal1.')
            if self.goal2_new:
                self._start('goal2')
            else:
                self.state = State.WAIT_GOAL2
                self.active = None
                self.get_logger().info('Holding position at goal1 until goal2 is received.')
        else:
            self.get_logger().info('Reached goal2. Mission complete; holding position.')
            self.state = State.DONE
            self.active = None

    def _on_failure(self):
        self.retries += 1
        if self.max_retries < 0 or self.retries <= self.max_retries:
            self._clear_costmaps()
            self.retry_at = self.get_clock().now() + Duration(seconds=self.retry_delay)
        else:
            self.get_logger().error(f'Giving up on {self.active} after {self.retries - 1} retries.')
            self.state = State.FAILED

    def _clear_costmaps(self):
        for client in self.clear_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    # ------------------------------------------------------------------ frames
    def _lookup_xy_yaw(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, Time(), timeout=Duration(seconds=0.2))
        t = tf.transform.translation
        return t.x, t.y, yaw_from_quat(tf.transform.rotation)

    def _make_goal_pose(self, point):
        # world -> goal_frame
        ox, oy, oyaw = self.origin
        dx, dy = point.x - ox, point.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        gx, gy = c * dx - s * dy, s * dx + c * dy

        frame, x, y = self.goal_frame, gx, gy
        if self.goal_frame != self.global_frame:
            try:
                tx, ty, tyaw = self._lookup_xy_yaw(self.global_frame, self.goal_frame)
                c, s = math.cos(tyaw), math.sin(tyaw)
                frame, x, y = self.global_frame, tx + c * gx - s * gy, ty + s * gx + c * gy
            except TransformException as e:
                self.get_logger().warn(f'{self.global_frame}<-{self.goal_frame} TF not available ({e}); '
                                       f'sending goal in {self.goal_frame}')

        # face along the straight line robot -> goal (Point carries no orientation;
        # the goal checker ignores yaw anyway)
        yaw = 0.0
        try:
            rx, ry, _ = self._lookup_xy_yaw(frame, self.robot_frame)
            if math.hypot(x - rx, y - ry) > 1e-3:
                yaw = math.atan2(y - ry, x - rx)
        except TransformException:
            pass

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = Time().to_msg()       # "latest available" transform
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ------------------------------------------------------------------ status
    def _publish_state(self):
        msg = String()
        pos = ''
        if self.odom is not None:
            pp = self.odom.pose.pose.position
            pos = f' odom=({pp.x:.2f},{pp.y:.2f})'
        dist = '' if math.isnan(self.dist_remaining) else f' remaining={self.dist_remaining:.2f}m'
        msg.data = f'{self.state.value}{pos}{dist}'
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
