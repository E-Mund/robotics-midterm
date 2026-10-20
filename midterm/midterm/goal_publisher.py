"""
Test node: publish goals to /goal1, /goal2, /goal3 to exercise sub.py.

Usage:
    ros2 run midterm goal_publisher --ros-args -p scenario:=emergency
    ros2 run midterm goal_publisher --ros-args -p scenario:=both \
        -p goal1:="[1.0, 0.5]" -p goal2:="[-1.0, 2.0]"

Scenarios (from the assignment spec), as (delay seconds, goal):
    goal1_only        goal1 at t=0
    goal2_only        goal2 at t=0 (robot should NOT go yet)
    both              goal1 and goal2 at t=0
    goal2_then_goal1  goal2 at t=0, goal1 at t=delay
    emergency         goal1 and goal2 at t=0, goal3 at t=delay
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose

SCENARIOS = {
    "goal1_only": [(0.0, "goal1")],
    "goal2_only": [(0.0, "goal2")],
    "both": [(0.0, "goal1"), (0.0, "goal2")],
    "goal2_then_goal1": [(0.0, "goal2"), (1.0, "goal1")],
    "emergency": [(0.0, "goal1"), (0.0, "goal2"), (1.0, "goal3")],
}


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.scenario = self.declare_parameter("scenario", "both").value
        self.delay = self.declare_parameter("delay", 10.0).value
        self.coords = {
            name: self.declare_parameter(name, default).value
            for name, default in (
                ("goal1", [-0.6, 1.2]),
                ("goal2", [2.7, 0.4]),
                ("goal3", [-0.4, 0.5]),
            )
        }

        if self.scenario not in SCENARIOS:
            raise ValueError(
                f"Unknown scenario '{self.scenario}', pick one of {list(SCENARIOS)}"
            )

        self.pubs = {
            name: self.create_publisher(PointStamped, f"/{name}", 10)
            for name in self.coords
        }
        self.events = [
            (delay * self.delay, name) for delay, name in SCENARIOS[self.scenario]
        ]
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.ready_since = None
        self.start = None
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Scenario '{self.scenario}', waiting for sub.py and Nav2..."
        )

    def tick(self):
        if self.start is None:
            ready = (
                self.pubs["goal1"].get_subscription_count() > 0
                and self.nav_client.server_is_ready()
            )
            now = self.get_clock().now()
            if not ready:
                self.ready_since = None
                return
            if self.ready_since is None:
                self.ready_since = now
            if (now - self.ready_since).nanoseconds < 2e9:
                return
            self.start = now

        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        while self.events and self.events[0][0] <= elapsed:
            _, name = self.events.pop(0)
            self.publish(name)

        if not self.events:
            self.get_logger().info("Scenario done")
            self.timer.cancel()

    def publish(self, name):
        x, y = self.coords[name]
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = float(x), float(y)
        self.pubs[name].publish(msg)
        self.get_logger().info(f"Published /{name}: ({x:.2f}, {y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
