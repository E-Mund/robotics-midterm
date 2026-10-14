import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

DEPTH = 10


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goals = ([1, None], [2, None], [3, None])
        self.goalHandle = None
        self.create_subscription(PointStamped, "/goal1", self.goal1Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal2", self.goal2Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal3", self.goal3Callback, DEPTH)

    def goal1Callback(self, msg):
        self.updateGoals(msg, 1)

    def goal2Callback(self, msg):
        self.updateGoals(msg, 2)

    def goal3Callback(self, msg):
        self.updateGoals(msg, 0)

    def updateGoals(self, msg, priority: int):
        self.goals[priority][1] = msg
        self.seekGoal(msg, priority)
        # if not any((goal[1] is not None) for goal in self.goals[priority:]):
        #     self.seekGoal(msg, priority)

    def seekGoal(self, msg, priority: int):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 is not ready")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header = msg.header
        goal.pose.pose.position.x = msg.point.x
        goal.pose.pose.position.y = msg.point.y
        goal.pose.pose.orientation.w = 1.0
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.doneCallback(f, priority))

    # def getGoal(self) -> int:
    #     return next((goal[1] for goal in self.goals if goal[1] is not None), -1)

    def doneCallback(self, future, priority: int):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Task {priority} was rejected by Nav2")
            return


def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
