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
        if not any((goal[1] is not None) for goal in self.goals[:priority]):
            self.seekGoal(msg, priority)

    def seekGoal(self, msg, priority: int):
        if not self.nav_client.server_is_ready():
            print("Nav server is not ready")
            exit()
        goal = NavigateToPose.Goal()
        goal.pose.header = msg.header
        goal.pose.pose.position.x = msg.point.x
        goal.pose.pose.position.y = msg.point.y
        goal.pose.pose.orientation.w = 1.0
        future = self.nav_client.send_goal_async(goal)

        def accepted(future):
            self.goalHandle = future.result()
            if self.goalHandle.accepted:
                print(f"Goal {priority} - in progress")
                result = self.goalHandle.get_result_async()
                result.add_done_callback(
                    lambda future: self.checkGoalFinished(future, priority)
                )

        future.add_done_callback(accepted)

    def checkGoalFinished(self, future, priority: int):
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            print(f"Goal {priority} - failure")
            return
        print(f"Goal {priority} - sucess")
        self.goals[priority][1] = None
        nextGoal = self.getGoal()
        if nextGoal is not None:
            self.seekGoal(nextGoal[1], self.goals.index(nextGoal))

    def getGoal(self):
        return next((goal for goal in self.goals if goal[1] is not None), None)


def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
