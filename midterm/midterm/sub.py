import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from frontier_exploration_ros2.srv import ControlExploration
from action_msgs.msg import GoalStatus
from dataclasses import dataclass
from enum import Enum

DEPTH = 10

Status = Enum("Status", ["PENDING", "COMPLETE"])


@dataclass
class Goal:
    message: PointStamped | None
    status: Status


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goals: list[Goal] = [
            Goal(None, Status.PENDING),
            Goal(None, Status.PENDING),
            Goal(None, Status.PENDING),
        ]
        self.goalHandle = None
        self.create_subscription(PointStamped, "/goal1", self.goal1Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal2", self.goal2Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal3", self.goal3Callback, DEPTH)
        self.exploration_client = self.create_client(
            ControlExploration,
            "/control_exploration",
        )
        while not self.exploration_client.wait_for_service(timeout_sec=1.0):
            print("Waiting on exploration client")
        self.explore(ControlExploration.Request.ACTION_START)

    def explore(self, action, on_idle=None):
        request = ControlExploration.Request()
        request.action = action
        future = self.exploration_client.call_async(request)

        def exploreCallback(future):
            response = future.result()
            if not response.accepted:
                raise RuntimeError(response.message)

            if on_idle is not None:
                if response.state == ControlExploration.Request.STATE_IDLE:
                    on_idle()
                else:
                    self.explore(action, on_idle)

        future.add_done_callback(exploreCallback)

    def goal1Callback(self, msg: PointStamped):
        self.updateGoals(msg, 0)

    def goal2Callback(self, msg: PointStamped):
        self.updateGoals(msg, 1)

    def goal3Callback(self, msg: PointStamped):
        self.updateGoals(msg, 2)

    def updateGoals(self, msg: PointStamped, goalNum: int):
        if self.goals[goalNum].status == Status.COMPLETE:
            return print("Goal already completed")
        self.goals[goalNum].message = msg
        if goalNum == 1 and self.goals[0].status == Status.PENDING:
            return print("Goal 2 is queued")
        if self.goalHandle is None:
            self.explore(
                ControlExploration.Request.ACTION_STOP,
                on_idle=lambda: self.seekGoal(goalNum),
            )
        elif goalNum == 2:
            raise NotImplementedError("Goal 3 logic not implemented")

    def seekGoal(self, goalNum: int):
        if not self.nav_client.server_is_ready():
            raise RuntimeError("Nav server is not ready")
        goal = self.goals[goalNum]
        if goal.message == None:
            raise RuntimeError("No message for goal")
        future = self.nav_client.send_goal_async(createGoal(goal.message))

        def accepted(future):
            self.goalHandle = future.result()
            if self.goalHandle.accepted:
                print(f"Goal {goalNum + 1} - in progress")
                result = self.goalHandle.get_result_async()
                result.add_done_callback(
                    lambda future: self.checkGoalComplete(future, goalNum)
                )

        future.add_done_callback(accepted)

    def checkGoalComplete(self, future, goalNum: int):
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            print(f"Goal {goalNum + 1} - failure")
            self.goalHandle = None
            self.explore(ControlExploration.Request.ACTION_START)
            return
        print(f"Goal {goalNum + 1} - success")
        self.goals[goalNum].status = Status.COMPLETE
        self.goalHandle = None
        nextGoalNum = self.getNextGoalNum()
        if nextGoalNum is not None:
            self.seekGoal(nextGoalNum)
        else:
            self.explore(ControlExploration.Request.ACTION_START)

    def getNextGoalNum(self) -> int | None:
        if self.goals[0].message is not None and self.goals[0].status == Status.PENDING:
            return 0
        elif (
            self.goals[0].status == Status.COMPLETE
            and self.goals[1].message is not None
            and self.goals[1].status == Status.PENDING
        ):
            return 1
        return None

def createGoal(msg: PointStamped) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose.header = msg.header
    goal.pose.pose.position.x = msg.point.x
    goal.pose.pose.position.y = msg.point.y
    goal.pose.pose.orientation.w = 1.0
    return goal

def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
