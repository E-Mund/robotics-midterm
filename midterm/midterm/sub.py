import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, PointStamped
from nav2_msgs.action import NavigateToPose
from frontier_exploration_ros2.srv import ControlExploration
from project_status.msg import Status as StatusMsg
from action_msgs.msg import GoalStatus
from dataclasses import dataclass
from enum import Enum

DEPTH = 10

Status = Enum("Status", ["PENDING", "ONGOING", "COMPLETE"])


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

        self.humanFound = False
        self.humanLocation: Point | None = None
        self.create_subscription(
            Point,
            "human_location",
            self.humanCallback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.status_pub = self.create_publisher(StatusMsg, "/status", DEPTH)
        self.create_timer(1.0, self.publishStatus)

        self.exploration_client = self.create_client(
            ControlExploration,
            "/control_exploration",
        )
        while not self.exploration_client.wait_for_service(timeout_sec=1.0):
            print("Waiting on exploration client")
        self.explore(ControlExploration.Request.ACTION_START)

    def explore(self, action, on_idle=None):
        if self.humanFound and action == ControlExploration.Request.ACTION_START:
            return
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

    def humanCallback(self, msg: Point):
        if self.humanFound:
            return
        self.humanFound = True
        self.humanLocation = msg
        print(f"Human found at ({msg.x:.2f}, {msg.y:.2f})")
        self.publishStatus()
        self.explore(ControlExploration.Request.ACTION_STOP)
        if self.goalHandle is not None:
            self.cancelGoal()

    def publishStatus(self):
        status = StatusMsg()
        status.goal1_received = self.goals[0].message is not None
        status.goal1_achieved = self.goals[0].status == Status.COMPLETE
        status.goal2_received = self.goals[1].message is not None
        status.goal2_achieved = self.goals[1].status == Status.COMPLETE
        status.emergency = self.goals[2].status == Status.ONGOING
        status.human_found = self.humanFound
        if self.humanLocation is not None:
            status.human_location = self.humanLocation
        self.status_pub.publish(status)

    def goal1Callback(self, msg: PointStamped):
        self.updateGoals(msg, 0)

    def goal2Callback(self, msg: PointStamped):
        self.updateGoals(msg, 1)

    def goal3Callback(self, msg: PointStamped):
        self.updateGoals(msg, 2)

    def isPending(self, goalNum: int) -> bool:
        return (
            self.goals[goalNum].status == Status.PENDING
            and self.goals[goalNum].message is not None
        )

    def cancelGoal(self):
        if self.goalHandle is None:
            return
        future = self.goalHandle.cancel_goal_async()
        return future.add_done_callback(checkCancellation)

    def updateGoals(self, msg: PointStamped, goalNum: int):
        if self.humanFound:
            return
        if self.goals[goalNum].status == Status.COMPLETE:
            return print("Goal already completed")
        if self.goals[goalNum].status == Status.ONGOING:
            return print("Goal currently ongoing")

        self.goals[goalNum].message = msg
        if goalNum == 1 and self.goals[0].status != Status.COMPLETE:
            return print("Goal 1 pending, goal 2 is queued")

        if any(goal.status == Status.ONGOING for goal in self.goals):
            if goalNum == 2 and self.goalHandle is not None:
                return self.cancelGoal()
            return print("Goal currently ongoing")

        self.goals[goalNum].status = Status.ONGOING
        self.explore(
            ControlExploration.Request.ACTION_STOP,
            on_idle=lambda: self.seekGoal(goalNum),
        )

    def seekGoal(self, goalNum: int):
        if self.humanFound:
            self.goals[goalNum].status = Status.PENDING
            return
        if not self.nav_client.server_is_ready():
            raise RuntimeError("Nav server is not ready")
        if self.goals[goalNum].message is None:
            raise RuntimeError("No message for goal")
        self.goals[goalNum].status = Status.ONGOING
        future = self.nav_client.send_goal_async(
            createGoal(self.goals[goalNum].message)
        )

        def accepted(future):
            self.goalHandle = future.result()
            if self.goalHandle.accepted:
                print(f"Goal {goalNum + 1} - in progress")
                result = self.goalHandle.get_result_async()
                result.add_done_callback(
                    lambda future: self.checkGoalComplete(future, goalNum)
                )
                if self.humanFound:
                    self.cancelGoal()
                    return
                if goalNum != 2 and self.isPending(2):
                    return self.cancelGoal()
            else:
                self.goalHandle = None
                self.goals[goalNum].status = Status.PENDING
                if goalNum != 2 and self.isPending(2):
                    return self.seekGoal(2)
                return self.explore(ControlExploration.Request.ACTION_START)

        future.add_done_callback(accepted)

    def checkGoalComplete(self, future, goalNum: int):
        self.goalHandle = None

        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.goals[goalNum].status = Status.COMPLETE
        else:
            self.goals[goalNum].status = Status.PENDING

        if self.humanFound:
            return

        if goalNum != 2 and self.isPending(2):
            return self.seekGoal(2)

        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            nextGoalNum = self.getNextGoalNum()
            if nextGoalNum is not None:
                return self.seekGoal(nextGoalNum)

        return self.explore(ControlExploration.Request.ACTION_START)

    def getNextGoalNum(self) -> int | None:
        if self.isPending(0):
            return 0
        if self.goals[0].status == Status.COMPLETE and self.isPending(1):
            return 1
        return None

def checkCancellation(future):
    response = future.result()
    if response.return_code == response.ERROR_GOAL_TERMINATED:
        return
    if not response.goals_canceling:
        raise RuntimeError("Nav2 did not accept goal cancellation")


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
