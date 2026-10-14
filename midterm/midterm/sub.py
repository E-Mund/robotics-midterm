import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

PERIOD = 1.0
DEPTH = 10


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.timer = self.create_timer(PERIOD, self.timerCallback)
        self.goals = []
        self.create_subscription(PointStamped, "/goal1", self.goal1Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal2", self.goal2Callback, DEPTH)
        self.create_subscription(PointStamped, "/goal3", self.goal3Callback, DEPTH)

    def timerCallback(self):
        pass

    def goal1Callback(self, msg):
        self.goalCallback(1, msg)

    def goal2Callback(self, msg):
        self.goalCallback(2, msg)

    def goal3Callback(self, msg):
        self.goalCallback(3, msg)

    def goalCallback(self, priority, msg):
        print(msg)
        self.timer.cancel()
        self.goals.append(priority)
        self.goals.remove(priority)


def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
