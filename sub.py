import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")

        for topic in ["/goal1", "/goal2", "/goal3"]:
            self.create_subscription(PoseStamped, topic, self.callback, 10)

    def callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y


def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
