import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")

        for topic in ["/goal1", "/goal2", "/goal3"]:
            self.create_subscription(Float32, topic, self.callback, 10)

    def callback(self, msg):
        print(msg.data)


def main(args=None):
    rclpy.init()
    node = Subscriber()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
