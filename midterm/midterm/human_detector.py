"""
ROS node: find the human in the camera feed.

Subscribes (relative name, resolves under the team namespace):
    camera/image_raw     sensor_msgs/Image

Person detection: YOLOX from OpenCV Zoo (Apache-2.0), run with cv2.dnn.
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import numpy as np


class HumanDetector(Node):
    def __init__(self):
        super().__init__("human_detector")
        self.bridge = CvBridge()
        self.K = None
        self.D = None

        model_path = self.declare_parameter("model_path", "").value
        self.net = cv2.dnn.readNet(model_path)

        self.create_subscription(
            Image, "camera/image_raw", self.image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            "camera/camera_info",
            self.info_callback,
            qos_profile_sensor_data,
        )

        self.marker_pub = self.create_publisher(Marker, "person_marker", 10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        people = self.detect_people(img)

        cv2.imshow("camera", img)
        cv2.waitKey(1)

    def info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def detect_people(self, img):
        # TODO 1: letterbox img to 640x640 (RGB, gray padding, no /255)
        blob, ratio = self.letterbox(img)
        # TODO 2: self.net.setInput(blob); run forward -> (8400, 85)
        # TODO 3: decode boxes with the grid + strides (see yolox.py)
        # TODO 4: keep person scores > 0.5, NMSBoxes, scale back to img
        return []

    def letterbox(self, img):
        (
            h,
            w,
        ) = img.shape[:2]
        ratio = min(640 / h, 640 / w)
        w_new = int(w * ratio)
        h_new = int(h * ratio)

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (w_new, h_new))
        padded = np.full((640, 640, 3), 144, dtype=np.float32)
        padded[:h_new, :w_new]

        blob = np.transpose(padded, (2, 0, 1))[np.newaxis]
        return blob, ratio


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
