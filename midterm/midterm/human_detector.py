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

INPUT_SIZE = 640
CONF_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45


class HumanDetector(Node):
    def __init__(self):
        super().__init__("human_detector")
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.strides = [8, 16, 32]

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

        self.grids = []
        self.expanded_strides = []
        sizes = [INPUT_SIZE // stride for stride in self.strides]
        for size, stride in zip(sizes, self.strides):
            xv, yv = np.meshgrid(np.arange(size), np.arange(size))
            grid = np.stack((xv, yv), 2).reshape(-1, 2)
            self.grids.append(grid)
            self.expanded_strides.append(np.full((size * size, 1), stride))

        self.grids = np.concatenate(self.grids)
        self.expanded_strides = np.concatenate(self.expanded_strides)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        people = self.detect_people(img)

        for (x, y, w, h), score in people:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                img,
                f"person {score:.2f}",
                (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

        cv2.imshow("camera", img)
        cv2.waitKey(1)

    def info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def detect_people(self, img):
        blob, ratio = self.letterbox(img)

        self.net.setInput(blob)
        output = self.net.forward()[0]

        dets = output
        dets[:, :2] = (dets[:, :2] + self.grids) * self.expanded_strides
        dets[:, 2:4] = np.exp(dets[:, 2:4]) * self.expanded_strides

        scores = dets[:, 4] * dets[:, 5]

        boxes = dets[:, :4]
        boxes_xywh = np.ones_like(boxes)
        boxes_xywh[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
        boxes_xywh[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
        boxes_xywh[:, 2] = boxes[:, 2]
        boxes_xywh[:, 3] = boxes[:, 3]

        keep = cv2.dnn.NMSBoxes(
            boxes_xywh.tolist(),
            scores.tolist(),
            CONF_THRESHOLD,
            NMS_THRESHOLD,
        )
        if len(keep) == 0:
            return []

        keep = np.array(keep).flatten()
        kept = self.unletterbox(boxes_xywh[keep], ratio).astype(int)
        return [
            (tuple(int(v) for v in box), float(score))
            for box, score in zip(kept, scores[keep])
        ]

    def letterbox(self, img):
        h, w = img.shape[:2]
        ratio = min(INPUT_SIZE / h, INPUT_SIZE / w)
        w_new = int(w * ratio)
        h_new = int(h * ratio)

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (w_new, h_new))
        padded = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.float32)
        padded[:h_new, :w_new] = resized

        blob = np.transpose(padded, (2, 0, 1))[np.newaxis]
        return blob, ratio

    def unletterbox(self, bbox, letterbox_scale):
        return bbox / letterbox_scale


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
