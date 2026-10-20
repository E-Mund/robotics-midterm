"""
ROS node: find the human in the camera feed.

Subscribes (relative name, resolves under the team namespace):
    camera/image_raw     sensor_msgs/Image
    camera/camera_info   sensor_msgs/CameraInfo

Publishes:
    human_location       geometry_msgs/Point   once, map frame (transient local)
    person_marker        visualization_msgs/Marker

Person detection: YOLOX from OpenCV Zoo (Apache-2.0), run with cv2.dnn.
"""

import os

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import numpy as np
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point, PointStamped
from tf2_geometry_msgs import do_transform_point

INPUT_SIZE = 640
CONF_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45
CONFIRM_FRAMES = 5


class HumanDetector(Node):
    def __init__(self):
        super().__init__("human_detector")
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.strides = [8, 16, 32]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        default_model = os.path.join(
            get_package_share_directory("midterm"),
            "models",
            "object_detection_yolox_2022nov.onnx",
        )
        model_path = self.declare_parameter("model_path", default_model).value
        self.net = cv2.dnn.readNet(model_path)

        self.show = self.declare_parameter("show", False).value

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

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.location_pub = self.create_publisher(Point, "human_location", latched)
        self.recent = []
        self.found = False

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
        located = None

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

            box = (x, y, w, h)
            loc = self.locate_person(box)

            if loc is not None:
                located = loc
                self.get_logger().info(f"person at {loc}")

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = msg.header.stamp
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD

                marker.pose.position.x = loc[0]
                marker.pose.position.y = loc[1]
                marker.pose.position.z = 0.85
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 1.7

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                self.marker_pub.publish(marker)

        self.confirm(located)

        if self.show:
            cv2.imshow("camera", img)
            cv2.waitKey(1)

    def confirm(self, loc):
        if self.found:
            return
        if loc is None:
            self.recent.clear()
            return

        self.recent.append(loc)
        if len(self.recent) < CONFIRM_FRAMES:
            return

        x, y = np.mean(self.recent, axis=0)
        self.location_pub.publish(Point(x=float(x), y=float(y), z=0.0))
        self.found = True
        self.get_logger().info(f"human found at ({x:.2f}, {y:.2f})")

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

    def locate_person(self, box):
        if self.K is None:
            return None

        x, y, w, h = box

        u = x + w / 2
        v = y + h

        ray = (
            (u - self.K[0, 2]) / self.K[0, 0],
            (v - self.K[1, 2]) / self.K[1, 1],
            1.0,
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "camera_rgb_optical_frame", Time()
            )
        except TransformException as e:
            self.get_logger().warn(f"no transform yet: {e}", throttle_duration_sec=2.0)
            return None

        point_a = PointStamped()
        point_b = PointStamped()

        point_b.point.x = ray[0]
        point_b.point.y = ray[1]
        point_b.point.z = ray[2]

        a_transform = do_transform_point(point_a, tf)
        b_transform = do_transform_point(point_b, tf)

        cam_pos = np.array(
            [a_transform.point.x, a_transform.point.y, a_transform.point.z]
        )
        ray_dir = np.array(
            [b_transform.point.x, b_transform.point.y, b_transform.point.z]
        )

        direction = ray_dir - cam_pos

        if direction[2] >= 0:
            return None

        t = -cam_pos[2] / direction[2]
        hit = cam_pos + t * direction

        if np.linalg.norm(hit[:2] - cam_pos[:2]) > 8:
            return None

        return float(hit[0]), float(hit[1])


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
