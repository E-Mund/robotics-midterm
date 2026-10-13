#!/usr/bin/env python3
"""
Monocular floor-obstacle detector (OpenCV) for objects the LDS LIDAR cannot see
(the TurtleBot3 LIDAR scans ~17 cm above the floor, so small objects lying on the floor are invisible to it).

Method: appearance-based obstacle detection (Ulrich & Nourbakhsh, "Appearance-Based Obstacle
Detection with Monocular Color Vision", AAAI 2000):
  1. A reference patch at the bottom-centre of the image (the floor right in front of the robot)
     gives a colour histogram of "floor" (Lab a*/b* channels, robust to shading).
  2. Every pixel is back-projected onto that histogram (cv2.calcBackProject). Pixels that are
     unlikely under the floor model, or much brighter/darker than the floor, are obstacle candidates.
  3. For each image column we scan upward from the bottom; the first run of obstacle pixels is the
     contact point between the obstacle and the floor.
  4. That pixel is projected onto the ground plane (z = 0 in base_footprint) using the camera
     intrinsics (CameraInfo) and the camera pose from TF -> metric (x, y) obstacle points.

Outputs (sensor_msgs/PointCloud2, frame base_footprint):
  <ns>/camera_obstacles/points : obstacle contact points -> marking source of a Nav2 costmap layer
  <ns>/camera_obstacles/free   : end points of free floor rays -> clearing source (removes
                                 obstacles that moved away = dynamic obstacles)
  <ns>/camera_obstacles/debug  : debug image (red = obstacle mask, green dots = detections)

The floor model adapts slowly, and is re-learned if the floor appearance changes (e.g. entering a
room with a different floor) while the LIDAR confirms the space right in front is empty.

Sources: Ulrich & Nourbakhsh (AAAI 2000); OpenCV histogram back-projection tutorial
(https://docs.opencv.org/4.x/dc/df6/tutorial_py_histogram_backprojection.html); pinhole camera model
(sensor_msgs/CameraInfo docs); Nav2 costmap ObstacleLayer PointCloud2 sources (https://docs.nav2.org).
"""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FloorObstacleDetector(Node):
    def __init__(self):
        super().__init__('floor_obstacle_detector')
        defaults = {
            'image_topic': 'camera/image_raw',
            'camera_info_topic': 'camera/camera_info',
            'scan_topic': 'scan',
            'base_frame': 'base_footprint',
            'processing_width': 320,       # image is downscaled to this width
            'column_step': 4,              # pixels per column bin (processed image)
            'max_range': 2.0,              # m, ignore ground further than this
            'min_range': 0.0,
            'hist_bins': 32,
            'backproject_threshold': 25,   # 0..255, lower = less floor-like
            'lightness_threshold': 70,     # |L - L_floor| above this = obstacle
            'min_run': 4,                  # consecutive obstacle rows needed
            'ref_rows_fraction': 0.12,     # reference patch height (bottom of image)
            'ref_cols_fraction': 0.30,     # reference patch width (centred)
            'patch_bad_fraction': 0.25,    # patch is "not floor" if this fraction is unlikely
            'adapt_rate': 0.05,
            'relearn_frames': 15,
            'front_clear_distance': 0.6,   # m, LIDAR check before re-learning the floor
            'clear_margin': 0.08,          # m, clearing rays stop this short of an obstacle
            'point_height': 0.05,          # z of published points (inside costmap height band)
            'publish_debug': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {n: self.get_parameter(n).value for n in defaults}

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['image_topic'], self._on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['scan_topic'], self._on_scan, qos_profile_sensor_data)
        self.pub_obs = self.create_publisher(PointCloud2, 'camera_obstacles/points', 10)
        self.pub_free = self.create_publisher(PointCloud2, 'camera_obstacles/free', 10)
        self.pub_dbg = self.create_publisher(Image, 'camera_obstacles/debug', 2)

        self.K = None
        self.cam_frame = None
        self.lut_key = None
        self.hist = None
        self.ref_L = None
        self.mismatch = 0
        self.front_clear = False
        self.get_logger().info('Floor obstacle detector started')

    # --------------------------------------------------------------- callbacks
    def _on_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.cam_frame = msg.header.frame_id

    def _on_scan(self, msg):
        # front sector +-25 deg (LDS frame looks forward on TurtleBot3)
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        sel = (np.abs(angles) < math.radians(25)) & np.isfinite(ranges) & (ranges > msg.range_min)
        self.front_clear = (not np.any(sel)) or float(np.min(ranges[sel])) > self.p['front_clear_distance']

    def _on_image(self, msg):
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h0, w0 = img.shape[:2]
        w = int(self.p['processing_width'])
        scale = w / float(w0)
        h = int(round(h0 * scale))
        small = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        frame = msg.header.frame_id or self.cam_frame
        if not self._ensure_lut(w, h, scale, frame):
            return

        lab = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2LAB)
        self._update_floor_model(lab)
        mask = self._segment(lab)
        obs, free, hit_rows = self._columns(mask)

        header = Header(stamp=msg.header.stamp, frame_id=self.p['base_frame'])
        z = float(self.p['point_height'])
        self.pub_obs.publish(point_cloud2.create_cloud_xyz32(header, [[float(x), float(y), z] for x, y in obs]))
        self.pub_free.publish(point_cloud2.create_cloud_xyz32(header, [[float(x), float(y), z] for x, y in free]))

        if self.p['publish_debug'] and self.pub_dbg.get_subscription_count() > 0:
            dbg = small.copy()
            dbg[mask > 0] = (0.5 * dbg[mask > 0] + np.array([0, 0, 127])).astype(np.uint8)
            step = self.step
            for j, v in hit_rows:
                cv2.circle(dbg, (int(j * step + step / 2), int(v)), 2, (0, 255, 0), -1)
            cv2.rectangle(dbg, (self.c0, self.r0), (self.c1, h - 1), (255, 255, 0), 1)
            out = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
            out.header = msg.header
            self.pub_dbg.publish(out)

    # ----------------------------------------------------------- ground LUT
    def _ensure_lut(self, w, h, scale, frame):
        key = (w, h, round(scale, 5), frame)
        if key == self.lut_key:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(self.p['base_frame'], frame, Time())
        except TransformException as e:
            self.get_logger().info(f'Waiting for TF {self.p["base_frame"]} <- {frame}: {e}',
                                   throttle_duration_sec=5.0)
            return False
        q, t = tf.transform.rotation, tf.transform.translation
        R = quat_to_mat(q.x, q.y, q.z, q.w)
        origin = np.array([t.x, t.y, t.z])

        fx, fy = self.K[0, 0] * scale, self.K[1, 1] * scale
        cx, cy = self.K[0, 2] * scale, self.K[1, 2] * scale
        self.step = int(self.p['column_step'])
        self.ncols = w // self.step
        us = np.arange(self.ncols) * self.step + self.step / 2.0
        vs = np.arange(h, dtype=float)
        U, V = np.meshgrid(us, vs)                               # (h, ncols)
        d_opt = np.stack([(U - cx) / fx, (V - cy) / fy, np.ones_like(U)], axis=-1)
        d = d_opt @ R.T                                          # ray directions in base frame
        with np.errstate(divide='ignore', invalid='ignore'):
            s = np.where(d[..., 2] < -1e-6, -origin[2] / d[..., 2], np.nan)
        X = origin[0] + s * d[..., 0]
        Y = origin[1] + s * d[..., 1]
        rng = np.hypot(X, Y)
        valid = np.isfinite(s) & (s > 0) & (rng <= self.p['max_range']) & (rng >= self.p['min_range'])

        self.X, self.Y, self.valid = X, Y, valid
        self.cam_xy = origin[:2]
        self.r0 = int(h * (1.0 - self.p['ref_rows_fraction']))
        self.c0 = int(w * (0.5 - self.p['ref_cols_fraction'] / 2))
        self.c1 = int(w * (0.5 + self.p['ref_cols_fraction'] / 2))
        self.lut_key = key
        if not valid.any():
            self.get_logger().warn('No image row sees the ground within max_range; check camera TF')
        else:
            self.get_logger().info(f'Ground LUT built ({w}x{h}, camera height {origin[2]:.3f} m)')
        return True

    # ----------------------------------------------------------- floor model
    def _hist(self, lab):
        b = int(self.p['hist_bins'])
        hst = cv2.calcHist([lab], [1, 2], None, [b, b], [0, 256, 0, 256])
        return hst / (hst.sum() + 1e-9)

    def _backproject(self, lab):
        model = cv2.GaussianBlur(self.hist, (3, 3), 0)         # generalise a little
        model = (model / (model.max() + 1e-9) * 255.0).astype(np.float32)
        return cv2.calcBackProject([lab], [1, 2], model, [0, 256, 0, 256], 1)

    def _update_floor_model(self, lab):
        patch = np.ascontiguousarray(lab[self.r0:, self.c0:self.c1])
        ph = self._hist(patch)
        pl = float(patch[..., 0].mean())
        if self.hist is None:
            self.hist, self.ref_L = ph, pl
            return
        bp = self._backproject(patch)
        bad = float(np.mean(bp < self.p['backproject_threshold']))
        if bad < self.p['patch_bad_fraction'] and abs(pl - self.ref_L) < self.p['lightness_threshold']:
            a = float(self.p['adapt_rate'])
            self.hist = (1 - a) * self.hist + a * ph
            self.ref_L = (1 - a) * self.ref_L + a * pl
            self.mismatch = 0
        else:
            # reference patch does not look like floor: probably an obstacle right in front.
            self.mismatch += 1
            if self.mismatch >= self.p['relearn_frames'] and self.front_clear:
                self.get_logger().warn('Floor appearance changed and LIDAR shows free space: re-learning floor')
                self.hist, self.ref_L, self.mismatch = ph, pl, 0

    def _segment(self, lab):
        bp = cv2.blur(self._backproject(lab), (5, 5))
        obstacle = bp < self.p['backproject_threshold']
        L = lab[..., 0].astype(np.int16)
        obstacle |= np.abs(L - int(self.ref_L)) > self.p['lightness_threshold']
        mask = obstacle.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        return mask

    # ------------------------------------------------------- column scanning
    def _columns(self, mask):
        h, n, s, k = mask.shape[0], self.ncols, self.step, int(self.p['min_run'])
        m = mask[:, :n * s].reshape(h, n, s).mean(axis=2) > 127
        m &= self.valid
        c = np.cumsum(m.astype(np.int32), axis=0)
        win = np.zeros_like(m)
        prev = np.vstack([np.zeros((1, n), np.int32), c[:-k]])
        win[k - 1:] = (c[k - 1:] - prev) == k                 # rows v-k+1..v all obstacle
        has_hit = win.any(axis=0)
        hit_row = h - 1 - np.argmax(win[::-1], axis=0)         # bottom-most = closest contact

        has_ground = self.valid.any(axis=0)
        far_row = np.argmax(self.valid, axis=0)                # top-most valid = farthest ground

        obs, free, hit_rows = [], [], []
        cxy = self.cam_xy
        for j in range(n):
            if not has_ground[j]:
                continue
            if has_hit[j]:
                v = hit_row[j]
                pt = np.array([self.X[v, j], self.Y[v, j]])
                obs.append(pt)
                hit_rows.append((j, v))
                ray = pt - cxy
                dist = np.linalg.norm(ray)
                if dist > self.p['clear_margin'] + 1e-3:
                    free.append(cxy + ray * (dist - self.p['clear_margin']) / dist)
            else:
                v = far_row[j]
                free.append(np.array([self.X[v, j], self.Y[v, j]]))
        return obs, free, hit_rows


def main(args=None):
    rclpy.init(args=args)
    node = FloorObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
