#!/usr/bin/env python3
"""Clean live views for D455 VISLAM.

Camera window: left IR image with the odometry's landmarks drawn on it
  green  = inlier (used in the pose estimate), with a short motion trail
  amber  = matched to the local map but rejected as an outlier
  grey   = detected, not matched
Also publishes, for RViz:
  /vislam/trajectory     nav_msgs/Path    full-rate odometry trail (odom frame)
  /vislam/camera_marker  Marker           camera frustum at camera_link
  /vislam/feature_image  Image            same picture as the window (only if subscribed)
"""
import collections
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

WINDOW = 'VISLAM - Camera + Landmarks'
INLIER = (90, 220, 70)
OUTLIER = (40, 180, 255)
UNMATCHED = (150, 150, 150)
LOST_RED = (60, 60, 230)
TRAIL_LEN = 8


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class VislamViewer(Node):
    def __init__(self, show_window):
        super().__init__('vislam_viewer')
        self.show_window = show_window
        self.images = collections.OrderedDict()
        self.trails = {}
        self.frame_idx = 0
        self.pending = None
        self.odom_times = collections.deque(maxlen=30)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        self.path = Path()
        self.last_path_pose = None

        self.create_subscription(Image, '/camera/infra1/image_rect_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(OdomInfo, '/odom_info', self.on_info, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.path_pub = self.create_publisher(Path, '/vislam/trajectory', 1)
        self.marker_pub = self.create_publisher(Marker, '/vislam/camera_marker', 1)
        self.image_pub = self.create_publisher(Image, '/vislam/feature_image', 1)
        self.create_timer(0.1, self.publish_path)
        self.create_timer(1.0, self.publish_frustum)

        if self.show_window:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(WINDOW, 1060, 600)

    # ---------------------------------------------------------------- inputs
    def on_image(self, msg):
        self.images[stamp_ns(msg.header)] = msg
        while len(self.images) > 30:
            self.images.popitem(last=False)

    def on_info(self, info):
        self.odom_times.append(time.monotonic())
        img = self.images.get(stamp_ns(info.header))
        if img is not None:
            self.pending = (img, info)

    def on_odom(self, msg):
        if msg.pose.covariance[0] >= 9999:  # null pose published while lost
            return
        p = msg.pose.pose
        if self.last_path_pose is not None:
            q = self.last_path_pose
            moved = math.dist((p.position.x, p.position.y, p.position.z),
                              (q.position.x, q.position.y, q.position.z))
            if moved < 0.01:
                return
        self.last_path_pose = p
        self.path.header.frame_id = msg.header.frame_id
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = p
        self.path.poses.append(ps)

    # -------------------------------------------------------------- outputs
    def publish_path(self):
        if self.path.poses:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)

    def publish_frustum(self):
        # D455 IR FOV ~87 x 58 deg; camera_link convention: x forward, y left, z up.
        d, hw, hh = 0.25, 0.25 * math.tan(math.radians(43.5)), 0.25 * math.tan(math.radians(29))
        corners = [Point(x=d, y=s * hw, z=t * hh) for s, t in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        origin = Point()
        m = Marker()
        m.header.frame_id = 'camera_link'
        m.ns, m.id, m.type, m.action = 'camera', 0, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.012
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.85, 1.0, 1.0
        for i, c in enumerate(corners):
            m.points += [origin, c, c, corners[(i + 1) % 4]]
        self.marker_pub.publish(m)

    # ------------------------------------------------------------- drawing
    def render(self, img_msg, info):
        h, w = img_msg.height, img_msg.width
        gray = np.frombuffer(img_msg.data, np.uint8).reshape(h, img_msg.step)[:, :w]
        base = (self.clahe.apply(gray) * 0.7).astype(np.uint8)
        canvas = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)

        self.frame_idx += 1
        inliers = set(info.word_inliers)
        matches = set(info.word_matches)
        for key, kp in zip(info.words_keys, info.words_values):
            pt = (kp.pt.x, kp.pt.y)
            if key in inliers:
                trail = self.trails.setdefault(key, collections.deque(maxlen=TRAIL_LEN))
                trail.append((pt, self.frame_idx))
            elif key in matches:
                cv2.circle(canvas, (round(pt[0]), round(pt[1])), 3, OUTLIER, 1, cv2.LINE_AA)
            else:
                cv2.circle(canvas, (round(pt[0]), round(pt[1])), 1, UNMATCHED, -1, cv2.LINE_AA)

        for key in list(self.trails):
            trail = self.trails[key]
            if self.frame_idx - trail[-1][1] > 2:
                del self.trails[key]
                continue
            pts = [p for p, _ in trail]
            for i in range(1, len(pts)):
                a = i / len(pts)
                color = tuple(int(c * a) for c in INLIER)
                cv2.line(canvas, tuple(map(round, pts[i - 1])), tuple(map(round, pts[i])),
                         color, 1, cv2.LINE_AA)
            if trail[-1][1] == self.frame_idx:
                cv2.circle(canvas, tuple(map(round, pts[-1])), 3, INLIER, -1, cv2.LINE_AA)

        self.draw_hud(canvas, info)
        return canvas

    def draw_hud(self, canvas, info):
        h, w = canvas.shape[:2]
        band = canvas[:34].copy()
        band[:] = (18, 18, 22)
        cv2.addWeighted(band, 0.75, canvas[:34], 0.25, 0, canvas[:34])

        if info.lost:
            status, color = 'LOST', LOST_RED
            cv2.rectangle(canvas, (0, 0), (w - 1, h - 1), LOST_RED, 3)
        else:
            status, color = 'TRACKING', INLIER
        cv2.circle(canvas, (16, 17), 6, color, -1, cv2.LINE_AA)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(canvas, status, (30, 23), font, 0.55, color, 1, cv2.LINE_AA)

        hz = 0.0
        if len(self.odom_times) > 1:
            hz = (len(self.odom_times) - 1) / (self.odom_times[-1] - self.odom_times[0] + 1e-9)
        stats = (f'inliers {info.inliers:4d}   features {info.features:4d}   '
                 f'odom {hz:4.1f} Hz   {info.time_estimation * 1000:3.0f} ms   '
                 f'dist {info.distance_travelled:6.2f} m')
        cv2.putText(canvas, stats, (150, 22), font, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

    def tick(self):
        if self.pending is None:
            return
        img_msg, info = self.pending
        self.pending = None
        canvas = self.render(img_msg, info)
        if self.image_pub.get_subscription_count() > 0:
            out = Image()
            out.header = img_msg.header
            out.height, out.width = canvas.shape[:2]
            out.encoding, out.step = 'bgr8', canvas.shape[1] * 3
            out.data = canvas.tobytes()
            self.image_pub.publish(out)
        if self.show_window:
            cv2.imshow(WINDOW, canvas)


def main():
    show = '--no-window' not in sys.argv
    rclpy.init(args=sys.argv)
    node = VislamViewer(show)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            node.tick()
            if node.show_window:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    node.show_window = False
                    cv2.destroyAllWindows()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
