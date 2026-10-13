"""ROS 2 glue shared by the autonomy nodes: message <-> numpy, TF to 4x4 matrices, sensor extrinsics in base_link.

Extrinsics: every node needs sensor poses in base_link. They come from TF (base_link <-> sensor frame); in arena.sh
the sim publishes camera_link -> base_link plus camera_link -> each sensor, and on the robot camera_mount gives
base_link -> camera_link. If no base_link link exists after tf_wait seconds, base_link -> camera_link falls back to
the team URDF's D455 mount (camera_mount parameter) and only camera_link -> sensor comes from TF.
"""
import math

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time

import autonomy_core as ac

TEAM_CAMERA_MOUNT = '0.535 0 0.37 0 0.6981 0'      # base_d455_joint in uic_bot_2027.xacro (x y z roll pitch yaw)


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xyz(msg):
    """x, y, z float32 fields of a PointCloud2 as an (N, 3) float64 array (NaN points dropped)."""
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, msg.point_step)
    P = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in 'xyz'], 1).astype(np.float64)
    return P[np.isfinite(P).all(1)]


def depth_image(msg):
    """32FC1 (m) or 16UC1 (mm) depth image as float metres."""
    if msg.encoding in ('32FC1',):
        a = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return a.astype(np.float64)
    if msg.encoding in ('16UC1', 'mono16'):
        a = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return a.astype(np.float64) / 1000.0
    raise ValueError(f'depth encoding {msg.encoding}')


def gray_image(msg):
    import cv2
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, '8UC1': 1}[msg.encoding]
    a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    if ch == 1:
        return a.reshape(msg.height, msg.width)
    a = a.reshape(msg.height, msg.width, ch)
    code = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY,
            'bgra8': cv2.COLOR_BGRA2GRAY}[msg.encoding]
    return cv2.cvtColor(np.ascontiguousarray(a), code)


def tf_to_T(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = ac.quat_to_R((q.x, q.y, q.z, q.w))
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def mount_T(text):
    x, y, z, r, p, yw = (float(v) for v in text.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, (x, y, z)
    return T


def quat_from_yaw(a):
    return 0.0, 0.0, math.sin(a / 2), math.cos(a / 2)


class Extrinsics:
    """Cached base_link -> sensor-frame transforms (static)."""

    def __init__(self, node, base_frame='base_link', camera_frame='camera_link', tf_wait=5.0):
        import tf2_ros
        self.node = node
        self.base, self.cam = base_frame, camera_frame
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, node)
        self.cache = {}
        self.t0 = node.get_clock().now()
        self.tf_wait = tf_wait
        node.declare_parameter('camera_mount', TEAM_CAMERA_MOUNT)
        self.fallback = mount_T(node.get_parameter('camera_mount').value)
        self.warned = False

    def get(self, frame):
        """4x4 pose of frame in base_link, or None while TF is not there yet."""
        if frame in self.cache:
            return self.cache[frame]
        try:
            T = tf_to_T(self.buf.lookup_transform(self.base, frame, Time()))
        except Exception:
            waited = (self.node.get_clock().now() - self.t0) > Duration(seconds=self.tf_wait)
            if not waited:
                return None
            try:
                T = self.fallback @ tf_to_T(self.buf.lookup_transform(self.cam, frame, Time()))
            except Exception:
                return None
            if not self.warned:
                self.node.get_logger().warn(f'no TF {self.base} <-> {frame}: using camera_mount '
                                            f'"{self.node.get_parameter("camera_mount").value}" for {self.base} -> '
                                            f'{self.cam}')
                self.warned = True
        self.cache[frame] = T
        self.node.get_logger().info(f'{self.base} -> {frame}: xyz {np.round(T[:3, 3], 3).tolist()}')
        return T


class PoseHistory:
    """Recent (t, x, y, yaw) from a PoseStamped topic, for placing sensor frames at their own stamp."""

    def __init__(self, n=400):
        self.h = []
        self.n = n

    def add(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.h.append((stamp_s(msg.header.stamp), p.x, p.y, ac.yaw_of((q.x, q.y, q.z, q.w))))
        if len(self.h) > self.n:
            self.h = self.h[-self.n // 2:]

    def at(self, t, max_gap=0.3):
        """Pose interpolated at t, or None if t is outside the history by more than max_gap."""
        if not self.h:
            return None
        ts = np.array([e[0] for e in self.h])
        k = int(np.searchsorted(ts, t))
        if k == 0:
            return np.array(self.h[0][1:]) if ts[0] - t < max_gap else None
        if k >= len(ts):
            return np.array(self.h[-1][1:]) if t - ts[-1] < max_gap else None
        a, b = np.array(self.h[k - 1][1:]), np.array(self.h[k][1:])
        f = (t - ts[k - 1]) / max(ts[k] - ts[k - 1], 1e-9)
        yaw = a[2] + f * ac.wrap(b[2] - a[2])
        return np.array([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), ac.wrap(yaw)])


    def latest(self):
        return self.h[-1][0] if self.h else None

    def yaw_rate(self, t, half=0.1):
        """Mean yaw rate (rad/s) over the 2 * half before t, or None if the history does not cover it."""
        a, b = self.at(t - 2 * half, max_gap=0.0), self.at(t, max_gap=0.0)
        if a is None or b is None:
            return None
        return ac.wrap(b[2] - a[2]) / (2 * half)


def xyz_cloud(P, frame, stamp):
    """(N, 3) points as a PointCloud2 with float32 x, y, z."""
    from sensor_msgs.msg import PointCloud2, PointField
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.height, msg.width = 1, len(P)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.is_bigendian, msg.point_step, msg.row_step = False, 12, 12 * len(P)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(P, np.float32).tobytes()
    return msg
