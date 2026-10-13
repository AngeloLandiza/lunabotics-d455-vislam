"""Read an arena.sh --autonomy bag (ros2 bag record, sqlite3 with or without zstd) into numpy arrays.

load(bag) -> dict with
  gt     (n, 4)  t, x, y, yaw of /ground_truth/base_pose (arena frame)
  wheel  (n, 2)  t, encoder forward speed (/wheel/odom twist.linear.x)
  imu    (n, 4)  t, gyro x, y, z in the IMU message frame;  imu_frame
  clouds list of (t, (N, 3) points in the cloud frame);      cloud_frame
  tags   (n, 4)  t, x, y, yaw of /autonomy/tag_fix
  est    (n, 4)  t, x, y, yaw of /autonomy/pose (the live localizer)
  T_base {frame: 4x4 base_link-from-frame} from /tf_static (the sim publishes camera_link -> everything, base_link
         included, so base-from-X = inv(cam-from-base) @ cam-from-X)
Needs ROS 2 Humble (rosbag2_py, rclpy) on the path.
"""
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def T_of(tr):
    q, p = tr.rotation, tr.translation
    w, x, y, z = q.w, q.x, q.y, q.z
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, msg.point_step)
    P = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in 'xyz'], 1).astype(np.float64)
    return P[np.isfinite(P).all(1)]


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def load(bag, topics=None):
    compressed = any(f.endswith('.zstd') for f in __import__('os').listdir(bag))
    r = rosbag2_py.SequentialCompressionReader() if compressed else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = topics or ['/ground_truth/base_pose', '/wheel/odom', '/camera/imu', '/l1/points', '/autonomy/tag_fix',
                      '/autonomy/pose', '/tf_static']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}
    out = {'gt': [], 'wheel': [], 'imu': [], 'clouds': [], 'tags': [], 'est': [], 'imu_frame': None,
           'cloud_frame': None}
    cam_from = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == '/ground_truth/base_pose':
            out['gt'].append((st(m.header), m.pose.position.x, m.pose.position.y, yaw_q(m.pose.orientation)))
        elif topic == '/autonomy/pose':
            out['est'].append((st(m.header), m.pose.position.x, m.pose.position.y, yaw_q(m.pose.orientation)))
        elif topic == '/wheel/odom':
            out['wheel'].append((st(m.header), m.twist.twist.linear.x))
        elif topic == '/camera/imu':
            w = m.angular_velocity
            out['imu'].append((st(m.header), w.x, w.y, w.z))
            out['imu_frame'] = m.header.frame_id
        elif topic == '/l1/points':
            out['clouds'].append((st(m.header), cloud_xyz(m)))
            out['cloud_frame'] = m.header.frame_id
        elif topic == '/autonomy/tag_fix':
            p = m.pose.pose
            out['tags'].append((st(m.header), p.position.x, p.position.y, yaw_q(p.orientation)))
        elif topic == '/tf_static':
            for tf in m.transforms:
                if tf.header.frame_id == 'camera_link':
                    cam_from[tf.child_frame_id] = T_of(tf.transform)
    for k in ('gt', 'wheel', 'imu', 'tags', 'est'):
        out[k] = np.array(out[k]) if out[k] else np.zeros((0, 4))
    if 'base_link' in cam_from:
        base_from_cam = np.linalg.inv(cam_from['base_link'])
        out['T_base'] = {f: base_from_cam @ T for f, T in cam_from.items()}
        out['T_base']['camera_link'] = base_from_cam
    return out


def interp_pose(arr, t):
    """(x, y, yaw) at times t from an (n, 4) t/x/y/yaw array (yaw unwrapped for interpolation)."""
    ts = arr[:, 0]
    yaw = np.unwrap(arr[:, 3])
    t = np.atleast_1d(t)
    return np.stack([np.interp(t, ts, arr[:, 1]), np.interp(t, ts, arr[:, 2]),
                     (np.interp(t, ts, yaw) + math.pi) % (2 * math.pi) - math.pi], -1)
