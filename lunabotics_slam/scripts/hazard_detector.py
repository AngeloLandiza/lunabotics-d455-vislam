#!/usr/bin/env python3
"""Boulder and crater detection from the L1 and the D455: the BP-1 sandbox's log-odds detector as a ROS 2 node.

Every L1 cloud (and every depth_every-th D455 depth image) becomes rays from the sensor through its returns, placed
in the arena with the robot pose at the frame's stamp. autonomy_core.HazardMap adds each frame's evidence per cell
(Pairet et al. log-odds steps, ground from the robust local plane fit) and every publish_s publishes the objects
whose pooled evidence reached the sensor's level (L1 5.1, D455 0.85; sim/bp1_sandbox/params.py DETECT).

Input:  /l1/points (PointCloud2), /camera/depth/image_rect_raw + camera_info, pose_topic (PoseStamped, arena frame)
Output: /autonomy/l1_arena      the L1 cloud in the arena frame, body returns removed (what the detector used)
        /autonomy/hazards       Float32MultiArray, 5 per hazard: kind (0 boulder, 1 crater), x, y, radius, evidence
        /autonomy/hazard_markers MarkerArray (discs + labels) for RViz
        /autonomy/hazard_grid   OccupancyGrid (100 = evidence for an object, 0 = seen, -1 = unseen)
"""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

import autonomy_core as ac
import autonomy_ros as ar


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        p = self.declare_parameter
        p('pose_topic', '/autonomy/pose')
        p('cloud_topic', '/l1/points')
        p('depth_topic', '/camera/depth/image_rect_raw')
        p('use_depth', True)
        p('depth_every', 3)
        p('depth_stride', 8)
        p('publish_s', 1.0)
        p('arena_frame', 'arena')
        p('base_frame', 'base_link')
        p('turn_max', 0.3)                  # rad/s: frames taken while turning faster are skipped
        p('pose_wait', 0.3)                 # s: a frame newer than the last pose waits this long for a later pose
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.arena_frame = g('arena_frame')
        self.ext = ar.Extrinsics(self, g('base_frame'))
        self.poses = ar.PoseHistory()
        self.map = ac.HazardMap()
        self.K = None
        self.n_depth = 0
        self.busy_s = []
        self.pending = []                   # frames waiting for a pose after their stamp
        self.n_turning = 0
        self.last_swept = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, g('pose_topic'), self.on_pose, 50)
        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud, 2)
        if g('use_depth'):
            self.create_subscription(Image, g('depth_topic'), self.on_depth, 2)
            self.create_subscription(CameraInfo, g('depth_topic').rsplit('/', 1)[0] + '/camera_info', self.on_info, 2)
        self.pub = self.create_publisher(Float32MultiArray, '/autonomy/hazards', latched)
        self.pub_m = self.create_publisher(MarkerArray, '/autonomy/hazard_markers', latched)
        self.pub_g = self.create_publisher(OccupancyGrid, '/autonomy/hazard_grid', latched)
        self.pub_c = self.create_publisher(PointCloud2, '/autonomy/l1_arena', 2)
        self.create_timer(g('publish_s'), self.publish)
        self.get_logger().info('hazard detector up: waiting for poses on ' + g('pose_topic'))

    def on_info(self, msg):
        self.K = np.array(msg.k, float).reshape(3, 3)

    def on_pose(self, msg):
        self.poses.add(msg)
        q = np.array(self.poses.h[-1][1:])
        if self.last_swept is None or np.hypot(*(q[:2] - self.last_swept[:2])) > 0.03 or \
                abs(ac.wrap(q[2] - self.last_swept[2])) > 0.05:
            self.map.drove(q)
            self.last_swept = q
        if self.pending:
            todo, self.pending = self.pending, []
            for f in todo:
                self._frame(*f)

    def _frame(self, P_sensor, frame, stamp, kind, sigma):
        """Place a frame with the pose interpolated at its stamp. Frames newer than the last pose wait for the next
        one (holding the last pose made the cloud lag the robot's turn: arena.sh run 5 smeared boulders 0.2-0.5 m on
        the localizer pose), and frames taken while turning faster than turn_max are skipped (the L1 frame is not
        deskewed, and the yaw error is largest in fast turns)."""
        t = ar.stamp_s(stamp)
        last = self.poses.latest()
        if last is not None and t > last:
            if t - last < self.get_parameter('pose_wait').value:
                self.pending.append((P_sensor, frame, stamp, kind, sigma))
            return
        rate = self.poses.yaw_rate(t)
        if rate is not None and abs(rate) > self.get_parameter('turn_max').value:
            self.n_turning += 1
            return
        pose = self.poses.at(t)
        T_bs = self.ext.get(frame)
        if pose is None or T_bs is None or len(P_sensor) == 0:
            return
        t0 = time.time()
        Pb = ac.apply_T(T_bs, P_sensor)
        keep = ~ac.body_mask(Pb)
        roll, pitch = self.poses.tilt_at(t)
        T_ab = ac.base_T(*pose, roll=roll, pitch=pitch)
        Pa = ac.apply_T(T_ab, Pb[keep])
        origin = ac.apply_T(T_ab, T_bs[:3, 3][None])[0]
        sig = sigma[keep] if np.ndim(sigma) else sigma
        self.map.add(ac.make_scan(origin, Pa, sig, kind))
        self.busy_s.append(time.time() - t0)
        if kind == 'l1':
            self.pub_c.publish(ar.xyz_cloud(Pa, self.arena_frame, stamp))

    def on_cloud(self, msg):
        self._frame(ar.cloud_xyz(msg), msg.header.frame_id, msg.header.stamp, 'l1', ac.val('l1_sigma'))

    def on_depth(self, msg):
        self.n_depth += 1
        if self.K is None or self.n_depth % self.get_parameter('depth_every').value:
            return
        P, cosang = ac.depth_cloud(ar.depth_image(msg), self.K, self.get_parameter('depth_stride').value)
        self._frame(P, msg.header.frame_id, msg.header.stamp, 'd455', ac.d455_sigma_r(P[:, 2], cosang))

    def publish(self):
        if self.map.frames == 0:
            return
        self.map.refit()
        objs = self.map.objects()
        data = []
        for o in objs:
            data += [0.0 if o['kind'] == 'boulder' else 1.0, o['x'], o['y'], o['r'], o['evidence']]
        self.pub.publish(Float32MultiArray(data=[float(x) for x in data]))
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for k, o in enumerate(objs):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.arena_frame, stamp
            m.ns, m.id, m.type = 'hazard', k, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = o['x'], o['y'], 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * o['r']
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = ((1.0, 0.45, 0.1, 0.8) if o['kind'] == 'boulder'
                                                          else (0.2, 0.5, 1.0, 0.8))
            ma.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type = 'label', k, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = o['x'], o['y'], 0.35
            t.pose.orientation.w = 1.0
            t.scale.z = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{o['kind']} {o['r'] * 200:.0f} cm ({o['sensor']})"
            ma.markers.append(t)
        self.pub_m.publish(ma)
        g = OccupancyGrid()
        g.header.frame_id, g.header.stamp = self.arena_frame, stamp
        grid = self.map.grid
        g.info.resolution = float(grid.cell)
        g.info.width, g.info.height = grid.nx, grid.ny
        g.info.origin.orientation.w = 1.0
        g.data = self.map.occupancy().T.ravel().tolist()        # rows = y
        self.pub_g.publish(g)
        if self.busy_s:
            self.get_logger().info(f'{len(objs)} hazards ({sum(o["kind"] == "boulder" for o in objs)} boulders), '
                                   f'{len(self.map.shallow)} shallow crater blobs dropped, '
                                   f'{sum(x[0] == "duct" for x in self.map.dropped)} touching a duct; '
                                   f'{self.n_turning} frames skipped while turning; '
                                   f'{self.map.frames} frames, {1000 * np.mean(self.busy_s):.0f} ms per frame',
                                   throttle_duration_sec=10.0)
            self.busy_s = []


def main():
    rclpy.init()
    rclpy.spin(HazardDetector())


if __name__ == '__main__':
    main()
