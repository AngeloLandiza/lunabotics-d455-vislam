#!/usr/bin/env python3
"""Robot pose in the arena: start-zone tag fix, wheel + gyro dead reckoning, and four-wall matching on the L1.

This is the step 2 odometry test's fused_walls estimator (sim/odometry/drift.py: about 1 cm median, 1.2-5.5 cm
95th percentile over a dig -> dump -> dig cycle in the sandbox), fed from topics:

  1. gyro bias: the yaw rate averaged while the wheels stand still for calib_s at startup (the "standstill
     calibration" drift.py assumes); subtracted from then on
  2. start: the first start-zone tag fix (start_tags.py) sets the pose; later fixes while the robot stands in the
     start zone update it (gated)
  3. prediction at the IMU rate: encoder speed x wheel_scale (BP-1 slip calibration) and the gyro yaw rate in
     base_link. The encoder yaw rate is not used (it over-reads pivots: about 2.6x in arena.sh)
  4. optional: a LiDAR odometry (lidar_odom_topic, e.g. KISS-ICP or Point-LIO) fused over fuse_every windows
  5. every wall_every s: L1 returns in the wall band (between the duct tops and the bin wall top), placed with the
     filter's own track, ICP-matched to the four bin walls; gated update with the relock rule

Input:  /wheel/odom (Odometry twist), /camera/imu (Imu), /l1/points, /autonomy/tag_fix (PoseWithCovarianceStamped)
Output: /autonomy/pose (PoseStamped, arena frame), /autonomy/odom (Odometry with covariance), TF arena -> tf_child
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import TransformBroadcaster

import autonomy_core as ac
import autonomy_ros as ar


class ArenaLocalizer(Node):
    def __init__(self):
        super().__init__('arena_localizer')
        p = self.declare_parameter
        p('wheel_topic', '/wheel/odom')
        p('imu_topic', '/camera/imu')
        p('cloud_topic', '/l1/points')
        p('tag_topic', '/autonomy/tag_fix')
        p('lidar_odom_topic', '')
        p('wheel_scale', 0.95)          # 1 - nominal mean slip (sim/realism.py Slip, Inferred); calibrate on BP-1
        p('calib_s', 2.0)
        p('initial_pose', [0.0])        # [x, y, yaw_deg] to start without tags (testing)
        p('arena_frame', 'arena')
        p('base_frame', 'base_link')
        p('tf_child', 'autonomy_base')  # TF arena -> this frame for RViz; empty = no TF
        p('walls', True)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.arena_frame, self.tf_child = g('arena_frame'), g('tf_child')
        self.ext = ar.Extrinsics(self, g('base_frame'))
        self.f = ac.WallFilter(scale=g('wheel_scale'))
        self.v_wheel = 0.0
        self.t_wheel = None
        self.calib = []
        self.bias = None
        self.t_calib0 = None
        self.lidar_prev = None
        self.wall_every = ac.drift.v(ac.drift.PARAMS, 'wall_every')
        self.t_last_fix = None
        self.counts = {}
        init = g('initial_pose')
        self.pending_init = (init[0], init[1], math.radians(init[2])) if len(init) == 3 else None
        self.create_subscription(Odometry, g('wheel_topic'), self.on_wheel, 50)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, 400)
        if g('walls'):
            self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud, 5)
        self.create_subscription(PoseWithCovarianceStamped, g('tag_topic'), self.on_tag, 10)
        if g('lidar_odom_topic'):
            self.create_subscription(Odometry, g('lidar_odom_topic'), self.on_lidar_odom, 50)
        self.pub = self.create_publisher(PoseStamped, '/autonomy/pose', 20)
        self.pub_o = self.create_publisher(Odometry, '/autonomy/odom', 20)
        self.tfb = TransformBroadcaster(self) if self.tf_child else None
        self.create_timer(5.0, self.report)
        self.get_logger().info('arena localizer up: hold still for the gyro calibration, then show it a start tag')

    # ---------------------------------------------------------------- inputs
    def on_wheel(self, msg):
        self.v_wheel = msg.twist.twist.linear.x
        self.t_wheel = ar.stamp_s(msg.header.stamp)

    def on_imu(self, msg):
        t = ar.stamp_s(msg.header.stamp)
        T = self.ext.get(msg.header.frame_id)
        if T is None:
            return
        w = msg.angular_velocity
        wz = float((T[:3, :3] @ np.array([w.x, w.y, w.z]))[2])      # yaw rate in base_link
        if self.bias is None:
            if self.t_wheel is None or abs(self.v_wheel) > 0.005:
                self.calib, self.t_calib0 = [], None                 # moving: restart the average
                return
            self.t_calib0 = self.t_calib0 or t
            self.calib.append(wz)
            if t - self.t_calib0 >= self.get_parameter('calib_s').value:
                self.bias = float(np.mean(self.calib))
                self.get_logger().info(f'gyro bias {math.degrees(self.bias):.3f} deg/s from {len(self.calib)} '
                                       f'samples at standstill')
                if self.pending_init is not None:
                    self.f.init(self.pending_init, t)
                    self.get_logger().info(f'initial pose from the parameter: {np.round(self.pending_init, 3)}')
            return
        if not self.f.ready:
            return
        self.f.predict(self.v_wheel, wz - self.bias, t)
        self.publish(msg.header.stamp)
        if self.t_last_fix is None:
            self.t_last_fix = t
        if t - self.t_last_fix >= self.wall_every:
            self.t_last_fix = t
            r = self.f.wall_fix()
            if r:
                self.counts[r] = self.counts.get(r, 0) + 1

    def on_tag(self, msg):
        if self.bias is None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        z = (p.x, p.y, ac.yaw_of((q.x, q.y, q.z, q.w)))
        t = ar.stamp_s(msg.header.stamp)
        if not self.f.ready:
            self.f.init(z, t)
            self.get_logger().info(f'initial pose from the start tags: x {z[0]:.3f} y {z[1]:.3f} '
                                   f'yaw {math.degrees(z[2]):.1f} deg')
            return
        x0, x1, y0, y1 = ac.v(ac.pr.ARENA, 'start_zone')
        if x0 <= self.f.x[0] <= x1 and y0 <= self.f.x[1] <= y1 and abs(self.v_wheel) < 0.005:
            ok = self.f.pose_update(z)
            self.counts['tag ' + ('used' if ok else 'gated')] = self.counts.get('tag ' + ('used' if ok else 'gated'),
                                                                                0) + 1

    def on_cloud(self, msg):
        if not self.f.ready:
            return
        T = self.ext.get(msg.header.frame_id)
        if T is None:
            return
        Pb = ac.apply_T(T, ar.cloud_xyz(msg))
        Pb = Pb[~ac.body_mask(Pb)]
        self.f.add_wall_points(Pb, ar.stamp_s(msg.header.stamp))

    def on_lidar_odom(self, msg):
        """Window increments from a LiDAR odometry's pose (its own frame): fused every fuse_every seconds."""
        if not self.f.ready:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        cur = np.array([p.x, p.y, ac.yaw_of((q.x, q.y, q.z, q.w))])
        t = ar.stamp_s(msg.header.stamp)
        if self.lidar_prev is None:
            self.lidar_prev = (t, cur)
            return
        t0, prev = self.lidar_prev
        if t - t0 >= ac.drift.v(ac.drift.PARAMS, 'fuse_every'):
            inc = ac._from_se2(np.linalg.inv(ac._se2(*prev)) @ ac._se2(*cur))
            self.f.lidar_window(inc)
            self.lidar_prev = (t, cur)

    # ---------------------------------------------------------------- outputs
    def publish(self, stamp):
        x, y, a = self.f.x
        qx, qy, qz, qw = ar.quat_from_yaw(a)
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, self.arena_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), ac.val('base_height')
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        self.pub.publish(ps)
        od = Odometry()
        od.header = ps.header
        od.child_frame_id = self.get_parameter('base_frame').value
        od.pose.pose = ps.pose
        cov = np.zeros((6, 6))
        cov[np.ix_([0, 1, 5], [0, 1, 5])] = self.f.Pc
        od.pose.covariance = cov.ravel().tolist()
        self.pub_o.publish(od)
        if self.tfb:
            tf = TransformStamped()
            tf.header = ps.header
            tf.child_frame_id = self.tf_child
            tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
            tf.transform.translation.z = ac.val('base_height')
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tfb.sendTransform(tf)

    def report(self):
        if not self.f.ready:
            return
        sd = np.sqrt(np.diag(self.f.Pc))
        self.get_logger().info(f'pose {np.round(self.f.x[:2], 3).tolist()} yaw {math.degrees(self.f.x[2]):.1f} deg, '
                               f'sd {sd[0] * 100:.1f}/{sd[1] * 100:.1f} cm {math.degrees(sd[2]):.2f} deg; '
                               f'wall fixes {self.counts}')


def main():
    rclpy.init()
    rclpy.spin(ArenaLocalizer())


if __name__ == '__main__':
    main()
