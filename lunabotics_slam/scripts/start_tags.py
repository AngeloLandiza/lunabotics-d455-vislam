#!/usr/bin/env python3
"""Start-zone pose from the two AprilTags (36h11 ids 0 and 1, one per start-zone bin wall) in the rear camera.

autonomy_core.TagFix: OpenCV's AprilTag 36h11 detector with sub-pixel corners; one tag -> IPPE_SQUARE (lower-error
solution), two tags -> SQPnP + LM refinement on all eight corners; camera pose -> base_link pose through TF. Offline
check (test_autonomy_core.py tags): 1.1 cm median, 0.4 deg median. Isaac two-tag check (sim/fiducial): 1.5 cm median.

Input:  /rear_camera/image_raw + camera_info
Output: /autonomy/tag_fix (PoseWithCovarianceStamped, arena frame; covariance from drift.PARAMS tag_sigma)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

import autonomy_core as ac
import autonomy_ros as ar


class StartTags(Node):
    def __init__(self):
        super().__init__('start_tags')
        p = self.declare_parameter
        p('image_topic', '/rear_camera/image_raw')
        p('rate_hz', 3.0)
        p('max_reproj_px', 2.0)
        p('arena_frame', 'arena')
        p('base_frame', 'base_link')
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.ext = ar.Extrinsics(self, g('base_frame'))
        self.fix = ac.TagFix()
        self.K = self.D = None
        self.t_last = -1e9
        self.create_subscription(Image, g('image_topic'), self.on_image, 2)
        self.create_subscription(CameraInfo, g('image_topic').rsplit('/', 1)[0] + '/camera_info', self.on_info, 2)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/autonomy/tag_fix', 10)

    def on_info(self, msg):
        self.K = np.array(msg.k, float).reshape(3, 3)
        self.D = np.array(msg.d, float) if len(msg.d) else None

    def on_image(self, msg):
        t = ar.stamp_s(msg.header.stamp)
        if self.K is None or t - self.t_last < 1.0 / self.get_parameter('rate_hz').value:
            return
        self.t_last = t
        T_bo = self.ext.get(msg.header.frame_id)
        if T_bo is None:
            return
        found = self.fix.detect(ar.gray_image(msg))
        if not found:
            return
        est = self.fix.solve(found, self.K, self.D, T_bo)
        if est is None or est[4] > self.get_parameter('max_reproj_px').value:
            return
        x, y, yaw, n, err = est
        out = PoseWithCovarianceStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.get_parameter('arena_frame').value
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.position.z = ac.val('base_height')
        _, _, qz, qw = ar.quat_from_yaw(yaw)
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        sx, sa = ac.drift.v(ac.drift.PARAMS, 'tag_sigma')
        cov = np.zeros((6, 6))
        cov[0, 0] = cov[1, 1] = sx ** 2
        cov[5, 5] = math.radians(sa) ** 2
        out.pose.covariance = cov.ravel().tolist()
        self.pub.publish(out)
        self.get_logger().info(f'tags {sorted(found)}: x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f} deg '
                               f'({err:.2f} px)', throttle_duration_sec=2.0)


def main():
    rclpy.init()
    rclpy.spin(StartTags())


if __name__ == '__main__':
    main()
