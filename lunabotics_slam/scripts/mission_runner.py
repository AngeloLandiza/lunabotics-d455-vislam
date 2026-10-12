#!/usr/bin/env python3
"""Dig -> dump cycles with the custom planner: tangent-graph routes, the shared executor, replanning on new hazards.

autonomy_core.MissionRunner on the localizer's pose and the detector's hazards (offline check, test_autonomy_core.py
planner: sandbox worlds with hazards revealed within 2.5 m, skid-steer with slip). Before the mission: stand still
for the gyro calibration, then turn in place slowly until the start tags give a pose (Isaac two-tag check: median
35 deg of turning). At a dig goal it asks the sim to dig in front of the robot (/arena/dig, sim only); at the dump
goal it dumps the same volume on the berm side.

Input:  /autonomy/pose (PoseStamped), /autonomy/hazards (Float32MultiArray: kind, x, y, r, evidence)
Output: /cmd_vel (Twist), /autonomy/path (Path), /autonomy/goals (MarkerArray), /arena/dig (sim)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

import autonomy_core as ac
import autonomy_ros as ar

HZ = 20.0                                   # arena2d.CONTROL_HZ


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_runner')
        p = self.declare_parameter
        p('pose_topic', '/autonomy/pose')
        p('cycles', 3)
        p('seed', 0)
        p('sim_dig', True)
        p('autostart', True)
        p('wait_s', 3.0)                    # stand still first: the localizer's gyro calibration takes calib_s
        p('search_rate', 0.25)              # rad/s, turn in place until the tags give a pose
        p('arena_frame', 'arena')
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.frame = g('arena_frame')
        self.pose = None
        self.t_pose = None
        self.hazards = []
        rng = np.random.default_rng(g('seed'))
        self.goals = ac.mission_goals(rng, g('cycles'))
        self.model = ac.RobotModel()
        self.run = ac.MissionRunner(self.goals, self.model)
        self.t0 = None
        self.searched = 0.0
        self.n_log = 0
        self.last_leg = None
        self.create_subscription(PoseStamped, g('pose_topic'), self.on_pose, 20)
        self.create_subscription(Float32MultiArray, '/autonomy/hazards', self.on_hazards, 5)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/autonomy/path', 2)
        self.pub_goal = self.create_publisher(MarkerArray, '/autonomy/goals', 2)
        self.pub_dig = self.create_publisher(Float32MultiArray, '/arena/dig', 10)
        self.create_timer(1.0 / HZ, self.tick)
        for k, (kind, gl) in enumerate(self.goals):
            self.get_logger().info(f'goal {k + 1}: {kind} at x {gl[0]:.2f} y {gl[1]:.2f} '
                                   f'heading {math.degrees(gl[2]):.0f} deg')

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.pose = np.array([p.x, p.y, ac.yaw_of((q.x, q.y, q.z, q.w))])
        self.t_pose = ar.stamp_s(msg.header.stamp)

    def on_hazards(self, msg):
        d = np.asarray(msg.data, float).reshape(-1, 5)
        self.hazards = [{'kind': 'boulder' if k < 0.5 else 'crater', 'x': x, 'y': y, 'r': r, 'evidence': e}
                        for k, x, y, r, e in d]

    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.get_parameter('autostart').value:
            return
        if self.t0 is None:
            self.t0 = now
        if self.pose is None:
            # stand still for the gyro calibration, then turn slowly until the start tags give a pose
            if now - self.t0 < self.get_parameter('wait_s').value or self.searched > 2 * math.pi:
                self.send(0.0, 0.0)
                return
            w = self.get_parameter('search_rate').value
            self.searched += w / HZ
            v, w = self.model.command(0.0, w, 1.0 / HZ)
            self.send(0.0, w)
            return
        if self.run.state == 'done':
            self.send(0.0, 0.0)
            return
        v, w, ev = self.run.step(self.pose, now, self.hazards, 1.0 / HZ)
        self.send(v, w)
        for t, m in self.run.log[self.n_log:]:
            self.get_logger().info(m)
        self.n_log = len(self.run.log)
        if self.run.leg is not self.last_leg and self.run.leg is not None and self.run.leg.ok:
            self.last_leg = self.run.leg
            self.publish_plan()
        if ev and self.get_parameter('sim_dig').value:
            kind, gl = ev
            r, depth = ac.val('dig_stroke')
            fx = ac.v(ac.a2.PARAMS, 'footprint')[1] + r          # just past the front edge (no bucket model yet)
            x = self.pose[0] + fx * math.cos(self.pose[2])
            y = self.pose[1] + fx * math.sin(self.pose[2])
            self.pub_dig.publish(Float32MultiArray(data=[float(x), float(y), float(r),
                                                         float(depth if kind == 'dig' else -depth)]))
            self.get_logger().info(f'{kind} at x {x:.2f} y {y:.2f}')

    def publish_plan(self):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id, path.header.stamp = self.frame, stamp
        for x, y in self.run.leg.route:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), 0.02
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)
        ma = MarkerArray()
        for k, (kind, gl) in enumerate(self.goals):
            m = Marker()
            m.header = path.header
            m.ns, m.id, m.type = 'goal', k, Marker.ARROW
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(gl[0]), float(gl[1]), 0.05
            _, _, m.pose.orientation.z, m.pose.orientation.w = ar.quat_from_yaw(gl[2])
            m.scale.x, m.scale.y, m.scale.z = 0.35, 0.06, 0.06
            done = k < self.run.k
            m.color.r, m.color.g, m.color.b, m.color.a = ((0.5, 0.5, 0.5, 0.6) if done else
                                                          (0.1, 0.9, 0.3, 0.9) if kind == 'dig' else (0.9, 0.8, 0.1, 0.9))
            ma.markers.append(m)
        self.pub_goal.publish(ma)


def main():
    rclpy.init()
    rclpy.spin(MissionNode())


if __name__ == '__main__':
    main()
