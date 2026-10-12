#!/usr/bin/env python3
"""Test driver for the simulated arena (sim only): mission-cycle-like survey loops.

Plans with A* around the round's boulders, craters and the column (known from the
shared layout seed) and follows the path using the simulator's ground-truth pose.
It only exercises SLAM / terrain mapping; it is not the robot's autonomy and SLAM
never sees its inputs. Parameters: seed, laps, speed.
"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as A  # noqa: E402

# Excavation zone -> along the far wall -> construction zone / berm -> back (one lap).
WAYPOINTS = [(1.3, 1.3), (1.2, 3.9), (3.0, 4.3), (5.9, 4.0), (6.2, 2.4), (5.6, 0.9),
             (4.3, 0.9), (2.6, 1.1)]


class ArenaDriver(Node):
    def __init__(self):
        super().__init__('arena_driver')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.speed, laps = p('speed', 0.3), p('laps', 2)
        boulders, craters = A.layout(p('seed', 0))
        self.occ = A.occupancy(boulders, craters, inflate=0.40)
        self.goals = WAYPOINTS * laps + [WAYPOINTS[0]]
        self.path = []
        self.pose = None
        self.start_time = None
        self.create_subscription(PoseStamped, '/ground_truth/base_pose', self.on_pose, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def replan(self):
        while self.goals:
            goal = A.nearest_free(self.occ, self.goals[0])
            path = A.plan(self.occ, self.pose[:2], goal) if goal else None
            if path:
                self.path = path[3::3] + [path[-1]]
                self.get_logger().info(f'heading to {self.goals[0]} ({len(path) * A.PLAN_RES:.1f} m path)')
                return
            self.get_logger().warn(f'waypoint {self.goals[0]} unreachable this round; skipping')
            self.goals.pop(0)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose is None or now == 0:
            return
        if self.start_time is None:
            self.start_time = now
        cmd = Twist()
        if now - self.start_time < 5.0 or (not self.goals and not self.path):  # IMU init / done
            self.pub.publish(cmd)
            return
        if not self.path:
            self.replan()
            if not self.path:
                self.get_logger().info('survey done')
                return
        x, y, yaw = self.pose
        while self.path and math.hypot(self.path[0][0] - x, self.path[0][1] - y) < 0.12:
            self.path.pop(0)
        if not self.path:
            self.get_logger().info(f'reached {self.goals.pop(0)} at sim t={now - self.start_time:.0f} s')
            if not self.goals:
                self.get_logger().info('survey done')
            return
        gx, gy = self.path[0]
        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        cmd.angular.z = max(-0.4, min(0.4, 1.5 * err))
        cmd.linear.x = self.speed * math.cos(err) if abs(err) < 0.5 else 0.0   # pivot first (skid steer)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ArenaDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
