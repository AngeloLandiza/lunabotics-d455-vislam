#!/usr/bin/env python3
"""Drive the simulated rover around a square survey loop (sim only).

Steers to the square's corners using the simulator's ground-truth pose
(/ground_truth/odom), so the loop closes regardless of wheel slip in regolith.
SLAM never sees this topic. Parameters: side (m), speed (m/s), laps.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SurveyDriver(Node):
    def __init__(self):
        super().__init__('survey_driver')
        p = lambda n, d: self.declare_parameter(n, d).value
        side, self.speed, laps = p('side', 2.5), p('speed', 0.3), p('laps', 2)
        corners = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
        self.waypoints = corners * laps
        self.pose = None
        self.start_time = None
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        cmd = Twist()
        if self.pose is None or now == 0:
            return
        if self.start_time is None:
            self.start_time = now
        if now - self.start_time < 5.0 or not self.waypoints:  # stand still for IMU init / done
            self.pub.publish(cmd)
            return
        x, y, yaw = self.pose
        gx, gy = self.waypoints[0]
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.15:
            self.waypoints.pop(0)
            self.get_logger().info(f'reached ({gx:.1f}, {gy:.1f}) at sim t={now - self.start_time:.0f} s; '
                                   f'{len(self.waypoints)} waypoints left')
            if not self.waypoints:
                self.get_logger().info('survey done')
            return
        err = math.atan2(gy - y, gx - x) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        cmd.angular.z = max(-0.4, min(0.4, 1.5 * err))
        # Skid steer: pivot in place (both sides counter-rotate) when far off heading;
        # a tight arc holds the inner wheels still and they anchor against the scrub.
        cmd.linear.x = self.speed * math.cos(err) if abs(err) < 0.5 else 0.0
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SurveyDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C / ros2 launch stop
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
