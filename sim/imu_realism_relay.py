"""IMU realism relay: /sim/imu_ideal (Isaac's ideal D455 IMU) -> /camera/imu with BMI055-class noise and per-run biases.

Isaac publishes the IMU from an OmniGraph at every 200 Hz physics step, which Python in the sim loop cannot reach, so
the noise (sim/realism.py ImuRealism: gyro 0.014 deg/s/sqrt(Hz), accel 150 ug/sqrt(Hz), turn-on biases drawn once per
run) is added here, one message in, one out, header unchanged. Started by run/sim/arena/arena.sh when --realism is
not off.
    python3 sim/imu_realism_relay.py --ros-args -p level:=nominal -p seed:=0
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import realism  # noqa: E402


class Relay(Node):
    def __init__(self):
        super().__init__('imu_realism_relay')
        level = self.declare_parameter('level', 'nominal').value
        seed = int(self.declare_parameter('seed', 0).value)
        self.imu = realism.ImuRealism(level, np.random.default_rng(seed + 505))
        self.pub = self.create_publisher(Imu, '/camera/imu', 50)    # reliable, as Isaac's own IMU publisher
        self.create_subscription(Imu, '/sim/imu_ideal', self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'IMU realism {level}: gyro bias {np.degrees(self.imu.bg).round(3)} deg/s, '
                               f'accel bias {self.imu.ba.round(3)} m/s^2')

    def cb(self, m):
        g, a = self.imu.apply([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z],
                              [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, g)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, a)
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Relay())


if __name__ == '__main__':
    main()
