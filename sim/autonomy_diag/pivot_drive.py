"""Drive the arena.sh rover through pivot tests with /cmd_vel only (sim/autonomy_diag pivot test; no mission).

Steps (sim time from /clock, pose from /ground_truth/base_pose):
  back   reverse at 0.15 m/s until base_link has moved --back m (out of the start zone corner)
  pivot  w = +0.8 rad/s for 8 s, stop 1 s, w = -0.8 rad/s for 8 s, stop 1 s          (case label from --case)
  hole   (with --hole) /arena/dig a 0.25 m, 5 cm hole whose rim sits under the right track, then pivot again
Writes the case time windows to --out (JSON) so pivot_icr.py's windows can be labelled.
Usage: python3 sim/autonomy_diag/pivot_drive.py --case rut --hole --out pivot_cases.json
"""
import argparse
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray

ap = argparse.ArgumentParser()
ap.add_argument('--case', default='rut')
ap.add_argument('--back', type=float, default=0.8)
ap.add_argument('--hole', action='store_true')
ap.add_argument('--out', required=True)
a = ap.parse_args()
rclpy.init()
n = rclpy.create_node('pivot_drive')
st = {'t': None, 'p': None}
n.create_subscription(Clock, '/clock', lambda m: st.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), 10)


def on_gt(m):
    q = m.pose.orientation
    st['p'] = (m.pose.position.x, m.pose.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


n.create_subscription(PoseStamped, '/ground_truth/base_pose', on_gt, 10)
cmd = n.create_publisher(Twist, '/cmd_vel', 10)
dig = n.create_publisher(Float32MultiArray, '/arena/dig', 10)
while st['t'] is None or st['p'] is None:
    rclpy.spin_once(n, timeout_sec=0.1)
cases = []


def drive(v, w, dur=None, until=None):
    t0 = st['t']
    m = Twist()
    m.linear.x, m.angular.z = float(v), float(w)
    while (dur is None or st['t'] - t0 < dur) and (until is None or not until()):
        cmd.publish(m)
        rclpy.spin_once(n, timeout_sec=0.05)
    return t0, st['t']


def stop(dur=1.0):
    drive(0.0, 0.0, dur)


def pivots(label):
    for w in (0.8, -0.8):
        t0, t1 = drive(0.0, w, 8.0)
        cases.append({'case': label, 'w': w, 't0': round(t0, 2), 't1': round(t1, 2), 'start': [round(v, 3) for v in st['p']]})
        print(f'{label} w {w:+.1f}: sim {t0:.1f}-{t1:.1f}', flush=True)
        stop()


p0 = st['p']
drive(-0.15, 0.0, until=lambda: math.hypot(st['p'][0] - p0[0], st['p'][1] - p0[1]) >= a.back)
stop()
pivots(a.case)
if a.hole:
    x, y, yaw = st['p']
    hx, hy = x + 0.52 * math.sin(yaw), y - 0.52 * math.cos(yaw)       # right of the robot: rim under the right track
    msg = Float32MultiArray()
    msg.data = [float(hx), float(hy), 0.25, 0.05]
    for _ in range(3):
        dig.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.1)
    print(f'hole at ({hx:.2f}, {hy:.2f}), robot at ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}', flush=True)
    cases.append({'case': 'hole_dug', 'xy': [round(hx, 3), round(hy, 3)], 't': round(st['t'], 2)})
    stop(2.0)
    pivots('hole')
json.dump(cases, open(a.out, 'w'), indent=1)
print('done', flush=True)
