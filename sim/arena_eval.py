#!/usr/bin/env python3
"""Arena SLAM / terrain-mapping check against the simulator's ground truth.

  1. survey lap (the terrain map's baseline)
  2. dig two holes beside the route and dump a pile in the berm box (/arena/dig)
  3. second lap, then report:
     - SLAM camera pose error vs truth over the run (position, tilt)
     - terrain map error vs the true heightmap
     - dug / dumped volume: SLAM (/terrain/volume) vs truth, counting only changes > 3 cm
       like the mapper does

Run with the arena sim + SLAM up (run/sim/arena/arena.sh --eval does all of it).
Parameters: seed (layout, must match the sim), laps_per_phase.
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import arena as A  # noqa: E402
from dig_tool import pose_to_T  # noqa: E402

EDITS = [(2.2, 3.6, 0.25, 0.12), (5.6, 3.2, 0.25, 0.12), (5.4, 0.9, 0.30, -0.10)]   # x, y, r, depth


class Eval(Node):
    def __init__(self):
        super().__init__('arena_eval')
        self.seed = self.declare_parameter('seed', 0).value
        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.H = self.slam_vol = self.cloud = self.T = None
        self.hist, self.err = deque(), []
        self.create_subscription(Image, '/arena/heightmap', self.on_height, latched)
        self.create_subscription(Float64MultiArray, '/terrain/volume', lambda m: setattr(self, 'slam_vol', list(m.data)), 1)
        self.create_subscription(PointCloud2, '/terrain/cloud', lambda m: setattr(self, 'cloud', m), 1)
        self.create_subscription(PoseStamped, '/ground_truth/camera_pose', self.on_cam, 50)
        self.dig = self.create_publisher(Float32MultiArray, '/arena/dig', 10)

    def on_height(self, m):
        self.H = np.frombuffer(m.data, np.float32).reshape(m.height, m.width).copy()

    def on_cam(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.hist.append((pose_to_T((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), Time.from_msg(m.header.stamp)))
        while len(self.hist) > 2 and (self.hist[-1][1] - self.hist[0][1]).nanoseconds > 7e8:
            T_gt, stamp = self.hist.popleft()
            try:
                t = self.tf.lookup_transform('map', 'camera_link', stamp)
            except Exception:
                continue
            a, b = t.transform.translation, t.transform.rotation
            Tm = pose_to_T((a.x, a.y, a.z), (b.x, b.y, b.z, b.w))
            if self.T is None:
                self.T = T_gt @ np.linalg.inv(Tm)       # arena <- map, from one instant of truth
            est = self.T @ Tm
            zt, ze = T_gt[:3, :3].T @ [0, 0, 1], est[:3, :3].T @ [0, 0, 1]
            self.err.append((stamp.nanoseconds * 1e-9, float(np.linalg.norm(est[:3, 3] - T_gt[:3, 3])),
                             math.degrees(math.acos(float(np.clip(zt @ ze, -1, 1))))))

    def map_error(self):
        m = self.cloud
        off = {f.name: f.offset // 4 for f in m.fields}
        a = np.frombuffer(m.data, np.float32).reshape(-1, m.point_step // 4)
        P = np.c_[a[:, [off['x'], off['y'], off['elevation']]], np.ones(len(a))] @ self.T.T
        xs, ys = A.grid()
        i = np.round((P[:, 0] - xs[0]) / A.RES).astype(int)
        j = np.round((P[:, 1] - ys[0]) / A.RES).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < len(xs)) & (j < len(ys))
        e = np.abs(P[ok, 2] - self.H[i[ok], j[ok]])
        return float(np.median(e)), float(np.percentile(e, 90)), int(ok.sum()) * 0.0025


def main():
    rclpy.init(args=sys.argv)
    n = Eval()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    def wait(cond, what, timeout=600):
        t = time.time()
        while not cond():
            if time.time() - t > timeout:
                raise SystemExit(f'timed out waiting for {what}')
            time.sleep(0.5)

    def lap():
        subprocess.run([sys.executable, os.path.join(HERE, 'arena_driver.py'), '--ros-args', '-p', 'use_sim_time:=true',
                        '-p', f'seed:={n.seed}', '-p', 'laps:=1'], check=False)

    b, c = A.layout(n.seed)
    H0 = A.heightmap(b, c, n.seed)[2]
    wait(lambda: n.H is not None and n.T is not None and n.cloud is not None and n.slam_vol is not None,
         'sim heightmap, SLAM alignment and terrain map')
    print('[eval] lap 1 (baseline survey)', flush=True)
    lap()
    time.sleep(12)                                      # let /terrain/volume (5 s period) catch up
    true0, slam0 = A.volumes(n.H, H0, 0.03), list(n.slam_vol)
    print(f'[eval] after lap 1: map |error| median/90% = {n.map_error()[0] * 100:.1f}/{n.map_error()[1] * 100:.1f} cm',
          flush=True)
    for e in EDITS:
        n.dig.publish(Float32MultiArray(data=list(e)))
        time.sleep(1.0)
    time.sleep(3)
    true1 = A.volumes(n.H, H0, 0.03)
    print('[eval] dug 2 holes beside the route, dumped on the berm; lap 2', flush=True)
    lap()
    time.sleep(12)
    true2, slam2 = A.volumes(n.H, H0, 0.03), list(n.slam_vol)
    med, p90, area = n.map_error()
    err = np.array(n.err)
    res = {
        'pose_error_m': {'median': float(np.median(err[:, 1])), 'max': float(err[:, 1].max()),
                         'final': float(err[-1, 1])},
        'tilt_error_deg': {'median': float(np.median(err[:, 2])), 'p95': float(np.percentile(err[:, 2], 95))},
        'map_error_cm': {'median': med * 100, 'p90': p90 * 100, 'area_m2': area},
        'dug_L': {'true': (true2[0] - true0[0]) * 1000, 'slam': (slam2[0] - slam0[0]) * 1000},
        'dumped_L': {'true': (true2[1] - true0[1]) * 1000, 'slam': (slam2[1] - slam0[1]) * 1000,
                     'true_in_berm_box': (true2[2] - true0[2]) * 1000},
        'edit_volume_check_L': (true1[0] - true0[0]) * 1000,
    }
    print('[eval] RESULT ' + json.dumps(res), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
