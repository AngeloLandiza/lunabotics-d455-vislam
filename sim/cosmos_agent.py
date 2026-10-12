#!/usr/bin/env python3
"""NVIDIA Cosmos 3 Edge as the decision maker of a full Lunabotics run, on top of SLAM.

Every step the model gets the rover's front camera image plus a status report built from
the SLAM stack (lunabotics_slam/launch/lunabotics.launch.py): the SLAM pose, the obstacles the segmenter
found on the SLAM terrain map, the berm volume the terrain map measured, hazards from the
depth camera, time left and bucket state. It answers with one high-level action, which
this node executes with the arena rules:

    go_to X Y [H]  A* path on the SLAM obstacle grid (/terrain/obstacle_grid), then follow it
                (and turn to heading H at the end); stops at depth-camera hazards and walls
    turn DEG    pivot in place (+ = counter-clockwise)
    drive M     straight (- = reverse), max 1.5 m
    dig         excavation/starting zone only: lowers the ground under the rover's front and
                fills the bucket (no moving parts, just a terrain edit via /arena/dig)
    dump        drops the bucket's load as a pile 0.65 m in front of the rover (construction zone only)
    wait S      do nothing

Localization: the SLAM map frame is tied to the arena frame once at the start, from one
ground-truth camera pose (the stand-in for localizing on the start-zone fiducials, which
the rules allow); after that the robot only knows where it is from SLAM. With
localization:=ground_truth the sim's true pose is used instead and the planner runs on
an empty map (depth-camera safety stop only).

Rules from the 2027 guidebook (sec. 11-17): 15 min run, dig only in the excavation and
starting zones, only regolith inside the 2.2 x 0.9 m berm box counts, -50 / -25 (max -100)
for each boulder contact or crater crossing in the obstacle zone, full autonomy (1400 pts)
needs >= 2 dig -> dump cycles and is scaled by min(1, berm / 25,000 cm3). The judges'
side (penalties, berm volume on the score card) uses ground truth.

Model: sim/cosmos_server.py at param `url`. Each step is saved to ~/.ros/cosmos_runs/<time>/.
"""
import base64
import collections
import json
import math
import os
import re
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import requests
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as A  # noqa: E402

BUCKET = 0.010                 # m^3 per load (~15 kg of BP-1); the guidebook's example run
                               # builds 50,000-77,000 cm^3 in 8 cycles
# A shallow scoop in front of the rover: (ahead, left) of its centre, m. Spread over ~0.5 x 0.75 m
# the load is a 3 cm deep dig (measured with arena.dig); four strokes in a line made a 12 cm
# trench the rover then bogged down in.
DIG_STROKES = tuple((a, l) for a in (0.2, 0.45, 0.7) for l in (-0.13, 0.13))
DIG_RADIUS, DIG_SECONDS = 0.24, 20.0
DUMP_AHEAD, DUMP_RADIUS, DUMP_SECONDS = 0.65, 0.24, 10.0   # pile stays clear of the front edge after
                                                          # slumping; closer, the rover sits on it and can't pivot
BRUSH = 0.2974                 # raised-cosine brush volume = BRUSH * pi r^2 * depth
SPEED, TURN_RATE = 0.30, 0.5   # m/s, rad/s
WALL_MARGIN = 0.05             # m between the rover's front/back edge and a wall
HALF_LEN, HALF_WID = 0.255, 0.215
INFLATE = 0.32                 # m: planner keeps the rover centre this far from obstacles and walls
STOP_RANGE = 0.75              # m from the rover centre: hazards closer than this in its path stop it
HAZARD_MEMORY = 120.0          # s: hazards that stopped the rover stay in the planning grid this long
CLEAR_RADIUS = 0.8             # m: planner recovery ignores SLAM obstacle cells this close to the rover
HAZ_HIGH, HAZ_LOW = 0.10, -0.08   # m above / below the local ground = boulder, wall / crater
MIN_CHANGE = 0.03              # m: terrain-map change that counts as dumped (lunabotics_terrain.yaml)
# Suggested targets, cycled per dig / dump (the model may pick others): fresh ground in the
# excavation zone, and dump spots along the berm box (the rover stands DUMP_AHEAD before them,
# facing +x). The box is filled far to near, so every stand point is short of all earlier piles
# (near to far put the next stand point on the last pile); after that, loads stack on the
# nearest pile, still reachable from 4.05.
DIG_SPOTS = [(1.2, 1.0), (1.7, 1.0), (1.2, 1.5), (1.7, 1.5), (1.2, 0.6), (1.7, 0.6), (0.9, 1.9), (1.5, 1.9)]
DUMP_SPOTS = [(6.1, 0.9), (5.6, 0.9), (5.1, 0.9), (4.7, 0.9)]


def dump_spot(n_dumps):
    return DUMP_SPOTS[min(n_dumps, len(DUMP_SPOTS) - 1)]

SYSTEM = f"""You are the autonomous controller of a NASA Lunabotics excavation rover in the Artemis Arena, \
for one 15-minute competition run. Goal: build the largest berm. Repeat cycles: dig in the excavation zone \
(one dig fills the bucket), carry the load through the obstacle zone, dump it inside the berm target box, \
go back for more. More cycles = more points; at least 2 complete cycles are needed.

Arena (meters): x runs along the 6.88 m length, y across the 5.0 m width, origin at the starting-zone corner.
- Starting zone: x 0-2, y 0-2. Excavation zone: x 0-2.5, all y. Digging is ONLY allowed in these.
- Obstacle zone: x 2.5-6.88. Boulders (30-40 cm) and craters (40-50 cm wide) at unknown positions. \
Touching a boulder or crossing a crater costs points.
- Construction zone: x 3.88-6.88, y 0-1.5. Berm target box: x 4.28-6.48, y 0.45-1.35 (center 5.38, 0.90). \
Only regolith dumped inside the box counts; dumps outside the construction zone are refused. A dump lands \
{DUMP_AHEAD} m in front of the rover, so stop about {DUMP_AHEAD} m short of the spot and face it. \
Spread dumps along the box instead of one spot.
- Central column (0.46 m wide, 1 m tall) at x 3.44, y 2.50. Walls all around: never drive into them.
Rover: tank drive, 0.51 x 0.43 m. Heading 0 deg = +x, 90 deg = +y (counter-clockwise positive).
The rover runs SLAM: its pose, the obstacles on its map and the measured berm volume are in the status \
report. go_to plans a path around the mapped obstacles by itself.

Each step you see the rover's front camera (grayscale, pitched down) and the status report with a suggested \
target. Think in a few short sentences (check the obstacles and the last result), then end with exactly one line:
ACTION: go_to X Y [H]  (plan a path around mapped obstacles, drive to the point, optionally end facing
                       heading H deg; stops early at a hazard)
ACTION: turn DEG    (pivot in place, positive = left)
ACTION: drive M     (straight, negative = reverse, at most 1.5)
ACTION: dig         (only in the excavation zone, fills the bucket, about 20 s)
ACTION: dump        (empties the bucket in front of the rover, about 10 s)
ACTION: wait S"""

ACTION_RE = re.compile(r'ACTION:\s*(?:go\s+)?(go_to|goto|turn|drive|dig|dump|wait)\b([^\n]*)', re.I)


def yaw_of_T(T):
    return math.atan2(T[1, 0], T[0, 0])


def pose_T(p, q):
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def tf_T(tf):
    return pose_T(tf.transform.translation, tf.transform.rotation)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def zones_at(x, y):
    names = [z for z in ('starting', 'excavation', 'obstacle', 'construction')
             if A.in_box(x, y, A.ZONES['start' if z == 'starting' else z])]
    if A.in_box(x, y, A.ZONES['berm']):
        names.append('berm box')
    return '+'.join(names) or 'outside'


def parse_action(text):
    """Last 'ACTION: ...' line -> (name, [numbers]) or None."""
    found = ACTION_RE.findall(text or '')
    if not found:
        return None
    name, rest = found[-1]
    name = 'go_to' if name.lower() == 'goto' else name.lower()
    nums = [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?', rest)]
    need = {'go_to': 2, 'turn': 1, 'drive': 1, 'wait': 1}.get(name, 0)
    if len(nums) < need:
        return None
    return name, nums[:3 if name == 'go_to' else need]   # go_to: optional final heading


def trouble(last_result):
    return last_result.startswith(('REFUSED', 'STOPPED', 'TIMED OUT')) or 'PENALTY' in last_result


def ahead_of(x, y, yaw, ahead, left=0.0):
    return (x + ahead * math.cos(yaw) - left * math.sin(yaw), y + ahead * math.sin(yaw) + left * math.cos(yaw))


def dig_problem(x, y, yaw):
    """Why digging here would be wrong, judged where the strokes land (not where the rover is)."""
    pts = [ahead_of(x, y, yaw, *s) for s in DIG_STROKES]
    if not all(0.15 < px < A.LENGTH - 0.15 and 0.15 < py < A.WIDTH - 0.15 for px, py in pts):
        return 'the dig would reach the wall/duct ahead; turn away from the wall or move out first'
    if not all(A.in_box(px, py, A.ZONES['excavation']) for px, py in pts):
        return 'digging is only allowed in the excavation/starting zone (x < 2.5); the dig would reach outside it'
    return None


def disk(radius_cells):
    r = int(math.ceil(radius_cells))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy <= radius_cells * radius_cells).astype(np.uint8)


class CosmosAgent(Node):
    def __init__(self):
        super().__init__('cosmos_agent')
        p = lambda n, d: self.declare_parameter(n, d).value
        self.seed = p('seed', 0)
        self.run_seconds = 60.0 * float(self.declare_parameter(   # accepts 5 or 5.0
            'minutes', 15.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.url = p('url', 'http://127.0.0.1:8000/v1/chat/completions')
        # Reasoning budget per decision (0 = answer directly). Measured with sim/cosmos_bench.py:
        # with the explicit status report, answering directly was as good as reasoning on routine
        # steps and 7x faster, so reasoning is only spent after something went wrong.
        self.budget = p('think_tokens', 0)
        self.trouble_budget = p('trouble_think_tokens', 128)
        self.slam = p('localization', 'slam') == 'slam'
        self.out = os.path.expanduser(time.strftime(p('out_dir', '~/.ros/cosmos_runs/%Y%m%d_%H%M%S')))
        os.makedirs(self.out, exist_ok=True)
        self.boulders, self.craters = A.layout(self.seed)   # for the judges' penalty count only

        self.pose = self.cam_T = self.image = self.depth = self.K = None
        self.gt_pose = self.gt_cam = None
        self.gt_hist = collections.deque()       # (stamp, T_arena_cam, T_arena_base) for the alignment
        self.T_arena_map = self.T_base_cam = None
        self.pose_err = []                       # SLAM vs truth, m (sampled ~1 Hz)
        self.last_err_t = 0.0
        self.obstacles = []                      # SLAM segmenter: (kind, label, x, y) in the arena frame
        self.occ = None                          # inflated arena-frame planning grid from SLAM
        self.remembered = []                     # (time, xs, ys): depth hazards that stopped the rover
        self.dug = []                            # (x, y): own dig strokes; their trench is drivable
        self.slam_berm = None                    # m^3 of fill inside the berm box, SLAM terrain map
        self.volume = (0.0, 0.0, 0.0)            # truth [cut, fill, berm] (judges)
        self.load = 0.0
        self.t0 = None
        self.action = None                       # dict while executing
        self.thinking = False
        self.history = []                        # (action text, result)
        self.last_result = 'none yet (run just started)'
        self.cycles, self.dug_since_dump = 0, False
        self.penalty, self.touching, self.penalty_note = 0, set(), ''
        self.step = 0
        self.n_digs = self.n_dumps = 0
        self.done = False
        # Arena bounds only: the guidebook rules out the walls and the column as a-priori map
        # features for travel autonomy, so the column has to come from SLAM like the rest.
        nx, ny = int(A.LENGTH / A.PLAN_RES) + 1, int(A.WIDTH / A.PLAN_RES) + 1
        X, Y = np.meshgrid(np.arange(nx) * A.PLAN_RES, np.arange(ny) * A.PLAN_RES, indexing='ij')
        self.walls = (X < INFLATE) | (Y < INFLATE) | (X > A.LENGTH - INFLATE) | (Y > A.WIDTH - INFLATE)
        self.grid_x, self.grid_y = X, Y
        self.odom_lost, self.last_tick = False, None   # SLAM visual odometry lost (null /odom)

        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, '/ground_truth/base_pose', self.on_gt_base, 10)
        self.create_subscription(PoseStamped, '/ground_truth/camera_pose', self.on_gt_cam, 10)
        self.create_subscription(Image, '/camera/infra1/image_rect_raw',
                                 lambda m: setattr(self, 'image', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 lambda m: setattr(self, 'depth', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info',
                                 lambda m: setattr(self, 'K', np.array(m.k).reshape(3, 3)), qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/arena/volume',
                                 lambda m: setattr(self, 'volume', tuple(m.data[:3])), latched)
        if self.slam:
            self.create_subscription(MarkerArray, '/terrain/obstacles', self.on_obstacles, 1)
            self.create_subscription(OccupancyGrid, '/terrain/obstacle_grid', self.on_grid, 1)
            self.create_subscription(PointCloud2, '/terrain/cloud', self.on_terrain, 1)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(Float64MultiArray, '/terrain/volume',
                                     lambda m: setattr(self, 'slam_volume', tuple(m.data[:2])), 1)
        self.slam_volume = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dig_pub = self.create_publisher(Float32MultiArray, '/arena/dig', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'run log: {self.out}  (localization: {"SLAM" if self.slam else "ground truth"})')

    # ---------------------------------------------------------------- localization
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt_base(self, msg):
        T = pose_T(msg.pose.position, msg.pose.orientation)
        self.gt_pose = (T[0, 3], T[1, 3], yaw_of_T(T))
        if not self.slam:
            self.pose = self.gt_pose
        self.judge_obstacles()
        if self.slam and self.T_arena_map is None and self.gt_cam is not None:
            self.gt_hist.append((Time.from_msg(msg.header.stamp), self.gt_cam, T))
            while len(self.gt_hist) > 200:
                self.gt_hist.popleft()

    def on_gt_cam(self, msg):
        self.gt_cam = pose_T(msg.pose.position, msg.pose.orientation)
        if not self.slam:
            self.cam_T = self.gt_cam

    def on_odom(self, msg):
        lost = msg.pose.covariance[0] >= 9999   # stereo odometry publishes a null pose while lost
        if lost != self.odom_lost:
            self.get_logger().warn('SLAM tracking lost: rover paused' if lost else 'SLAM tracking back')
        self.odom_lost = lost

    def align(self):
        """Tie the SLAM map frame to the arena once (start-zone localization stand-in)."""
        while len(self.gt_hist) > 1 and (self.gt_hist[-1][0] - self.gt_hist[0][0]).nanoseconds > 5e8:
            stamp, T_cam, T_base = self.gt_hist.popleft()   # SLAM publishes a frame ~0.5 s late
            try:
                T_map_cam = tf_T(self.tf.lookup_transform('map', 'camera_link', stamp))
            except Exception:
                continue
            self.T_arena_map = T_cam @ np.linalg.inv(T_map_cam)
            self.T_base_cam = np.linalg.inv(T_base) @ T_cam
            self.get_logger().info('SLAM map aligned to the arena (one ground-truth camera pose)')
            self.gt_hist.clear()
            return

    def update_slam_pose(self):
        if self.T_arena_map is None:
            return
        try:
            T_map_cam = tf_T(self.tf.lookup_transform('map', 'camera_link', Time()))
        except Exception:
            return
        self.cam_T = self.T_arena_map @ T_map_cam
        T_base = self.cam_T @ np.linalg.inv(self.T_base_cam)
        self.pose = (T_base[0, 3], T_base[1, 3], yaw_of_T(T_base))
        now = self.now()
        if self.gt_pose and now - self.last_err_t >= 1.0 and self.t0 is not None and not self.done:
            self.last_err_t = now
            self.pose_err.append(math.hypot(self.pose[0] - self.gt_pose[0], self.pose[1] - self.gt_pose[1]))

    def to_arena(self, frame, x, y):
        """Points in a SLAM frame (map) -> arena frame."""
        if self.T_arena_map is None or frame not in ('map', ''):
            return None
        T = self.T_arena_map
        return T[0, 0] * x + T[0, 1] * y + T[0, 3], T[1, 0] * x + T[1, 1] * y + T[1, 3]

    # ---------------------------------------------------------------- SLAM map
    def on_obstacles(self, msg):
        found = []
        for m in msg.markers:
            if m.type != Marker.TEXT_VIEW_FACING or m.ns == 'wall':
                continue
            p = self.to_arena(m.header.frame_id, m.pose.position.x, m.pose.position.y)
            if p:
                found.append((m.ns, m.text, p[0], p[1]))
        self.obstacles = found

    def on_grid(self, g):
        """SLAM obstacle grid (map frame) -> inflated planning grid in the arena frame."""
        if self.T_arena_map is None:
            return
        w, h, r = g.info.width, g.info.height, g.info.resolution
        data = np.asarray(g.data, np.int8).reshape(h, w)            # rows = y, cols = x
        jj, ii = np.nonzero(data >= 100)
        occ = np.zeros_like(self.walls, np.uint8)
        if len(ii):
            mx = g.info.origin.position.x + (ii + 0.5) * r
            my = g.info.origin.position.y + (jj + 0.5) * r
            ax, ay = self.to_arena(g.header.frame_id, mx, my)
            ci = np.round(ax / A.PLAN_RES).astype(int)
            cj = np.round(ay / A.PLAN_RES).astype(int)
            ok = (ci >= 0) & (ci < occ.shape[0]) & (cj >= 0) & (cj < occ.shape[1])
            for dx_, dy_ in self.dug:   # the segmenter maps the own trench as a 'dug pit'
                ok &= np.hypot(ax - dx_, ay - dy_) > DIG_RADIUS + 0.1
            occ[ci[ok], cj[ok]] = 1
            occ = cv2.dilate(occ, disk(INFLATE / A.PLAN_RES))
        self.occ = occ.astype(bool) | self.walls

    def on_terrain(self, msg):
        """Fill inside the berm box, as measured by SLAM's terrain map."""
        if self.T_arena_map is None or msg.width == 0:
            return
        off = {f.name: f.offset // 4 for f in msg.fields}
        a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
        ax, ay = self.to_arena(msg.header.frame_id, a[:, off['x']], a[:, off['y']])
        x0, y0, x1, y1 = A.ZONES['berm']
        inside = (ax >= x0) & (ax <= x1) & (ay >= y0) & (ay <= y1)
        ch = a[:, off['change']][inside]
        self.slam_berm = float(np.where(ch > MIN_CHANGE, ch, 0).sum() * 0.05 * 0.05)   # 5 cm cells

    def remember(self, haz):
        """Put the depth hazards that just stopped the rover into the planning grid, so the next
        plan goes around them even when SLAM hasn't mapped them (yet)."""
        x, y, yaw = self.pose
        pts = [(f, l) for f, l, _ in haz or [] if f < 2.0]
        if pts:
            f, l = np.array(pts).T
            self.remembered.append((self.now(), x + f * math.cos(yaw) - l * math.sin(yaw),
                                    y + f * math.sin(yaw) + l * math.cos(yaw)))

    def plan_path(self, goal):
        """A* on the SLAM grid + remembered depth hazards; returns (path, goal used, note) or
        (None, why, '').

        If that boxes the rover in (mapping artifacts right next to it are common, e.g. just
        after a dump), retry like a costmap-clearing recovery: first without the SLAM cells
        within CLEAR_RADIUS of the rover, then with only the walls and remembered hazards. The
        depth-camera stop still guards every move.
        """
        self.remembered = [r for r in self.remembered if self.now() - r[0] < HAZARD_MEMORY]
        hazard = np.zeros(self.walls.shape, np.uint8)
        for _, hx, hy in self.remembered:
            ci, cj = np.round(hx / A.PLAN_RES).astype(int), np.round(hy / A.PLAN_RES).astype(int)
            ok = (ci >= 0) & (ci < hazard.shape[0]) & (cj >= 0) & (cj < hazard.shape[1])
            hazard[ci[ok], cj[ok]] = 1
        hazard = cv2.dilate(hazard, disk(INFLATE / A.PLAN_RES)).astype(bool) if self.remembered else hazard.astype(bool)
        slam = self.occ if self.occ is not None else self.walls
        x, y, _ = self.pose
        near = np.hypot(self.grid_x - x, self.grid_y - y) < CLEAR_RADIUS
        why = ''
        for occ, note in [(slam | hazard, ''),
                          ((slam & ~near) | self.walls | hazard,
                           f'recovery: planned without the SLAM map cells within {CLEAR_RADIUS} m of the rover '
                           '(they boxed it in, likely mapping artifacts)'),
                          (self.walls | hazard,
                           'recovery: the SLAM obstacle map blocked every way, so this path only avoids walls and '
                           'hazards seen by the depth camera')]:
            path, used = self.astar(occ.copy(), goal)
            if path is not None:
                return path, used, note
            why = used
        return None, why, ''

    def path_blocked(self, path):
        """Any remaining waypoint inside the current SLAM obstacles or remembered hazards?"""
        occ = self.occ if self.occ is not None else self.walls
        for _, hx, hy in self.remembered:
            if any(math.hypot(px - qx, py - qy) < INFLATE for px, py in path for qx, qy in zip(hx[::5], hy[::5])):
                return True
        for px, py in path:
            i, j = int(round(px / A.PLAN_RES)), int(round(py / A.PLAN_RES))
            if 0 <= i < occ.shape[0] and 0 <= j < occ.shape[1] and occ[i, j]:
                return True
        return False

    def astar(self, occ, goal):
        x, y, _ = self.pose
        i, j = int(round(x / A.PLAN_RES)), int(round(y / A.PLAN_RES))
        occ[max(i - 4, 0):i + 5, max(j - 4, 0):j + 5] = False            # never boxed in by its own cell
        g = goal
        gi, gj = int(round(g[0] / A.PLAN_RES)), int(round(g[1] / A.PLAN_RES))
        if not (0 <= gi < occ.shape[0] and 0 <= gj < occ.shape[1]) or occ[gi, gj]:
            g = A.nearest_free(occ, goal, max_dist=0.6)
            if g is None:
                return None, f'({goal[0]:.2f}, {goal[1]:.2f}) is inside a mapped obstacle or too close to a wall'
        path = A.plan(occ, (x, y), g)
        if path is None:
            return None, f'no path on the SLAM obstacle map to ({goal[0]:.2f}, {goal[1]:.2f})'
        return path[1:] or [g], g

    # ---------------------------------------------------------------- sensing
    def hazards(self):
        """Depth image -> points in the arena frame -> what sticks out of the local ground.

        Returns [(forward, left, kind)] in the rover frame, kind 'high' (boulder, wall,
        column, berm) or 'low' (crater), or None without data. Reactive safety layer.
        """
        if self.depth is None or self.K is None or self.cam_T is None or self.pose is None:
            return None
        try:
            T_link_opt = tf_T(self.tf.lookup_transform('camera_link', 'camera_depth_optical_frame', Time()))
        except Exception:
            return None
        d = np.frombuffer(self.depth.data, np.float32).reshape(self.depth.height, -1)[::6, ::6]
        v, u = np.mgrid[0:self.depth.height:6, 0:self.depth.width:6]
        ok = (d > 0.2) & (d < 3.5)
        z = d[ok]
        pts = np.stack([(u[ok] - self.K[0, 2]) * z / self.K[0, 0], (v[ok] - self.K[1, 2]) * z / self.K[1, 1],
                        z, np.ones_like(z)])
        P = (self.cam_T @ T_link_opt @ pts)[:3]
        x, y, yaw = self.pose
        dx, dy = P[0] - x, P[1] - y
        fwd, left = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
        near = (fwd > 0.3) & (fwd < 1.5) & (np.abs(left) < 1.0)
        if near.sum() < 20:
            return []
        dz = P[2] - np.median(P[2][near])
        kind = np.where(dz > HAZ_HIGH, 1, np.where(dz < HAZ_LOW, -1, 0))
        for dx_, dy_ in self.dug:   # the rover's own ~6-10 cm trench is drivable, not a crater
            kind[(kind < 0) & (np.hypot(P[0] - dx_, P[1] - dy_) < DIG_RADIUS + 0.1)] = 0
        keep = kind != 0
        return [(f, l, 'high' if k > 0 else 'low') for f, l, k in zip(fwd[keep], left[keep], kind[keep])]

    def hazard_report(self, haz):
        if haz is None:
            return 'no data'
        sectors = {'left': (15, 50), 'ahead': (-15, 15), 'right': (-50, -15)}
        parts = []
        for name, (a0, a1) in sectors.items():
            hits = [(math.hypot(f, l), k) for f, l, k in haz if a0 <= math.degrees(math.atan2(l, f)) < a1]
            if hits:
                r, k = min(hits)
                parts.append(f'{name}: {"raised (boulder/wall/column/pile)" if k == "high" else "dip (crater/hole)"} '
                             f'at {r:.1f} m')
            else:
                parts.append(f'{name}: clear')
        return '; '.join(parts)

    def obstacle_report(self):
        if not self.slam:
            return ''
        if self.T_arena_map is None:
            return 'SLAM obstacle map: not ready\n'
        x, y, yaw = self.pose
        near = sorted(self.obstacles, key=lambda o: math.hypot(o[2] - x, o[3] - y))[:6]
        if not near:
            return 'SLAM obstacle map: nothing mapped yet\n'
        lines = [f'- {label} at ({ox:.2f}, {oy:.2f}): {math.hypot(ox - x, oy - y):.1f} m, '
                 f'{math.degrees(wrap(math.atan2(oy - y, ox - x) - yaw)):+.0f} deg'
                 for _, label, ox, oy in near]
        return 'SLAM obstacle map (nearest):\n' + '\n'.join(lines) + '\n'

    def blocked(self, haz, direction):
        """A hazard in the rover's path, or a wall just ahead (direction +1 fwd / -1 back)."""
        x, y, yaw = self.pose
        edge = (HALF_LEN + WALL_MARGIN) * direction
        fx, fy = x + edge * math.cos(yaw), y + edge * math.sin(yaw)
        if not (0.0 < fx < A.LENGTH and 0.0 < fy < A.WIDTH):
            return 'wall ahead' if direction > 0 else 'wall behind'
        if direction > 0 and haz:
            for f, l, k in haz:
                if f < STOP_RANGE and abs(l) < HALF_WID + 0.08:
                    return f'{"raised obstacle" if k == "high" else "dip/crater"} {f:.2f} m ahead'
        return None

    def judge_obstacles(self):
        """Count obstacle-zone contacts like the judges (true pose): boulder touched or crater crossed."""
        x, y, _ = self.gt_pose
        if not A.in_box(x, y, A.ZONES['obstacle']) or self.t0 is None or self.done:
            return
        for i, o in enumerate([*self.boulders, *self.craters]):
            hit = math.hypot(x - o.x, y - o.y) < (o.diameter / 2 + HALF_WID if isinstance(o, A.Boulder)
                                                  else o.diameter / 2)
            if hit and i not in self.touching:
                cost = 50 if self.penalty == 0 else 25
                self.penalty = min(self.penalty + cost, 100)
                what = 'touched a boulder' if isinstance(o, A.Boulder) else 'drove into a crater'
                self.penalty_note += f' | PENALTY: {what} at ({o.x:.1f}, {o.y:.1f})'
                self.get_logger().warn(f'penalty: {what} at ({o.x:.2f}, {o.y:.2f})')
            if hit:
                self.touching.add(i)
            else:
                self.touching.discard(i)

    # ---------------------------------------------------------------- decide
    def status(self, haz):
        x, y, yaw = self.pose
        left = max(self.run_seconds - (self.now() - self.t0), 0.0)
        rel = lambda gx, gy: (math.hypot(gx - x, gy - y), math.degrees(wrap(math.atan2(gy - y, gx - x) - yaw)))
        d_berm, b_berm = rel(5.38, 0.90)
        # Spell out the consequence of the two key actions right here: the small model
        # misreads its state otherwise (e.g. "just dumped" while the bucket is full).
        px, py = x + DUMP_AHEAD * math.cos(yaw), y + DUMP_AHEAD * math.sin(yaw)
        if self.load > 0:
            where = ('INSIDE the berm box' if A.in_box(px, py, A.ZONES['berm']) else
                     'in the construction zone but outside the box' if A.in_box(px, py, A.ZONES['construction'])
                     else 'outside the construction zone (would be refused)')
            bucket = (f'FULL ({self.load * 1e6:,.0f} cm3, not dumped yet). A dump right now would land at '
                      f'({px:.2f}, {py:.2f}): {where}')
            sx, sy = dump_spot(self.n_dumps)
            hint = ('dump now' if where.startswith('INSIDE') else
                    f'carry the load to the berm box: go_to {sx - DUMP_AHEAD:.2f} {sy:.2f} 0 (ends facing +x), '
                    f'then dump (the pile lands at ({sx:.2f}, {sy:.2f}))')
        else:
            why = dig_problem(x, y, yaw)
            dig_ok = why is None
            bucket = f'empty. Digging here is {"allowed" if dig_ok else "NOT possible: " + why}'
            sx, sy = DIG_SPOTS[self.n_digs % len(DIG_SPOTS)]
            hint = ('dig now' if dig_ok and math.hypot(sx - x, sy - y) < 0.3 else
                    f'go dig fresh ground in the excavation zone: go_to {sx:.2f} {sy:.2f}, then dig')
        if self.slam:
            berm = (f'{self.slam_berm * 1e6:,.0f} cm3 (SLAM terrain map)' if self.slam_berm is not None
                    else 'not measured yet')
        else:
            berm = f'{self.volume[2] * 1e6:,.0f} cm3'
        recent = '\n'.join(f'- {a} -> {r}' for a, r in self.history[-4:]) or '- none'
        return (f'Time left: {int(left // 60)}:{int(left % 60):02d}\n'
                f'Pose ({"SLAM" if self.slam else "true"}): x={x:.2f} y={y:.2f} heading={math.degrees(yaw):.0f} deg, '
                f'zone: {zones_at(x, y)}\n'
                f'Bucket: {bucket}\n'
                f'Berm volume in the target box: {berm}; complete cycles: {self.cycles}; '
                f'obstacle penalties: -{self.penalty}\n'
                f'Berm box center: {d_berm:.1f} m away, {b_berm:+.0f} deg from your heading.\n'
                f'{self.obstacle_report()}'
                f'Depth camera: {self.hazard_report(haz)}\n'
                f'Recent actions:\n{recent}\n'
                f'Last action result: {self.last_result}\n'
                f'Suggested next action: {hint}')   # last: a small model weighs the end most

    def budget_for(self, last_result):
        """Reason only after trouble: a refused, stopped or timed-out action, or a penalty."""
        return self.trouble_budget if trouble(last_result) else self.budget

    def ask_model(self, text, image_jpg, budget):
        think = budget > 0
        content = [{'type': 'image_url', 'image_url': {'url': 'data:image/jpeg;base64,' + image_jpg}},
                   {'type': 'text', 'text': text}]
        r = requests.post(self.url, json={
            'model': 'cosmos3-edge', 'messages': [{'role': 'system', 'content': SYSTEM},
                                                  {'role': 'user', 'content': content}],
            'max_tokens': budget if think else 32, 'temperature': 0.3,
            'chat_template_kwargs': {'enable_thinking': think},
            # sim/cosmos_server.py extension: stop reasoning after this many tokens and force the
            # answer to start with "ACTION:" (with vLLM, drop these and raise max_tokens).
            'thinking_budget': budget or None, 'answer_prefix': 'ACTION:', 'answer_tokens': 24}, timeout=300)
        r.raise_for_status()
        m = r.json()['choices'][0]['message']
        return m.get('content') or '', m.get('reasoning_content') or ''

    def decide(self, haz):
        """Runs in a worker thread: one model call (plus one strict retry) -> self.action."""
        try:
            self._decide(haz)
        except Exception as e:   # never leave the agent waiting on a dead thread
            self.get_logger().error(f'decision failed: {e!r}')
            self.start_action('wait', [2.0])
        finally:
            self.thinking = False

    def _decide(self, haz):
        self.step += 1
        img = np.frombuffer(self.image.data, np.uint8).reshape(self.image.height, -1)[:, :self.image.width]
        img = cv2.resize(img, (512, 320), interpolation=cv2.INTER_AREA)
        _, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        jpg64 = base64.b64encode(jpg).decode()
        cv2.imwrite(os.path.join(self.out, f'step_{self.step:03d}.jpg'), img)
        text = self.status(haz)
        t = time.time()
        answer = reasoning = ''
        try:
            budget = self.budget_for(self.last_result)
            answer, reasoning = self.ask_model(text, jpg64, budget)
            act = parse_action(answer) or parse_action(reasoning)
            if act is None:   # the model rambled or got cut off: ask again for just the line
                answer, _ = self.ask_model(text + '\n\nReply with only the ACTION line.', jpg64, 0)
                act = parse_action(answer)
        except Exception as e:
            self.get_logger().error(f'model call failed: {e}')
            act = None
        if act is None:
            act = ('wait', [2.0])
        wall = time.time() - t
        name, args = act
        said = (reasoning or answer).strip()
        self.get_logger().info(f'step {self.step} ({wall:.1f} s): {name} {" ".join(f"{a:g}" for a in args)}'
                               f'  | {said.splitlines()[-1][:120] if said else ""}')
        with open(os.path.join(self.out, 'log.jsonl'), 'a') as f:
            f.write(json.dumps({'step': self.step, 'sim_time': self.now() - self.t0, 'model_seconds': wall,
                                'status': text, 'reasoning': reasoning, 'answer': answer,
                                'action': [name, args], 'true_pose': self.gt_pose}) + '\n')
        self.start_action(name, args)

    # ---------------------------------------------------------------- act
    def start_action(self, name, args):
        x, y, yaw = self.pose
        a = {'name': name, 'args': args, 't': self.now(), 'start': (x, y, yaw),
             'text': f'{name} {" ".join(f"{v:g}" for v in args)}'.strip()}
        if name == 'go_to':
            goal = (min(max(args[0], 0.35), A.LENGTH - 0.35), min(max(args[1], 0.35), A.WIDTH - 0.35))
            path, used, note = self.plan_path(goal)
            if path is None:
                return self.finish(a, f'REFUSED: {used}; pick another goal')
            a.update(goal=used, path=path, replanned=self.now(), timeout=120.0, note=note,
                     heading=math.radians(args[2]) if len(args) > 2 else None)
        elif name == 'turn':
            a.update(target=wrap(yaw + math.radians(args[0])), timeout=30.0)
        elif name == 'drive':
            dist = max(min(args[0], 1.5), -1.5)
            a.update(dist=dist, timeout=abs(dist) / SPEED * 2 + 5)
        elif name == 'dig':
            why = dig_problem(x, y, yaw)
            if why:
                return self.finish(a, f'REFUSED: {why}')
            if self.load > 0:
                return self.finish(a, 'REFUSED: the bucket is already full; go dump it in the berm box')
            a.update(strokes=list(DIG_STROKES), timeout=DIG_SECONDS)
        elif name == 'dump':
            if self.load <= 0:
                return self.finish(a, 'REFUSED: the bucket is empty; dig first')
            px, py = x + DUMP_AHEAD * math.cos(yaw), y + DUMP_AHEAD * math.sin(yaw)
            if not A.in_box(px, py, A.ZONES['construction']):   # interlock: don't waste a load
                sx, sy = dump_spot(self.n_dumps)
                return self.finish(a, f'REFUSED: the pile would land at ({px:.2f}, {py:.2f}), outside the construction '
                                      f'zone; go_to ({sx - DUMP_AHEAD:.2f}, {sy:.2f}) and face +x first')
            a.update(timeout=DUMP_SECONDS)
        else:
            a.update(timeout=max(min(args[0], 30.0), 0.5))
        self.action = a

    def finish(self, a, result):
        self.cmd_pub.publish(Twist())
        result, self.penalty_note = result + self.penalty_note, ''
        self.last_result = result
        self.history.append((a['text'], result))
        self.action = None

    def edit_terrain(self, ahead, left, radius, volume):
        """Dig (volume > 0) or deposit (volume < 0) with the sim's terrain brush, at the true pose
        (the bucket acts where the rover really is, whatever SLAM thinks)."""
        depth = volume / (BRUSH * math.pi * radius ** 2)
        msg = Float32MultiArray()
        msg.data = [*map(float, ahead_of(*self.gt_pose, ahead, left)), float(radius), float(depth)]
        self.dig_pub.publish(msg)
        return msg.data[0], msg.data[1]

    def follow(self, a, cmd):
        """Pure pursuit along the planned path; replans on the newest SLAM grid every 2 s."""
        x, y, yaw = self.pose
        gx, gy = a['goal']
        dist = math.hypot(gx - x, gy - y)
        if dist < 0.12 or a.get('arrived'):
            a['arrived'] = True
            if a['heading'] is not None and abs(wrap(a['heading'] - yaw)) > 0.05:   # final turn
                cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 2.0 * wrap(a['heading'] - yaw)))
                return None
            return (f'arrived at ({x:.2f}, {y:.2f}), heading {math.degrees(yaw):.0f} deg'
                    + (f' ({a["note"]})' if a.get('note') else ''))
        if self.now() - a['replanned'] > 2.0:
            # Replan only when the newest map blocks the current path: replanning on every map
            # update made the rover chase a different first waypoint each time and re-pivot
            # instead of driving (a 0.7 m move timed out after 120 s).
            a['replanned'] = self.now()
            if not a.get('note') and self.path_blocked(a['path']):
                path, used, note = self.plan_path(a['goal'])
                if path is None:
                    return f'STOPPED at ({x:.2f}, {y:.2f}): {used}'
                a['path'], a['note'] = path, note
        path = a['path']
        while len(path) > 1 and math.hypot(path[0][0] - x, path[0][1] - y) < 0.35:
            path.pop(0)
        tx, ty = path[0]
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(err) > 0.5:   # pivot first (skid steer)
            cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 2.0 * err))
            return None
        haz = self.hazards()
        why = self.blocked(haz, 1)
        if why:
            self.remember(haz)
            moved = math.hypot(x - a['start'][0], y - a['start'][1])
            return (f'STOPPED after {moved:.2f} m at ({x:.2f}, {y:.2f}): {why}; {dist:.2f} m short of the goal. '
                    'The hazard is now on the map: go_to plans around it')
        cmd.linear.x = SPEED * math.cos(err) * min(1.0, dist / 0.3 + 0.3)
        cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 1.5 * err))
        return None

    def tick(self):
        now = self.now()
        if self.slam:
            if self.T_arena_map is None:
                self.align()
            self.update_slam_pose()
        if self.pose is None or self.gt_pose is None or self.image is None or now == 0 or self.done:
            return
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info(f'run started: {self.run_seconds / 60:.0f} min, start {zones_at(*self.pose[:2])} '
                                   f'at ({self.pose[0]:.2f}, {self.pose[1]:.2f}), heading '
                                   f'{math.degrees(self.pose[2]):.0f} deg')
        over = now - self.t0 >= self.run_seconds
        a = self.action
        dt, self.last_tick = (now - self.last_tick if self.last_tick else 0.0), now
        if self.slam and self.odom_lost and a is not None and a['name'] in ('go_to', 'drive', 'turn'):
            # Don't steer on a frozen pose: stop and hold the action (and its timeout) until
            # tracking is back. If odometry resets instead, the move ends on the new pose.
            self.cmd_pub.publish(Twist())
            a['t'] += dt
            if 'replanned' in a:
                a['replanned'] += dt
            return
        if over and not (a and a['name'] == 'dump'):   # time's up; a dump in progress may finish
            self.cmd_pub.publish(Twist())
            self.done = True
            self.report()
            return
        if a is None:
            if not self.thinking and not over:
                self.thinking = True
                threading.Thread(target=self.decide, args=(self.hazards(),), daemon=True).start()
            return
        elapsed = now - a['t']
        x, y, yaw = self.pose
        cmd = Twist()
        name = a['name']
        if name in ('go_to', 'drive', 'turn'):
            if name == 'turn':
                err = wrap(a['target'] - yaw)
                if abs(err) < 0.05:
                    return self.finish(a, f'done, heading now {math.degrees(yaw):.0f} deg')
                cmd.angular.z = max(-TURN_RATE, min(TURN_RATE, 2.0 * err))
            elif name == 'drive':
                moved = math.hypot(x - a['start'][0], y - a['start'][1])
                if moved >= abs(a['dist']) - 0.03:
                    return self.finish(a, f'done, drove {moved:.2f} m')
                haz = self.hazards()
                why = self.blocked(haz, 1 if a['dist'] > 0 else -1)
                if why:
                    self.remember(haz)
                    return self.finish(a, f'STOPPED after {moved:.2f} m: {why}')
                cmd.linear.x = math.copysign(SPEED, a['dist'])
                cmd.angular.z = 1.5 * wrap(a['start'][2] - yaw)
            else:
                result = self.follow(a, cmd)
                if result:
                    return self.finish(a, result)
            if elapsed > a['timeout']:
                return self.finish(a, f'TIMED OUT at ({x:.2f}, {y:.2f}), heading {math.degrees(yaw):.0f} deg '
                                      '(stuck? try another direction)')
            self.cmd_pub.publish(cmd)
        elif name == 'dig':
            self.cmd_pub.publish(cmd)
            n = len(DIG_STROKES)
            while a['strokes'] and elapsed >= (n - len(a['strokes'])) * DIG_SECONDS / n:   # spread over the dig
                stroke = a['strokes'].pop(0)
                self.edit_terrain(*stroke, DIG_RADIUS, BUCKET / n)
                self.dug.append(ahead_of(x, y, yaw, *stroke))
            if elapsed >= DIG_SECONDS:
                self.load = BUCKET
                self.n_digs += 1
                self.dug_since_dump = True
                self.finish(a, f'dug {BUCKET * 1e6:,.0f} cm3 at ({x:.2f}, {y:.2f}); bucket FULL')
        elif name == 'dump':
            self.cmd_pub.publish(cmd)
            if elapsed >= DUMP_SECONDS:
                px, py = self.edit_terrain(DUMP_AHEAD, 0.0, DUMP_RADIUS, -self.load)
                load, self.load = self.load, 0.0
                self.n_dumps += 1
                inside = A.in_box(px, py, A.ZONES['berm'])   # the judges' view (true pile position)
                if inside and self.dug_since_dump:
                    self.cycles += 1
                self.dug_since_dump = False
                sx, sy = x + DUMP_AHEAD * math.cos(yaw), y + DUMP_AHEAD * math.sin(yaw)
                self.finish(a, f'dumped {load * 1e6:,.0f} cm3 at ({sx:.2f}, {sy:.2f}) by SLAM'
                               f'{"" if inside else " (the berm measurement will show whether it landed in the box)"}')
        else:
            self.cmd_pub.publish(cmd)
            if elapsed >= a['timeout']:
                self.finish(a, 'waited')

    # ---------------------------------------------------------------- score
    def report(self):
        cut, fill, berm = self.volume
        berm_cm3 = berm * 1e6
        autonomy = (1400 if self.cycles >= 2 else 0) * min(1.0, berm_cm3 / 25000)
        lines = [
            '==================== Lunabotics run (Cosmos 3 Edge) ====================',
            f'  localization       {"SLAM" if self.slam else "ground truth"}',
            f'  duration           {self.run_seconds / 60:.0f} min sim time, {self.step} model decisions',
            f'  berm (in box)      {berm_cm3:,.0f} cm3 true'
            + (f', {self.slam_berm * 1e6:,.0f} cm3 measured by SLAM' if self.slam_berm is not None else '')
            + f'   (all dumped: {fill * 1e6:,.0f} cm3, dug: {cut * 1e6:,.0f} cm3)',
            f'  complete cycles    {self.cycles}  (dig -> dump inside the berm box)',
            f'  obstacle penalty   -{self.penalty}',
            f'  autonomy points    {autonomy:,.0f}  (1400 if >= 2 cycles, x min(1, berm / 25,000 cm3))',
        ]
        if self.pose_err:
            e = np.array(self.pose_err)
            lines.append(f'  SLAM pose error    mean {e.mean() * 100:.0f} cm, max {e.max() * 100:.0f} cm (vs truth)')
        lines += ['  not simulated: BCP mass/energy, bandwidth, dust', f'  log + camera frames: {self.out}']
        print('\n'.join(lines), flush=True)
        with open(os.path.join(self.out, 'score.txt'), 'w') as f:
            f.write('\n'.join(lines) + '\n')


def main():
    rclpy.init()
    node = CosmosAgent()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            ex.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node.done and node.t0 is not None:
            node.report()
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
