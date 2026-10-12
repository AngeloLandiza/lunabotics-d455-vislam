#!/usr/bin/env python3
"""Arena dig tool: edit the simulated regolith with the mouse and compare SLAM to the truth.

Left panel  TRUE terrain from Isaac Sim (/arena/heightmap), with zones, obstacles,
            rover and the D455's ground footprint (what the camera can see right now).
Right panel SLAM terrain map (/terrain/cloud) placed in the arena frame. Toggle with E:
            elevation | change (SLAM's dug/dumped map) | error (SLAM - truth).

Mouse (either panel): left = dig, right = dump (hold and drag to keep going),
wheel or [ ] = brush radius, - / = = depth per stroke.
Keys: E view, R re-align SLAM to the arena, Q quit.
Driving (parameter drive:=true): hold W A S D for tank drive (W/S both tracks forward /
back, A/D tracks opposite = pivot, W+A / W+D arc), 1 2 3 speed. The window needs focus.

Placing the SLAM map in the arena frame uses ground truth once (the camera's true pose
at one instant), like a judge's survey; afterwards the panel shows SLAM's own map, so
drift and missed terrain changes show up as error.
"""
import collections
import math
import os
import sys
import time

import tkinter as tk

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as A  # noqa: E402

WINDOW = 'Arena Dig Tool'
SCALE = 1.6                       # screen pixels per 2 cm cell
H_RANGE = (-0.25, 0.15)           # colour scale for elevations (m)
ERR_RANGE = 0.10                  # +- m for the error / change views
FOV_H, FOV_V, MAX_RANGE = math.radians(90.4), math.radians(64.5), 3.5   # sim D455 IR + mapper range
VIEWS = ['elevation', 'change', 'error']
SPEEDS = [(0.15, 0.3), (0.3, 0.5), (0.5, 0.8)]   # (m/s, rad/s) at full track command, keys 1 2 3
TRACKS = {'w': (1, 1), 's': (-1, -1), 'a': (-1, 1), 'd': (1, -1)}   # (left, right) per key


def pose_to_T(p, q):
    x, y, z, w = q
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = p
    return T


class DigTool(Node):
    def __init__(self):
        super().__init__('dig_tool')
        self.seed = self.declare_parameter('seed', 0).value
        self.boulders, self.craters = A.layout(self.seed)
        self.xs, self.ys = A.grid()
        self.H = self.H0 = None
        self.true_vol = (0.0, 0.0, 0.0)
        self.slam_vol = None
        self.slam = None                 # (x, y, elevation, change) in the map frame
        self.rover = None                # (x, y, yaw) arena
        self.gt_cam = None               # 4x4 arena <- camera_link, with stamp
        self.gt_hist = collections.deque()
        self.T_arena_map = None
        self.drift = None
        self.view = 0
        self.radius, self.depth = 0.15, 0.04
        self.mouse = None                # (x, y, button) while held
        self.last_stroke = 0.0
        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/arena/heightmap', self.on_height, latched)
        self.create_subscription(Float32MultiArray, '/arena/volume', self.on_true_vol, latched)
        self.create_subscription(PoseStamped, '/ground_truth/base_pose', self.on_base, 10)
        self.create_subscription(PoseStamped, '/ground_truth/camera_pose', self.on_cam, 10)
        self.create_subscription(PointCloud2, '/terrain/cloud', self.on_cloud, 1)
        self.create_subscription(Float64MultiArray, '/terrain/volume', self.on_slam_vol, 1)
        self.dig_pub = self.create_publisher(Float32MultiArray, '/arena/dig', 10)
        self.drive = self.declare_parameter('drive', False).value
        self.held = set()                # drive keys held down
        self.speed = 1
        self.tracks = (0.0, 0.0)
        if self.drive:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.create_timer(0.05, self.publish_drive, clock=Clock())   # wall clock, 20 Hz

    # ---------------------------------------------------------------- inputs
    def on_height(self, msg):
        H = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
        if H.shape != (len(self.xs), len(self.ys)):
            self.get_logger().error(f'heightmap {H.shape} is not the arena grid; is the sim in --world arena?')
            return
        self.H = H

    def on_true_vol(self, msg):
        self.true_vol = tuple(msg.data[:3])

    def on_slam_vol(self, msg):
        self.slam_vol = tuple(msg.data[:2])

    def on_base(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.rover = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_cam(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.gt_cam = (pose_to_T((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), Time.from_msg(msg.header.stamp))
        # SLAM publishes a frame's pose a little after the image; match GT poses ~0.5 s old.
        self.gt_hist.append(self.gt_cam)
        while len(self.gt_hist) > 2 and (self.gt_cam[1] - self.gt_hist[0][1]).nanoseconds > 5e8:
            T_gt, stamp = self.gt_hist.popleft()
            try:
                tf = self.tf.lookup_transform('map', 'camera_link', stamp)
            except Exception:
                continue
            t, r = tf.transform.translation, tf.transform.rotation
            T_map_cam = pose_to_T((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
            if self.T_arena_map is None:
                self.T_arena_map = T_gt @ np.linalg.inv(T_map_cam)
                self.get_logger().info('SLAM map aligned to the arena frame')
            est = self.T_arena_map @ T_map_cam
            self.drift = (float(np.linalg.norm(est[:3, 3] - T_gt[:3, 3])),
                          float(np.degrees(np.arccos(np.clip((np.trace(est[:3, :3].T @ T_gt[:3, :3]) - 1) / 2,
                                                             -1, 1)))))

    def on_cloud(self, msg):
        off = {f.name: f.offset // 4 for f in msg.fields}
        a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
        self.slam = a[:, [off['x'], off['y'], off['elevation'], off['change']]].copy()

    # ---------------------------------------------------------------- driving
    def publish_drive(self):
        left = float(np.clip(sum(TRACKS[k][0] for k in self.held), -1, 1))
        right = float(np.clip(sum(TRACKS[k][1] for k in self.held), -1, 1))
        self.tracks = (left, right)
        v, w = SPEEDS[self.speed]
        cmd = Twist()
        cmd.linear.x = v * (left + right) / 2
        cmd.angular.z = w * (right - left) / 2
        self.cmd_pub.publish(cmd)

    # ---------------------------------------------------------------- mouse
    def to_arena(self, u, v):
        w = len(self.xs) * SCALE
        if u >= w + 10:
            u -= w + 10
        return self.xs[0] + u / SCALE * A.RES, self.ys[-1] - v / SCALE * A.RES

    def stroke(self):
        if not self.mouse or time.monotonic() - self.last_stroke < 0.15 or self.H is None:
            return
        u, v, sign = self.mouse
        if v > len(self.ys) * SCALE:
            return
        x, y = self.to_arena(u, v)
        self.dig_pub.publish(Float32MultiArray(data=[x, y, self.radius, sign * self.depth]))
        self.last_stroke = time.monotonic()

    # ---------------------------------------------------------------- drawing
    def px(self, x, y):
        return int((x - self.xs[0]) / A.RES * SCALE), int((self.ys[-1] - y) / A.RES * SCALE)

    def colorize(self, Z, lo, hi, cmap=cv2.COLORMAP_TURBO, shade=None):
        z = np.clip((Z - lo) / (hi - lo), 0, 1)
        img = cv2.applyColorMap((np.nan_to_num(z) * 255).astype(np.uint8), cmap).astype(np.float32)
        if shade is not None:
            img *= shade[..., None]
        img[np.isnan(Z)] = (28, 28, 30)
        return np.clip(img, 0, 255).astype(np.uint8)

    def to_screen(self, grid_img):
        """Grid image (rows = x, cols = y) -> screen (x right, y up)."""
        img = np.transpose(grid_img, (1, 0, 2))[::-1]
        return cv2.resize(img, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)

    def overlays(self, img):
        col = {'start': (90, 220, 90), 'excavation': (255, 190, 60), 'obstacle': (170, 170, 170),
               'construction': (60, 200, 255), 'berm': (60, 60, 255)}
        for k, (x0, y0, x1, y1) in A.ZONES.items():
            cv2.rectangle(img, self.px(x0, y0), self.px(x1, y1), col[k], 2 if k == 'berm' else 1)
        cx, cy, s = A.COLUMN
        cv2.rectangle(img, self.px(cx - s / 2, cy - s / 2), self.px(cx + s / 2, cy + s / 2), (200, 200, 200), -1)
        for b in self.boulders:
            cv2.circle(img, self.px(b.x, b.y), int(b.diameter / 2 / A.RES * SCALE), (240, 240, 240), 2, cv2.LINE_AA)
        for x, y in A.POLES:
            cv2.circle(img, self.px(x, y), 3, (40, 40, 230), -1, cv2.LINE_AA)
        if self.rover:
            x, y, yaw = self.rover
            c, s_ = math.cos(yaw), math.sin(yaw)
            pts = [self.px(x + c * dx - s_ * dy, y + s_ * dx + c * dy)
                   for dx, dy in ((0.255, 0.215), (0.255, -0.215), (-0.255, -0.215), (-0.255, 0.215))]
            cv2.polylines(img, [np.array(pts)], True, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(img, self.px(x, y), self.px(x + 0.4 * c, y + 0.4 * s_), (255, 255, 255), 2,
                            cv2.LINE_AA, tipLength=0.3)
        if self.gt_cam:
            T = self.gt_cam[0]
            o = T[:3, 3]
            corners = []
            for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
                d = T[:3, :3] @ np.array([1.0, sy * math.tan(FOV_H / 2), sz * math.tan(FOV_V / 2)])
                d /= np.linalg.norm(d)
                t = -o[2] / d[2] if d[2] < -1e-3 else MAX_RANGE
                p = o + d * min(t, MAX_RANGE)
                corners.append(self.px(p[0], p[1]))
            cv2.polylines(img, [np.array(corners)], True, (0, 255, 255), 1, cv2.LINE_AA)
        if getattr(self, 'cursor', None):
            u, v = self.cursor
            w = len(self.xs) * SCALE
            u = int(u - (w + 10)) if u >= w + 10 else int(u)
            cv2.circle(img, (u, int(v)), int(self.radius / A.RES * SCALE), (255, 255, 255), 1, cv2.LINE_AA)

    def slam_grid(self):
        """Rasterize the SLAM terrain cells (map frame) onto the arena grid."""
        Z = np.full((len(self.xs), len(self.ys)), np.nan, np.float32)
        C = Z.copy()
        if self.slam is None or self.T_arena_map is None:
            return Z, C
        P = np.c_[self.slam[:, :3], np.ones(len(self.slam))] @ self.T_arena_map.T
        half = int(round(0.025 / A.RES))           # mapper cells are 5 cm
        i = np.round((P[:, 0] - self.xs[0]) / A.RES).astype(int)
        j = np.round((P[:, 1] - self.ys[0]) / A.RES).astype(int)
        ok = (i >= half) & (j >= half) & (i < len(self.xs) - half) & (j < len(self.ys) - half)
        for di in range(-half, half + 1):
            for dj in range(-half, half + 1):
                Z[i[ok] + di, j[ok] + dj] = P[ok, 2]
                C[i[ok] + di, j[ok] + dj] = self.slam[ok, 3]
        return Z, C

    def render(self):
        if self.H is None:
            img = np.zeros((300, 900, 3), np.uint8)
            cv2.putText(img, 'waiting for /arena/heightmap (start the sim with --world arena)', (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1, cv2.LINE_AA)
            return img
        gx, gy = np.gradient(self.H.astype(np.float64), A.RES)
        shade = np.clip(0.75 + 0.9 * (gx - gy) / np.sqrt(2) * -1 * 0.5, 0.35, 1.15).astype(np.float32)
        left = self.to_screen(self.colorize(self.H, *H_RANGE, shade=shade))
        Z, C = self.slam_grid()
        view = VIEWS[self.view]
        if view == 'elevation':
            right = self.to_screen(self.colorize(Z, *H_RANGE, shade=shade))
        elif view == 'change':
            right = self.to_screen(self.colorize(C, -ERR_RANGE, ERR_RANGE, cv2.COLORMAP_JET))
        else:
            right = self.to_screen(self.colorize(Z - self.H, -ERR_RANGE, ERR_RANGE, cv2.COLORMAP_JET))
        for img in (left, right):
            self.overlays(img)
        gap = np.full((left.shape[0], 10, 3), 40, np.uint8)
        top = np.hstack([left, gap, right])
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(top, 'TRUE terrain (sim)', (8, 20), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(top, f'SLAM terrain map: {view}  [E]', (left.shape[1] + 18, 20), font, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        bar = np.full((124 if self.drive else 96, top.shape[1], 3), 24, np.uint8)
        cut, fill, berm = self.true_vol
        line1 = (f'brush r={self.radius * 100:.0f} cm  depth/stroke={self.depth * 100:.0f} cm     '
                 f'TRUE  dug {cut * 1000:6.1f} L  dumped {fill * 1000:6.1f} L  berm box {berm * 1000:6.1f} L')
        sv = self.slam_vol
        line2 = ('SLAM  dug %6.1f L  dumped %6.1f L' % (sv[0] * 1000, sv[1] * 1000) if sv else 'SLAM  (no /terrain/volume yet)')
        valid = ~np.isnan(Z)
        if valid.any() and self.T_arena_map is not None:
            err = np.abs(Z - self.H)[valid]
            dug = valid & (self.H < self.H0 - 0.03) if self.H0 is not None else valid & False
            line2 += f'     map |error| median {np.median(err) * 100:.1f} cm, 90% {np.percentile(err, 90) * 100:.1f} cm'
            if dug.any():
                line2 += f'   in dug cells {np.median(np.abs(Z - self.H)[dug]) * 100:.1f} cm ({dug.sum()} px)'
        if self.drift:
            line2 += f'     camera drift {self.drift[0] * 100:.1f} cm / {self.drift[1]:.1f} deg'
        elif self.T_arena_map is None:
            line2 += '     (waiting for SLAM to align)'
        lines = [line1, line2]
        if self.drive:
            left, right = self.tracks
            v, w = SPEEDS[self.speed]
            lines.append(f'DRIVE (hold W A S D, 1 2 3 speed)  speed {self.speed + 1}: {v:.2f} m/s, {w:.1f} rad/s'
                         f'     tracks  L {left:+.0f}  R {right:+.0f}     keys {"".join(sorted(self.held)).upper() or "-"}')
        lines.append('left-drag dig | right-drag dump | wheel/[ ] radius | -/= depth | E view | R re-align | Q quit')
        for k, t in enumerate(lines):
            cv2.putText(bar, t, (10, 24 + 28 * k), font, 0.5, (230, 230, 230) if k < len(lines) - 1 else (150, 150, 150),
                        1, cv2.LINE_AA)
        return np.vstack([top, bar])

    def key(self, k):
        if k in (ord('q'), 27):
            return False
        if k == ord('e'):
            self.view = (self.view + 1) % len(VIEWS)
        elif k == ord('r'):
            self.T_arena_map = None
        elif k == ord('['):
            self.radius = max(0.04, self.radius / 1.15)
        elif k == ord(']'):
            self.radius = min(0.6, self.radius * 1.15)
        elif k == ord('-'):
            self.depth = max(0.01, self.depth - 0.01)
        elif k in (ord('='), ord('+')):
            self.depth = min(0.2, self.depth + 0.01)
        elif k in (ord('1'), ord('2'), ord('3')):
            self.speed = k - ord('1')
        return True


class Window:
    """Tk window: unlike an OpenCV window it reports key releases, which hold-to-drive needs."""

    def __init__(self, node):
        self.node, self.closed = node, False
        self.root = tk.Tk()
        self.root.title(WINDOW)
        self.root.resizable(False, False)
        self.label = tk.Label(self.root, bd=0, highlightthickness=0)
        self.label.pack()
        self.photo = None
        self.release_jobs = {}
        r, lb = self.root, self.label
        r.protocol('WM_DELETE_WINDOW', self.close)
        r.bind('<KeyPress>', self.on_key)
        r.bind('<KeyRelease>', self.on_key_release)
        r.bind('<FocusOut>', lambda e: node.held.clear())        # never keep driving without focus
        # Come up in front of the Isaac / RViz windows (GNOME would open it behind them) with
        # keyboard focus, since WASD needs it.
        r.attributes('-topmost', True)
        r.after(2000, lambda: r.attributes('-topmost', False))
        r.after(200, r.focus_force)
        for b, sign in ((1, 1), (3, -1)):
            lb.bind(f'<ButtonPress-{b}>', lambda e, s=sign: self.mouse('down', e, s))
            lb.bind(f'<B{b}-Motion>', lambda e: self.mouse('move', e))
            lb.bind(f'<ButtonRelease-{b}>', lambda e: self.mouse('up', e))
        lb.bind('<Motion>', lambda e: self.mouse('move', e))
        lb.bind('<Button-4>', lambda e: self.mouse('wheel', e, 1))
        lb.bind('<Button-5>', lambda e: self.mouse('wheel', e, -1))

    def mouse(self, what, e, sign=0):
        n = self.node
        if what == 'down':
            n.mouse, n.last_stroke = [e.x, e.y, sign], 0.0
        elif what == 'move' and n.mouse:
            n.mouse[:2] = [e.x, e.y]
        elif what == 'up':
            n.mouse = None
        elif what == 'wheel':
            n.radius = float(np.clip(n.radius * (1.15 if sign > 0 else 1 / 1.15), 0.04, 0.6))
        n.cursor = (e.x, e.y)

    def on_key(self, e):
        k = e.keysym.lower()
        if k in self.release_jobs:                  # X11 auto-repeat sends release + press pairs
            self.root.after_cancel(self.release_jobs.pop(k))
        if self.node.drive and k in TRACKS:
            self.node.held.add(k)
        elif len(k) == 1 or k in ('escape', 'minus', 'equal', 'plus', 'bracketleft', 'bracketright'):
            ch = {'escape': 27, 'minus': ord('-'), 'equal': ord('='), 'plus': ord('+'),
                  'bracketleft': ord('['), 'bracketright': ord(']')}.get(k, ord(k[0]))
            if not self.node.key(ch):
                self.close()

    def on_key_release(self, e):
        k = e.keysym.lower()
        if k in TRACKS:
            self.release_jobs[k] = self.root.after(40, lambda: (self.node.held.discard(k), self.release_jobs.pop(k, None)))

    def show(self, bgr):
        h, w = bgr.shape[:2]
        ppm = b'P6 %d %d 255 ' % (w, h) + cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
        self.photo = tk.PhotoImage(width=w, height=h, data=ppm, format='PPM')
        self.label.configure(image=self.photo)
        self.root.update()

    def close(self):
        self.closed = True
        self.node.held.clear()


def main():
    rclpy.init(args=sys.argv)
    node = DigTool()
    win = Window(node)
    try:
        while rclpy.ok() and not win.closed:
            t = time.monotonic()
            while time.monotonic() - t < 0.06:
                rclpy.spin_once(node, timeout_sec=0.01)
                win.root.update()                   # key events between frames
            if node.H is not None and node.H0 is None:
                node.H0 = A.heightmap(node.boulders, node.craters, node.seed)[2]
            node.stroke()
            win.show(node.render())
    except (KeyboardInterrupt, ExternalShutdownException, tk.TclError):
        pass
    finally:
        if node.drive and rclpy.ok():               # (Ctrl+C has already shut ROS down)
            node.held.clear()
            node.publish_drive()                    # stop the rover
        try:
            win.root.destroy()
        except tk.TclError:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
