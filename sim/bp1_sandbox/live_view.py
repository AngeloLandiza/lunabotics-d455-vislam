"""Live RViz2 view of one approach run: the robot drives through the obstacle zone and the log-odds detector
updates frame by frame, exactly as in approach.trial, for one arena seed, one sensor and clear air or dust.

Usage (ROS 2 Humble sourced; run_live.sh also starts rviz2 with live_view.rviz):
  python3 live_view.py [--seed 1] [--sensor spin16|d455] [--dust] [--rate 2.0] [--l-dec-lidar 5.1] [--l-dec-d455 0.85]
                       [--paused] [--loop]
Keys in the terminal: space pause/resume, n next frame (while paused), r restart, q quit.

Topics (frame "map", the arena frame: metres, z up, origin at the start-zone corner):
  /bp1/terrain (PointCloud2, latched)      height field every 2 cm, coloured by height
  /bp1/truth (MarkerArray, latched)        true boulders / craters / column as translucent labelled cylinders
  /bp1/robot (MarkerArray)                 footprint box, corridor ahead, plume ellipsoid
  /bp1/path (nav_msgs/Path)                path driven so far; TF map -> base_link is the robot pose
  /bp1/scan_surface, /bp1/scan_dust        current scan's surface returns and dust echoes (PointCloud2)
  /bp1/lp, /bp1/ln (MarkerArray)           boulder / crater log-odds per cell, diverging colours
  /bp1/candidates (MarkerArray)            every cluster: confirmed / unresolved / faint, with evidence and truth class
  /bp1/gate (MarkerArray)                  speed-gate text and the stopping-distance line ahead of the front
"""
import argparse
import os
import queue
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import params as pr                                             # noqa: E402
from params import v                                            # noqa: E402
from approach import DS, FRONT, HALF_W, LOOK, PLUME_AHEAD, X_END, path_frame   # noqa: E402
from arena import Arena                                         # noqa: E402
from detect import Grid, add_scan, clusters, fit_ground, logodds_delta, logodds_zref, new_layers   # noqa: E402
from dust import Plume                                          # noqa: E402
from sensors import D455Depth, Lidar                            # noqa: E402

import rclpy                                                    # noqa: E402
from builtin_interfaces.msg import Duration                     # noqa: E402
from geometry_msgs.msg import Point, PoseStamped, TransformStamped   # noqa: E402
from nav_msgs.msg import Path                                   # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile              # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField             # noqa: E402
from std_msgs.msg import ColorRGBA, Header                      # noqa: E402
from tf2_ros import TransformBroadcaster                        # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray          # noqa: E402

# Speed gate overlay (Assumed numbers from the coordinator's brief; speed_gate.py holds the evaluated policy)
V_FAST, V_SLOW = 1.0, 0.25          # m/s
T_REACT, DECEL, STOP_MARGIN = 0.3, 0.5, 0.10   # s, m/s^2, m
FAINT = 0.85                        # evidence below one net observation (l_occ) is drawn as faint
ROBOT_LEN, ROBOT_W, ROBOT_H = 2 * FRONT, 0.60, 0.30   # footprint box (Assumed; FRONT is the centre-to-front distance)


def P(x=0.0, y=0.0, z=0.0):
    """geometry_msgs Point; rclpy asserts plain floats."""
    return Point(x=float(x), y=float(y), z=float(z))


def d_stop(vel):
    return vel * T_REACT + vel ** 2 / (2 * DECEL) + STOP_MARGIN


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def diverging(x, neg, pos, lim):
    """White at 0, towards `neg` for negative and `pos` for positive values, saturating at |x| = lim."""
    t = np.clip(np.asarray(x) / lim, -1, 1)[:, None]
    w = np.ones(3)
    return np.where(t < 0, w + (-t) * (np.asarray(neg) - w), w + t * (np.asarray(pos) - w))


def height_rgb(z, lo=-0.25, hi=0.25):
    """Terrain colour: blue (low) - tan (ground) - brown/red (high); walls and the column above hi show grey."""
    t = np.clip((z - lo) / (hi - lo), 0, 1)
    stops = np.array([[0.15, 0.35, 0.85], [0.80, 0.75, 0.62], [0.65, 0.30, 0.15]])
    seg = np.where(t < 0.5, 0, 1)
    f = np.where(t < 0.5, t / 0.5, (t - 0.5) / 0.5)[:, None]
    rgb = stops[seg] * (1 - f) + stops[seg + 1] * f
    rgb[z > 0.6] = [0.55, 0.55, 0.58]
    return rgb


def cloud(header, xyz, rgb=None):
    """PointCloud2 with float32 x, y, z and, when given, a packed rgb field (RViz RGB8 colour transformer)."""
    xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    if rgb is None:
        data = xyz
    else:
        c = (np.clip(np.asarray(rgb), 0, 1) * 255).astype(np.uint32)
        packed = ((c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]).view(np.float32)
        data = np.concatenate([xyz, packed[:, None]], 1)
        fields.append(PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1))
    msg = PointCloud2(header=header, height=1, width=len(xyz), fields=fields, is_bigendian=False,
                      point_step=4 * data.shape[1], row_step=4 * data.shape[1] * len(xyz), is_dense=True)
    msg.data = np.ascontiguousarray(data, np.float32).tobytes()
    return msg


# ---------------------------------------------------------------- the run (approach.trial for one sensor and dust)
class Run:
    def __init__(self, seed, sensor, dust_on, l_dec):
        rng = np.random.default_rng(seed)          # same draws as approach.trial(seed): arena, then start pose
        self.arena = arena = Arena(rng)
        x0, y0 = rng.uniform(0.3, 0.8), rng.uniform(0.8, 4.2)
        yaw = float(np.clip(rng.normal(0, 10), -25, 25))
        self.rng = rng
        self.x0, self.y0, self.yaw = x0, y0, yaw
        self.h, self.n = path_frame(x0, y0, yaw)
        h = self.h
        lim = [(X_END - x0) / h[0]]
        if h[1] > 1e-6:
            lim.append((arena.W - 0.5 - y0) / h[1])
        elif h[1] < -1e-6:
            lim.append((0.5 - y0) / h[1])
        self.length = max(0.0, min(lim))
        self.n_frames = int(self.length // DS) + 1
        self.grid = grid = Grid(arena.L, arena.W, v(pr.DETECT, "cell"))
        X, Y = grid.centers()
        self.along_c = (X - x0) * h[0] + (Y - y0) * h[1]
        lat_c = np.abs((X - x0) * self.n[0] + (Y - y0) * self.n[1])
        inside = np.zeros_like(X, bool)
        inside[1:-1, 1:-1] = True
        self.corridor = (lat_c < HALF_W) & inside
        self.sensor_name = sensor
        self.sensor = D455Depth() if sensor == "d455" else Lidar(sensor)
        self.dust = Plume(x0 + PLUME_AHEAD * h[0], y0 + PLUME_AHEAD * h[1]) if dust_on else None
        self.l_dec = l_dec

    def classify(self, d):
        """Same truth classes as approach.trial: obstacle / near / wall / fa."""
        arena, margin, band = self.arena, v(pr.DETECT, "match_margin"), v(pr.DETECT, "wall_band")
        want = {"pos": ("boulder", "column"), "neg": ("crater",)}[d["kind"]]
        if any(o.kind in want and np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin for o in arena.obstacles):
            return "obstacle"
        if any(np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin for o in arena.obstacles):
            return "near"
        if min(d["x"], d["y"], arena.L - d["x"], arena.W - d["y"]) < band:
            return "wall"
        return "fa"

    def frames(self, stop):
        grid, h, n = self.grid, self.h, self.n
        lmin, lmax = v(pr.DETECT, "l_min"), v(pr.DETECT, "l_max")
        L = new_layers(grid)
        lp, ln = np.zeros((grid.nx, grid.ny)), np.zeros((grid.nx, grid.ny))
        for i in range(self.n_frames):
            if stop.is_set():
                return
            s = i * DS
            robot = (self.x0 + s * h[0], self.y0 + s * h[1], self.yaw)
            scan = self.sensor.scan(self.arena, robot, self.rng, dust=self.dust)
            add_scan(L, scan, grid)
            g = fit_ground(L, grid)
            dp, dn = logodds_delta(scan, grid, g, logodds_zref(L, g))
            lp = np.clip(lp + dp, lmin, lmax)
            ln = np.clip(ln + dn, lmin, lmax)
            dets = []
            for d in clusters(lp > 0, ln > 0, grid, lp, ln, l_dec=-np.inf):
                rel = np.array([d["x"] - self.x0, d["y"] - self.y0])
                d.update(cls=self.classify(d), along=float(rel @ h), lateral=float(abs(rel @ n)))
                dets.append(d)
            seen = (L["count"] > 0) | np.isfinite(L["carve"])
            win = self.corridor & (self.along_c > s + FRONT) & (self.along_c < s + FRONT + LOOK) & ~seen
            first_unseen = float(self.along_c[win].min() - (s + FRONT)) if win.any() else None
            yield {"i": i, "s": s, "robot": robot, "pts": scan["pts"][scan["valid"]], "dust": scan["is_dust"][scan["valid"]],
                   "lp": lp.copy(), "ln": ln.copy(), "g": g, "dets": dets, "first_unseen": first_unseen}

    def gate(self, fr):
        """Speed gate (overlay only): slow when unobserved ground, an unresolved candidate or a confirmed hazard lies
        in the corridor within the fast stopping distance plus 0.5 m ahead of the front; otherwise fast."""
        horizon = d_stop(V_FAST) + 0.5
        reasons = []
        if fr["first_unseen"] is not None and fr["first_unseen"] < d_stop(V_FAST):
            reasons.append(f"unobserved ground {fr['first_unseen']:.2f} m ahead")
        ahead = [d for d in fr["dets"] if d["lateral"] < HALF_W + 0.1
                 and 0 < d["along"] - (fr["s"] + FRONT) < horizon]
        if any(d["evidence"] >= self.l_dec for d in ahead):
            reasons.append("confirmed hazard")
        elif any(FAINT <= d["evidence"] < self.l_dec for d in ahead):
            reasons.append("unresolved candidate")
        return (V_SLOW if reasons else V_FAST), reasons


# ---------------------------------------------------------------- ROS publishing
class Viewer:
    def __init__(self, node, run):
        self.node, self.run = node, run
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher
        self.p_terrain = pub(PointCloud2, "/bp1/terrain", latched)
        self.p_truth = pub(MarkerArray, "/bp1/truth", latched)
        self.p_robot = pub(MarkerArray, "/bp1/robot", 5)
        self.p_path = pub(Path, "/bp1/path", 5)
        self.p_surf = pub(PointCloud2, "/bp1/scan_surface", 5)
        self.p_dust = pub(PointCloud2, "/bp1/scan_dust", 5)
        self.p_lp = pub(MarkerArray, "/bp1/lp", 5)
        self.p_ln = pub(MarkerArray, "/bp1/ln", 5)
        self.p_cand = pub(MarkerArray, "/bp1/candidates", 5)
        self.p_gate = pub(MarkerArray, "/bp1/gate", 5)
        self.tf = TransformBroadcaster(node)
        self.path = Path()

    def header(self, frame="map"):
        return Header(stamp=self.node.get_clock().now().to_msg(), frame_id=frame)

    def marker(self, ns, mid, mtype, frame="map", **kw):
        if "text" in kw:   # rviz2 (Humble) draws spaces in text markers several characters wide, even U+00A0
            kw["text"] = "_".join(kw["text"].split(" ")).replace("__", "_").replace("__", "_")
        m = Marker(header=self.header(frame), ns=ns, id=mid, type=mtype, action=Marker.ADD)
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(sec=0)
        for k, val in kw.items():
            setattr(m, k, val)
        return m

    @staticmethod
    def clear(ns):
        return Marker(ns=ns, action=Marker.DELETEALL)

    def publish_static(self):
        a = self.run.arena
        hs = a.h[::2, ::2]
        X, Y = a.X[::2, ::2], a.Y[::2, ::2]
        xyz = np.stack([X.ravel(), Y.ravel(), hs.ravel()], 1)
        xyz = xyz[(xyz[:, 0] >= 0) & (xyz[:, 0] <= a.L) & (xyz[:, 1] >= 0) & (xyz[:, 1] <= a.W)]   # walls: outline only
        self.p_terrain.publish(cloud(self.header(), xyz, height_rgb(xyz[:, 2])))
        arr = MarkerArray()
        style = {"boulder": (0.85, 0.45, 0.15), "crater": (0.20, 0.45, 1.0), "column": (0.6, 0.6, 0.65)}
        for k, o in enumerate(a.obstacles):
            hgt = max(o.size, 0.02)
            z = -hgt / 2 if o.kind == "crater" else hgt / 2
            m = self.marker("truth", k, Marker.CYLINDER, color=color(*style[o.kind], 0.35))
            m.pose.position = P(x=o.x, y=o.y, z=z)
            m.scale.x = m.scale.y = 2 * o.radius
            m.scale.z = hgt
            arr.markers.append(m)
            t = self.marker("truth_label", k, Marker.TEXT_VIEW_FACING, color=color(*style[o.kind], 1.0),
                            text=f"{o.kind} {2 * o.radius * 100:.0f}x{o.size * 100:.0f}cm")
            t.pose.position = P(x=o.x, y=o.y, z=max(o.size, 0.0) + 0.25 if o.kind != "column" else 1.4)
            t.scale.z = 0.09
            arr.markers.append(t)
        # Arena outline
        ol = self.marker("truth_outline", 0, Marker.LINE_STRIP, color=color(1, 1, 1, 0.8))
        ol.scale.x = 0.02
        ol.points = [P(x=x, y=y, z=0.0) for x, y in ((0, 0), (a.L, 0), (a.L, a.W), (0, a.W), (0, 0))]
        arr.markers.append(ol)
        self.p_truth.publish(arr)

    def publish_frame(self, fr, paused):
        run = self.run
        x, y, yaw = fr["robot"]
        q = (np.cos(np.radians(yaw) / 2), np.sin(np.radians(yaw) / 2))
        ts = TransformStamped(header=self.header(), child_frame_id="base_link")
        ts.transform.translation.x, ts.transform.translation.y = float(x), float(y)
        ts.transform.rotation.w, ts.transform.rotation.z = float(q[0]), float(q[1])
        self.tf.sendTransform(ts)

        ps = PoseStamped(header=self.header())
        ps.pose.position = P(x=float(x), y=float(y), z=0.02)
        ps.pose.orientation.w, ps.pose.orientation.z = float(q[0]), float(q[1])
        if fr["i"] == 0:
            self.path = Path()
        self.path.header = self.header()
        self.path.poses.append(ps)
        self.p_path.publish(self.path)

        # Robot footprint, corridor ahead, plume
        rob = MarkerArray()
        box = self.marker("robot", 0, Marker.CUBE, frame="base_link", color=color(0.1, 0.85, 1.0, 0.85))
        box.pose.position.z = ROBOT_H / 2
        box.scale.x, box.scale.y, box.scale.z = ROBOT_LEN, ROBOT_W, ROBOT_H
        rob.markers.append(box)
        cor = self.marker("corridor", 0, Marker.CUBE, frame="base_link", color=color(0.3, 1.0, 0.4, 0.12))
        cor.pose.position.x, cor.pose.position.z = FRONT + LOOK / 2, 0.01
        cor.scale.x, cor.scale.y, cor.scale.z = LOOK, 2 * HALF_W, 0.01
        rob.markers.append(cor)
        edges = self.marker("corridor_edges", 0, Marker.LINE_LIST, frame="base_link", color=color(0.3, 1.0, 0.4, 0.7))
        edges.scale.x = 0.01
        for sy in (-HALF_W, HALF_W):
            edges.points += [P(x=FRONT, y=sy, z=0.02), P(x=FRONT + LOOK, y=sy, z=0.02)]
        edges.points += [P(x=FRONT, y=-HALF_W, z=0.02), P(x=FRONT, y=HALF_W, z=0.02)]
        rob.markers.append(edges)
        if run.dust is not None:
            p = run.dust
            pl = self.marker("plume", 0, Marker.SPHERE, color=color(1.0, 0.6, 0.1, 0.18))
            pl.pose.position = P(x=p.cx, y=p.cy, z=p.cz)
            pl.scale.x = pl.scale.y = 4 * p.sh      # +-2 sigma
            pl.scale.z = 4 * p.sv
            rob.markers.append(pl)
        self.p_robot.publish(rob)

        # Current scan
        pts, dmask = fr["pts"], fr["dust"]
        self.p_surf.publish(cloud(self.header(), pts[~dmask]))
        self.p_dust.publish(cloud(self.header(), pts[dmask]))

        # Log-odds layers drawn just above the fitted ground
        grid = run.grid
        X, Y = grid.centers()
        for pub_, lay, ns, neg, pos, size, dz in ((self.p_lp, fr["lp"], "lp", (0.2, 0.4, 1.0), (1.0, 0.15, 0.1), 1.0, 0.03),
                                                 (self.p_ln, fr["ln"], "ln", (0.1, 0.75, 0.75), (0.75, 0.1, 0.9), 0.6, 0.05)):
            sel = np.abs(lay) > 0.3
            arr = MarkerArray(markers=[self.clear(ns)])
            m = self.marker(ns, 0, Marker.CUBE_LIST, color=color(1, 1, 1, 0.7))
            m.scale.x = m.scale.y = grid.cell * size
            m.scale.z = 0.01
            zs = fr["g"][sel] + dz
            m.points = [P(x=float(a), y=float(b), z=float(c)) for a, b, c in zip(X[sel], Y[sel], zs)]
            m.colors = [color(*c, 0.5) for c in diverging(lay[sel], neg, pos, v(pr.DETECT, "l_max"))]
            arr.markers.append(m)
            pub_.publish(arr)

        # Candidates
        arr = MarkerArray(markers=[self.clear("cand"), self.clear("cand_label")])
        for k, d in enumerate(fr["dets"]):
            ev = d["evidence"]
            state = "confirmed" if ev >= run.l_dec else ("unresolved" if ev >= FAINT else "faint")
            col = {"confirmed": (1.0, 0.1, 0.1, 0.85), "unresolved": (1.0, 0.85, 0.1, 0.8),
                   "faint": (0.6, 0.6, 0.6, 0.35)}[state]
            r = max(np.sqrt(d["cells"] / np.pi) * grid.cell, 0.05)
            m = self.marker("cand", k, Marker.CYLINDER, color=color(*col))
            m.pose.position = P(x=d["x"], y=d["y"], z=0.40 if d["kind"] == "pos" else 0.30)
            m.scale.x = m.scale.y = 2 * r
            m.scale.z = 0.03
            arr.markers.append(m)
            if state != "faint" or d["cls"] == "obstacle":
                t = self.marker("cand_label", k, Marker.TEXT_VIEW_FACING, color=color(*col[:3], 1.0),
                                text=f"{'B' if d['kind'] == 'pos' else 'C'} {ev:.1f} {state} [{d['cls']}]")
                t.pose.position = P(x=d["x"], y=d["y"], z=0.55)
                t.scale.z = 0.08
                arr.markers.append(t)
        self.p_cand.publish(arr)

        # Speed gate
        vel, reasons = run.gate(fr)
        arr = MarkerArray()
        line = self.marker("gate", 0, Marker.LINE_LIST, frame="base_link",
                           color=color(1.0, 0.3, 0.2, 1.0) if reasons else color(0.3, 1.0, 0.3, 1.0))
        line.scale.x = 0.025
        xs = FRONT + d_stop(vel)
        line.points = [P(x=FRONT, y=0.0, z=0.05), P(x=xs, y=0.0, z=0.05),
                       P(x=xs, y=-HALF_W, z=0.05), P(x=xs, y=HALF_W, z=0.05)]
        arr.markers.append(line)
        txt = (f"v = {vel:.2f} m/s {'SLOW' if reasons else 'FAST'}   d_stop = {d_stop(vel):.2f} m\n"
               + ("; ".join(reasons) if reasons else "corridor observed and clear"))
        t = self.marker("gate_text", 0, Marker.TEXT_VIEW_FACING, frame="base_link", text=txt,
                        color=color(1.0, 0.45, 0.35, 1.0) if reasons else color(0.5, 1.0, 0.5, 1.0))
        t.pose.position.z = 0.75
        t.scale.z = 0.10
        arr.markers.append(t)
        n_conf = sum(d["evidence"] >= run.l_dec for d in fr["dets"])
        n_unres = sum(FAINT <= d["evidence"] < run.l_dec for d in fr["dets"])
        hud = self.marker("hud", 0, Marker.TEXT_VIEW_FACING, color=color(1, 1, 1, 1),
                          text=f"seed {self.node.args.seed}  {run.sensor_name}  {'DUST' if run.dust else 'clear'}  "
                               f"frame {fr['i'] + 1}/{run.n_frames}  s = {fr['s']:.2f} m{'  [PAUSED]' if paused else ''}\n"
                               f"threshold {run.l_dec:g}   confirmed {n_conf}   unresolved {n_unres}   "
                               f"dust echoes {int(fr['dust'].sum())}")
        hud.pose.position = P(x=run.arena.L / 2, y=run.arena.W + 0.6, z=1.2)
        hud.scale.z = 0.16
        arr.markers.append(hud)
        self.p_gate.publish(arr)
        return vel, reasons


# ---------------------------------------------------------------- keys and main loop
class Keys:
    def __init__(self):
        self.q = queue.Queue()
        self.ok = sys.stdin.isatty()
        if self.ok:
            import termios
            import tty
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        while True:
            c = sys.stdin.read(1)
            if c:
                self.q.put(c)

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None

    def restore(self):
        if self.ok:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sensor", choices=["spin16", "d455"], default="spin16")
    ap.add_argument("--dust", action="store_true")
    ap.add_argument("--rate", type=float, default=2.0, help="frames per second shown")
    ap.add_argument("--l-dec-lidar", type=float, default=v(pr.DETECT, "l_decide_lidar"))
    ap.add_argument("--l-dec-d455", type=float, default=v(pr.DETECT, "l_decide_d455"))
    ap.add_argument("--paused", action="store_true", help="start paused (space resumes, n steps)")
    ap.add_argument("--loop", action="store_true", help="restart the run when it ends")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames (0 = whole run); for tests")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = rclpy.create_node("bp1_live_view")
    node.args = args
    keys = Keys()
    l_dec = args.l_dec_d455 if args.sensor == "d455" else args.l_dec_lidar
    cache, shown = None, 0
    try:
        while True:
            run = Run(args.seed, args.sensor, args.dust, l_dec)
            viewer = Viewer(node, run)
            viewer.publish_static()
            print(f"[live] seed {args.seed} {args.sensor} {'dust' if args.dust else 'clear'}: {run.n_frames} frames, "
                  f"start ({run.x0:.2f}, {run.y0:.2f}) yaw {run.yaw:.1f} deg, threshold {l_dec:g}"
                  + ("   keys: space pause, n step, r restart, q quit" if keys.ok else ""), flush=True)
            stop = threading.Event()
            fq = queue.Queue()
            if cache is None:      # first pass: frames stream as the worker computes them (rate may drop below --rate)
                computed = []
                worker = threading.Thread(target=lambda: [fq.put(f) or computed.append(f) for f in run.frames(stop)]
                                          + [fq.put(None)], daemon=True)
                worker.start()
            else:                  # later loops: replay the cached frames at --rate
                computed = None
                for f in cache:
                    fq.put(f)
                fq.put(None)
            paused, step, restart, quit_ = args.paused, False, False, False
            fr, nxt = None, False
            while True:
                t0 = time.time()
                while time.time() - t0 < 1.0 / args.rate or (paused and not step):
                    k = keys.get()
                    if k == " ":
                        paused = not paused
                        if fr is not None:
                            viewer.publish_frame(fr, paused)
                    elif k == "n":
                        step = True
                    elif k == "r":
                        restart = True
                    elif k == "q":
                        quit_ = True
                    if restart or quit_ or step:
                        break
                    rclpy.spin_once(node, timeout_sec=0.02)
                if restart or quit_:
                    break
                step = False
                nxt = fq.get()
                if nxt is None:
                    break
                fr = nxt
                vel, reasons = viewer.publish_frame(fr, paused)
                shown += 1
                conf = [f"{'B' if d['kind'] == 'pos' else 'C'}({d['x']:.2f},{d['y']:.2f}) {d['evidence']:.1f} {d['cls']}"
                        for d in fr["dets"] if d["evidence"] >= l_dec]
                print(f"[live] frame {fr['i'] + 1}/{run.n_frames} s={fr['s']:.2f} v={vel} "
                      f"{'; '.join(reasons) or 'clear'} | confirmed: {', '.join(conf) or '-'}", flush=True)
                if args.frames and shown >= args.frames:
                    quit_ = True
                    break
            stop.set()
            if computed is not None and nxt is None:
                cache = computed
            if quit_ or not (args.loop or restart):
                if not quit_:
                    print("[live] run finished; the last frame stays up (Ctrl+C or q to exit)", flush=True)
                    while keys.get() != "q":
                        rclpy.spin_once(node, timeout_sec=0.1)
                break
    except KeyboardInterrupt:
        pass
    finally:
        keys.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
