"""Shared 2D arena, mission and skid-steer robot for the planner head-to-head (custom stack vs Nav2).

Every planner runs against this one world: the same obstacles (the sandbox arenas, sim/bp1_sandbox/arena.py, so seed
3000 here is seed 3000 there), the same missions (dig point -> berm dump pose -> next dig point), the same robot
footprint from the team URDF and the same skid-steer model with slip. Planners get the true pose (localization is tested
elsewhere) and the obstacle list; they send (v, w) commands at CONTROL_HZ. Every number is in PARAMS with its status.

Frames: arena x along the 6.8 m side from the start-zone corner, y along the 5.0 m side, metres; robot pose (x, y, yaw)
is base_link on the ground, yaw from +x.
"""
import json
import os
import sys
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SANDBOX = os.path.join(os.path.dirname(HERE), "bp1_sandbox")
sys.path.insert(0, SANDBOX)
import params as pr          # noqa: E402  (sandbox params: arena, berm, obstacles)
from params import P, v      # noqa: E402

GUIDEBOOK = pr.GUIDEBOOK
PARAMS = {
    # robot (team URDF via sim/bp1_sandbox/robot.py; robot_cache keeps the numbers so no mesh checkout is needed)
    "footprint": P((-0.438, 0.558, -0.353, 0.346), "m (x0,x1,y0,y1) about base_link", "Verified",
                   "uic_bot_2027.xacro body extents (robot.py Robot.extents)"),
    "track": P(0.54, "m", "Verified", "wheel centres at y = +-0.27 (uic_bot_2027.xacro)"),
    "v_wheel_max": P(0.75, "m/s", "Assumed", "motor limit unknown; 0.75 m/s is the fastest speed with no missed "
                     "hazards in the sandbox speed runs"),
    "a_wheel_max": P(0.5, "m/s^2", "Assumed", "wheel acceleration limit"),
    # skid-steer slip on BP-1 (no measured data for BP-1 or this robot: all Assumed, swept by SLIP_LEVELS)
    "slip_long": P(0.10, "", "Assumed", "mean longitudinal slip ratio on loose regolith"),
    "slip_long_sd": P(0.05, "", "Assumed", "variation of the slip ratio, redrawn every SLIP_TAU seconds"),
    "icr_factor": P(1.5, "", "Assumed", "effective / geometric track width when turning (skid-steer ICR expansion)"),
    "icr_factor_sd": P(0.15, "", "Assumed", "variation of the ICR factor"),
    "slip_tau": P(1.0, "s", "Assumed", "how long one slip draw lasts"),
    # mission (zones beyond the berm are not in the guidebook text: Assumed)
    "dig_box": P((0.7, 1.8, 0.7, 4.3), "m (x0,x1,y0,y1)", "Assumed",
                 "excavation zone overlaps the start zone (16.1.1); its size is in the unreadable Figure 9"),
    "dump_x": P((4.70, 5.38, 6.06), "m", "Assumed", "three dump spots along the 2.2 m berm side, used in turn; a spot "
                "an obstacle blocks is skipped"),
    "dump_clear": P(0.05, "m", "Assumed", "gap between the robot's front and the berm box edge at the dump pose"),
    "dump_heading": P(-90.0, "deg", "Assumed", "front faces the berm (robot drives in forwards); the team's dump "
                      "direction is not known"),
    "goal_tol": P((0.10, 10.0), "m, deg", "Assumed", "pose accepted at a dig point or the dump pose"),
    "berm_tol": P((0.05, 5.0), "m, deg", "Assumed", "dump pose error that counts as on target (scored, not enforced)"),
    "cycles": P(3, "", "Assumed", "dig -> dump -> dig cycles per mission"),
    "leg_timeout": P(60.0, "s", "Assumed", "a leg that takes longer counts as failed"),
}
SLIP_LEVELS = {             # (mean slip ratio, its sd, ICR factor, its sd); all Assumed
    "none": (0.0, 0.0, 1.0, 0.0),
    "nominal": (0.10, 0.05, 1.5, 0.15),
    "heavy": (0.25, 0.08, 2.0, 0.25),
}
SIM_DT = 0.01               # s, physics step
CONTROL_HZ = 20.0           # controller rate (Nav2 controller_server default is 20 Hz)


@dataclass
class Disc:
    kind: str
    x: float
    y: float
    r: float


@dataclass
class World:
    seed: int
    L: float
    W: float
    obstacles: list
    berm: tuple             # (x0, x1, y0, y1)
    meta: dict = field(default_factory=dict)


def world(seed, cache_dir=os.path.join(HERE, "world_cache")):
    """The sandbox arena for this seed, reduced to discs (boulders, craters, column). Cached as JSON."""
    path = os.path.join(cache_dir, f"world_{seed}.json")
    if os.path.exists(path):
        d = json.load(open(path))
    else:
        from arena import Arena
        a = Arena(np.random.default_rng(seed))
        d = {"seed": seed, "L": a.L, "W": a.W, "obstacles": [[o.kind, o.x, o.y, o.radius] for o in a.obstacles]}
        os.makedirs(cache_dir, exist_ok=True)
        json.dump(d, open(path, "w"))
    (bx, by), (bl, bw) = v(pr.ARENA, "berm_center"), v(pr.ARENA, "berm_size")
    return World(seed, d["L"], d["W"], [Disc(*o) for o in d["obstacles"]],
                 (bx - bl / 2, bx + bl / 2, by - bw / 2, by + bw / 2))


def mission(w, rng):
    """Start pose in the start zone, then cycles of (dig pose, dump pose). Dig poses face the obstacle zone."""
    x0, x1, y0, y1 = v(pr.ARENA, "start_zone")
    fp = v(PARAMS, "footprint")
    rad = np.hypot(max(-fp[0], fp[1]), max(-fp[2], fp[3]))
    goals = []
    start = None
    for _ in range(1000):
        s = (rng.uniform(x0 + rad, x1 - rad), rng.uniform(y0 + rad, y1 - rad), rng.uniform(-np.pi, np.pi))
        if turn_clearance(w, s) > 0.05:
            start = s
            break
    dx0, dx1, dy0, dy1 = v(PARAMS, "dig_box")
    dump_y = w.berm[3] + v(PARAMS, "dump_clear") + fp[1]
    dumps = [(dx, dump_y, np.radians(v(PARAMS, "dump_heading"))) for dx in v(PARAMS, "dump_x")]
    dumps = [d for d in dumps if clearance(w, d, berm=False) > 0.10] or dumps[-1:]   # skip spots an obstacle blocks
    for k in range(v(PARAMS, "cycles")):
        for _ in range(1000):
            g = (rng.uniform(dx0, dx1), rng.uniform(dy0, dy1), rng.uniform(-np.pi / 4, np.pi / 4))
            if clearance(w, g) > 0.15 and turn_clearance(w, g) > 0.05:      # any planner can turn in place there
                break
        goals.append(("dig", g))
        goals.append(("dump", dumps[k % len(dumps)]))
    return start, goals


def footprint_pts(pose, n=6):
    """Points on the footprint rectangle's boundary (world frame) for clearance checks."""
    x0, x1, y0, y1 = v(PARAMS, "footprint")
    t = np.linspace(0, 1, n, endpoint=False)
    loc = np.concatenate([np.stack([x0 + (x1 - x0) * t, np.full(n, y0)], 1), np.stack([np.full(n, x1), y0 + (y1 - y0) * t], 1),
                          np.stack([x1 - (x1 - x0) * t, np.full(n, y1)], 1), np.stack([np.full(n, x0), y1 - (y1 - y0) * t], 1)])
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return np.stack([pose[0] + c * loc[:, 0] - s * loc[:, 1], pose[1] + s * loc[:, 0] + c * loc[:, 1]], 1)


def turn_radius():
    """Radius swept by the footprint when the robot turns in place about base_link."""
    x0, x1, y0, y1 = v(PARAMS, "footprint")
    return float(np.hypot(max(-x0, x1), max(-y0, y1)))


def _rect_rect_gap(P, rect):
    """Gap between a convex polygon P (corners, world frame) and an axis-aligned rectangle (negative = overlap),
    by separating axes (the four axes of the two rectangles)."""
    bx0, bx1, by0, by1 = rect
    Q = np.array([[bx0, by0], [bx1, by0], [bx1, by1], [bx0, by1]])
    axes = [np.array([1.0, 0.0]), np.array([0.0, 1.0]), P[1] - P[0], P[3] - P[0]]
    gaps = []
    for ax in axes:
        ax = ax / np.linalg.norm(ax)
        p, q = P @ ax, Q @ ax
        gaps.append(max(q.min() - p.max(), p.min() - q.max()))
    g = max(gaps)
    if g > 0:   # separated: the true distance is at least the largest axis gap; use corner-to-rectangle distances
        dp = min(np.hypot(max(bx0 - x, 0, x - bx1), max(by0 - y, 0, y - by1)) for x, y in P)
        dq = min(_pt_poly_dist(qq, P) for qq in Q)
        return float(min(dp, dq))
    return float(g)


def _pt_poly_dist(q, P):
    best = np.inf
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        t = np.clip(np.dot(q - a, b - a) / np.dot(b - a, b - a), 0, 1)
        best = min(best, np.hypot(*(a + t * (b - a) - q)))
    return best


def clearance(w, pose, berm=True):
    """Signed distance from the footprint rectangle to the nearest obstacle disc, wall or the berm box (negative =
    overlap). The berm box is a keep-out area: driving into it would flatten the berm."""
    x0, x1, y0, y1 = v(PARAMS, "footprint")
    c, s = np.cos(pose[2]), np.sin(pose[2])
    best = np.inf
    for o in w.obstacles:
        dx, dy = o.x - pose[0], o.y - pose[1]
        lx, ly = c * dx + s * dy, -s * dx + c * dy        # disc centre in the robot frame
        qx, qy = np.clip(lx, x0, x1), np.clip(ly, y0, y1)
        inside = (x0 < lx < x1) and (y0 < ly < y1)
        d = -min(lx - x0, x1 - lx, ly - y0, y1 - ly) if inside else np.hypot(lx - qx, ly - qy)
        best = min(best, d - o.r)
    corners = footprint_pts(pose, 1)
    best = min(best, corners[:, 0].min(), w.L - corners[:, 0].max(), corners[:, 1].min(), w.W - corners[:, 1].max())
    if berm:
        best = min(best, _rect_rect_gap(corners, w.berm))
    return float(best)


def turn_clearance(w, xy):
    """Clearance of the circle the footprint sweeps when turning in place at xy."""
    rho = turn_radius()
    best = min(np.hypot(o.x - xy[0], o.y - xy[1]) - o.r for o in w.obstacles) - rho
    best = min(best, xy[0] - rho, w.L - xy[0] - rho, xy[1] - rho, w.W - xy[1] - rho)
    bx0, bx1, by0, by1 = w.berm
    best = min(best, np.hypot(max(bx0 - xy[0], 0, xy[0] - bx1), max(by0 - xy[1], 0, xy[1] - by1)) - rho)
    return float(best)


class SkidSteer:
    """Kinematic skid-steer with slip. The base driver turns commands (v, w) into wheel speeds with an effective track
    width B * mult, as ros2_control's diff_drive_controller does with wheel_separation_multiplier, calibrated to the
    level's mean ICR factor; speeds are then rate- and speed-limited. The body moves with
    v_true = (1 - slip) (vl + vr) / 2 and w_true = (vr - vl) / (icr * B), with slip and icr redrawn every slip_tau
    seconds from this robot's own random stream. Planners see the true pose, not the slip."""

    def __init__(self, pose, level, rng):
        self.pose = np.array(pose, float)
        self.vl = self.vr = 0.0
        self.level, self.rng = level, rng
        self.t_next = 0.0
        self.t = 0.0
        self.slip, self.icr = 0.0, 1.0
        self.B = v(PARAMS, "track")
        self.mult = SLIP_LEVELS[level][2]
        self.vmax, self.amax = v(PARAMS, "v_wheel_max"), v(PARAMS, "a_wheel_max")

    @property
    def w_max(self):
        """Fastest commanded turn in place."""
        return 2 * self.vmax / (self.B * self.mult)

    def _draw(self):
        m, sd, im, isd = SLIP_LEVELS[self.level]
        self.slip = float(np.clip(self.rng.normal(m, sd), 0.0, 0.6))
        self.icr = float(max(1.0, self.rng.normal(im, isd)))
        self.t_next = self.t + v(PARAMS, "slip_tau")

    def step(self, cmd_v, cmd_w, dt):
        if self.t >= self.t_next:
            self._draw()
        tl, tr = cmd_v - cmd_w * self.B * self.mult / 2, cmd_v + cmd_w * self.B * self.mult / 2
        k = max(1.0, abs(tl) / self.vmax, abs(tr) / self.vmax)        # keep the turn ratio when saturating
        tl, tr = tl / k, tr / k
        da = self.amax * dt
        self.vl += np.clip(tl - self.vl, -da, da)
        self.vr += np.clip(tr - self.vr, -da, da)
        vb = (1 - self.slip) * (self.vl + self.vr) / 2
        wb = (self.vr - self.vl) / (self.icr * self.B)
        x, y, a = self.pose
        am = a + wb * dt / 2
        self.pose = np.array([x + vb * np.cos(am) * dt, y + vb * np.sin(am) * dt, (a + wb * dt + np.pi) % (2 * np.pi) - np.pi])
        self.t += dt
        return vb, wb


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def run_leg(w, robot, goal, controller, timeout=None, log=None):
    """Drive one leg with controller(pose, t) -> (v, w, done). Returns a metrics dict."""
    timeout = timeout or v(PARAMS, "leg_timeout")
    n_sub = int(round(1 / (CONTROL_HZ * SIM_DT)))
    t0 = robot.t
    min_clear, contact_steps, dist, worst = np.inf, 0, 0.0, None
    cmd = (0.0, 0.0)
    done = False
    tk = 0
    while robot.t - t0 < timeout:
        if tk % n_sub == 0:
            c = controller(robot.pose.copy(), robot.t - t0)
            cmd, done = c[:2], c[2]
            if done:
                break
        p0 = robot.pose[:2].copy()
        robot.step(cmd[0], cmd[1], SIM_DT)
        dist += float(np.hypot(*(robot.pose[:2] - p0)))
        cl = clearance(w, robot.pose)
        if cl < min_clear:
            min_clear, worst = cl, robot.pose.copy()
        contact_steps += cl < 0
        if log is not None and tk % n_sub == 0:
            log.append([robot.t, *robot.pose.tolist(), cmd[0], cmd[1]])
        tk += 1
    # stop: let the wheels spin down so the next leg starts from rest (counted in the leg time)
    while abs(robot.vl) + abs(robot.vr) > 1e-6:
        robot.step(0.0, 0.0, SIM_DT)
    e_xy = float(np.hypot(*(robot.pose[:2] - np.asarray(goal[:2]))))
    e_yaw = float(abs(np.degrees(wrap(robot.pose[2] - goal[2]))))
    return {"time": robot.t - t0, "reached": bool(done), "e_xy": e_xy, "e_yaw": e_yaw, "dist": dist,
            "min_clearance": float(min_clear), "contact_s": contact_steps * SIM_DT,
            "worst_pose": None if worst is None else [round(float(x), 3) for x in worst]}
