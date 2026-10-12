"""arena-GCS-TOPP: the custom planner candidate as planned (plan doc, Step 2, "Custom planner versus Nav2").

1. Free space as convex regions: IRIS (Deits and Tedrake 2014, Drake's implementation), seeded first in the middle of
   every gap between grown obstacles, then on a grid in order of clearance. Obstacles are the discs and the berm box,
   grown by `inflate`, so the regions hold base_link positions.
2. A route through the regions: GCS (Marcucci et al., Drake's GcsTrajectoryOptimization) with one straight piece per
   region, minimum length, solved by the convex relaxation plus rounding with Clarabel (open source; the papers used
   MOSEK). The last piece runs along the goal heading. Cubic Bezier pieces were tried first and took about 7 s per
   solve against about 1 s, so the straight pieces are kept and their corners rounded afterwards (fillet).
3. A minimum-time speed profile along the route: TOPP-RA (Pham and Pham 2018, the toppra package) with each wheel's
   speed and acceleration limits. The path is written in wheel coordinates, q_l,r(s) = s -+ (B mult / 2)(theta(s) -
   theta(0)), so the wheel limits are TOPP-RA's ordinary joint limits.
4. Execution (execute.py, shared with the other route planners): back out if the robot cannot turn where it stands,
   stop and turn at sharp corners, follow each piece at TOPP-RA's speed for the robot's current path position, and
   steer with the Kanayama et al. (1990) law on the closest path point.

The footprint is checked along the route with the heading on the tangent; if it touches anything, the regions are
rebuilt with the next inflation in INFLATIONS. Planning uses Drake, which has no Linux ARM wheel on PyPI (checked
2026-10-12), so the Orin NX version would need its own GCS on an ARM-ready solver; this file is for the comparison.
"""
import time

import numpy as np
from scipy.interpolate import CubicSpline

import arena2d as a2

INFLATIONS = (0.45, 0.40, 0.37)   # m: half-width 0.353 + 0.1 margin, then less where passages are narrow
SEED_GRID = 0.20                  # m, candidate IRIS seeds
MAX_REGIONS = 30


def _disc_poly(x, y, r, n=16):
    from pydrake.geometry.optimization import VPolytope
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    R = r / np.cos(np.pi / n)       # circumscribe the disc
    return VPolytope(np.stack([x + R * np.cos(a), y + R * np.sin(a)]))


class Regions:
    """IRIS regions covering the free space. Seeds: first the middle of every gap between two grown obstacles (or an
    obstacle and a wall) wide enough to pass, so narrow passages get a region, then a grid in order of clearance."""

    def __init__(self, w, inflate):
        from pydrake.geometry.optimization import HPolyhedron, Iris, IrisOptions, VPolytope
        t0 = time.time()
        self.inflate = inflate
        self.obs = [_disc_poly(o.x, o.y, o.r + inflate) for o in w.obstacles]
        b = w.berm
        self.obs.append(VPolytope(np.array([[b[0] - inflate, b[1] + inflate, b[1] + inflate, b[0] - inflate],
                                            [b[2] - inflate, b[2] - inflate, b[3] + inflate, b[3] + inflate]])))
        lo, hi = np.array([inflate, inflate]), np.array([w.L - inflate, w.W - inflate])
        self.dom = HPolyhedron.MakeBox(lo, hi)
        self.opts = IrisOptions()
        self.opts.require_sample_point_is_contained = True
        # grown obstacles as discs (the berm box as its four corner discs) for the gap seeds and clearances
        discs = [(o.x, o.y, o.r + inflate) for o in w.obstacles] + [(x, y, inflate) for x in b[:2] for y in b[2:]]
        self.free = lambda p: self._clear(p, discs, b, lo, hi) > 0.01
        seeds = []
        for i in range(len(discs)):
            xi, yi, ri = discs[i]
            for j in range(i + 1, len(discs)):
                xj, yj, rj = discs[j]
                d = np.hypot(xj - xi, yj - yi)
                if d - ri - rj > 0.02:
                    u = np.array([xj - xi, yj - yi]) / d
                    seeds.append(np.array([xi, yi]) + u * (ri + (d - ri - rj) / 2))
            for k, (pt, gap) in enumerate([((xi - ri) / 2 + lo[0] / 2, xi - ri - lo[0]), ((xi + ri + hi[0]) / 2, hi[0] - xi - ri),
                                           ((yi - ri) / 2 + lo[1] / 2, yi - ri - lo[1]), ((yi + ri + hi[1]) / 2, hi[1] - yi - ri)]):
                if gap > 0.02:
                    seeds.append(np.array([pt, yi]) if k < 2 else np.array([xi, pt]))
        xs, ys = np.meshgrid(np.arange(lo[0], hi[0] + 1e-9, SEED_GRID), np.arange(lo[1], hi[1] + 1e-9, SEED_GRID))
        grid = np.stack([xs.ravel(), ys.ravel()], 1)
        clr = np.array([self._clear(p, discs, b, lo, hi) for p in grid])
        grid = grid[np.argsort(-clr, kind="stable")][np.sort(clr)[::-1] > 0.01]
        self.regions = []
        for p in [q for q in seeds if self.free(q)] + list(grid):
            if len(self.regions) >= MAX_REGIONS:
                break
            self.add(p)
        cov = [self.contains(p) for p in grid]
        self.coverage = float(np.mean(cov)) if cov else 0.0
        self.seconds = time.time() - t0

    def _clear(self, p, discs, b, lo, hi):
        c = min(np.hypot(p[0] - x, p[1] - y) - r for x, y, r in discs)
        dx, dy = max(b[0] - p[0], 0, p[0] - b[1]), max(b[2] - p[1], 0, p[1] - b[3])
        inside = -min(p[0] - b[0], b[1] - p[0], p[1] - b[2], b[3] - p[1]) if dx == dy == 0 else np.hypot(dx, dy)
        return min(c, inside - self.inflate, p[0] - lo[0], hi[0] - p[0], p[1] - lo[1], hi[1] - p[1])

    def add(self, p):
        """Grow one more region from p unless p is already covered or not free."""
        from pydrake.geometry.optimization import Iris
        p = np.asarray(p, float)
        if self.contains(p) or not self.free(p):
            return False
        try:
            self.regions.append(Iris(self.obs, p, self.dom, self.opts))
        except RuntimeError:
            return False
        return True

    def contains(self, p):
        return any(r.PointInSet(np.asarray(p, float)) for r in self.regions)


def gcs_route(regions, start, goal, goal_heading, final_straight=0.25):
    """Shortest polyline start -> goal through the regions (one straight piece per region), whose last piece runs
    along goal_heading for at least final_straight metres. Returns the polyline vertices (N, 2), or None."""
    from pydrake.geometry.optimization import GraphOfConvexSetsOptions, Point
    from pydrake.planning import GcsTrajectoryOptimization
    from pydrake.solvers import ClarabelSolver
    gcs = GcsTrajectoryOptimization(2)
    main = gcs.AddRegions(regions.regions, order=1)
    src = gcs.AddRegions([Point(np.asarray(start, float))], order=0)
    tgt = gcs.AddRegions([Point(np.asarray(goal, float))], order=0)
    gcs.AddEdges(src, main)
    out = gcs.AddEdges(main, tgt)
    u, _ = out.edge_constituent_vertex_control_points()
    d = u[:, 1] - u[:, 0]
    h = np.array([np.cos(goal_heading), np.sin(goal_heading)])
    out.AddEdgeConstraint(d[0] * h[1] - d[1] * h[0] == 0)      # last piece along the goal heading ...
    out.AddEdgeConstraint(d[0] * h[0] + d[1] * h[1] >= final_straight)    # ... and at least this long
    gcs.AddPathLengthCost(1.0)
    opts = GraphOfConvexSetsOptions()
    opts.convex_relaxation = True
    opts.preprocessing = True
    opts.max_rounded_paths = 5
    opts.solver = ClarabelSolver()
    traj, res = gcs.SolvePath(src, tgt, opts)
    if not res.is_success():
        return None
    pts = [np.asarray(start, float)]
    for k in range(traj.get_number_of_segments()):
        cp = np.asarray(traj.segment(k).control_points())
        pts.extend(cp.reshape(2, -1).T)
    pts.append(np.asarray(goal, float))
    out_pts = [pts[0]]
    for p in pts[1:]:
        if np.hypot(*(p - out_pts[-1])) > 1e-4:
            out_pts.append(p)
    return np.array(out_pts)


def fillet(poly, max_cut=0.05, step=0.01):
    """Round each corner of a polyline with a circular arc that cuts the corner by at most max_cut metres and keeps
    its tangent points inside the two pieces (at most half of each). Returns densely sampled points."""
    P = np.asarray(poly, float)
    if len(P) < 3:
        n = max(int(np.hypot(*(P[-1] - P[0])) / step), 2)
        return np.linspace(P[0], P[-1], n)
    out = [P[0]]
    prev_end = P[0]
    for i in range(1, len(P) - 1):
        a, b = P[i] - P[i - 1], P[i + 1] - P[i]
        la, lb = np.linalg.norm(a), np.linalg.norm(b)
        ua, ub = a / la, b / lb
        phi = np.arccos(np.clip(np.dot(ua, ub), -1, 1))          # turn angle
        if phi < 1e-3:
            continue
        lim_seg = min(la if i == 1 else la / 2, lb if i == len(P) - 2 else lb / 2)
        rho_seg = lim_seg / np.tan(phi / 2)
        rho_cut = max_cut / (1 / np.cos(phi / 2) - 1) if phi < np.pi - 1e-3 else 0.0
        rho = min(rho_seg, rho_cut)
        t = rho * np.tan(phi / 2)
        p_in, p_out = P[i] - ua * t, P[i] + ub * t
        n = max(int(np.hypot(*(p_in - prev_end)) / step), 1)
        out.extend(np.linspace(prev_end, p_in, n + 1)[1:])
        if rho > 1e-4:
            nrm = np.array([-ua[1], ua[0]]) * np.sign(ua[0] * ub[1] - ua[1] * ub[0])
            c = p_in + nrm * rho
            a0, a1 = np.arctan2(*(p_in - c)[::-1]), np.arctan2(*(p_out - c)[::-1])
            da = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
            m = max(int(abs(da) * rho / step), 2)
            out.extend(c + rho * np.stack([np.cos(a0 + da * np.linspace(0, 1, m)), np.sin(a0 + da * np.linspace(0, 1, m))], 1)[1:])
        prev_end = p_out
    n = max(int(np.hypot(*(P[-1] - prev_end)) / step), 1)
    out.extend(np.linspace(prev_end, P[-1], n + 1)[1:])
    return np.array(out)


class Profile:
    """Arc-length spline of a route and its TOPP-RA timing in wheel coordinates (without the toppra package:
    the same limits by a forward-backward pass in the phase plane, _numpy_profile)."""

    def __init__(self, pts, B_eff, vmax, amax, ds=0.01):
        try:
            import toppra as ta
            import toppra.algorithm as algo
            import toppra.constraint as constraint
        except ImportError:
            ta = None
        seg = np.hypot(*np.diff(pts, axis=0).T)
        keep = np.concatenate([[True], seg > 1e-6])
        pts = pts[keep]
        s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
        self.length = float(s[-1])
        n = max(int(self.length / ds) + 1, 4)
        ss = np.linspace(0, self.length, n)
        cx, cy = CubicSpline(s, pts[:, 0]), CubicSpline(s, pts[:, 1])
        x, y = cx(ss), cy(ss)
        # re-spline on the resampled arc length so the parameter is close to true arc length
        s2 = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        self.sx, self.sy = CubicSpline(s2, x), CubicSpline(s2, y)
        self.length = float(s2[-1])
        th = np.unwrap(np.arctan2(self.sy(s2, 1), self.sx(s2, 1)))
        self.sth = CubicSpline(s2, th)
        if ta is None:
            self._numpy_profile(s2, B_eff, vmax, amax)
            return
        q = np.stack([s2 - B_eff / 2 * (th - th[0]), s2 + B_eff / 2 * (th - th[0]), s2], 1)
        path = ta.SplineInterpolator(s2, q)
        vlim = np.array([[-vmax, vmax], [-vmax, vmax], [-10.0, 10.0]])
        alim = np.array([[-amax, amax], [-amax, amax], [-100.0, 100.0]])
        inst = algo.TOPPRA([constraint.JointVelocityConstraint(vlim), constraint.JointAccelerationConstraint(alim)],
                           path, parametrizer="ParametrizeConstAccel", gridpoints=np.linspace(0, self.length, min(n, 400)))
        self.traj = inst.compute_trajectory(0, 0)
        self.ok = self.traj is not None
        self.T = float(self.traj.duration) if self.ok else np.inf
        if self.ok:
            tt = np.linspace(0, self.T, 2000)
            st = np.clip(self.traj(tt)[:, 2], 0, self.length)
            sd = np.clip(self.traj(tt, 1)[:, 2], 0, None)
            order = np.argsort(st, kind="stable")
            self._s, self._sd = st[order], sd[order]

    def _numpy_profile(self, s, B_eff, vmax, amax):
        """Minimum-time path speed under each wheel's speed and acceleration limits, starting and ending at rest.

        Wheel speeds are sd (1 -+ B k / 2) with k the path curvature, so the speed limit is vmax / (1 + B |k| / 2);
        wheel accelerations are sdd (1 -+ B k / 2) -+ sd^2 B k' / 2. On the grid of x = sd^2, a forward pass
        accelerates as hard as both wheels allow and a backward pass brakes as hard (the classic phase-plane method that
        TOPP-RA solves exactly; here the acceleration bound is taken at the start of each step, slightly conservative).
        """
        k = self.sth(s, 1)
        dk = self.sth(s, 2)
        c = np.stack([1 - B_eff * k / 2, 1 + B_eff * k / 2])          # wheel speed per unit path speed
        d = np.stack([-B_eff * dk / 2, B_eff * dk / 2])               # wheel acceleration per unit sd^2
        xmax = (vmax / np.maximum(np.abs(c).max(0), 1e-6)) ** 2

        def acc(x, i):
            lim = (amax - np.abs(d[:, i]) * x) / np.maximum(np.abs(c[:, i]), 1e-6)
            return max(float(lim.min()), 1e-3)
        ds = np.diff(s)
        x = np.minimum(xmax, np.inf)
        x[0] = 0.0
        for i in range(len(s) - 1):
            x[i + 1] = min(x[i + 1], x[i] + 2 * ds[i] * acc(x[i], i))
        x[-1] = 0.0
        for i in range(len(s) - 1, 0, -1):
            x[i - 1] = min(x[i - 1], x[i] + 2 * ds[i - 1] * acc(x[i], i))
        sd = np.sqrt(np.maximum(x, 0))
        self._s, self._sd = s, sd
        dt = 2 * ds / np.maximum(sd[1:] + sd[:-1], 1e-6)
        self.T = float(dt.sum())
        self.ok = True
        self.traj = None

    def speed_at(self, s):
        """TOPP-RA's path speed at path position s (m/s)."""
        return float(np.interp(s, self._s, self._sd))

    def project(self, xy, s_prev, window=0.6, step=0.005):
        """Path position closest to xy, searched near s_prev so the robot never skips ahead across a loop."""
        ss = np.arange(max(0.0, s_prev - 0.1), min(self.length, s_prev + window) + step, step)
        d = (self.sx(ss) - xy[0]) ** 2 + (self.sy(ss) - xy[1]) ** 2
        return float(ss[np.argmin(d)])

    def ref(self, t):
        t = float(np.clip(t, 0, self.T))
        s = float(np.clip(self.traj(t)[2], 0, self.length))
        sd = float(self.traj(t, 1)[2])
        th = float(self.sth(s))
        k = float(self.sth(s, 1))
        return float(self.sx(s)), float(self.sy(s)), th, sd, k * sd

    def poses(self, step=0.02):
        ss = np.linspace(0, self.length, max(int(self.length / step), 2))
        return np.stack([self.sx(ss), self.sy(ss), self.sth(ss)], 1)


class GcsController:
    """Plans one leg at construction (regions, route, pieces, profiles), then drives it as the run_leg controller."""

    def __init__(self, w, robot, goal, regions_cache):
        import execute as ex
        self.goal = np.asarray(goal, float)
        t0 = time.time()
        self.info = {"inflate": None, "regions_s": 0.0, "solve_s": 0.0, "fail": None, "pieces": 0, "length": None}
        backup, p = ex.backup_distance(w, robot.pose)
        actions, why = None, []
        for inf in INFLATIONS:
            if inf not in regions_cache:
                regions_cache[inf] = Regions(w, inf)
                self.info["regions_s"] += regions_cache[inf].seconds
            reg = regions_cache[inf]
            ta = time.time()
            reg.add(p[:2])                  # grow a region from the start or goal if none holds them yet
            reg.add(self.goal[:2])
            self.info["regions_s"] += time.time() - ta
            if not (reg.contains(p[:2]) and reg.contains(self.goal[:2])):
                why.append(f"{inf}: no region")
                continue
            ts = time.time()
            poly = gcs_route(reg, p[:2], self.goal[:2], self.goal[2])
            self.info["solve_s"] += time.time() - ts
            if poly is None:
                why.append(f"{inf}: no route")
                continue
            actions, reason = ex.build(w, robot, poly, self.goal, start_heading=p[2])
            why.append(f"{inf}: {reason}")
            if actions is not None:
                self.info.update(inflate=inf, pieces=sum(a[0] in ("follow", "line") for a in actions),
                                 length=float(np.hypot(*np.diff(poly, axis=0).T).sum()))
                break
        self.info["fail"] = None if actions is not None else "; ".join(why)
        self.info["plan_s"] = time.time() - t0
        self.run = ex.Executor(robot, actions or [], backup) if actions is not None else None

    def __call__(self, pose, t):
        if self.run is None:
            return 0.0, 0.0, True
        return self.run(pose, t)
