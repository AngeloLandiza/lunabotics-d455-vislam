"""Turning a route (a polyline of base_link positions) into robot motion, shared by every planner in this folder that
outputs a geometric route, so route planners can be compared with the same execution.

A skid-steer can turn in place, so the route is split at sharp corners: the robot stops there and turns. Each piece
between sharp corners gets its gentle corners rounded (custom_gcs.fillet) and a minimum-time speed profile (TOPP-RA,
custom_gcs.Profile). Actions: back out (if the robot cannot turn where it stands), turn, follow, turn, follow, ...,
then a final correction to the goal pose. The footprint is checked along every piece (heading on the tangent) and the
turning circle at every turn.
"""
import numpy as np

import arena2d as a2

SHARP = np.radians(45.0)          # corners sharper than this become stop-and-turn points (Assumed)
CHECK_MARGIN = 0.04               # m, clearance every planned pose and turn must keep
GAINS = (16.0, 8.0)               # Kanayama ky (1/m^2), ktheta (1/m); picked on tuning arenas 1000-1011 from (8, 3), (8, 6),
                                  # (16, 8), (8, 10): a stiff heading term keeps the 1 m body from yawing into obstacles
APPROACH_MIN = 0.25               # m, a final straight run along the goal heading this long is driven as an approach


def backup_distance(w, pose, limit=0.6, step=0.05):
    """How far to back out straight before the robot can turn in place (0 if it already can)."""
    p, d = np.array(pose, float), 0.0
    while a2.turn_clearance(w, p[:2]) < CHECK_MARGIN and d < limit:
        q = p - step * np.array([np.cos(p[2]), np.sin(p[2]), 0])
        if a2.clearance(w, q) < CHECK_MARGIN:
            break
        p, d = q, d + step
    return d, p


def split(poly, sharp=SHARP):
    """Pieces of the polyline between corners sharper than `sharp` (each piece keeps its end vertices)."""
    P = np.asarray(poly, float)
    cuts = [0]
    for i in range(1, len(P) - 1):
        a, b = P[i] - P[i - 1], P[i + 1] - P[i]
        cos = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
        if np.arccos(np.clip(cos, -1, 1)) > sharp:
            cuts.append(i)
    cuts.append(len(P) - 1)
    return [P[cuts[k]:cuts[k + 1] + 1] for k in range(len(cuts) - 1)]


def turn_sweep_clear(w, xy, th0, th1, step=np.radians(4.0)):
    """Smallest footprint clearance while turning in place at xy from th0 to th1 the short way (as Executor turns)."""
    e = a2.wrap(th1 - th0)
    n = max(int(np.ceil(abs(e) / step)), 1)
    return min(a2.clearance(w, (xy[0], xy[1], th0 + e * f)) for f in np.linspace(0, 1, n + 1))


def approach_start(poly, goal, min_len=APPROACH_MIN):
    """Index of the point where the route's final straight run along the goal heading begins, if that run is at least
    min_len long (else None)."""
    P, g = np.asarray(poly, float), np.asarray(goal, float)
    h = np.array([np.cos(g[2]), np.sin(g[2])])
    k = len(P) - 1
    while k > 0:
        q = g[:2] - P[k - 1]
        if abs(q[0] * h[1] - q[1] * h[0]) > 2e-3 or q @ h <= 0:
            break
        k -= 1
    return k if (g[:2] - P[k]) @ h >= min_len else None


def build(w, robot, poly, goal, start_heading=None, sharp=SHARP):
    """Actions for a route, or (None, reason) if the footprint would touch something: along every piece, and while
    turning in place at its start (from the robot's heading for the first piece) and at the goal. A final straight run
    along the goal heading becomes a stop, an aim at the goal and a straight drive in (see Executor 'line')."""
    from custom_gcs import Profile, fillet
    actions = []
    g = np.asarray(goal, float)
    ka = approach_start(poly, g)
    main = np.asarray(poly, float)[: ka + 1] if ka is not None else np.asarray(poly, float)
    pieces = split(main, sharp) if len(main) > 1 else []
    heading = start_heading
    for k, piece in enumerate(pieces):
        if np.hypot(*(piece[-1] - piece[0])) < 1e-3:
            continue
        prof = Profile(fillet(piece), robot.B * robot.mult, robot.vmax, robot.amax)
        if not prof.ok:
            return None, "profile"
        if min(a2.clearance(w, q) for q in prof.poses()[::2]) < CHECK_MARGIN:
            return None, "footprint"
        th = float(prof.sth(0))
        if heading is not None and turn_sweep_clear(w, piece[0], heading, th) < CHECK_MARGIN:
            return None, "turn"
        actions.append(("turn", th))
        actions.append(("follow", prof))
        heading = float(prof.sth(prof.length))
    if ka is not None:
        A = main[-1]
        if heading is not None and turn_sweep_clear(w, A, heading, g[2]) < CHECK_MARGIN:
            return None, "turn"
        n = max(int(np.hypot(*(g[:2] - A)) / 0.03), 1)
        if min(a2.clearance(w, (*p, g[2])) for p in np.linspace(A, g[:2], n + 1)) < CHECK_MARGIN:
            return None, "footprint"
        actions.append(("line", (g[:2].copy(), 1, 0.4)))
        heading = g[2]
    if heading is not None and turn_sweep_clear(w, g[:2], heading, g[2]) < CHECK_MARGIN:
        return None, "turn at goal"
    actions.append(("final", g))
    return actions, None


class Executor:
    """run_leg controller for a list of actions."""

    def __init__(self, robot, actions, backup=0.0):
        self.r = robot
        self.actions = ([("backup", backup)] if backup > 0 else []) + list(actions)
        self.k = 0
        self.tol = a2.v(a2.PARAMS, "goal_tol")
        self.state = {}

    def _still(self):
        """Wheels (encoders) nearly stopped."""
        return abs(self.r.vl) + abs(self.r.vr) < 0.02

    def _turn_rate(self, e):
        """Turn-in-place rate that can still stop at the target heading with the wheels' acceleration limit."""
        alpha = 0.8 * 2 * self.r.amax / (self.r.B * self.r.mult)
        w = min(0.9 * self.r.w_max, np.sqrt(2 * alpha * abs(e)), 4.0 * abs(e))
        return float(np.sign(e) * w)

    def _next(self):
        self.k += 1
        self.state = {}

    def __call__(self, pose, t):
        r = self.r
        wmax = 0.9 * r.w_max
        while self.k < len(self.actions):
            kind, arg = self.actions[self.k]
            if kind in ("turn", "line", "final") and not self.state.get("settled"):
                # come to rest first: a turn commanded while the wheels still roll forward becomes an arc that
                # carries the robot off the route (and, before a run in, off the line to the dump spot)
                if not self._still():
                    return 0.0, 0.0, False
                self.state["settled"] = True
            if kind == "backup":
                st = self.state.setdefault("xy", pose[:2].copy())
                left = arg - np.hypot(*(pose[:2] - st))
                if left <= 0.01:
                    self._next()
                    continue
                return -min(0.3, 1.5 * left + 0.05), 0.0, False
            if kind == "turn":
                e = a2.wrap(arg - pose[2])
                if abs(e) < np.radians(2.0) and self._still():
                    self._next()
                    continue
                return 0.0, self._turn_rate(e), False
            if kind == "follow":
                prof = arg
                s = self.state["s"] = prof.project(pose[:2], self.state.get("s", 0.0))
                left = prof.length - s
                end = np.array([float(prof.sx(prof.length)), float(prof.sy(prof.length))])
                if left < 0.02 or (left < 0.10 and np.hypot(*(pose[:2] - end)) < 0.03):
                    self._next()
                    continue
                xr, yr, thr = float(prof.sx(s)), float(prof.sy(s)), float(prof.sth(s))
                kap = float(prof.sth(s, 1))
                ey = -np.sin(pose[2]) * (xr - pose[0]) + np.cos(pose[2]) * (yr - pose[1])
                eth = a2.wrap(thr - pose[2])
                vr = max(prof.speed_at(s + 0.05), min(0.6 * left, 0.15), 0.05)
                ky, kth = GAINS
                vc = vr * np.cos(eth)
                wc = kap * vr + vr * (ky * ey + kth * np.sin(eth))
                return float(np.clip(vc, -r.vmax, r.vmax)), float(np.clip(wc, -r.w_max, r.w_max)), False
            if kind == "line":
                # straight to a point, forwards (+1) or backwards (-1): aim from where the robot really is, then drive
                # straight. Used for the run in to the goal: a lateral error left by the previous piece then becomes a
                # small heading error at the goal instead of a sideways miss (the dump spot is 5 cm from the berm).
                g, sgn, vcap = arg
                dv = g - pose[:2]
                along = sgn * (np.cos(pose[2]) * dv[0] + np.sin(pose[2]) * dv[1])
                aim = np.arctan2(dv[1], dv[0]) + (0.0 if sgn > 0 else np.pi)
                if not self.state.get("aimed"):
                    e = a2.wrap(aim - pose[2])
                    if np.hypot(*dv) > 0.03 and abs(e) > np.radians(1.0):
                        return 0.0, self._turn_rate(e), False
                    if not self._still():
                        return 0.0, 0.0, False
                    self.state["aimed"] = True
                if along < 0.015:
                    self._next()
                    continue
                eb = a2.wrap(aim - pose[2]) if along > 0.10 else 0.0
                sp = min(vcap, np.sqrt(2 * 0.8 * r.amax * along), 1.2 * along + 0.03)
                return float(sgn * sp), float(np.clip(2.0 * eb, -0.5, 0.5)), False
            if kind == "final":
                g = arg
                d = np.hypot(*(g[:2] - pose[:2]))
                if d > 0.15 and not self.state.get("placed"):
                    # far from the spot (the follow went wrong): drive there, forwards or backwards
                    bearing = np.arctan2(g[1] - pose[1], g[0] - pose[0])
                    eb = a2.wrap(bearing - pose[2])
                    sgn = 1.0 if abs(eb) <= np.pi / 2 else -1.0
                    eb = eb if sgn > 0 else a2.wrap(eb + np.pi)
                    if abs(eb) > np.radians(25):
                        return 0.0, self._turn_rate(eb), False
                    vmax_stop = np.sqrt(2 * 0.8 * r.amax * d)         # can still stop at the spot
                    return float(sgn * min(0.3, vmax_stop, 1.2 * d + 0.03)), float(np.clip(3.0 * eb, -wmax, wmax)), False
                if not self.state.get("placed"):
                    # near the spot: close the distance along the current heading only; steering here would swing the
                    # corners, and the spot can be 5 cm from the berm. What is left sideways stays.
                    along = np.cos(pose[2]) * (g[0] - pose[0]) + np.sin(pose[2]) * (g[1] - pose[1])
                    if abs(along) > 0.015:
                        sp = min(0.2, np.sqrt(2 * 0.8 * r.amax * abs(along)), 1.2 * abs(along) + 0.02)
                        return float(np.sign(along) * sp), 0.0, False
                    if not self._still():
                        return 0.0, 0.0, False
                    self.state["placed"] = True
                # then turn in place to the goal heading (build checked this turn's sweep)
                e = a2.wrap(g[2] - pose[2])
                if abs(e) < np.radians(2.0):
                    return 0.0, 0.0, self._still()
                if d > self.tol[0]:
                    self.state["placed"] = False
                return 0.0, self._turn_rate(e), False
        return 0.0, 0.0, True
