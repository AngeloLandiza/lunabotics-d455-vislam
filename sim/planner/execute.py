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
CHECK_MARGIN = 0.02               # m, clearance every planned pose and turn must keep
GAINS = (8.0, 3.0)                # Kanayama ky (1/m^2), ktheta (1/m); Assumed, hand-set


def backup_distance(w, pose, limit=0.6, step=0.05):
    """How far to back out straight before the robot can turn in place (0 if it already can)."""
    p, d = np.array(pose, float), 0.0
    while a2.turn_clearance(w, p[:2]) < CHECK_MARGIN and d < limit:
        q = p - step * np.array([np.cos(p[2]), np.sin(p[2]), 0])
        if a2.clearance(w, q) < CHECK_MARGIN:
            break
        p, d = q, d + step
    return d, p


def split(poly):
    """Pieces of the polyline between sharp corners (each piece keeps its end vertices)."""
    P = np.asarray(poly, float)
    cuts = [0]
    for i in range(1, len(P) - 1):
        a, b = P[i] - P[i - 1], P[i + 1] - P[i]
        cos = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
        if np.arccos(np.clip(cos, -1, 1)) > SHARP:
            cuts.append(i)
    cuts.append(len(P) - 1)
    return [P[cuts[k]:cuts[k + 1] + 1] for k in range(len(cuts) - 1)]


def build(w, robot, poly, goal):
    """Actions for a route, or (None, reason) if the footprint would touch something."""
    from custom_gcs import Profile, fillet
    actions = []
    pieces = split(poly)
    for k, piece in enumerate(pieces):
        if np.hypot(*(piece[-1] - piece[0])) < 1e-3:
            continue
        prof = Profile(fillet(piece), robot.B * robot.mult, robot.vmax, robot.amax)
        if not prof.ok:
            return None, "profile"
        if min(a2.clearance(w, q) for q in prof.poses()[::2]) < CHECK_MARGIN:
            return None, "footprint"
        if k > 0 and a2.turn_clearance(w, piece[0]) < CHECK_MARGIN:
            return None, "turn"
        actions.append(("turn", float(prof.sth(0))))
        actions.append(("follow", prof))
    actions.append(("final", np.asarray(goal, float)))
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
            if kind == "final":
                g = arg
                d = np.hypot(*(g[:2] - pose[:2]))
                if d > self.tol[0] * 0.5 and not self.state.get("placed"):
                    # drive to the spot: forwards if it is ahead, backwards if it is behind (no turn needed)
                    bearing = np.arctan2(g[1] - pose[1], g[0] - pose[0])
                    eb = a2.wrap(bearing - pose[2])
                    sgn = 1.0 if abs(eb) <= np.pi / 2 else -1.0
                    eb = eb if sgn > 0 else a2.wrap(eb + np.pi)
                    if abs(eb) > np.radians(25) and d > 0.15:
                        return 0.0, self._turn_rate(eb), False
                    vmax_stop = np.sqrt(2 * 0.8 * r.amax * d)         # can still stop at the spot
                    return float(sgn * min(0.3, vmax_stop, 1.2 * d + 0.03)), float(np.clip(3.0 * eb, -wmax, wmax)), False
                self.state["placed"] = True
                e = a2.wrap(g[2] - pose[2])
                if abs(e) < np.radians(self.tol[1]) * 0.5:
                    return 0.0, 0.0, self._still()
                if d > self.tol[0]:
                    self.state["placed"] = False
                return 0.0, self._turn_rate(e), False
        return 0.0, 0.0, True
