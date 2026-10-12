"""Paths and wheel-rut stamping for drives, shared by isaac_drive_ruts.py (Isaac Sim) and isaac_drive_eval.py (plain
Python), so both rebuild exactly the same ruts frame by frame.

Paths are lists of (x, y, yaw_deg) robot poses (robot frame origin on the ground under base_link):
- approach_path(seed): approach.trial(seed)'s straight line, one pose every approach.DS (isaac_approach.approach_poses).
- trip_path(arena, rng): one obstacle-avoiding trip from ruts.earlier_trips (straight legs between waypoints whose
  corridor misses every obstacle), resampled every DS along each leg; at each corner the robot pivots in place in
  steps of PIVOT_STEP degrees (skid steer), one pose per step.
"""
import numpy as np

from approach import DS
from ruts import earlier_trips

PIVOT_STEP = 15.0   # deg per frame while turning in place (Assumed)


def approach_path(seed):
    from isaac_approach import approach_poses
    return approach_poses(seed)


def trip_path(arena, rng, ds=DS):
    trip = earlier_trips(arena, rng, 1)[0]
    legs = [(trip[i], trip[i + 1]) for i in range(0, len(trip), 2)]
    out = []
    for n, (a, b) in enumerate(legs):
        yaw = a[2]
        if out:                                              # pivot from the previous heading to this leg's
            y0 = out[-1][2]
            turn = (yaw - y0 + 180) % 360 - 180
            steps = int(np.ceil(abs(turn) / PIVOT_STEP))
            for k in range(1, steps + 1):
                out.append((a[0], a[1], y0 + turn * k / steps))
        L = np.hypot(b[0] - a[0], b[1] - a[1])
        m = int(np.floor(L / ds + 1e-9))
        start = 1 if out else 0
        for k in range(start, m + 1):
            f = k * ds / L
            out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), yaw))
    return out


def wheel_track_points(poses, k, x_rear, x_front):
    """Robot-centre path that the wheel tracks follow up to frame k: from the rear wheels' start to the front wheels
    now (straight extensions along the first and current headings)."""
    x0, y0, yaw0 = poses[0]
    xk, yk, yawk = poses[k]
    h0 = np.array([np.cos(np.radians(yaw0)), np.sin(np.radians(yaw0))])
    hk = np.array([np.cos(np.radians(yawk)), np.sin(np.radians(yawk))])
    start = (x0 + x_rear * h0[0], y0 + x_rear * h0[1], yaw0)
    end = (xk + x_front * hk[0], yk + x_front * hk[1], yawk)
    return [start] + list(poses[: k + 1]) + [end]


def stamp_to(ruts, poses, k, x_rear, x_front, lat):
    """Ruts for every wheel track up to frame k; ruts.apply() writes them into the arena height field."""
    ruts.path(wheel_track_points(poses, k, x_rear, x_front), lat)
    ruts.apply()


def track_segments(poses, k, x_rear, x_front, lat):
    """Wheel-track centre lines up to frame k as segments ((x0, y0), (x1, y1)), one list per lateral offset."""
    P = wheel_track_points(poses, k, x_rear, x_front)
    segs = []
    for la in lat:
        pts = [(x - la * np.sin(np.radians(yw)), y + la * np.cos(np.radians(yw))) for x, y, yw in P]
        segs += [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    return segs


def dist_to_segments(x, y, segs):
    """Distance from points (x, y) to the nearest segment."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    best = np.full(x.shape, np.inf)
    for (ax, ay), (bx, by) in segs:
        dx, dy = bx - ax, by - ay
        L2 = max(dx * dx + dy * dy, 1e-12)
        s = np.clip(((x - ax) * dx + (y - ay) * dy) / L2, 0, 1)
        best = np.minimum(best, np.hypot(x - (ax + s * dx), y - (ay + s * dy)))
    return best
