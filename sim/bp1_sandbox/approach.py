"""Approach runs: the robot drives a straight line through the obstacle zone, and the log-odds detector updates
frame by frame, as it would on the robot. For every frame it records each candidate object (kind, position, pooled
evidence, what it really is) and how far ahead the path has been observed. speed_gate.py turns these records into
missed-hazard rates, phantom stops and speeds for any threshold and speed policy, without re-running the scans.

Usage: python3 approach.py [n_arenas] [out_dir] [first_seed] [--rms M] [--undulation M] [--pitch-err DEG] [--clear-only]
       [--ground plane|local] [--ground-sigma M] [--ground-spread M] [--seeds S1,S2,...] [--workers N]
       [--robot] [--lidar spin16|l1] [--body none|raw|box|table] [--inflate DEG] [--mask-err DEG]
       [--ruts M] [--rut-trips N] [--sensors lidar,d455] [--frame-s S] [--label TEXT]
--robot takes the sensor mounts and the robot's size from the team's URDF (robot.py); --body none keeps the body
transparent, raw lets it block and return rays with no mask, box and table add the self-mask (selfmask.py).
--mask-err tilts the real LiDAR by that many degrees against its nominal mount. --ruts stamps ruts of that depth from
--rut-trips earlier drives (ruts.py) into each arena, from their own random stream. --sensors runs only the listed
sensors (the D455 never sees the body, so mask cases can skip it). --frame-s sets how long the L1 collects per frame:
frames are DS apart, so DS / speed matches the rays a robot at that speed gets (0.15 s at 1 m/s, 0.6 s at 0.25 m/s).
--rms sets the floor roughness (params.py default 0.01 m), --undulation adds long-wavelength floor waves of that RMS,
--pitch-err makes the robot's attitude estimate wrong by that many degrees (sign alternates with the seed). The extra
draws use their own random stream, so every case sees the same arenas, starts and sensor noise as the others. The
dust runs share the sensor stream, so D455 noise matches only between folders that both run or both skip dust.
Every number below is Assumed unless marked; the robot's real size, braking and frame rate replace them later.
"""
import argparse
import json
import os
import time
from multiprocessing import Pool

import numpy as np

import params as pr
from params import v
from arena import Arena
from detect import Grid, add_scan, clusters, fit_ground, logodds_delta, logodds_zref, new_layers
from dust import Plume
from raycast import flat_ground_range
from sensors import Lidar, D455Depth
import selfmask
from ruts import add_earlier_ruts

try:
    from telemetry import Telemetry
except ImportError:      # the dashboard's telemetry module is optional
    Telemetry = None

DS = 0.15            # m driven between frames (Assumed: 10 Hz at 1.5 m/s; a slower robot gets more frames per metre)
FRONT = 0.35         # m from the robot centre to its front edge (Assumed, = D455 mount x)
HALF_W = 0.40        # m, half-width of the swept corridor: robot half-width plus margin (Assumed)
X_END = 4.9          # m, robot centre stops here, short of the berm wall
PLUME_AHEAD = 1.2    # m ahead of the start, dust plume centre (Assumed: a dig site in front)
LOOK = 3.0           # m beyond the front edge searched for unobserved corridor cells
UNDULATION_WL = (1.5, 3.0)   # m, wavelength range of the floor waves (Assumed)
EXTRA_SEED = 7_000_003       # offset of the random stream for floor waves and attitude error
RUT_SEED = 9_000_011         # offset of the random stream for earlier drives (ruts)
BACK = 0.45          # m from the robot centre to its rear edge (set from the URDF with --robot)
CASE = {"undulation": 0.0, "pitch_err": 0.0, "clear_only": False, "robot": False, "lidar": "spin16", "body": "none",
        "inflate": None, "mask_err": 0.0, "ruts": 0.0, "rut_trips": 6, "sensors": ["lidar", "d455"]}
_CACHE = {}


def robot_parts():
    """Per-process robot model, body caster and self-mask (Embree scenes cannot be pickled to the workers)."""
    if "robot" not in _CACHE:
        from robot import Robot, BodyCaster
        rb = Robot()
        xyz, R = rb.mount(v(pr.ROBOT, "lidar_link"))
        body = BodyCaster(rb, v(pr.ROBOT, "lidar_link"))
        _CACHE.update(robot=rb, body=body, lidar_mount=(xyz, R), d455_mount=rb.mount_pitch(v(pr.ROBOT, "d455_link")))
        if CASE["body"] == "table":
            _CACHE["table"] = selfmask.MaskTable(body, xyz, R, CASE["inflate"])
        if CASE["body"] == "box":
            _CACHE["box"] = selfmask.box_of(rb)
    return _CACHE


def path_frame(x0, y0, yaw):
    h = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
    return h, np.array([-h[1], h[0]])


def undulate(arena, seed, rms, n=12):
    """Add long-wavelength floor waves of the given RMS inside the walls (the robot still rides at z = 0)."""
    r = np.random.default_rng(seed + EXTRA_SEED)
    lam, th, ph = r.uniform(*UNDULATION_WL, n), r.uniform(0, 2 * np.pi, n), r.uniform(0, 2 * np.pi, n)
    k = 2 * np.pi / lam
    f = sum(np.cos(k[i] * (np.cos(th[i]) * arena.X + np.sin(th[i]) * arena.Y) + ph[i]) for i in range(n))
    inside = (arena.X >= 0) & (arena.X <= arena.L) & (arena.Y >= 0) & (arena.Y <= arena.W)
    arena.h[inside] += (f * rms / np.sqrt(n / 2))[inside]


def believed(scan, robot, pitch_err):
    """The scan as the robot places it when its pitch estimate is off by pitch_err degrees: every ray and sample is
    rotated about the robot's ground point. Positive error makes the ground ahead look lower."""
    x, y, yaw = robot
    c = np.array([x, y, 0.0])
    a, p = np.radians(yaw), np.radians(pitch_err)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    R = Rz @ Ry @ Rz.T
    out = dict(scan)
    out["origin"] = c + R @ (scan["origin"] - c)
    out["dirs"] = scan["dirs"] @ R.T
    out["pts"] = c + (scan["pts"] - c) @ R.T
    out["r_flat"] = flat_ground_range(out["origin"], out["dirs"])
    sm = dict(scan["samples"])
    P = c + (np.stack([sm["x"], sm["y"], sm["z"]], -1) - c) @ R.T
    sm["x"], sm["y"], sm["z"] = P[:, 0], P[:, 1], P[:, 2]
    out["samples"] = sm
    return out


def trial(seed):
    rng = np.random.default_rng(seed)
    arena = Arena(rng)
    if CASE["undulation"] > 0:
        undulate(arena, seed, CASE["undulation"])
    parts = robot_parts() if (CASE["robot"] or CASE["ruts"] > 0) else {}
    if CASE["ruts"] > 0:
        add_earlier_ruts(arena, np.random.default_rng(seed + RUT_SEED), CASE["rut_trips"], CASE["ruts"], parts["robot"])
    pitch_err = CASE["pitch_err"] * (1 if seed % 2 == 0 else -1)
    x0, y0 = rng.uniform(0.3, 0.8), rng.uniform(0.8, 4.2)
    yaw = float(np.clip(rng.normal(0, 10), -25, 25))
    h, n = path_frame(x0, y0, yaw)
    # path length until the centre reaches X_END or comes within 0.5 m of a side wall
    lim = [(X_END - x0) / h[0]]
    if h[1] > 1e-6:
        lim.append((arena.W - 0.5 - y0) / h[1])
    elif h[1] < -1e-6:
        lim.append((0.5 - y0) / h[1])
    length = max(0.0, min(lim))
    n_frames = int(length // DS) + 1
    grid = Grid(arena.L, arena.W, v(pr.DETECT, "cell"))
    X, Y = grid.centers()
    along_c = (X - x0) * h[0] + (Y - y0) * h[1]
    lat_c = np.abs((X - x0) * n[0] + (Y - y0) * n[1])
    inside = np.zeros_like(X, bool)
    inside[1:-1, 1:-1] = True
    corridor = (lat_c < HALF_W) & inside

    margin, band = v(pr.DETECT, "match_margin"), v(pr.DETECT, "wall_band")
    obst = []
    for k, o in enumerate(arena.obstacles):
        rel = np.array([o.x - x0, o.y - y0])
        a, lat = float(rel @ h), float(abs(rel @ n))
        edge = a - o.radius
        if lat < o.radius + HALF_W and edge > FRONT + 0.3 and edge < length + FRONT:
            obst.append({"k": k, "kind": o.kind, "along_edge": edge, "lateral": lat, "radius": o.radius, "size": o.size})

    def classify(d):
        want = {"pos": ("boulder", "column"), "neg": ("crater",)}[d["kind"]]
        match = [k for k, o in enumerate(arena.obstacles)
                 if o.kind in want and np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin]
        if match:
            return "obstacle", match
        if any(np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin for o in arena.obstacles):
            return "near", []
        if min(d["x"], d["y"], arena.L - d["x"], arena.W - d["y"]) < band:
            return "wall", []
        return "fa", []

    lmin, lmax = v(pr.DETECT, "l_min"), v(pr.DETECT, "l_max")
    runs = []
    for sname in [CASE["lidar"] if k == "lidar" else k for k in CASE["sensors"]]:
        if not CASE["robot"]:
            sensor = D455Depth() if sname == "d455" else Lidar(sname)
        elif sname == "d455":
            sensor = D455Depth(*parts["d455_mount"])
        else:
            xyz, R = parts["lidar_mount"]
            sensor = Lidar(sname, mount_xyz=xyz, mount_R=R, body=None if CASE["body"] == "none" else parts["body"],
                           mount_err_deg=CASE["mask_err"])
        for dust_on in (False,) if CASE["clear_only"] else (False, True):
            dust = Plume(x0 + PLUME_AHEAD * h[0], y0 + PLUME_AHEAD * h[1]) if dust_on else None
            L = new_layers(grid)
            lp, ln = np.zeros((grid.nx, grid.ny)), np.zeros((grid.nx, grid.ny))
            frames = []
            for i in range(n_frames):
                s = i * DS
                robot = (x0 + s * h[0], y0 + s * h[1], yaw)
                scan = sensor.scan(arena, robot, rng, dust=dust)
                if pitch_err:
                    scan = believed(scan, robot, pitch_err)
                body_pts, lost = 0, 0
                if "is_body" in scan:
                    if CASE["body"] in ("box", "table"):
                        was_body = scan["is_body"]
                        scan, drop = selfmask.apply(scan, robot, table=parts.get("table"), box=parts.get("box"))
                        lost = int((drop & ~was_body & ~scan["is_dust"]).sum())     # ground thrown away
                    body_pts = int(scan["is_body"].sum())
                add_scan(L, scan, grid)
                g = fit_ground(L, grid)
                dp, dn = logodds_delta(scan, grid, g, logodds_zref(L, g))
                lp = np.clip(lp + dp, lmin, lmax)
                ln = np.clip(ln + dn, lmin, lmax)
                dets = []
                for d in clusters(lp > 0, ln > 0, grid, lp, ln, l_dec=-np.inf):
                    cls, match = classify(d)
                    rel = np.array([d["x"] - x0, d["y"] - y0])
                    dets.append([d["kind"], round(d["x"], 3), round(d["y"], 3), round(d["evidence"], 3), d["cells"],
                                 round(float(rel @ h), 3), round(float(abs(rel @ n)), 3), cls, match])
                seen = (L["count"] > 0) | np.isfinite(L["carve"])
                win = corridor & (along_c > s + FRONT) & (along_c < s + FRONT + LOOK) & ~seen
                first_unseen = float(along_c[win].min() - (s + FRONT)) if win.any() else None
                # this scan alone: nearest ground return ahead of the front edge inside the corridor
                P = scan["pts"][scan["valid"] & ~scan.get("is_body", np.zeros(len(scan["valid"]), bool))]
                rel = P[:, :2] - [x0 + s * h[0], y0 + s * h[1]]
                a_p, l_p = rel @ h, np.abs(rel @ n)
                ahead = (l_p < HALF_W) & (a_p > FRONT) & (P[:, 2] < 0.15)
                blind = float(a_p[ahead].min() - FRONT) if ahead.any() else None
                frames.append({"s": round(s, 3), "dets": dets, "first_unseen": first_unseen,
                               "dust_echoes": int(scan["is_dust"].sum()), "body_pts": body_pts, "lost": lost,
                               "blind_ahead": None if blind is None else round(blind, 3)})
            runs.append({"sensor": sname, "dust": dust_on, "frames": frames})
    every = [[o.kind, o.x, o.y, o.radius] for o in arena.obstacles]
    return {"seed": seed, "start": [x0, y0, yaw], "length": length, "obstacles": obst, "arena_obstacles": every,
            "pitch_err": pitch_err, "runs": runs}


def main():
    ap = argparse.ArgumentParser(description="Frame-by-frame approach runs")
    ap.add_argument("n", nargs="?", type=int, default=48)
    ap.add_argument("out", nargs="?", default="results_approach")
    ap.add_argument("seed0", nargs="?", type=int, default=3000)
    ap.add_argument("--rms", type=float, default=None, help="floor roughness RMS in m (default from params.py)")
    ap.add_argument("--undulation", type=float, default=0.0, help="RMS of long-wavelength floor waves in m")
    ap.add_argument("--pitch-err", type=float, default=0.0, help="attitude (pitch) estimate error in degrees")
    ap.add_argument("--clear-only", action="store_true", help="skip the dust runs")
    ap.add_argument("--ground", choices=["plane", "local"], default=None, help="ground fit (default from params.py)")
    ap.add_argument("--ground-sigma", type=float, default=None, help="window width in m for the local ground fit")
    ap.add_argument("--ground-spread", type=float, default=None, help="local ground fit skips cells whose returns span more (m)")
    ap.add_argument("--seeds", default=None, help="comma-separated seeds to run instead of n from seed0")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--robot", action="store_true", help="sensor mounts and robot size from the team's URDF")
    ap.add_argument("--lidar", default="spin16", choices=sorted(k for k in pr.LIDAR_PROFILES if k != "rosette70"))
    ap.add_argument("--body", default="none", choices=["none", "raw", "box", "table"],
                    help="robot body: transparent, blocking with no mask, or masked by a crop box or a range table")
    ap.add_argument("--inflate", type=float, default=None, help="self-mask table widening in degrees")
    ap.add_argument("--mask-err", type=float, default=0.0, help="real LiDAR tilted this many degrees off its mount")
    ap.add_argument("--ruts", type=float, default=0.0, help="depth (m) of ruts from earlier drives; 0 = none")
    ap.add_argument("--rut-trips", type=int, default=v(pr.RUTS, "trips"))
    ap.add_argument("--sensors", default="lidar,d455", help="comma-separated subset of lidar,d455")
    ap.add_argument("--frame-s", type=float, default=None, help="L1 collection time per frame in s (default 0.1)")
    ap.add_argument("--label", default="", help="case name shown on the dashboard")
    a = ap.parse_args()
    assert a.body == "none" or a.robot, "--body needs --robot"
    sensors = a.sensors.split(",")
    assert sensors and set(sensors) <= {"lidar", "d455"}, "--sensors takes lidar, d455 or both"
    if a.frame_s is not None:  # set before the pool forks, like --rms
        assert pr.LIDAR_PROFILES[a.lidar]["kind"] == "l1", "--frame-s is for the l1 profile"
        pr.LIDAR_PROFILES[a.lidar] = dict(pr.LIDAR_PROFILES[a.lidar], frame_s=a.frame_s)
    if a.rms is not None:      # set before the pool forks, so every worker builds arenas with it
        old = pr.ARENA["surface_rms"]
        pr.ARENA["surface_rms"] = pr.P(a.rms, old.unit, "Assumed", "approach.py --rms (robustness case)")
    if a.ground is not None:
        old = pr.DETECT["ground_fit"]
        pr.DETECT["ground_fit"] = pr.P(a.ground, old.unit, "Assumed", "approach.py --ground")
    if a.ground_sigma is not None:
        old = pr.DETECT["ground_sigma"]
        pr.DETECT["ground_sigma"] = pr.P(a.ground_sigma, old.unit, "Assumed", "approach.py --ground-sigma")
    if a.ground_spread is not None:
        old = pr.DETECT["ground_spread"]
        pr.DETECT["ground_spread"] = pr.P(a.ground_spread, old.unit, "Assumed", "approach.py --ground-spread")
    CASE.update(undulation=a.undulation, pitch_err=a.pitch_err, clear_only=a.clear_only, robot=a.robot,
                lidar=a.lidar, body=a.body, inflate=a.inflate, mask_err=a.mask_err, ruts=a.ruts, rut_trips=a.rut_trips,
                sensors=sensors)
    if a.robot:          # robot size from the URDF, set before the pool forks
        global FRONT, BACK, HALF_W
        parts = robot_parts()
        lo, hi = parts["robot"].extents()
        FRONT, BACK = float(hi[0]), float(-lo[0])
        HALF_W = float(max(hi[1], -lo[1])) + 0.05
        _CACHE.clear()   # each worker builds its own (Embree scenes do not survive the fork reliably)
    seeds = [int(x) for x in a.seeds.split(",")] if a.seeds else list(range(a.seed0, a.seed0 + a.n))
    a.n = len(seeds)
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    tel = Telemetry(kind="approach", total=a.n, args=vars(a), label=a.label or os.path.basename(a.out.rstrip("/"))) \
        if Telemetry else None
    with Pool(a.workers) as p:
        res = []
        for k, r in enumerate(p.imap(trial, seeds)):
            res.append(r)
            print(f"{k + 1}/{a.n} arenas, {time.time() - t0:.0f} s", flush=True)
            if tel:
                fr = [f for run in r["runs"] for f in run["frames"]]
                tel.progress(k + 1, seed=r["seed"], frames=len(fr), body_pts_per_frame=round(
                    sum(f.get("body_pts", 0) for f in fr) / max(len(fr), 1), 2))
    meta = {"n_arenas": a.n, "seed0": seeds[0], "seconds": time.time() - t0,
            "DS": DS, "FRONT": FRONT, "HALF_W": HALF_W, "X_END": X_END, "PLUME_AHEAD": PLUME_AHEAD, "LOOK": LOOK,
            "surface_rms": v(pr.ARENA, "surface_rms"), "ground_fit": v(pr.DETECT, "ground_fit"), "ground_sigma": v(pr.DETECT, "ground_sigma"),
            "ground_spread": v(pr.DETECT, "ground_spread"), "BACK": BACK, "label": a.label, **CASE,
            "frame_s": pr.LIDAR_PROFILES[a.lidar].get("frame_s"),
            "det_fields": ["kind", "x", "y", "evidence", "cells", "along", "lateral", "class", "match"]}
    with open(os.path.join(a.out, "approaches.json"), "w") as f:
        json.dump({"meta": meta, "trials": res}, f)
    print(f"{a.n} arenas in {time.time() - t0:.0f} s -> {a.out}/approaches.json")
    if tel:
        tel.end(seconds=round(time.time() - t0, 1))


if __name__ == "__main__":
    main()
