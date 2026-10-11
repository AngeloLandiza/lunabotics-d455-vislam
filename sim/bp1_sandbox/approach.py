"""Approach runs: the robot drives a straight line through the obstacle zone, and the log-odds detector updates
frame by frame, as it would on the robot. For every frame it records each candidate object (kind, position, pooled
evidence, what it really is) and how far ahead the path has been observed. speed_gate.py turns these records into
missed-hazard rates, phantom stops and speeds for any threshold and speed policy, without re-running the scans.

Usage: python3 approach.py [n_arenas] [out_dir] [first_seed] [--rms M] [--undulation M] [--pitch-err DEG] [--clear-only]
       [--ground plane|local] [--ground-sigma M] [--workers N]
--rms sets the floor roughness (params.py default 0.01 m), --undulation adds long-wavelength floor waves of that RMS,
--pitch-err makes the robot's attitude estimate wrong by that many degrees (sign alternates with the seed). The extra
draws use their own random stream, so every case sees the same arenas, starts and sensor noise as the others.
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

SENSORS = ["spin16", "d455"]
DS = 0.15            # m driven between frames (Assumed: 10 Hz at 1.5 m/s; a slower robot gets more frames per metre)
FRONT = 0.35         # m from the robot centre to its front edge (Assumed, = D455 mount x)
HALF_W = 0.40        # m, half-width of the swept corridor: robot half-width plus margin (Assumed)
X_END = 4.9          # m, robot centre stops here, short of the berm wall
PLUME_AHEAD = 1.2    # m ahead of the start, dust plume centre (Assumed: a dig site in front)
LOOK = 3.0           # m beyond the front edge searched for unobserved corridor cells
UNDULATION_WL = (1.5, 3.0)   # m, wavelength range of the floor waves (Assumed)
EXTRA_SEED = 7_000_003       # offset of the random stream for floor waves and attitude error
CASE = {"undulation": 0.0, "pitch_err": 0.0, "clear_only": False}


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
    for sname in SENSORS:
        sensor = D455Depth() if sname == "d455" else Lidar(sname)
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
                frames.append({"s": round(s, 3), "dets": dets, "first_unseen": first_unseen,
                               "dust_echoes": int(scan["is_dust"].sum())})
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
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    if a.rms is not None:      # set before the pool forks, so every worker builds arenas with it
        old = pr.ARENA["surface_rms"]
        pr.ARENA["surface_rms"] = pr.P(a.rms, old.unit, "Assumed", "approach.py --rms (robustness case)")
    if a.ground is not None:
        old = pr.DETECT["ground_fit"]
        pr.DETECT["ground_fit"] = pr.P(a.ground, old.unit, "Assumed", "approach.py --ground")
    if a.ground_sigma is not None:
        old = pr.DETECT["ground_sigma"]
        pr.DETECT["ground_sigma"] = pr.P(a.ground_sigma, old.unit, "Assumed", "approach.py --ground-sigma")
    CASE.update(undulation=a.undulation, pitch_err=a.pitch_err, clear_only=a.clear_only)
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    with Pool(a.workers) as p:
        res = []
        for k, r in enumerate(p.imap(trial, range(a.seed0, a.seed0 + a.n))):
            res.append(r)
            print(f"{k + 1}/{a.n} arenas, {time.time() - t0:.0f} s", flush=True)
    meta = {"n_arenas": a.n, "seed0": a.seed0, "seconds": time.time() - t0,
            "DS": DS, "FRONT": FRONT, "HALF_W": HALF_W, "X_END": X_END, "PLUME_AHEAD": PLUME_AHEAD, "LOOK": LOOK,
            "surface_rms": v(pr.ARENA, "surface_rms"), "ground_fit": v(pr.DETECT, "ground_fit"), "ground_sigma": v(pr.DETECT, "ground_sigma"), **CASE,
            "det_fields": ["kind", "x", "y", "evidence", "cells", "along", "lateral", "class", "match"]}
    with open(os.path.join(a.out, "approaches.json"), "w") as f:
        json.dump({"meta": meta, "trials": res}, f)
    print(f"{a.n} arenas in {time.time() - t0:.0f} s -> {a.out}/approaches.json")


if __name__ == "__main__":
    main()
