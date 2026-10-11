"""Approach runs: the robot drives a straight line through the obstacle zone, and the log-odds detector updates
frame by frame, as it would on the robot. For every frame it records each candidate object (kind, position, pooled
evidence, what it really is) and how far ahead the path has been observed. speed_gate.py turns these records into
missed-hazard rates, phantom stops and speeds for any threshold and speed policy, without re-running the scans.

Usage: python3 approach.py [n_arenas] [out_dir] [first_seed]
Every number below is Assumed unless marked; the robot's real size, braking and frame rate replace them later.
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

import params as pr
from params import v
from arena import Arena
from detect import Grid, add_scan, clusters, fit_ground, logodds_delta, logodds_zref, new_layers
from dust import Plume
from sensors import Lidar, D455Depth

SENSORS = ["spin16", "d455"]
DS = 0.15            # m driven between frames (Assumed: 10 Hz at 1.5 m/s; a slower robot gets more frames per metre)
FRONT = 0.35         # m from the robot centre to its front edge (Assumed, = D455 mount x)
HALF_W = 0.40        # m, half-width of the swept corridor: robot half-width plus margin (Assumed)
X_END = 4.9          # m, robot centre stops here, short of the berm wall
PLUME_AHEAD = 1.2    # m ahead of the start, dust plume centre (Assumed: a dig site in front)
LOOK = 3.0           # m beyond the front edge searched for unobserved corridor cells


def path_frame(x0, y0, yaw):
    h = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
    return h, np.array([-h[1], h[0]])


def trial(seed):
    rng = np.random.default_rng(seed)
    arena = Arena(rng)
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
        for dust_on in (False, True):
            dust = Plume(x0 + PLUME_AHEAD * h[0], y0 + PLUME_AHEAD * h[1]) if dust_on else None
            L = new_layers(grid)
            lp, ln = np.zeros((grid.nx, grid.ny)), np.zeros((grid.nx, grid.ny))
            frames = []
            for i in range(n_frames):
                s = i * DS
                robot = (x0 + s * h[0], y0 + s * h[1], yaw)
                scan = sensor.scan(arena, robot, rng, dust=dust)
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
    return {"seed": seed, "start": [x0, y0, yaw], "length": length, "obstacles": obst, "arena_obstacles": every, "runs": runs}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 48
    out = sys.argv[2] if len(sys.argv) > 2 else "results_approach"
    seed0 = int(sys.argv[3]) if len(sys.argv) > 3 else 3000
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    with Pool(4) as p:
        res = []
        for k, r in enumerate(p.imap(trial, range(seed0, seed0 + n))):
            res.append(r)
            print(f"{k + 1}/{n} arenas, {time.time() - t0:.0f} s", flush=True)
    meta = {"n_arenas": n, "seed0": seed0, "seconds": time.time() - t0,
            "DS": DS, "FRONT": FRONT, "HALF_W": HALF_W, "X_END": X_END, "PLUME_AHEAD": PLUME_AHEAD, "LOOK": LOOK,
            "det_fields": ["kind", "x", "y", "evidence", "cells", "along", "lateral", "class", "match"]}
    with open(os.path.join(out, "approaches.json"), "w") as f:
        json.dump({"meta": meta, "trials": res}, f)
    print(f"{n} arenas in {time.time() - t0:.0f} s -> {out}/approaches.json")


if __name__ == "__main__":
    main()
