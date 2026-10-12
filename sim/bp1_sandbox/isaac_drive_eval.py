"""Do wheel ruts make false craters or boulders? Isaac Sim drives (isaac_drive_ruts.py) with and without ruts, through
the BP-1 layer, the L1 self-mask and the log-odds detector, frame by frame.

Usage: python3 isaac_drive_eval.py RUTS.npz NORUTS.npz [--l-dec-l1 6.8] [--l-dec-d455 0.85] [--out DIR]
Both captures must be the same path on the same arena (same seed, trip, poses).

Per frame k of each capture:
- the arena height field is rebuilt with the ruts stamped up to frame k (drive_path.stamp_to, as in the Isaac drive),
  for LiDAR surface normals and the truth classes;
- L1: one L1-like frame of rays (sensors.l1_dirs, rng fixed by (seed, k), identical for both captures) is taken from
  the captured hemisphere grid (nearest 1 deg grid ray; its Isaac direction and range are used as they are), passed
  through the BP-1 layer (isaac_bridge.dust_march + Lidar("l1").apply, normals from the rutted arena), then the table
  self-mask (selfmask.MaskTable for robot.BodyCaster at robot.mount("lidar3d_link"));
- D455: isaac_bridge.d455_from_ideal on the captured depth image;
- each sensor feeds its own detector (detect.add_scan / fit_ground / logodds_delta / clusters, as approach.trial).
Truth classes as approach.trial (obstacle / near / wall / fa); an fa candidate whose centre lies within RUT_NEAR of a
wheel-track centre line laid so far (drive_path.track_segments; the same lines in both captures) is "on the ruts".
Phantom stop (speed_gate.py rule): an fa candidate at or above the threshold inside the corridor (lateral < HALF_W +
0.25 m) and ahead of the front edge by at most d_stop(1.0 m/s) + DS. The robot's attitude in every frame is the
settled wheels-on-terrain pose the drive used.
"""
import argparse
import json
import os

import numpy as np

import params as pr
import selfmask
from params import v
from approach import DS, FRONT, HALF_W
from arena import Arena
from detect import Grid, add_scan, clusters, fit_ground, logodds_delta, logodds_zref, new_layers
from drive_path import dist_to_segments, stamp_to, track_segments
from isaac_bridge import d455_from_ideal, dust_march
from robot import BodyCaster, Robot
from ruts import Ruts, wheel_offsets
from sensors import D455Depth, Lidar, l1_dirs
from speed_gate import d_stop

RUT_NEAR = 0.15     # m from a wheel-track centre line: "on or near the ruts"


class GridLookup:
    """Nearest captured hemisphere-grid ray for any sensor-frame direction (91 elevations x 360 azimuths, 1 deg)."""

    def __init__(self, d_s):
        self.d_s = d_s
        ok = np.isfinite(d_s[:, 0])
        el = np.degrees(np.arcsin(np.clip(d_s[ok, 2], -1, 1)))
        az = np.degrees(np.arctan2(d_s[ok, 1], d_s[ok, 0]))
        self.map = np.full((91, 360), -1, int)
        self.map[np.clip(np.rint(el).astype(int), 0, 90), np.rint(az + 180).astype(int) % 360] = np.nonzero(ok)[0]

    def __call__(self, d):
        el = np.degrees(np.arcsin(np.clip(d[:, 2], -1, 1)))
        az = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
        return self.map[np.clip(np.rint(el).astype(int), 0, 90), np.rint(az + 180).astype(int) % 360]


def evaluate(path, l_dec, rb, table, caster):
    cap = np.load(path, allow_pickle=True)
    seed = int(cap["seed"])
    poses = [tuple(p) for p in cap["poses"]]
    arena = Arena(np.random.default_rng(seed))
    depth = float(cap["rut_depth"])
    ruts = Ruts(arena, depth, width=rb.wheels[0][2]) if depth > 0 else None
    lat = wheel_offsets(rb)
    wheels = np.array([c for c, _, _ in rb.wheels])
    x_front, x_rear = wheels[:, 0].max(), wheels[:, 0].min()
    xyz, R_m = rb.mount("lidar3d_link")
    lidar = Lidar("l1", mount_xyz=xyz, mount_R=R_m)
    cam = D455Depth()
    look = GridLookup(cap["l1_d_s"])
    grid = Grid(arena.L, arena.W, v(pr.DETECT, "cell"))
    lmin, lmax = v(pr.DETECT, "l_min"), v(pr.DETECT, "l_max")
    margin, band = v(pr.DETECT, "match_margin"), v(pr.DETECT, "wall_band")
    state = {s: {"L": new_layers(grid), "lp": np.zeros((grid.nx, grid.ny)), "ln": np.zeros((grid.nx, grid.ny))}
             for s in ("l1", "d455")}
    frames = []
    for k, pose in enumerate(poses):
        if ruts is not None:
            stamp_to(ruts, poses, k, x_rear, x_front, lat)
        segs = track_segments(poses, k, x_rear, x_front, lat)
        T = cap[f"p{k}_base"]
        lo, lR = cap[f"p{k}_l1_origin"], cap[f"p{k}_l1_R"]
        rng = np.random.default_rng([seed, k, 7])           # same draws for both captures
        idx = look(l1_dirs(pr.LIDAR_PROFILES["l1"], rng))
        idx = idx[idx >= 0]
        d_s = cap["l1_d_s"][idx]
        dirs = d_s @ lR.T
        t = cap[f"p{k}_l1_r"][idx].astype(float)
        P = lo + np.nan_to_num(t)[:, None] * dirs
        normals = arena.normal(P[:, 0], P[:, 1])
        # body hits: the return comes back within 2 cm of the body's range on that ray (robot.BodyCaster, exact)
        t_body, _ = caster.cast(np.asarray(xyz, float), d_s @ R_m.T)
        is_body = np.isfinite(t) & np.isfinite(t_body) & (np.abs(t - np.nan_to_num(t_body)) < 0.02)
        m = dust_march(lo, dirs, t, None, lidar.rmax)
        # body hits: params ROBOT body_reflectivity; their incidence is not captured, so cos = 1 (nan -> 1 in apply)
        scan = lidar.apply(lo, dirs, t, normals, m, rng, dust_on=False, is_body=is_body,
                           cos_override=np.full(len(t), np.nan))
        scan["dirs_s"] = d_s
        scan, dropped = selfmask.apply(scan, pose, table=table)
        cam_scan = d455_from_ideal(cam, cap[f"p{k}_d455_origin"], cap[f"p{k}_d455_R"],
                                   cap[f"p{k}_depth"].astype(float), rng)
        x, y, yaw = pose
        h = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
        n = np.array([-h[1], h[0]])
        rec = {"k": k, "pose": list(pose), "l1_returns": int(np.isfinite(t).sum()), "l1_body_dropped": int(dropped.sum()),
               "l1_body_hits": int(is_body.sum()), "l1_body_leaked": int((scan["is_body"] & scan["valid"]).sum())}
        for sname, sc in (("l1", scan), ("d455", cam_scan)):
            st = state[sname]
            add_scan(st["L"], sc, grid)
            g = fit_ground(st["L"], grid)
            dp, dn = logodds_delta(sc, grid, g, logodds_zref(st["L"], g))
            st["lp"] = np.clip(st["lp"] + dp, lmin, lmax)
            st["ln"] = np.clip(st["ln"] + dn, lmin, lmax)
            dets = []
            for d in clusters(st["lp"] > 0, st["ln"] > 0, grid, st["lp"], st["ln"], l_dec=-np.inf):
                want = {"pos": ("boulder", "column"), "neg": ("crater",)}[d["kind"]]
                if any(o.kind in want and np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin for o in arena.obstacles):
                    cls = "obstacle"
                elif any(np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + 2 * margin for o in arena.obstacles):
                    cls = "near"
                elif min(d["x"], d["y"], arena.L - d["x"], arena.W - d["y"]) < band:
                    cls = "wall"
                else:
                    cls = "fa"
                rel = np.array([d["x"] - x, d["y"] - y])
                gap, lat_d = float(rel @ h) - FRONT, float(abs(rel @ n))
                rut_d = float(dist_to_segments(d["x"], d["y"], segs))
                dets.append({"kind": d["kind"], "x": round(d["x"], 3), "y": round(d["y"], 3),
                             "evidence": round(d["evidence"], 2), "cells": d["cells"], "cls": cls,
                             "on_ruts": cls == "fa" and rut_d < RUT_NEAR, "rut_dist": round(rut_d, 3),
                             "ahead": 0 < gap <= d_stop(1.0) + DS and lat_d < HALF_W + 0.25})
            thr = l_dec[sname]
            conf_fa = [d for d in dets if d["cls"] == "fa" and d["evidence"] >= thr]
            rec[sname] = {"dets": dets, "fa_confirmed": len(conf_fa),
                          "fa_on_ruts": sum(d["on_ruts"] for d in conf_fa),
                          "phantom": any(d["ahead"] for d in conf_fa),
                          "phantom_on_ruts": any(d["ahead"] and d["on_ruts"] for d in conf_fa),
                          "obstacles_confirmed": sum(d["cls"] == "obstacle" and d["evidence"] >= thr for d in dets)}
        frames.append(rec)
    return {"capture": os.path.relpath(path), "seed": seed, "rut_depth": depth, "n_frames": len(poses),
            "trip_seed": int(cap["trip_seed"]) if "trip_seed" in cap else None, "frames": frames}


def summary(run, sname, thr):
    fr = run["frames"]
    uniq, uniq_rut = set(), set()
    for f in fr:
        for d in f[sname]["dets"]:
            if d["cls"] == "fa" and d["evidence"] >= thr:
                key = (d["kind"], round(d["x"] / 0.1), round(d["y"] / 0.1))
                uniq.add(key)
                if d["on_ruts"]:
                    uniq_rut.add(key)
    return {"fa_frames": sum(f[sname]["fa_confirmed"] for f in fr),
            "fa_frames_on_ruts": sum(f[sname]["fa_on_ruts"] for f in fr),
            "fa_unique": len(uniq), "fa_unique_on_ruts": len(uniq_rut),
            "phantom_frames": sum(f[sname]["phantom"] for f in fr),
            "phantom_frames_on_ruts": sum(f[sname]["phantom_on_ruts"] for f in fr),
            "last_frame_fa": fr[-1][sname]["fa_confirmed"],
            "last_frame_fa_kinds": sorted(d["kind"] for d in fr[-1][sname]["dets"]
                                          if d["cls"] == "fa" and d["evidence"] >= thr)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("ruts")
    ap.add_argument("noruts")
    ap.add_argument("--l-dec-l1", type=float, default=6.8, help="L1 threshold (cloud sweep value, Inferred)")
    ap.add_argument("--l-dec-d455", type=float, default=v(pr.DETECT, "l_decide_d455"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    l_dec = {"l1": a.l_dec_l1, "d455": a.l_dec_d455}
    rb = Robot()
    xyz, R_m = rb.mount("lidar3d_link")
    table = selfmask.MaskTable(BodyCaster(rb, "lidar3d_link"), xyz, R_m)   # cached in robot_cache/ after the first run
    caster = BodyCaster(rb, "lidar3d_link")
    runs = {"ruts": evaluate(a.ruts, l_dec, rb, table, caster), "noruts": evaluate(a.noruts, l_dec, rb, table, caster)}
    assert np.allclose([f["pose"] for f in runs["ruts"]["frames"]], [f["pose"] for f in runs["noruts"]["frames"]]), \
        "captures are not the same path"
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.ruts)), "drive_eval")
    os.makedirs(out, exist_ok=True)
    summ = {s: {c: summary(runs[c], s, l_dec[s]) for c in ("ruts", "noruts")} for s in ("l1", "d455")}
    with open(os.path.join(out, "drive_eval.json"), "w") as f:
        json.dump({"thresholds": l_dec, "rut_near_m": RUT_NEAR, "summary": summ, "runs": runs}, f, indent=1, default=float)
    r0 = runs["ruts"]
    L = [f"# Wheel ruts and false detections (Isaac drive, seed {r0['seed']}, trip seed {r0['trip_seed']})", "",
         f"{r0['n_frames']} frames; ruts {100 * r0['rut_depth']:.1f} cm vs none; thresholds L1 {l_dec['l1']} (Inferred, "
         f"cloud sweep), D455 {l_dec['d455']}. 'On the ruts': false candidate within {RUT_NEAR} m of a wheel track. "
         "Phantom stop: confirmed false candidate in the corridor within d_stop(1.0) + 0.15 m ahead.", "",
         "| sensor | case | confirmed false-candidate frames (on ruts) | distinct false candidates (on ruts) | "
         "phantom-stop frames (from ruts) | false candidates in the final map |", "| --- | --- | --- | --- | --- | --- |"]
    for s in ("l1", "d455"):
        for c in ("ruts", "noruts"):
            x = summ[s][c]
            L.append(f"| {s.upper()} | {'with ruts' if c == 'ruts' else 'no ruts'} | {x['fa_frames']} ({x['fa_frames_on_ruts']}) | "
                     f"{x['fa_unique']} ({x['fa_unique_on_ruts']}) | {x['phantom_frames']} ({x['phantom_frames_on_ruts']}) | "
                     f"{x['last_frame_fa']} {x['last_frame_fa_kinds']} |")
    leak = [f["l1_body_leaked"] for f in r0["frames"]]
    hits = [f["l1_body_hits"] for f in r0["frames"]]
    L += ["", f"L1 self-mask (with ruts): {np.mean(hits):.0f} body returns per frame before the mask (within 2 cm of "
              f"robot.BodyCaster's range), {np.mean(leak):.1f} left after it (max {max(leak)})."]
    md = "\n".join(L) + "\n"
    open(os.path.join(out, "drive_eval.md"), "w").write(md)
    print(md)


if __name__ == "__main__":
    main()
