"""Check Isaac Sim's ideal hits against the sandbox geometry, then run them through the BP-1 layer and the detectors.

Usage: python3 isaac_validate.py [isaac_out/capture.npz]   (plain Python: numpy + scipy; no Isaac Sim needed)

1. Geometry: for every captured pose, re-create Arena(default_rng(1)) and cast the sandbox's own Lidar / D455Depth
   rays (sensors.spinning_dirs and D455Depth.d_s through sensors.sensor_pose, raycast.march). Compare range per
   matching ray: median and 95th percentile of |Isaac - sandbox|, and the share of rays where only one side hits.
   The same is done against an exact ray cast of the exported USD triangles ("mesh"), which separates Isaac's own
   error from the difference between the 2 cm USD mesh and the 1 cm height field.
2. BP-1 layer and detection: the Isaac hits go through isaac_bridge.lidar_from_ideal (normals from arena.normal) and
   d455_from_ideal, in clear air and with a dust.Plume 0.8 m ahead of the first pose (as in evaluate.py). Then
   detect.detect(method='logodds') and detect.score, next to the sandbox's own scans for the same poses and seeds.
"""
import json
import os
import sys

import numpy as np

from arena import Arena
from detect import Grid, accumulate, detect, score
from dust import Plume
from isaac_bridge import d455_from_ideal, lidar_from_ideal
from raycast import march
from sensors import D455Depth, Lidar, sensor_pose, spinning_dirs


class MeshHeight:
    """Height of the exported USD mesh: export_arena_usd samples arena.h every `stride` cells and splits each
    quad a(i,j) b(i+1,j) c(i+1,j+1) d(i,j+1) into triangles abc and acd."""

    def __init__(self, arena, stride=2):
        self.h = arena.h[::stride, ::stride]
        self.x0, self.y0, self.res, self.wall = arena.x0, arena.y0, arena.res * stride, arena.WALL_H

    def height(self, x, y):
        fx, fy = (x - self.x0) / self.res, (y - self.y0) / self.res
        nx, ny = self.h.shape
        out = (fx < 0) | (fy < 0) | (fx > nx - 1) | (fy > ny - 1)
        fx, fy = np.clip(fx, 0, nx - 1.0001), np.clip(fy, 0, ny - 1.0001)
        i, j = fx.astype(np.int64), fy.astype(np.int64)
        ax, ay, h = fx - i, fy - j, self.h
        ha, hb, hc, hd = h[i, j], h[i + 1, j], h[i + 1, j + 1], h[i, j + 1]
        z = np.where(ax >= ay, ha + ax * (hb - ha) + ay * (hc - hb), ha + ay * (hd - ha) + ax * (hc - hd))
        return np.where(out, self.wall, z)


def agreement(a, b, hit_a, hit_b):
    both = hit_a & hit_b
    e = np.abs(a - b)[both]
    return {"n_both": int(both.sum()), "median_cm": float(np.median(e) * 100), "p95_cm": float(np.percentile(e, 95) * 100),
            "one_side_only_pct": float(np.mean(hit_a != hit_b) * 100)}


def geometry(cap, k, arena, mesh):
    res = {}
    lid = Lidar("spin16")
    lo, lR = cap[f"p{k}_lidar_origin"], cap[f"p{k}_lidar_R"]
    o2, R2 = sensor_pose(tuple(cap["poses"][k]), lid.mount, lid.pitch)
    assert np.allclose(lo, o2) and np.allclose(lR, R2), "capture used a different mount than params.MOUNTS"
    dirs = spinning_dirs(lid.prof) @ lR.T
    di = cap[f"p{k}_lidar_dirs_isaac"]
    have = np.isfinite(di[:, 0])
    ang = np.degrees(np.arccos(np.clip((dirs[have] * di[have]).sum(1), -1, 1)))
    ri = cap[f"p{k}_lidar_range"]
    for name, H, step in (("sandbox", arena, 0.02), ("mesh", mesh, 0.005)):
        t = march(H, lo, dirs, lid.rmax, step=step)["t_hit"]
        res[f"lidar_vs_{name}"] = agreement(ri, t, np.isfinite(ri), np.isfinite(t))
        if name == "mesh" and f"p{k}_lidar_range_per_scan" in cap:
            r1 = cap[f"p{k}_lidar_range_per_scan"][0].astype(float)    # one RTX scan, before merging
            res["lidar_single_scan_vs_mesh"] = agreement(r1, t, np.isfinite(r1), np.isfinite(t))
    res["lidar_rays_missing"] = int((~have).sum())
    res["lidar_max_dir_err_deg"] = float(ang.max())

    cam = D455Depth()
    co, cR = cap[f"p{k}_cam_origin"], cap[f"p{k}_cam_R"]
    dirs = cam.d_s @ cR.T
    zi = cap[f"p{k}_depth"].reshape(-1)
    for name, H, step in (("sandbox", arena, 0.02), ("mesh", mesh, 0.005)):
        t = march(H, co, dirs, cam.zmax / cam.d_s[:, 0].min(), step=step)["t_hit"]
        z = t * cam.d_s[:, 0]
        hz = np.isfinite(z) & (np.nan_to_num(z, nan=1e9) <= cam.zmax)
        hi = np.isfinite(zi) & (np.nan_to_num(zi, nan=1e9) <= cam.zmax)
        res[f"depth_vs_{name}"] = agreement(zi, z, hi, hz)
    return res


def isaac_lidar_scan(cap, k, arena, lid, rng, dust):
    lo, lR = cap[f"p{k}_lidar_origin"], cap[f"p{k}_lidar_R"]
    dirs = spinning_dirs(lid.prof) @ lR.T
    di = cap[f"p{k}_lidar_dirs_isaac"]
    dirs = np.where(np.isfinite(di), di, dirs)          # Isaac's own ray directions (they agree to < 1e-3 deg)
    t = cap[f"p{k}_lidar_range"]
    P = lo + np.nan_to_num(t)[:, None] * dirs
    return lidar_from_ideal(lid, lo, dirs, t, arena.normal(P[:, 0], P[:, 1]), rng, dust=dust)


def isaac_d455_scan(cap, k, cam, rng, dust):
    return d455_from_ideal(cam, cap[f"p{k}_cam_origin"], cap[f"p{k}_cam_R"], cap[f"p{k}_depth"], rng, dust=dust)


def summarize(dets, arena, L, grid, origin):
    rec, fa, wall = score(dets, arena, L, grid, origin)
    hits = [f"{r['kind'][0].upper()}{'+' if r['detected'] else ('-' if r['in_view'] else '.')}" for r in rec]
    return {"detected": sum(r["detected"] for r in rec), "in_view": sum(r["in_view"] for r in rec),
            "per_obstacle": " ".join(hits), "false_alarms": fa, "wall": wall}


def detection(cap, arena, seqs):
    grid = Grid(arena.L, arena.W, 0.05)
    rows = []
    for si, seq in enumerate(seqs):
        x, y, yaw = cap["poses"][seq[0]]
        plume_xy = (x + 0.8 * np.cos(np.radians(yaw)), y + 0.8 * np.sin(np.radians(yaw)))
        for sname in ("spin16", "d455"):
            for dust_on in (False, True):
                dust = Plume(*plume_xy) if dust_on else None
                for src in ("sandbox", "isaac"):
                    seed = 100 * si + 10 * (sname == "d455") + dust_on
                    rng = np.random.default_rng(seed)
                    sensor = D455Depth() if sname == "d455" else Lidar(sname)
                    scans = []
                    for k in seq:
                        if src == "sandbox":
                            scans.append(sensor.scan(arena, tuple(cap["poses"][k]), rng, dust=dust))
                        elif sname == "d455":
                            scans.append(isaac_d455_scan(cap, k, sensor, rng, dust))
                        else:
                            scans.append(isaac_lidar_scan(cap, k, arena, sensor, rng, dust))
                    for nf in (1, len(seq)):
                        L = accumulate(scans[:nf], grid)
                        dets, _ = detect(L, grid, "logodds", scans[:nf])
                        s = summarize(dets, arena, L, grid, scans[nf - 1]["origin"])
                        rows.append(dict(s, seq=si, sensor=sname, dust=dust_on, source=src, frames=nf,
                                         dust_echoes=int(sum(sc["is_dust"].sum() for sc in scans[:nf]))))
    return rows


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join("isaac_out", "capture.npz")
    cap = np.load(path)
    arena = Arena(np.random.default_rng(1))
    mesh = MeshHeight(arena)
    n = len(cap["poses"])
    geo = [geometry(cap, k, arena, mesh) for k in range(n)]
    print("Geometry: Isaac vs sandbox ray cast (and vs exact USD mesh), |range difference| per matching ray")
    print(f"{'pose':>24} | {'LiDAR med/p95 cm, one-side %':>30} | {'vs mesh':>18} | {'depth med/p95 cm, one-side %':>30} | {'vs mesh':>18}")
    for k, g in enumerate(geo):
        f = lambda d: f"{d['median_cm']:.2f}/{d['p95_cm']:.2f}, {d['one_side_only_pct']:.2f}"  # noqa: E731
        print(f"{str(tuple(np.round(cap['poses'][k], 3))):>24} | {f(g['lidar_vs_sandbox']):>30} | {f(g['lidar_vs_mesh']):>18} | "
              f"{f(g['depth_vs_sandbox']):>30} | {f(g['depth_vs_mesh']):>18}")
    if "lidar_single_scan_vs_mesh" in geo[0]:
        print("one RTX scan alone vs mesh, one-side-only %: "
              + ", ".join(f"{g['lidar_single_scan_vs_mesh']['one_side_only_pct']:.2f}" for g in geo))
    print(f"max LiDAR direction error {max(g['lidar_max_dir_err_deg'] for g in geo):.2e} deg, "
          f"missing rays {sum(g['lidar_rays_missing'] for g in geo)}")
    seqs = [list(range(i, min(i + 4, n))) for i in range(0, n, 4)]
    rows = detection(cap, arena, seqs)
    print("\nDetection, logodds (per obstacle: B/C = boulder/crater, + detected, - missed in view, . not in view)")
    print(f"{'seq':>3} {'sensor':>6} {'dust':>5} {'frames':>6} | {'sandbox':>32} | {'isaac':>32}")
    key = lambda r: (r["seq"], r["sensor"], r["dust"], r["frames"])  # noqa: E731
    by = {}
    for r in rows:
        by.setdefault(key(r), {})[r["source"]] = r
    for kk, d in by.items():
        f = lambda r: f"{r['detected']}/{r['in_view']} {r['per_obstacle']} fa={r['false_alarms']} w={r['wall']}"  # noqa: E731
        print(f"{kk[0]:>3} {kk[1]:>6} {str(kk[2]):>5} {kk[3]:>6} | {f(d['sandbox']):>32} | {f(d['isaac']):>32}")
    out = os.path.join(os.path.dirname(path), "validation.json")
    with open(out, "w") as fh:
        json.dump({"capture": path, "poses": cap["poses"].tolist(), "geometry": geo, "detection": rows}, fh, indent=1)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
