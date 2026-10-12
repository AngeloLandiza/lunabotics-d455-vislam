"""LiDAR localization against the arena walls, for different LiDAR mounts.

The robot stands at random poses in random arenas. The L1 (sensors.Lidar with the robot body and the table
self-mask) scans for a while; returns between z_lo and z_hi above the floor, projected onto the floor plane, are
matched to the four walls (the only geometry known before the run: the column's position and every obstacle are left
out of the map) by robust point-to-line ICP in (x, y, yaw), from a start pose off by odometry-like errors.
Per pose: wall points used, the final pose error, and the weakest direction of the fit (the smallest eigenvalue of the
normal matrix per point, near 0 when the visible walls leave a direction unconstrained). Attitude is taken as known
(IMU), as the planar fit assumes. Every number here is Assumed unless marked.

Usage: python3 localize.py out_dir [--arenas N] [--poses N] [--seed0 S] [--workers N] [--frames N,N]
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
from sensors import Lidar
import selfmask

Z_LO, Z_HI = 0.30, 1.10     # m, wall band: above boulders (<= 0.40 m tall... minus clutter) and below the wall top
HUBER = 0.05                # m, robust loss scale
GOOD = (0.05, 2.0)          # m, deg: a fit within these counts as localized (Assumed)
STARTS = {                  # start pose error sigmas (m, deg): odometry drift, and a bad guess such as after a slip
    "normal": (0.15, 5.0),
    "hard": (0.40, 15.0),
}
MOUNTS = {
    "rear_real": None,                                       # URDF mount: upside down at the rear top
    "front45": (0.45, 0.0, 0.75, 180.0, -45.0, 0.0),         # front top, axis 45 deg forward of straight down
}
_CACHE = {}


def parts(name):
    if name not in _CACHE:
        from robot import Robot, BodyCaster, rpy_matrix
        rb = _CACHE.get("robot") or Robot()
        _CACHE["robot"] = rb
        body = _CACHE.get("body") or BodyCaster(rb, v(pr.ROBOT, "lidar_link"))
        _CACHE["body"] = body
        m = MOUNTS[name]
        if m is None:
            xyz, R = rb.mount(v(pr.ROBOT, "lidar_link"))
        else:
            xyz, R = tuple(m[:3]), rpy_matrix(*np.radians(m[3:]))
        _CACHE[name] = (xyz, R, selfmask.MaskTable(body, xyz, R))
    return _CACHE[name]


def wall_residuals(P, pose, L, W):
    """Signed distance of 2D points (robot frame) to the nearest wall, and its Jacobian in (x, y, yaw)."""
    x, y, a = pose
    c, s = np.cos(a), np.sin(a)
    Q = np.stack([x + c * P[:, 0] - s * P[:, 1], y + s * P[:, 0] + c * P[:, 1]], -1)
    d = np.stack([Q[:, 0], L - Q[:, 0], Q[:, 1], W - Q[:, 1]], -1)       # distance to x=0, x=L, y=0, y=W
    k = np.argmin(np.abs(d), 1)
    r = d[np.arange(len(d)), k]
    # d(Q)/d(yaw) = (-s px - c py, c px - s py)
    dqa = np.stack([-s * P[:, 0] - c * P[:, 1], c * P[:, 0] - s * P[:, 1]], -1)
    sign = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]], float)[k]      # gradient of d w.r.t. Q
    J = np.stack([sign[:, 0], sign[:, 1], (sign * dqa).sum(1)], -1)
    return r, J


def icp(P, pose0, L, W, iters=30):
    pose = np.array(pose0, float)
    for _ in range(iters):
        r, J = wall_residuals(P, pose, L, W)
        near = np.abs(r) < 0.5                                   # gate: points far from every wall are clutter
        if near.sum() < 10:
            return pose, None, int(near.sum())
        r, J = r[near], J[near]
        w = np.where(np.abs(r) <= HUBER, 1.0, HUBER / np.abs(r))
        H = (J * w[:, None]).T @ J
        g = (J * w[:, None]).T @ r
        step = -np.linalg.solve(H + 1e-9 * np.eye(3), g)
        pose += step
        if np.abs(step[:2]).max() < 1e-4 and abs(step[2]) < 1e-5:
            break
    r, J = wall_residuals(P, pose, L, W)
    near = np.abs(r) < HUBER * 2
    H = J[near].T @ J[near] / max(near.sum(), 1)
    return pose, float(np.linalg.eigvalsh(H[:2, :2]).min()) if near.sum() >= 3 else None, int(near.sum())


def free_pose(arena, rng):
    for _ in range(1000):
        x, y = rng.uniform(0.6, arena.L - 0.6), rng.uniform(0.6, arena.W - 0.6)
        if all(np.hypot(x - o.x, y - o.y) > o.radius + 0.6 for o in arena.obstacles):
            return x, y, rng.uniform(-180, 180)
    raise RuntimeError("no free pose")


def trial(args):
    seed, n_poses, frames_list = args
    rng = np.random.default_rng(seed)
    arena = Arena(rng)
    prng = np.random.default_rng(seed + 11_000_003)
    out = []
    for k in range(n_poses):
        x, y, yaw = free_pose(arena, prng)
        errs = {n: (prng.normal(0, sx), prng.normal(0, sx), prng.normal(0, sa)) for n, (sx, sa) in STARTS.items()}
        for name in MOUNTS:
            xyz, R, table = parts(name)
            lidar = Lidar("l1", mount_xyz=xyz, mount_R=R, body=_CACHE["body"])
            srng = np.random.default_rng([seed, k, list(MOUNTS).index(name)])
            pts = []
            for f in range(max(frames_list)):
                scan = lidar.scan(arena, (x, y, yaw), srng)
                scan, _ = selfmask.apply(scan, (x, y, yaw), table=table)
                p = scan["pts"][scan["valid"] & ~scan["is_body"]]
                pts.append(p[(p[:, 2] > Z_LO) & (p[:, 2] < Z_HI)])
            a = np.radians(yaw)
            for nf in frames_list:
                P = np.concatenate(pts[:nf])
                d = P[:, :2] - [x, y]                              # world -> robot frame (true pose)
                Pr = np.stack([np.cos(a) * d[:, 0] + np.sin(a) * d[:, 1], -np.sin(a) * d[:, 0] + np.cos(a) * d[:, 1]], -1)
                for sname, err0 in errs.items():
                    start = (x + err0[0], y + err0[1], a + np.radians(err0[2]))
                    pose, lam, n_used = icp(Pr, start, arena.L, arena.W)
                    e_xy = float(np.hypot(pose[0] - x, pose[1] - y))
                    e_yaw = float(abs((np.degrees(pose[2] - a) + 180) % 360 - 180))
                    out.append({"seed": seed, "k": k, "mount": name, "frames": nf, "start": sname, "pose": [x, y, yaw],
                                "start_err": [float(np.hypot(*err0[:2])), float(abs(err0[2]))],
                                "points": int(len(Pr)), "wall_points": n_used, "e_xy": e_xy, "e_yaw": e_yaw,
                                "lam_min": lam, "ok": bool(e_xy < GOOD[0] and e_yaw < GOOD[1])})
    return out


def summarize(rows):
    res = []
    keys = [(sn, m, nf) for sn in STARTS for m in MOUNTS for nf in sorted({r["frames"] for r in rows})]
    for sname, name, nf in keys:
        sel = [r for r in rows if r["mount"] == name and r["frames"] == nf and r["start"] == sname]
        e = np.array([r["e_xy"] for r in sel])
        ey = np.array([r["e_yaw"] for r in sel])
        lam = np.array([r["lam_min"] if r["lam_min"] is not None else 0.0 for r in sel])
        res.append({"start": sname, "mount": name, "frames": nf, "n": len(sel), "ok": float(np.mean([r["ok"] for r in sel])),
                    "wall_points_median": float(np.median([r["wall_points"] for r in sel])),
                    "e_xy_median_cm": float(100 * np.median(e)), "e_xy_p95_cm": float(100 * np.percentile(e, 95)),
                    "e_yaw_median_deg": float(np.median(ey)), "e_yaw_p95_deg": float(np.percentile(ey, 95)),
                    "lam_min_p5": float(np.percentile(lam, 5))})
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--arenas", type=int, default=12)
    ap.add_argument("--poses", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--frames", default="1,10", help="scan lengths to test, in 0.1 s L1 frames")
    a = ap.parse_args()
    frames = [int(f) for f in a.frames.split(",")]
    t0 = time.time()
    with Pool(a.workers) as p:
        rows = [r for part in p.imap(trial, [(a.seed0 + i, a.poses, frames) for i in range(a.arenas)]) for r in part]
    summ = summarize(rows)
    os.makedirs(a.out, exist_ok=True)
    meta = {"arenas": a.arenas, "poses": a.poses, "seed0": a.seed0, "frames": frames, "mounts": MOUNTS, "z_band": [Z_LO, Z_HI],
            "huber": HUBER, "good": GOOD, "start_err_sigma": STARTS, "seconds": time.time() - t0}
    with open(os.path.join(a.out, "localize.json"), "w") as f:
        json.dump({"meta": meta, "summary": summ, "rows": rows}, f)
    L = ["| Start error | Mount | Scan | Poses | Localized (< 5 cm, < 2 deg) | Wall points used | Position error median / 95th pct | "
         "Yaw error median / 95th pct |", "|" + " --- |" * 8]
    for s in summ:
        sx, sa = STARTS[s["start"]]
        L.append(f"| {s['start']} ({sx * 100:.0f} cm, {sa:g} deg) | {s['mount']} | {s['frames'] / 10:g} s | {s['n']} | {100 * s['ok']:.0f}% | {s['wall_points_median']:.0f} | "
                 f"{s['e_xy_median_cm']:.1f} / {s['e_xy_p95_cm']:.1f} cm | {s['e_yaw_median_deg']:.2f} / {s['e_yaw_p95_deg']:.2f} deg |")
    with open(os.path.join(a.out, "localize.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))
    print(f"{time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
