"""Odometry drift over one dig -> dump -> dig round trip: wheel, gyro, L1 LiDAR (KISS-ICP), fused, and wall matching.

The robot drives the custom planner's first mission cycle (sim/planner: tangent route + executor, start -> dig 1 ->
dump 1 at the berm -> dig 2) in the sandbox arena of the same seed, with the skid-steer slip model of the chosen level.
The controller sees the true pose, so every estimator below sees the same drive and is scored against the same truth
(open loop: estimator errors do not change the path). The pose starts from a start-zone tag fix whose error is drawn
from the Isaac two-tag result (sim/fiducial, results/isaac_tags_v1).

Sensors (every number in PARAMS with its status):
- wheel encoders: the left and right wheel speeds the skid-steer model turns, exact (quantisation ignored);
- gyro: true yaw rate + a constant bias left after the standstill calibration + white noise;
- Unitree L1: sensors.Lidar("l1") on the team robot at the 2027 front mount (45 deg down), body occlusion and the
  table self-mask, clear air. Each 0.1 s frame is cast in SUB slices, each from the robot's true pose at that slice's
  time, so frames carry real motion skew; points go to KISS-ICP in the base frame with per-point timestamps.

Estimators (planar x, y, yaw):
- wheel: wheel speeds only, track multiplier and distance scale as the base driver has them (no slip calibration);
- wheel_cal: wheel only, distance scaled by (1 - mean slip) of the level as if calibrated on BP-1, with a
  per-run calibration error (wheel_cal_err);
- wheel_gyro: calibrated wheel speed and gyro yaw rate;
- lidar: KISS-ICP alone (its 6-DoF pose, projected to the plane), constant-velocity guess and deskew as shipped;
- lidar_gyro: KISS-ICP with the wheel_gyro motion over each frame as its initial guess and deskew motion (the role
  the IMU plays in FAST-LIO2 / Point-LIO);
- fused: wheel_gyro prediction; every fuse_every s the window's lidar_gyro increment is fused with the window's
  wheel_gyro increment by inverse covariance, with a chi-square gate;
- fused_walls: fused, plus a wall-map ICP fix (sim/bp1_sandbox/localize.icp, the four walls only) every WALL_EVERY s,
  fed to the filter as an absolute pose measurement when it is well conditioned.

Usage: python3 drift.py out_dir [--seeds 30] [--seed0 3000] [--slip nominal,heavy] [--workers 4]
"""
import argparse
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(SIM, "planner"))
sys.path.insert(0, os.path.join(SIM, "bp1_sandbox"))
import arena2d as a2                     # noqa: E402
from params import P, v                  # noqa: E402

PARAMS = {
    "tag_sigma": P((0.013, 1.2), "m per axis, deg", "Inferred",
                   "start fix error; Isaac two-tag run: 1.5 cm median xy, 0.8 deg median yaw (sim/fiducial)"),
    "gyro_noise": P(0.014, "deg/s/sqrt(Hz)", "Inferred", "BMI055 (D435i IMU chip) noise density from a Bosch forum post; the D455 chip is unconfirmed (research/step2/22-imu-noise.md)"),
    "gyro_bias": P(0.05, "deg/s", "Assumed", "1-sigma bias left after averaging the gyro at standstill; BMI055 offset drifts +-0.015 deg/s/K (Bosch forum, 22-imu-noise.md), so a few kelvin of warm-up"),
    "imu_hz": P(100.0, "Hz", "Assumed", "gyro rate used here (the sim step)"),
    "lidar_hz": P(10.0, "Hz", "Assumed", "frames handed to KISS-ICP (sensors.l1 frame_s 0.1)"),
    "sub": P(5, "", "Assumed", "time slices per L1 frame for motion skew"),
    "kiss_max_range": P(8.0, "m", "Assumed", "KISS-ICP max range; voxel = max_range / 100 (KISS-ICP default rule)"),
    "kiss_min_range": P(0.3, "m", "Assumed", "drop returns closer than this (L1 minimum range is about 0.05 m)"),
    "fuse_every": P(1.0, "s", "Assumed", "window over which LiDAR and wheel+gyro increments are fused"),
    "sig_win_lidar": P((0.012, 0.4), "m, deg per window", "Inferred", "lidar_gyro increment error over one window, "
                       "measured on tuning arenas 1000-1003 (tune.py)"),
    "sig_v_wheel": P(0.05, "fraction of distance", "Assumed", "slip variation within a window (SLIP_LEVELS sd)"),
    "wheel_cal_err": P(0.03, "fraction of distance", "Inferred", "error of the BP-1 slip calibration: the run's "
                       "true mean slip differs from the calibrated one by this 1-sigma (research/step2/23: flat-ground "
                       "slip sd 0.03, Inferred)"),
    "gate_chi2": P(11.34, "", "Verified", "chi-square 3 dof, 99%"),
    "wall_every": P(1.0, "s", "Assumed", "wall-map fix period"),
    "wall_band": P((0.30, 1.10), "m", "Assumed", "z band of wall points (localize.py)"),
    "wall_sigma": P(0.02, "m", "Assumed", "per-point wall residual used to turn the ICP fit into a covariance"),
    "wall_lam_min": P(0.15, "", "Assumed", "weakest-direction information per point below which a fix is skipped"),
    "wall_min_pts": P(200, "", "Assumed", "wall points needed for a fix"),
}
LEGS = 3                                 # start -> dig 1 -> dump 1 -> dig 2
MOUNT = "front45"


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class LoggedSkid(a2.SkidSteer):
    """SkidSteer that records the truth and the wheel speeds every physics step."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.log = [[0.0, *self.pose, 0.0, 0.0, 0.0]]

    def step(self, cmd_v, cmd_w, dt):
        vb, wb = super().step(cmd_v, cmd_w, dt)
        self.log.append([self.t, *self.pose, self.vl, self.vr, wb])
        return vb, wb


def drive(seed, level):
    from tangent_route import TangentController
    w = a2.world(seed)
    start, goals = a2.mission(w, np.random.default_rng(seed))
    robot = LoggedSkid(start, level, np.random.default_rng([seed, 7]))
    marks = []                                                    # (log index at each leg end, kind, ok)
    tol = v(a2.PARAMS, "goal_tol")
    for kind, g in goals[:LEGS]:
        c = TangentController(w, robot, g)
        if c.info.get("fail"):
            break
        m = a2.run_leg(w, robot, g, c)
        ok = bool(m["reached"] and m["e_xy"] <= tol[0] and m["e_yaw"] <= tol[1])
        marks.append((len(robot.log) - 1, kind, ok))
        if not ok:
            break
    return np.array(robot.log), marks


def lidar_frames(seed, log, rng):
    """L1 frames along the drive: (log index at the frame end, points in the base frame of their own time slice,
    per-point time in [0, 1] within the frame)."""
    import localize as lz
    import selfmask
    from arena import Arena
    from sensors import Lidar
    arena = Arena(np.random.default_rng(seed))
    xyz, R, table = lz.parts(MOUNT)
    lid = Lidar("l1", mount_xyz=xyz, mount_R=R, body=lz._CACHE["body"])
    lid.prof = dict(lid.prof)
    sub = v(PARAMS, "sub")
    T = 1.0 / v(PARAMS, "lidar_hz")
    lid.prof["frame_s"] = T / sub
    t = log[:, 0]
    frames = []
    for te in np.arange(T, t[-1] + 1e-9, T):
        pts, ts = [], []
        for k in range(sub):
            tk = te - T + (k + 0.5) * T / sub
            i = min(int(np.searchsorted(t, tk)), len(t) - 1)
            x, y, a = log[i, 1:4]
            pose = (x, y, np.degrees(a))
            s = lid.scan(arena, pose, rng)
            s, _ = selfmask.apply(s, pose, table=table)
            ok = s["valid"] & ~s["is_body"]
            r = s["r"][ok]
            ok_r = (r >= v(PARAMS, "kiss_min_range")) & (r <= v(PARAMS, "kiss_max_range"))
            Pw = s["pts"][ok][ok_r] - [x, y, 0.0]
            c, sn = np.cos(a), np.sin(a)
            Pb = np.stack([c * Pw[:, 0] + sn * Pw[:, 1], -sn * Pw[:, 0] + c * Pw[:, 1], Pw[:, 2]], -1)
            pts.append(Pb)
            ts.append(np.full(len(Pb), (k + 0.5) / sub))
        i_end = min(int(np.searchsorted(t, te)), len(t) - 1)
        frames.append((i_end, np.concatenate(pts), np.concatenate(ts)))
    return arena, frames


def se2(x, y, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def from_se2(M):
    return np.array([M[0, 2], M[1, 2], np.arctan2(M[1, 0], M[0, 0])])


def run_estimators(seed, level, log, frames, arena, rng):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP
    import localize as lz
    dt = a2.SIM_DT
    B, mult = v(a2.PARAMS, "track"), a2.SLIP_LEVELS[level][2]
    # distance scale from a BP-1 slip calibration, off by wheel_cal_err for this run
    scale = (1.0 - a2.SLIP_LEVELS[level][0]) * (1.0 + rng.normal(0, v(PARAMS, "wheel_cal_err")))
    # start fix
    sx, sa = v(PARAMS, "tag_sigma")
    true0 = log[0, 1:4]
    start = true0 + [rng.normal(0, sx), rng.normal(0, sx), np.radians(rng.normal(0, sa))]
    # gyro
    bias = np.radians(rng.normal(0, v(PARAMS, "gyro_bias")))
    n_sd = np.radians(v(PARAMS, "gyro_noise")) * np.sqrt(1.0 / dt)
    gyro = log[:, 6] + bias + rng.normal(0, n_sd, len(log))
    vl, vr = log[:, 4], log[:, 5]
    v_wheel = (vl + vr) / 2
    w_wheel = (vr - vl) / (B * mult)

    def integrate(vv, ww, s0):
        out = np.zeros((len(vv), 3))
        out[0] = s0
        for i in range(1, len(vv)):
            x, y, a = out[i - 1]
            am = a + ww[i] * dt / 2
            out[i] = [x + vv[i] * np.cos(am) * dt, y + vv[i] * np.sin(am) * dt, a + ww[i] * dt]
        return out

    est = {"wheel": integrate(v_wheel, w_wheel, start), "wheel_cal": integrate(scale * v_wheel, w_wheel, start),
           "wheel_gyro": integrate(scale * v_wheel, gyro, start)}
    # KISS-ICP
    cfg = KISSConfig()
    cfg.data.max_range = v(PARAMS, "kiss_max_range")
    cfg.data.min_range = v(PARAMS, "kiss_min_range")
    cfg.data.deskew = True
    cfg.mapping.voxel_size = cfg.data.max_range / 100.0
    S0 = se2(*start)

    def wg_increment(i0, i1):
        inc = np.zeros(3)
        for i in range(i0 + 1, i1 + 1):
            am = inc[2] + gyro[i] * dt / 2
            inc += [scale * v_wheel[i] * np.cos(am) * dt, scale * v_wheel[i] * np.sin(am) * dt, gyro[i] * dt]
        return inc

    lidar_idx = [f[0] for f in frames]
    kiss_runs = {}
    for name, prior in (("lidar", False), ("lidar_gyro", True)):
        kiss = KissICP(cfg)
        poses, prev = [], 0
        for i_end, pts, ts in frames:
            if prior:                  # wheel+gyro motion over this frame as the initial guess and for deskewing
                x_, y_, a_ = wg_increment(prev, i_end)
                D = np.eye(4)
                D[:2, :2] = [[np.cos(a_), -np.sin(a_)], [np.sin(a_), np.cos(a_)]]
                D[0, 3], D[1, 3] = x_, y_
                kiss.last_delta = D
            prev = i_end
            kiss.register_frame(pts.astype(np.float64), ts.astype(np.float64))
            Pk = kiss.last_pose
            poses.append(se2(Pk[0, 3], Pk[1, 3], np.arctan2(Pk[1, 0], Pk[0, 0])))   # heading of the 3D rotation
        kiss_runs[name] = poses
    kiss_poses = kiss_runs["lidar_gyro"]
    # fused filter: wheel+gyro prediction every frame; every FUSE_EVERY s the window's lidar_gyro increment is fused
    # with the window's wheel+gyro increment (single frames are too noisy: see sig_win_lidar), then the wall fix
    sig_l = np.array([v(PARAMS, "sig_win_lidar")[0]] * 2 + [np.radians(v(PARAMS, "sig_win_lidar")[1])])
    gate = v(PARAMS, "gate_chi2")
    win_n = int(round(v(PARAMS, "fuse_every") * v(PARAMS, "lidar_hz")))
    wall_n = int(round(v(PARAMS, "wall_every") * v(PARAMS, "lidar_hz")))
    zlo, zhi = v(PARAMS, "wall_band")
    sig_d = np.hypot(v(PARAMS, "sig_v_wheel"), v(PARAMS, "wheel_cal_err"))
    out = {}
    for variant in ("fused", "fused_walls"):
        x = start.copy()
        Pc = np.diag([sx ** 2, sx ** 2, np.radians(sa) ** 2])
        track = np.zeros((len(log), 3))
        track[0] = x
        prev_i = 0
        x_w0, P_w0, K_w0, i_w0 = x.copy(), Pc.copy(), np.eye(3), 0
        n_gated = n_walls = n_win = 0
        buf = []                                             # wall points in the world, by the estimated track
        T = 1.0 / v(PARAMS, "lidar_hz")
        for f, (i_end, pts, ts) in enumerate(frames):
            for i in range(prev_i + 1, i_end + 1):           # wheel+gyro prediction step by step
                am = x[2] + gyro[i] * dt / 2
                x = x + [scale * v_wheel[i] * np.cos(am) * dt, scale * v_wheel[i] * np.sin(am) * dt, gyro[i] * dt]
                track[i] = x
            prev_i = i_end
            if (f + 1) % win_n == 0:
                inc = from_se2(np.linalg.inv(se2(*x_w0)) @ se2(*x))
                n_steps = i_end - i_w0
                d = float(np.sum(np.abs(scale * v_wheel[i_w0 + 1:i_end + 1])) * dt)
                sv = sig_d * d + 1e-4
                syaw2 = (np.radians(v(PARAMS, "gyro_noise")) ** 2 * n_steps * dt
                         + (np.radians(v(PARAMS, "gyro_bias")) * n_steps * dt) ** 2) + 1e-10
                S_w = np.diag([sv ** 2, sv ** 2, syaw2])
                K = kiss_poses[f]
                inc_l = from_se2(np.linalg.inv(K_w0) @ K)
                S_l = np.diag(sig_l ** 2)
                r = inc_l - inc
                r[2] = wrap(r[2])
                n_win += 1
                if r @ np.linalg.solve(S_w + S_l, r) < gate:
                    Sf = np.linalg.inv(np.linalg.inv(S_w) + np.linalg.inv(S_l))
                    incf = Sf @ (np.linalg.solve(S_w, inc) + np.linalg.solve(S_l, inc + r))
                else:
                    Sf, incf = S_w, inc
                    n_gated += 1
                c, s = np.cos(x_w0[2]), np.sin(x_w0[2])
                J1 = np.array([[1, 0, -s * incf[0] - c * incf[1]], [0, 1, c * incf[0] - s * incf[1]], [0, 0, 1]])
                J2 = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                x = from_se2(se2(*x_w0) @ se2(*incf))
                Pc = J1 @ P_w0 @ J1.T + J2 @ Sf @ J2.T
                track[i_end] = x
            if variant == "fused_walls":
                band = (pts[:, 2] > zlo) & (pts[:, 2] < zhi)
                # each slice's points to the world with the estimated pose at the slice time (deskewed by the filter's
                # own wheel+gyro track), then into the current estimated body frame at the fix
                t_sl = log[i_end, 0] - T + ts[band] * T
                idx = np.clip(np.searchsorted(log[:, 0], t_sl), 0, i_end)
                xb = track[idx]
                q = pts[band, :2]
                c_, s_ = np.cos(xb[:, 2]), np.sin(xb[:, 2])
                buf.append(np.stack([xb[:, 0] + c_ * q[:, 0] - s_ * q[:, 1], xb[:, 1] + s_ * q[:, 0] + c_ * q[:, 1]], -1))
                if (f + 1) % wall_n == 0:
                    Xi = np.linalg.inv(se2(*x))
                    Pl = np.concatenate(buf) @ Xi[:2, :2].T + Xi[:2, 2] if buf else np.zeros((0, 2))
                    buf = []
                    if len(Pl) >= v(PARAMS, "wall_min_pts"):
                        pose, lam, n_used = lz.icp(Pl, x, arena.L, arena.W)
                        if lam is not None and lam > v(PARAMS, "wall_lam_min") and n_used >= v(PARAMS, "wall_min_pts"):
                            rr, Jw = lz.wall_residuals(Pl, pose, arena.L, arena.W)
                            near = np.abs(rr) < 0.1
                            H = Jw[near].T @ Jw[near]
                            Rw = v(PARAMS, "wall_sigma") ** 2 * np.linalg.inv(H + 1e-6 * np.eye(3))
                            Rw = 10 * Rw                     # inflated: neighbouring points are not independent
                            z = pose - x
                            z[2] = wrap(z[2])
                            S = Pc + Rw
                            if z @ np.linalg.solve(S, z) < gate:
                                Kg = Pc @ np.linalg.inv(S)
                                x = x + Kg @ z
                                x[2] = wrap(x[2])
                                Pc = (np.eye(3) - Kg) @ Pc
                                n_walls += 1
                                track[i_end] = x
            if (f + 1) % win_n == 0:
                x_w0, P_w0, K_w0, i_w0 = x.copy(), Pc.copy(), kiss_poses[f], i_end
        # fill steps between frame ends (already filled during prediction) and after the last frame
        for i in range(prev_i + 1, len(log)):
            track[i] = track[i - 1]
        est[variant] = track
        out[variant] = {"windows": n_win, "gated": n_gated, "wall_fixes": n_walls, "frames": len(frames)}
    for name, poses in kiss_runs.items():
        tr = np.zeros((len(log), 3))
        tr[0] = start
        j = 0
        for i in range(1, len(log)):
            while j < len(lidar_idx) and lidar_idx[j] <= i:
                j += 1
            tr[i] = from_se2(S0 @ poses[j - 1]) if j > 0 else start
        est[name] = tr
    return est, out, start


def mission(args):
    seed, level = args
    t0 = time.time()
    log, marks = drive(seed, level)
    rng = np.random.default_rng([seed, 99, list(a2.SLIP_LEVELS).index(level)])
    arena, frames = lidar_frames(seed, log, rng)
    est, info, start = run_estimators(seed, level, log, frames, arena, rng)
    dist = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(log[:, 1:3], axis=0).T))])
    rows = []
    for idx, kind, ok in marks:
        tr = log[idx, 1:4]
        row = {"seed": seed, "slip": level, "leg_end": kind, "leg_ok": ok, "t": float(log[idx, 0]),
               "dist": float(dist[idx]), "turned_deg": float(np.degrees(np.abs(log[1:idx + 1, 6]).sum() * a2.SIM_DT))}
        for name, e in est.items():
            row[f"{name}_xy"] = float(np.hypot(*(e[idx, :2] - tr[:2])))
            row[f"{name}_yaw"] = float(abs(np.degrees(wrap(e[idx, 2] - tr[2]))))
        rows.append(row)
    for k, kind in enumerate(m[1] for m in marks):
        rows[k]["kind_n"] = sum(1 for m in marks[:k + 1] if m[1] == kind)
    return {"seed": seed, "slip": level, "legs": len(marks), "rows": rows, "filter": info,
            "seconds": time.time() - t0, "start_err_cm": float(100 * np.hypot(*(start[:2] - log[0, 1:3])))}


EST = ["wheel", "wheel_cal", "wheel_gyro", "lidar", "lidar_gyro", "fused", "fused_walls"]
POINTS = [("dump", 1, "first berm arrival"), ("dig", 2, "back at the dig area (round trip)")]


def boot_ci(x, rng, n=2000, q=50):
    x = np.asarray(x)
    if len(x) < 3:
        return (float("nan"), float("nan"))
    b = np.percentile(x[rng.integers(0, len(x), (n, len(x)))], q, axis=1)
    return float(np.percentile(b, 2.5)), float(np.percentile(b, 97.5))


def summarize(runs):
    rng = np.random.default_rng(0)
    out = []
    for level in sorted({r["slip"] for r in runs}, key=list(a2.SLIP_LEVELS).index):
        for kind, n, label in POINTS:
            rows = [x for r in runs if r["slip"] == level for x in r["rows"]
                    if x["leg_end"] == kind and x.get("kind_n") == n]
            if not rows:
                continue
            for e in EST:
                xy = 100 * np.array([x[f"{e}_xy"] for x in rows])
                yaw = np.array([x[f"{e}_yaw"] for x in rows])
                out.append({"slip": level, "point": label, "estimator": e, "n": len(rows),
                            "dist_median_m": float(np.median([x["dist"] for x in rows])),
                            "xy_median_cm": float(np.median(xy)), "xy_ci_cm": boot_ci(xy, rng),
                            "xy_p95_cm": float(np.percentile(xy, 95)),
                            "yaw_median_deg": float(np.median(yaw)), "yaw_p95_deg": float(np.percentile(yaw, 95))})
    return out


def table(summ):
    L = ["| Slip | Where | Estimator | Runs | Distance (median) | Position error median [95% CI] | Position error 95th pct | "
         "Heading error median / 95th pct |", "|" + " --- |" * 8]
    for s in summ:
        L.append(f"| {s['slip']} | {s['point']} | {s['estimator']} | {s['n']} | {s['dist_median_m']:.1f} m | "
                 f"{s['xy_median_cm']:.1f} [{s['xy_ci_cm'][0]:.1f}, {s['xy_ci_cm'][1]:.1f}] cm | {s['xy_p95_cm']:.1f} cm | "
                 f"{s['yaw_median_deg']:.2f} / {s['yaw_p95_deg']:.2f} deg |")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--slip", default="nominal,heavy")
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    for s in range(a.seed0, a.seed0 + a.seeds):
        a2.world(s)
    jobs = [(s, lv) for lv in a.slip.split(",") for s in range(a.seed0, a.seed0 + a.seeds)]
    os.makedirs(a.out, exist_ok=True)
    t0 = time.time()
    runs = []
    with Pool(a.workers) as pool:
        for i, r in enumerate(pool.imap_unordered(mission, jobs)):
            runs.append(r)
            print(f"{i + 1}/{len(jobs)} seed {r['seed']} {r['slip']} legs {r['legs']} {r['seconds']:.0f} s "
                  f"(total {time.time() - t0:.0f} s)", flush=True)
            json.dump({"runs": runs}, open(os.path.join(a.out, "partial.json"), "w"))
    summ = summarize(runs)
    meta = {"seeds": [a.seed0, a.seed0 + a.seeds - 1], "slip_levels": {k: a2.SLIP_LEVELS[k] for k in a.slip.split(",")},
            "mount": MOUNT, "params": {k: [p.value, p.unit, p.status, p.source] for k, p in PARAMS.items()},
            "seconds": time.time() - t0}
    json.dump({"meta": meta, "summary": summ, "runs": runs}, open(os.path.join(a.out, "drift.json"), "w"))
    json.dump({"meta": meta, "summary": summ}, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    md = table(summ)
    open(os.path.join(a.out, "drift.md"), "w").write(md + "\n")
    os.remove(os.path.join(a.out, "partial.json"))
    print(md)


if __name__ == "__main__":
    main()
