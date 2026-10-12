"""Stereo visual odometry on the Isaac D455 renders (isaac_vo.py), scored at the drift drive's leg ends.

Usage: vo_eval.py RAW_DIR RESULTS_DIR (OpenCV with numpy; run_isaac_vo.sh runs it on Isaac's Python)
Per frame pair at 15 Hz (every rendered pose): GFTT corners on the left image at frame k, matched into the right
image with pyramidal KLT (rectified pair: kept if the row differs by < 1 px and the disparity is positive; depth
Z = f B / d, kept to 8 m), tracked into the left image at k+1 with KLT and a forward-backward check (< 1 px), then
solvePnPRansac on the 3D-2D pairs (iterative refine on the inliers). A frame "loses track" when fewer than
MIN_INLIERS inliers remain; that step reuses the previous step's motion (constant velocity).
Two estimators of the planar pose (x, y, yaw of base_link; the camera-to-base transform is the known mount):
  vo       camera only: the composed 6-DoF camera pose, projected to the plane;
  vo_gyro  yaw from a gyro (true yaw rate + 0.05 deg/s bias + 0.014 deg/s/sqrt(Hz) white noise, drift.py PARAMS);
           VO supplies only the translation step, expressed in the robot frame and rotated by the gyro yaw.
Both start at the true pose of the second rendered frame (drift.py adds the start-tag error separately). Errors are reported at every leg end
(dig 1, dump 1, dig 2) and at the end of the drive, with tracked-feature and lost-frame counts.
"""
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
raw, res = sys.argv[1], sys.argv[2]
os.makedirs(res, exist_ok=True)
meta = json.load(open(os.path.join(raw, "meta.json")))
TRAJ = os.path.join(HERE, meta["traj"])
W, H = meta["image"]
F = (W / 2) / math.tan(math.radians(meta["hfov_deg"]) / 2)
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
B = meta["baseline_m"]
MAX_DEPTH = 8.0
MIN_INLIERS = 15
GYRO_BIAS, GYRO_NOISE = 0.05, 0.014          # deg/s, deg/s/sqrt(Hz) (drift.py PARAMS gyro_bias, gyro_noise)
LK = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
# sandbox camera frame (x forward, y left, z up) -> OpenCV optical frame (x right, y down, z forward)
S2O = np.array([[0, -1.0, 0], [0, 0, -1.0], [1.0, 0, 0]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def planar(T):
    return T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])


def stereo_points(L, R):
    """Corners on L with stereo depth: (pixels (n, 2), 3D points in the left optical frame (n, 3))."""
    p = cv2.goodFeaturesToTrack(L, maxCorners=600, qualityLevel=0.005, minDistance=8, blockSize=7)
    if p is None:
        return np.zeros((0, 2), np.float32), np.zeros((0, 3))
    p = p.reshape(-1, 2).astype(np.float32)
    q, st, _ = cv2.calcOpticalFlowPyrLK(L, R, p, p.copy(), **LK)
    st = st.ravel() == 1
    d = p[:, 0] - q[:, 0]
    ok = st & (np.abs(p[:, 1] - q[:, 1]) < 1.0) & (d > 0.3)
    p, d = p[ok], d[ok]
    Z = F * B / d
    ok = Z < MAX_DEPTH
    p, Z = p[ok], Z[ok]
    X = np.stack([(p[:, 0] - W / 2) * Z / F, (p[:, 1] - H / 2) * Z / F, Z], 1)
    return p, X


def track(A, Bimg, p):
    q, st, _ = cv2.calcOpticalFlowPyrLK(A, Bimg, p, None, **LK)
    r, st2, _ = cv2.calcOpticalFlowPyrLK(Bimg, A, q, None, **LK)
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm(r - p, axis=1) < 1.0)
    ok &= (q[:, 0] >= 0) & (q[:, 0] < W) & (q[:, 1] >= 0) & (q[:, 1] < H)
    return q, ok


def run_drive(key):
    d = np.load(os.path.join(raw, f"{key}.npz"))
    seed = int(key.split("_")[0])
    tr = np.load(os.path.join(TRAJ, f"traj_{seed}_nominal.npz"))
    # the first render of each drive comes out before the scene settles (brighter, different contrast), so every
    # drive starts at its second rendered pose (1/15 s in), from the true pose there
    Ls, Rs, idx, camT, baseT = d["left"][1:], d["right"][1:], d["idx"][1:], d["cam_T"][1:], d["base_T"][1:]
    T_bc = np.linalg.inv(baseT[0]) @ camT[0]                       # base from left camera (fixed mount)
    T_bc_inv = np.linalg.inv(T_bc)
    n = len(idx)
    # camera-only VO: world-from-camera in the sandbox camera convention
    Tw = camT[0].copy()
    est_vo = [planar(Tw @ T_bc_inv)]
    # vo_gyro: gyro yaw integrated at the drive's 30 Hz log, VO translation steps in the robot frame
    rng = np.random.default_rng([seed, 11])
    t = tr["t"]
    dt = np.diff(t, prepend=t[0])
    hz = 1.0 / np.median(np.diff(t))
    # integrated gyro = true yaw + integrated bias and white noise (the log is ~33 Hz, too coarse to integrate the
    # true rate itself without a rectangle-rule error of ~0.9 deg over a drive; a 100+ Hz gyro has no such error)
    err = (math.radians(GYRO_BIAS) + math.radians(GYRO_NOISE) * math.sqrt(hz) * rng.standard_normal(len(t))) * dt
    yaw_g = np.unwrap(tr["yaw"]) + np.cumsum(err)
    x0, y0, _ = planar(baseT[0])
    pos_g = np.array([x0, y0])
    est_g = [(x0, y0, yaw_g[idx[0]])]
    n_feat, n_inl, lost = [], [], []
    last = np.eye(4)                                                 # previous step's camera motion (k -> k+1)
    for k in range(n - 1):
        p, X = stereo_points(Ls[k], Rs[k])
        q, ok = track(Ls[k], Ls[k + 1], p) if len(p) else (p, np.zeros(0, bool))
        n_feat.append(int(ok.sum()))
        M = None
        if ok.sum() >= 6:
            good, rvec, tvec, inl = cv2.solvePnPRansac(X[ok], q[ok], K, None, reprojectionError=1.5,
                                                       iterationsCount=200, flags=cv2.SOLVEPNP_EPNP)
            if good and inl is not None and len(inl) >= MIN_INLIERS:
                ii = inl.ravel()
                rvec, tvec = cv2.solvePnPRefineLM(X[ok][ii], q[ok][ii], K, None, rvec, tvec)
                Ro, _ = cv2.Rodrigues(rvec)
                Mo = np.eye(4)
                Mo[:3, :3], Mo[:3, 3] = Ro, tvec.ravel()                # optical k -> optical k+1
                S = np.eye(4)
                S[:3, :3] = S2O
                M = np.linalg.inv(S) @ Mo @ S                           # sandbox-camera k -> k+1
                n_inl.append(len(ii))
        if M is None:
            lost.append(k)
            n_inl.append(0)
            M = last
        last = M
        step = np.linalg.inv(M)                                          # camera k+1 pose in camera k frame
        Tw = Tw @ step
        est_vo.append(planar(Tw @ T_bc_inv))
        sb = T_bc @ step @ T_bc_inv                                      # base k+1 pose in base k frame
        a = yaw_g[idx[k]]
        c, s = math.cos(a), math.sin(a)
        pos_g = pos_g + np.array([c * sb[0, 3] - s * sb[1, 3], s * sb[0, 3] + c * sb[1, 3]])
        est_g.append((pos_g[0], pos_g[1], yaw_g[idx[k + 1]]))
    truth = [planar(T) for T in baseT]
    path = np.r_[0, np.cumsum(np.hypot(np.diff([p[0] for p in truth]), np.diff([p[1] for p in truth])))]
    out = {"frames": n, "lost_frames": len(lost), "lost_at": lost[:50],
           "features_tracked_median": float(np.median(n_feat)), "features_tracked_min": int(np.min(n_feat)),
           "inliers_median": float(np.median(n_inl)), "path_m": float(path[-1]), "estimators": {}}
    ends = np.minimum(tr["leg_end_idx"], len(tr["t"]) - 1)      # export_traj's last leg end can be one past the end
    marks = [(int(np.searchsorted(idx, i)), "dump" if dump else "dig") for i, dump in
             zip(ends, tr["leg_end_is_dump"]) if i <= idx[-1]]
    names = {0: "dig1", 1: "dump1", 2: "dig2"}
    for name, est in (("vo", est_vo), ("vo_gyro", est_g)):
        rows = {}
        for j, (k, kind) in enumerate(marks + [(n - 1, "end")]):
            e = est[k]
            g = truth[k]
            rows[names.get(j, "end")] = {"kind": kind, "frame": k, "path_m": float(path[k]),
                                         "e_xy": float(math.hypot(e[0] - g[0], e[1] - g[1])),
                                         "e_yaw_deg": float(abs(math.degrees(wrap(e[2] - g[2]))))}
        out["estimators"][name] = rows
    # sample frame with the tracked features
    k = n // 3
    p, X = stereo_points(Ls[k], Rs[k])
    q, ok = track(Ls[k], Ls[k + 1], p)
    vis = cv2.cvtColor(Ls[k + 1], cv2.COLOR_GRAY2BGR)
    for a_, b_ in zip(p[ok], q[ok]):
        cv2.line(vis, tuple(int(v) for v in a_), tuple(int(v) for v in b_), (0, 200, 0), 1)
        cv2.circle(vis, tuple(int(v) for v in b_), 2, (0, 0, 255), -1)
    cv2.imwrite(os.path.join(res, f"{key}_tracks_f{k:03d}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return out


summary = {"raw": os.path.relpath(raw, HERE), "meta": {k: meta[k] for k in ("image", "hfov_deg", "baseline_m",
                                                                             "every", "average", "lights")},
           "method": {"detector": "GFTT 600, quality 0.005, min distance 8 px", "stereo": "KLT left->right, |dy| < 1 px",
                      "temporal": "KLT with forward-backward < 1 px", "pose": "EPnP RANSAC 1.5 px + LM refine",
                      "min_inliers": MIN_INLIERS, "max_depth_m": MAX_DEPTH, "gyro_bias_deg_s": GYRO_BIAS,
                      "gyro_noise_deg_s_rthz": GYRO_NOISE, "opencv": cv2.__version__},
           "drives": {}}
for key in meta["drives"]:
    r = run_drive(key)
    summary["drives"][key] = r
    for name, rows in r["estimators"].items():
        cells = "  ".join(f"{leg} {v['e_xy'] * 100:.1f} cm / {v['e_yaw_deg']:.2f} deg" for leg, v in rows.items())
        print(f"{key:14s} {name:8s} {cells}", flush=True)
    print(f"{key:14s} features median {r['features_tracked_median']:.0f} (min {r['features_tracked_min']}), "
          f"inliers median {r['inliers_median']:.0f}, lost {r['lost_frames']}/{r['frames'] - 1}, path {r['path_m']:.1f} m",
          flush=True)
json.dump(summary, open(os.path.join(res, "summary.json"), "w"), indent=1)
print(f"wrote {res}/summary.json")
