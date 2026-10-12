"""Detect the start-zone tags in the Isaac renders (isaac_tags.py) and score the pose against tag_start's prediction.

Usage: python3 isaac_tags_eval.py RAW_DIR RESULTS_DIR
Detector: OpenCV aruco DICT_APRILTAG_36h11 with sub-pixel corner refinement. Pose: tag_start.solve() (IPPE for tags on
one wall, SQPnP + LM for both walls), robot pose from tag_start.robot_from_cam(). Scored twice: with the exact tag map
(render + detector error only) and with tag_start's placement error (5 mm / 1 deg, Assumed), which is what its
prediction includes. The prediction is tag_start.trial() on the same poses and settings. Writes RESULTS_DIR/summary.json
and a few frames with the detections drawn.
"""
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tag_start as ts  # noqa: E402

raw, res = sys.argv[1], sys.argv[2]
os.makedirs(res, exist_ok=True)
meta = json.load(open(os.path.join(raw, "meta.json")))
ts.PARAMS["tag_size"]["value"] = meta["tag_size"]
ts.PARAMS["tag_height"]["value"] = meta["tag_height"]
TAGS = ts.layouts()[meta["layout"]]

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
if hasattr(cv2.aruco, "ArucoDetector"):                       # OpenCV >= 4.7
    _p = cv2.aruco.DetectorParameters()
    _p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    _det = cv2.aruco.ArucoDetector(DICT, _p)
    detect = lambda g: _det.detectMarkers(g)[:2]              # noqa: E731
else:
    _p = cv2.aruco.DetectorParameters_create()
    _p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detect = lambda g: cv2.aruco.detectMarkers(g, DICT, parameters=_p)[:2]   # noqa: E731


def project(corners, R_wc, t_wc):
    pc = (corners - t_wc) @ R_wc
    return (pc @ ts.K.T)[:, :2] / pc[:, 2:3]


def stats(rows):
    got = [r for r in rows if r]
    exy = [r["e_xy"] for r in got]
    eyaw = [r["e_yaw"] for r in got]
    q = lambda x, p: float(np.percentile(x, p)) if x else None      # noqa: E731
    return {"seen_frac": len(got) / len(rows), "n_seen": len(got),
            "e_xy_median": q(exy, 50), "e_xy_p95": q(exy, 95), "e_yaw_median": q(eyaw, 50), "e_yaw_p95": q(eyaw, 95),
            "heading_flips": int(sum(e > 5.0 for e in eyaw)),
            "two_walls_frac": float(np.mean([r["planes"] == 2 for r in got])) if got else None}


out = {"raw": os.path.relpath(raw, HERE), "meta": {k: meta[k] for k in ("seed", "layout", "tag_size", "tag_height",
                                                                         "image", "average", "light")},
       "n_poses": len(meta["poses"]), "detector": f"cv2 {cv2.__version__} aruco DICT_APRILTAG_36h11, CORNER_REFINE_SUBPIX",
       "cameras": {}}
for cam, cfg in meta["cameras"].items():
    ts.set_hfov(cfg["hfov"])
    ts.set_pitch(cfg["pitch_deg_up"])
    d = np.load(os.path.join(raw, f"{cam}.npz"))
    grays, poses = d["gray"], [tuple(p) for p in d["poses"]]
    prng = np.random.default_rng([meta["seed"], 4])            # placement error draws for the Isaac scoring
    exact, placed, corner_err, wrong_ids, n_det, n_truth_vis = [], [], [], 0, 0, 0
    for k, (g, pose) in enumerate(zip(grays, poses)):
        R_wc, t_wc = ts.cam_pose(pose)
        n_truth_vis += sum(ts.visible(c, n, R_wc, t_wc, np.random.default_rng(0)) is not None for c, n in TAGS)
        corners, ids = detect(g)
        ids = [] if ids is None else [int(i) for i in np.ravel(ids)]
        keep = [(i, c.reshape(4, 2)) for i, c in zip(ids, corners) if 0 <= i < len(TAGS)]
        wrong_ids += len(ids) - len(keep)
        n_det += len(keep)
        for i, uv in keep:
            corner_err.append(float(np.median(np.linalg.norm(uv - project(TAGS[i][0], R_wc, t_wc), axis=1))))
        if k < 4:
            vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            if keep:
                cv2.aruco.drawDetectedMarkers(vis, [c.reshape(1, 4, 2) for _, c in keep], np.array([[i] for i, _ in keep]))
            cv2.imwrite(os.path.join(res, f"{cam}_f{k:02d}.jpg"), cv2.resize(vis, (640, 360)), [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not keep:
            exact.append(None)
            placed.append(None)
            continue
        img = np.vstack([uv for _, uv in keep])
        planes = len({tuple(np.round(TAGS[i][1], 6)) for i, _ in keep})
        believed = ts.perturb(TAGS, prng)
        for tagmap, rows in ((TAGS, exact), (believed, placed)):
            obj = np.vstack([tagmap[i][0] for i, _ in keep])
            x, y, yaw = ts.robot_from_cam(*ts.solve(obj, img, coplanar=planes == 1))
            rows.append({"e_xy": float(math.hypot(x - pose[0], y - pose[1])),
                         "e_yaw": float(abs(math.degrees(ts.wrap(yaw - pose[2])))), "planes": planes})
    trng = np.random.default_rng([meta["seed"], 1])
    pred = [ts.trial(p, TAGS, trng) for p in poses]
    pred_turn = [ts.turn_to_see(p, TAGS, trng) for p in poses]
    isaac_turn = [None if np.isnan(t) else float(t) for t in d["turn"]]

    def turn_stats(t):
        ok = [x for x in t if x is not None]
        return {"turn_median_deg": float(np.median(ok)) if ok else None,
                "turn_p95_deg": float(np.percentile(ok, 95)) if ok else None,
                "never_frac": float(np.mean([x is None for x in t])), "per_pose_deg": t}
    out["cameras"][cam] = {"hfov_rad": cfg["hfov"], "pitch_deg_up": cfg["pitch_deg_up"],
                           "isaac_exact_map": stats(exact), "isaac_placement_error": stats(placed),
                           "python_prediction_same_poses": stats(pred),
                           "turn_isaac": turn_stats(isaac_turn), "turn_python": turn_stats(pred_turn),
                           "detections": n_det, "decodable_by_python_model": n_truth_vis, "wrong_ids": wrong_ids,
                           "corner_err_px_median": float(np.median(corner_err)) if corner_err else None,
                           "corner_err_px_p95": float(np.percentile(corner_err, 95)) if corner_err else None}
    c = out["cameras"][cam]
    for key in ("isaac_exact_map", "isaac_placement_error", "python_prediction_same_poses"):
        s = c[key]
        f = lambda x, m=1: "-" if x is None else f"{x * m:.1f}"     # noqa: E731
        print(f"{cam:5s} {key:29s} seen {s['seen_frac']:.2f}  xy median {f(s['e_xy_median'], 100)} cm, "
              f"p95 {f(s['e_xy_p95'], 100)} cm  yaw median {f(s['e_yaw_median'])} deg, p95 {f(s['e_yaw_p95'])} deg  "
              f"flips {s['heading_flips']}")
    for key in ("turn_isaac", "turn_python"):
        t = c[key]
        print(f"{cam:5s} {key:11s} median {t['turn_median_deg']} deg, p95 {t['turn_p95_deg']} deg, never {t['never_frac']:.2f}")
    print(f"{cam:5s} detections {n_det} (python model says {n_truth_vis} decodable), wrong ids {wrong_ids}, "
          f"corner error median {c['corner_err_px_median']} px")
json.dump(out, open(os.path.join(res, "summary.json"), "w"), indent=1)
print(f"wrote {res}/summary.json")
