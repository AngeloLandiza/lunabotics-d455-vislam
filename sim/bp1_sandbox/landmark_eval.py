"""Features and relocalization before and after wheel ruts, from isaac_landmarks.py images.

Usage: python3 landmark_eval.py [isaac_out/landmarks_seed1] [--out DIR]
Per pass (A, B32, B32cam, B50) and light (diffuse, low), per frame:
- ORB keypoints (OpenCV, nfeatures 2000, fastThreshold FAST_T; Assumed settings for low-contrast regolith);
- each keypoint's world point from the distance_to_image_plane image and the camera pose; its distance to the nearest
  rut edge (trench edge = wheel-track centre line +- half the wheel width, the same tracks in every pass) and whether
  it lies within EDGE_NEAR of one;
- relocalization: ORB descriptors of pass A (the map, made before the ruts) matched to pass B at the same frame
  (brute force Hamming, Lowe ratio RATIO). A match counts as correct when A's keypoint, lifted to 3D with A's depth
  and reprojected into B's camera, lands within REPROJ_PX of B's keypoint (ground truth from the known poses, no
  RANSAC). Survival = correct matches / A keypoints that are visible in B (reprojected inside the image).
Output: <out>/landmarks.json, landmarks.md, landmarks.png.
"""
import argparse
import json
import os

import cv2
import numpy as np

from arena import Arena
from drive_path import dist_to_segments, track_segments
from robot import Robot
from ruts import wheel_offsets

FAST_T = 7          # ORB FAST threshold (OpenCV default 20 finds few corners on dark, low-contrast regolith)
RATIO = 0.8         # Lowe ratio test
REPROJ_PX = 3.0     # px: a match is correct when the reprojection agrees this well
EDGE_NEAR = 0.03    # m: "within 3 cm of a rut edge"
OFF_RUT = 0.15      # m from every wheel-track centre line: an "off-rut" landmark


def intrinsics(meta):
    w, h = meta["image"]
    f = (w / 2) / np.tan(np.radians(meta["hfov_deg"] / 2))
    return f, (w - 1) / 2, (h - 1) / 2


def lift(kp_uv, depth, T, f, cx, cy):
    """World points of pixels (u, v) from the depth along the optical axis (camera x forward, y left, z up)."""
    u, v = kp_uv[:, 0], kp_uv[:, 1]
    iu = np.clip(np.rint(u).astype(int), 0, depth.shape[1] - 1)
    iv = np.clip(np.rint(v).astype(int), 0, depth.shape[0] - 1)
    z = depth[iv, iu].astype(float)
    p = np.stack([z, -(u - cx) / f * z, -(v - cy) / f * z], 1)
    return p @ T[:3, :3].T + T[:3, 3], np.isfinite(z)


def project(P, T, f, cx, cy, shape):
    q = (P - T[:3, 3]) @ T[:3, :3]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx - f * q[:, 1] / q[:, 0]
        v = cy - f * q[:, 2] / q[:, 0]
    ok = (q[:, 0] > 0.05) & (u >= 0) & (u <= shape[1] - 1) & (v >= 0) & (v <= shape[0] - 1)
    return np.stack([u, v], 1), ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir", nargs="?", default=os.path.join("isaac_out", "landmarks_seed1"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.dir, "meta.json")))
    f, cx, cy = intrinsics(meta)
    rb = Robot()
    wheels = np.array([c for c, _, _ in rb.wheels])
    poses = [tuple(p) for p in meta["poses"]]
    segs = track_segments(poses, len(poses) - 1, wheels[:, 0].min(), wheels[:, 0].max(), wheel_offsets(rb))
    half = rb.wheels[0][2] / 2
    orb = cv2.ORB_create(nfeatures=2000, fastThreshold=FAST_T)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    lights = sorted({k.split("_")[1][:-4] for k in os.listdir(a.dir) if k.endswith(".npz") and "_" in k})
    passes = [p for p in ("A", "Arep", "Acam32", "B32", "B32cam", "B50") if os.path.exists(os.path.join(a.dir, f"{p}_{lights[0]}.npz"))]
    res = {}
    for light in lights:
        data = {p: np.load(os.path.join(a.dir, f"{p}_{light}.npz")) for p in passes}
        feats = {}
        for p in passes:
            out = []
            for k in range(len(data[p]["gray"])):
                kps, des = orb.detectAndCompute(data[p]["gray"][k], None)
                uv = np.array([kp.pt for kp in kps]) if kps else np.zeros((0, 2))
                P, okz = lift(uv, data[p]["depth"][k], data[p]["cam_T"][k], f, cx, cy) if len(uv) else (np.zeros((0, 3)), np.zeros(0, bool))
                d_c = dist_to_segments(P[:, 0], P[:, 1], segs) if len(uv) else np.zeros(0)
                edge = okz & (np.abs(d_c - half) < EDGE_NEAR)
                off = okz & (d_c > OFF_RUT)
                out.append({"uv": uv, "des": des, "P": P, "okz": okz, "edge": edge, "off": off})
            feats[p] = out
        res[light] = {}
        for p in passes:
            fr = feats[p]
            row = {"features_mean": float(np.mean([len(x["uv"]) for x in fr])),
                   "near_rut_edge_share": float(np.sum([x["edge"].sum() for x in fr]) / max(1, np.sum([x["okz"].sum() for x in fr])))}
            if p != "A":
                surv, surv_off, n_vis, n_ok, n_raw = [], [], 0, 0, 0
                for k in range(len(fr)):
                    A, B = feats["A"][k], fr[k]
                    if A["des"] is None or B["des"] is None or len(A["uv"]) == 0 or len(B["uv"]) == 0:
                        surv.append(0.0)
                        surv_off.append(0.0)
                        continue
                    pred, vis = project(A["P"], data[p]["cam_T"][k], f, cx, cy, data[p]["gray"][k].shape)
                    vis &= A["okz"]
                    good, good_off = 0, 0
                    for m in bf.knnMatch(A["des"], B["des"], k=2):
                        if len(m) < 2 or m[0].distance >= RATIO * m[1].distance:
                            continue
                        i, j = m[0].queryIdx, m[0].trainIdx
                        n_raw += 1
                        if vis[i] and np.hypot(*(pred[i] - B["uv"][j])) <= REPROJ_PX:
                            good += 1
                            good_off += bool(A["off"][i])
                    surv.append(good / max(1, vis.sum()))
                    surv_off.append(good_off / max(1, (vis & A["off"]).sum()))
                    n_vis += int(vis.sum())
                    n_ok += good
                row.update({"survival_mean": float(np.mean(surv)), "survival_min": float(np.min(surv)),
                            "survival_off_rut_mean": float(np.mean(surv_off)),
                            "survival_per_frame": [round(s, 4) for s in surv], "ratio_matches": n_raw,
                            "correct_matches": n_ok, "visible_A_features": n_vis})
            res[light][p] = row
    out_dir = a.out or a.dir
    with open(os.path.join(out_dir, "landmarks.json"), "w") as fh:
        json.dump({"settings": {"orb_nfeatures": 2000, "fast_threshold": FAST_T, "ratio": RATIO, "reproj_px": REPROJ_PX,
                                "edge_near_m": EDGE_NEAR, "intrinsics": [f, cx, cy]}, "meta": meta, "results": res},
                  fh, indent=1)
    L = [f"# Landmarks before and after wheel ruts (Isaac, seed {meta['seed']}, trip {meta['trip_seed']}, "
         f"{len(poses)} frames, D455-like IR camera {meta['image'][0]}x{meta['image'][1]})", "",
         f"ORB (2000 max, FAST threshold {FAST_T}); a relocalization match is correct when it reprojects within "
         f"{REPROJ_PX} px using the known poses. Survival: share of the map's (pass A) visible features still matched "
         "correctly in pass B at the same frame. Near rut edge: feature's ground point within 3 cm of a trench edge.", "",
         f"Off-rut survival: the same for map features more than {OFF_RUT} m from every wheel track.", "",
         "| light | pass | features per image | near a rut edge | survival vs A (mean / worst frame) | off-rut survival |",
         "| --- | --- | --- | --- | --- | --- |"]
    names = {"A": "A: no ruts (the map)", "Arep": "control: A rendered again", "Acam32": "control: no ruts, B32's camera poses", "B32": "B: 3.2 cm ruts, robot in them", "B32cam": "B: 3.2 cm ruts, A's camera poses",
             "B50": "B: 5 cm ruts, robot in them"}
    for light in lights:
        for p in passes:
            r = res[light][p]
            s = "-" if p == "A" else f"{100 * r['survival_mean']:.0f} % / {100 * r['survival_min']:.0f} %"
            so = "-" if p == "A" else f"{100 * r['survival_off_rut_mean']:.0f} %"
            L.append(f"| {light} | {names[p]} | {r['features_mean']:.0f} | {100 * r['near_rut_edge_share']:.1f} % | {s} | {so} |")
    md = "\n".join(L) + "\n"
    open(os.path.join(out_dir, "landmarks.md"), "w").write(md)
    print(md)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, len(lights), figsize=(6 * len(lights), 3.6), sharey=True)
        axs = np.atleast_1d(axs)
        for ax, light in zip(axs, lights):
            for p in passes[1:]:
                ax.plot(100 * np.array(res[light][p]["survival_per_frame"]), label=names[p])
            ax.set_title(f"{light} light")
            ax.set_xlabel("frame")
            ax.grid(alpha=0.3)
        axs[0].set_ylabel("A features still matched (%)")
        axs[0].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "landmarks.png"), dpi=110)
    except Exception as e:      # noqa: BLE001
        print("plot skipped:", e)


if __name__ == "__main__":
    main()
