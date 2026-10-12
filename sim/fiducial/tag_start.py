"""Start-zone tag test: robot pose from the rear camera with one tag, two tags on one wall, or two perpendicular tags.

The 2027 rules let a team put its own tags on the bin frame along two sides of the starting zone (2027 guidebook
13.7, Verified); the zone sits in a corner of the arena (11.2, Verified), so the two sides are likely perpendicular
(Inferred). No source we opened measured what a perpendicular pair gains over one tag or a coplanar pair
(research/step1/11-fiducial-gaps.md). This script measures it in simulation.

Each trial draws a random robot pose in the start zone (random heading, 2027 13.9.3), projects the tag corners
into the rear camera (team URDF mount and image size), adds corner noise, keeps tags a detector would decode,
estimates the camera pose with OpenCV (IPPE for planar sets, SQPnP + LM for non-coplanar sets) and scores the
robot pose against the truth. The robot may rotate in place to localize (2027 16.5), so the script also reports
how far it must turn before a usable set of tags is in view.

Every number below is labelled Verified, Inferred or Assumed. Run:
    python3 tag_start.py results/v1 --trials 3000
"""
import argparse
import json
import math
import os

import cv2
import numpy as np


def P(value, unit, status, source):
    return {"value": value, "unit": unit, "status": status, "source": source}


PARAMS = {
    "start_zone": P((0.0, 2.0, 0.0, 2.0), "m (x0,x1,y0,y1)", "Assumed",
                    "zone size is in the unreadable Figure 9; same as the BP-1 sandbox (bp1_sandbox/params.py)"),
    "walls": P("x = 0 and y = 0", "", "Inferred",
               "the zone is in the arena corner (11.2), so its two bin-frame sides are the walls meeting there"),
    "tag_size": P(0.16, "m", "Assumed", "black-square edge of a tag36h11 tag; team's tag family and size are open"),
    "tag_height": P(0.30, "m", "Assumed", "tag centre above the regolith on the bin frame"),
    "cam_mount": P((-0.42, 0.0, 0.37, 15.0), "m, m, m, deg up", "Verified",
                   "rear_cam_link in sim/robot/uic_bot_2027.xacro (facing back, pitched 15 deg up)"),
    "base_height": P(0.2651, "m", "Verified", "base_link height above ground (team URDF)"),
    "image": P((1280, 720), "px", "Verified", "rear_cam sensor in the xacro"),
    "hfov": P(1.2, "rad", "Verified", "rear_cam horizontal_fov in the xacro"),
    "corner_noise": P(0.5, "px", "Assumed", "1-sigma corner error after sub-pixel refinement; not measured on BP-1 light"),
    "min_side_px": P(20.0, "px", "Assumed", "smallest tag edge in pixels a tag36h11 detector decodes reliably"),
    "max_view_angle": P(70.0, "deg", "Assumed", "largest angle between the tag normal and the line of sight that decodes"),
    "placement_err": P((0.005, 1.0), "m, deg", "Assumed", "how well a taped or clamped tag's pose is known after setup"),
    "body_radius": P(0.66, "m", "Verified", "circumscribed radius of the 1.0 x 0.7 m body about base_link"),
}


def v(name):
    return PARAMS[name]["value"]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


# camera optical frame (z forward, x right, y down) relative to base_link, from the xacro joints
MX, MY, MZ, PITCH_UP = v("cam_mount")
R_BASE_CAMLINK = rot_z(math.pi) @ rot_y(-math.radians(PITCH_UP))      # rpy (0, -0.2618, pi): yaw pi, then pitch
R_CAMLINK_OPT = rot_z(-math.pi / 2) @ rot_x(-math.pi / 2)              # rpy (-pi/2, 0, -pi/2)
R_BASE_OPT = R_BASE_CAMLINK @ R_CAMLINK_OPT
T_BASE_OPT = np.array([MX, MY, MZ])



def set_pitch(deg):
    """Change the rear camera pitch (deg up) for a what-if run; the xacro value is 15."""
    global R_BASE_OPT
    PARAMS["cam_mount"] = P((MX, MY, MZ, deg), "m, m, m, deg up", "Assumed", f"what-if pitch {deg} deg (xacro: 15)")
    R_BASE_OPT = rot_z(math.pi) @ rot_y(-math.radians(deg)) @ R_CAMLINK_OPT


W_PX, H_PX = v("image")
FX = (W_PX / 2) / math.tan(v("hfov") / 2)
K = np.array([[FX, 0, W_PX / 2], [0, FX, H_PX / 2], [0, 0, 1.0]])


def set_hfov(rad):
    """What-if horizontal field of view (pinhole, no distortion: Assumed for wide lenses)."""
    global FX, K
    PARAMS["hfov"] = P(rad, "rad", "Assumed", f"what-if field of view (xacro: 1.2); pinhole, lens distortion ignored")
    FX = (W_PX / 2) / math.tan(rad / 2)
    K = np.array([[FX, 0, W_PX / 2], [0, FX, H_PX / 2], [0, 0, 1.0]])


def tag(x, y, normal_yaw):
    """A tag on a wall: centre (x, y, tag_height) facing into the arena along normal_yaw. Returns 4x3 corners in
    OpenCV IPPE_SQUARE order (top-left, top-right, bottom-right, bottom-left, as seen from the front)."""
    s = v("tag_size") / 2
    n = np.array([math.cos(normal_yaw), math.sin(normal_yaw), 0.0])
    right = np.array([-math.sin(normal_yaw), math.cos(normal_yaw), 0.0])       # viewer's right when facing the tag
    up = np.array([0, 0, 1.0])
    c = np.array([x, y, v("tag_height")])
    return np.array([c - right * s + up * s, c + right * s + up * s, c + right * s - up * s, c - right * s - up * s]), n


def layouts():
    """Tag sets on the start-zone walls (wall y = 0 faces +y, wall x = 0 faces +x). Positions Assumed."""
    return {
        "one": [tag(1.0, 0.0, math.pi / 2)],
        "two_coplanar": [tag(0.6, 0.0, math.pi / 2), tag(1.4, 0.0, math.pi / 2)],
        "two_perpendicular": [tag(1.0, 0.0, math.pi / 2), tag(0.0, 1.0, 0.0)],
        "four": [tag(0.6, 0.0, math.pi / 2), tag(1.4, 0.0, math.pi / 2), tag(0.0, 0.6, 0.0), tag(0.0, 1.4, 0.0)],
    }


def cam_pose(robot):
    """World pose (R_wc, t_wc) of the optical frame for robot (x, y, yaw)."""
    x, y, yaw = robot
    R_wb = rot_z(yaw)
    t_wb = np.array([x, y, v("base_height")])
    return R_wb @ R_BASE_OPT, t_wb + R_wb @ T_BASE_OPT


def visible(corners, normal, R_wc, t_wc, rng):
    """Pixel corners if a detector would decode the tag, else None."""
    pc = (corners - t_wc) @ R_wc            # world -> camera: R_wc^T (p - t)
    if np.any(pc[:, 2] < 0.1):
        return None
    centre = corners.mean(0)
    to_cam = t_wc - centre
    view = math.degrees(math.acos(np.clip(normal @ to_cam / np.linalg.norm(to_cam), -1, 1)))
    if view > v("max_view_angle"):
        return None
    uv = (pc @ K.T)[:, :2] / pc[:, 2:3]
    if np.any(uv < 0) or np.any(uv[:, 0] >= W_PX) or np.any(uv[:, 1] >= H_PX):
        return None
    side = min(np.linalg.norm(uv[i] - uv[(i + 1) % 4]) for i in range(4))
    if side < v("min_side_px"):
        return None
    return uv + rng.normal(0, v("corner_noise"), uv.shape)


def solve(obj, img, coplanar):
    """Camera pose from 3D-2D points: candidate poses, keep the lowest reprojection error (what a robot would do).
    Returns (R_cw, t_cw) mapping world points into the camera."""
    obj = np.ascontiguousarray(obj, np.float64)
    img = np.ascontiguousarray(img, np.float64)
    if coplanar:
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, None, flags=cv2.SOLVEPNP_IPPE)
        if n:
            k = int(np.argmin(np.ravel(errs)))
            rvec, tvec = rvecs[k], tvecs[k]
            R, _ = cv2.Rodrigues(rvec)
            return R, tvec.ravel()
        # IPPE refuses points that are not exactly coplanar (two separately placed tags); fall through
    if True:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_SQPNP)
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel()


def robot_from_cam(R_cw, t_cw):
    R_wc = R_cw.T
    t_wc = -R_wc @ t_cw
    R_wb = R_wc @ R_BASE_OPT.T
    t_wb = t_wc - R_wb @ T_BASE_OPT
    return t_wb[0], t_wb[1], math.atan2(R_wb[1, 0], R_wb[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def perturb(tags, rng):
    """Tag map as the robot believes it: each tag off by the placement error (rotation about its centre)."""
    d, a = v("placement_err")
    out = []
    for corners, n in tags:
        c = corners.mean(0)
        R = rot_z(math.radians(rng.normal(0, a))) @ rot_x(math.radians(rng.normal(0, a)))
        out.append(((corners - c) @ R.T + c + rng.normal(0, d, 3), n))
    return out


def trial(robot, tags, rng):
    R_wc, t_wc = cam_pose(robot)
    seen = [(k, uv) for k, (c, n) in enumerate(tags) if (uv := visible(c, n, R_wc, t_wc, rng)) is not None]
    if not seen:
        return None
    believed = perturb(tags, rng)
    obj = np.vstack([believed[k][0] for k, _ in seen])
    img = np.vstack([uv for _, uv in seen])
    normals = {tuple(np.round(tags[k][1], 6)) for k, _ in seen}
    R_cw, t_cw = solve(obj, img, coplanar=len(normals) == 1)
    x, y, yaw = robot_from_cam(R_cw, t_cw)
    return {"n_tags": len(seen), "planes": len(normals), "e_xy": float(math.hypot(x - robot[0], y - robot[1])),
            "e_yaw": float(abs(math.degrees(wrap(yaw - robot[2]))))}


def turn_to_see(robot, tags, rng, step=5.0):
    """Smallest in-place turn (either way, deg) after which at least one tag decodes; None if no heading works."""
    for d in np.arange(0, 180 + step, step):
        for sgn in (1, -1):
            r = (robot[0], robot[1], robot[2] + sgn * math.radians(d))
            R_wc, t_wc = cam_pose(r)
            if any(visible(c, n, R_wc, t_wc, rng) is not None for c, n in tags):
                return float(d)
    return None


def boot_ci(x, f, rng, n=2000):
    x = np.asarray(x)
    if len(x) == 0:
        return [None, None]
    s = [f(x[rng.integers(0, len(x), len(x))]) for _ in range(n)]
    return [float(np.percentile(s, 2.5)), float(np.percentile(s, 97.5))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--trials", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag-size", type=float, default=None)
    ap.add_argument("--tag-height", type=float, default=None)
    ap.add_argument("--cam-pitch", type=float, default=None, help="rear camera pitch, deg up (xacro: 15)")
    ap.add_argument("--hfov", type=float, default=None, help="rear camera horizontal field of view, rad (xacro: 1.2)")
    a = ap.parse_args()
    if a.tag_size:
        PARAMS["tag_size"]["value"] = a.tag_size
    if a.tag_height:
        PARAMS["tag_height"]["value"] = a.tag_height
    if a.cam_pitch is not None:
        set_pitch(a.cam_pitch)
    if a.hfov:
        set_hfov(a.hfov)
    rng = np.random.default_rng(a.seed)
    x0, x1, y0, y1 = v("start_zone")
    r = v("body_radius")
    poses = [(rng.uniform(x0 + r, x1 - r), rng.uniform(y0 + r, y1 - r), rng.uniform(-math.pi, math.pi))
             for _ in range(a.trials)]
    out = {"params": PARAMS, "trials": a.trials, "seed": a.seed, "layouts": {}}
    per_pose = {}
    for name, tags in layouts().items():
        trng = np.random.default_rng([a.seed, 1])        # same noise stream for every layout (paired comparison)
        res = [trial(p, tags, trng) for p in poses]
        turns = [turn_to_see(p, tags, trng) for p in poses]
        got = [x for x in res if x]
        exy = [x["e_xy"] for x in got]
        eyaw = [x["e_yaw"] for x in got]
        flip = [x["e_yaw"] > 5.0 for x in got]
        brng = np.random.default_rng([a.seed, 2])
        out["layouts"][name] = {
            "seen_frac": len(got) / len(poses),
            "e_xy_median": float(np.median(exy)) if exy else None,
            "e_xy_median_ci": boot_ci(exy, np.median, brng),
            "e_xy_p95": float(np.percentile(exy, 95)) if exy else None,
            "e_xy_p95_ci": boot_ci(exy, lambda s: np.percentile(s, 95), brng),
            "e_yaw_median": float(np.median(eyaw)) if eyaw else None,
            "e_yaw_p95": float(np.percentile(eyaw, 95)) if eyaw else None,
            "bad_yaw_frac": float(np.mean(flip)) if flip else None,
            "bad_yaw_frac_ci": boot_ci(np.array(flip, float), np.mean, brng),
            "two_planes_frac": float(np.mean([x["planes"] == 2 for x in got])) if got else None,
            "turn_median_deg": float(np.median([t for t in turns if t is not None])) if any(t is not None for t in turns) else None,
            "turn_p95_deg": float(np.percentile([t for t in turns if t is not None], 95)) if any(t is not None for t in turns) else None,
            "never_frac": float(np.mean([t is None for t in turns])),
        }
        per_pose[name] = res
        L = out["layouts"][name]
        print(f"{name:18s} seen {L['seen_frac']:.2f}  xy median {fmt(L['e_xy_median'], 3)} m, p95 {fmt(L['e_xy_p95'], 3)} m"
              f"  yaw median {fmt(L['e_yaw_median'], 2)} deg, >5 deg {fmt(L['bad_yaw_frac'], 3)}"
              f"  turn median {fmt(L['turn_median_deg'], 0)} deg, never {L['never_frac']:.2f}"
              f"  both walls {fmt(L['two_planes_frac'], 2)}", flush=True)
    # paired: poses where both layouts see tags
    out["paired"] = {}
    for A, B in [("one", "two_perpendicular"), ("two_coplanar", "two_perpendicular"), ("two_perpendicular", "four")]:
        pairs = [(p, q) for p, q in zip(per_pose[A], per_pose[B]) if p and q]
        if not pairs:
            continue
        d = np.array([q["e_xy"] - p["e_xy"] for p, q in pairs])
        out["paired"][f"{B}_minus_{A}"] = {"n": len(pairs), "e_xy_diff_median": float(np.median(d)),
                                           "e_xy_diff_median_ci": boot_ci(d, np.median, np.random.default_rng([a.seed, 3])),
                                           "B_better_frac": float(np.mean(d < 0))}
    os.makedirs(a.out, exist_ok=True)
    json.dump(out, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    print(json.dumps(out["paired"], indent=1))


def fmt(x, n):
    return "-" if x is None else f"{x:.{n}f}"


if __name__ == "__main__":
    main()
