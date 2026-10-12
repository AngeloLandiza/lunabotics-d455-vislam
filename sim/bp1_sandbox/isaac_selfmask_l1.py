"""Robot self-occlusion for the team's Unitree L1 in Isaac Sim, at its real (upside-down) mount.

Usage (Isaac Sim's Python; run_isaac_selfmask.sh passes the environment): isaac_selfmask_l1.py [--headless]
The L1 covers the hemisphere about its own z axis (360 x 90 deg). Isaac fires that hemisphere as a ROTARY pattern:
91 emitters at elevations 0, 1, ..., 90 deg and 360 azimuth ticks of 1 deg (32760 rays per frame), about the L1's z
axis at its mount. Each returned ray's direction comes from Isaac's own azimuth/elevation output, so the body can be
ray cast on exactly the rays Isaac fired. (SOLID_STATE mode with one emitter per L1 ray was tried first, 2026-10-12:
Isaac fired the requested directions, with azimuth reported negated, but flagged every return invalid, even 2-6 m
off a flat plane; ROTARY with a single tick per scan returned no points. Both are left out.) Other settings as in isaac_capture.py: idealized rays, noise off, nearRangeM and minDistBetweenEchosM
0.01 m, instantLidar, several scans merged. The robot is robot.py's collision mesh without the L1 housing
(lidar3d_link). Poses as isaac_selfmask.py: flat plane at (3.0, 2.5, 0), seed-1 arena at (1.0, 2.5, 0); each captured
with and without the robot prim. Output isaac_out/selfmask_l1.npz/.json: d_s (sensor-frame rays), p{k}_r_with,
p{k}_r_without (nan = no return), d_s and the ray index are the same for every capture: index = tick * 91 + emitter,
d_s from Isaac's reported azimuth/elevation; p{k}_origin, p{k}_R (world-from-sensor), poses, mounts.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--usd", default=os.path.join(HERE, "usd", "arena_seed1.usda"))
parser.add_argument("--out", default=os.path.join(HERE, "isaac_out", "selfmask_l1.npz"))
parser.add_argument("--scans", type=int, default=4)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gui-hold", type=float, default=1.5)
parser.add_argument("--debug-scan", default=None, help="save one raw scan (emitterId, azimuth, elevation, range) here")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt  # noqa: E402

sys.path.insert(0, HERE)
import params as pr  # noqa: E402
from robot import Robot  # noqa: E402
from sensors import sensor_pose_R  # noqa: E402

POSES = [((3.0, 2.5, 0.0), "flat"), ((1.0, 2.5, 0.0), "arena_seed1")]
PROF = pr.LIDAR_PROFILES["l1"]
EL_GRID = np.arange(0.0, 90.0 + 1e-9, 1.0)          # emitter elevations, deg
N_EM, N_TICK = len(EL_GRID), 360
N = N_EM * N_TICK
D_S = None                                          # filled from Isaac's reported directions


def quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method), as in isaac_capture.py."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(1 + t)
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q if q[0] >= 0 else -q


ok, stage = stage_utils.open_stage(args.usd)
assert ok, args.usd
terrain = stage.GetPrimAtPath("/Arena/Terrain")
plane = UsdGeom.Mesh.Define(stage, "/World/bp1_flat_plane")
plane.CreatePointsAttr([Gf.Vec3f(-20, -20, 0), Gf.Vec3f(20, -20, 0), Gf.Vec3f(20, 20, 0), Gf.Vec3f(-20, 20, 0)])
plane.CreateFaceVertexCountsAttr([3, 3])
plane.CreateFaceVertexIndicesAttr([0, 1, 2, 0, 2, 3])
plane.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
app_utils.update_app(steps=10)

robot = Robot()
TRIS, _ = robot.triangles(exclude=("lidar3d_link",))
MOUNT_XYZ, MOUNT_R = robot.mount("lidar3d_link")
print(f"[selfmask_l1] {N} rays (hemisphere grid), mount {MOUNT_XYZ}, sensor z axis {np.round(MOUNT_R[:, 2], 3)}, "
      f"robot {len(TRIS)} triangles", flush=True)
ROBOT_PATH = "/World/bp1_robot"


def add_robot(pose):
    x, y, yaw = pose
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    P = TRIS.reshape(-1, 3) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]).T + [x, y, 0.0]
    m = UsdGeom.Mesh.Define(stage, ROBOT_PATH)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRIS), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(P), dtype=np.int32)))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.9)])


# ROTARY hemisphere grid (the rotary mode is the one validated against the sandbox in isaac_capture.py)
DEBUG = {} if args.debug_scan else None
SCAN_HZ = 10
ATTRS = {
    "omni:sensor:Core:scanType": "ROTARY",
    "omni:sensor:Core:numberOfEmitters": N_EM, "omni:sensor:Core:numberOfChannels": N_EM,
    "omni:sensor:Core:scanRateBaseHz": SCAN_HZ, "omni:sensor:Core:patternFiringRateHz": SCAN_HZ * N_TICK,
    "omni:sensor:Core:farRangeM": float(PROF["rmax"]), "omni:sensor:Core:nearRangeM": 0.01,
    "omni:sensor:Core:minDistBetweenEchosM": 0.01, "omni:sensor:Core:maxReturns": 1,
    "omni:sensor:Core:instantLidar": True, "omni:sensor:Core:skipDroppingInvalidPoints": True,
    "omni:sensor:Core:rayType": "IDEALIZED", "omni:sensor:Core:rangeAccuracyM": 0.0,
    "omni:sensor:Core:azimuthErrorMean": 0.0, "omni:sensor:Core:azimuthErrorStd": 0.0,
    "omni:sensor:Core:elevationErrorMean": 0.0, "omni:sensor:Core:elevationErrorStd": 0.0,
    "omni:sensor:Core:originErrorMean": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:originErrorStd": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:rangeResolutionM": 0.001,
    "omni:sensor:Core:emitterState:s001:azimuthDeg": Vt.FloatArray([0.0] * N_EM),
    "omni:sensor:Core:emitterState:s001:elevationDeg": Vt.FloatArray(EL_GRID.tolist()),
    "omni:sensor:Core:emitterState:s001:fireTimeNs": Vt.UIntArray([0] * N_EM),
    "omni:sensor:Core:emitterState:s001:channelId": Vt.UIntArray(list(range(1, N_EM + 1))),
}
lidar = Lidar("/World/bp1_l1", attributes=ATTRS, tick_rate=SCAN_HZ, aux_output_level="FULL")
lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])


def collect(origin, q, n_scans, skip=3, max_updates=600):
    """Merged ranges per emitter over n_scans scans at the pose, after skipping `skip` scans (scene changed)."""
    r_all, last_ts, seen, n_el = [], None, 0, set()
    for _ in range(max_updates):
        app_utils.update_app()
        data, _ = lidar_sensor.get_data("generic-model-output")
        if data is None:
            continue
        g = parse_generic_model_output_data(data)
        if g.numElements == 0 or g.timestampNs == last_ts:
            continue
        last_ts = g.timestampNs
        qs = np.array(g.frameEnd.orientation)[[3, 0, 1, 2]]
        if not (np.allclose(g.frameEnd.posM, origin, atol=1e-4) and abs(np.dot(qs, q)) > 1 - 1e-6):
            continue
        seen += 1
        if seen <= skip:
            continue
        global D_S
        em = np.asarray(g.emitterId).astype(int)
        tk = np.asarray(g.tickId).astype(int)
        idx = tk * N_EM + em
        valid = (np.asarray(g.flags) & 64) > 0
        rng_m = np.asarray(g.z, float)
        r = np.full(N, np.nan)
        r[idx[valid]] = rng_m[valid]
        az, el = np.radians(np.asarray(g.x, float)), np.radians(np.asarray(g.y, float))
        d = np.full((N, 3), np.nan)
        d[idx] = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
        if D_S is None:
            D_S = d
        daz = np.nan_to_num(np.abs(np.degrees(np.arccos(np.clip((d * D_S).sum(1), -1, 1)))))   # same rays every scan
        del_ = np.abs(np.degrees(el) - EL_GRID[em])
        n_el.add(int(g.numElements))
        r_all.append(r)
        if DEBUG is not None and not DEBUG:
            DEBUG.update(em=em, az=np.asarray(g.x, float), el=np.asarray(g.y, float), r=rng_m, valid=valid,
                         ox=np.asarray(g.originX, float) if len(np.asarray(g.originX)) else np.zeros(0),
                         frame=str(g.frameOfReference), coords=str(g.elementsCoordsType))
        if len(r_all) == n_scans:
            R = np.stack(r_all)
            hit = np.isfinite(R).any(0)
            spread = np.nanmax(R[:, hit], 0) - np.nanmin(R[:, hit], 0) if hit.any() else np.zeros(1)
            merged = np.where(hit, np.nanmax(np.where(np.isfinite(R), R, -np.inf), 0), np.nan)
            return merged, {"elements_per_scan": sorted(n_el), "max_dir_change_deg": float(daz.max()),
                            "max_el_err_deg": float(del_.max()), "scan_range_spread_m": float(spread.max())}
    raise RuntimeError("not enough L1 scans")


if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    app_utils.enable_extension("isaacsim.util.debug_draw")
    from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
    UsdLux.DomeLight.Define(stage, "/World/bp1_view_dome").CreateIntensityAttr(400.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/bp1_view_sun")
    sun.CreateIntensityAttr(2500.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(40.0, 0.0, 30.0))
    chase = UsdGeom.Camera.Define(stage, "/World/bp1_chase_cam")
    chase.CreateFocalLengthAttr(14.0)
    chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    cxf = UsdGeom.Xformable(chase.GetPrim())
    chase_t = cxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    chase_q = cxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    get_active_viewport().camera_path = chase.GetPath().pathString


def show(pose, origin, dirs_w, r, label):
    if args.headless:
        return
    x, y, yaw = pose
    hv = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), 0.0])
    side = np.array([-hv[1], hv[0], 0.0])
    eye = np.array([x, y, 0.0]) + 1.8 * hv + 1.4 * side + np.array([0, 0, 1.6])   # front-left, looking back
    f = np.array([x, y, 0.4]) - eye
    f /= np.linalg.norm(f)
    rr = np.cross(f, [0, 0, 1.0])
    rr /= np.linalg.norm(rr)
    chase_t.Set(Gf.Vec3d(*eye.tolist()))
    chase_q.Set(Gf.Quatd(*quat_wxyz(np.stack([rr, np.cross(rr, f), -f], 1)).tolist()))
    draw.clear_points()
    ok = np.isfinite(r)
    pts = origin + r[ok, None] * dirs_w[ok]
    near = r[ok] < 1.2
    draw.draw_points([tuple(map(float, p)) for p in pts],
                     [(1.0, 0.25, 0.2, 1.0) if n else (0.2, 1.0, 0.4, 1.0) for n in near], [5.0] * len(pts))
    print(f"[selfmask_l1] showing {label}", flush=True)
    t_end = time.time() + args.gui_hold
    while time.time() < t_end:
        app.update()


out = {"poses": np.array([p for p, _ in POSES]), "scenes": np.array([s for _, s in POSES])}
mounts = {"lidar3d": {"xyz": list(MOUNT_XYZ), "R_robot_from_sensor": MOUNT_R.tolist()},
          "base_height": robot.base_height, "excluded_links": ["lidar3d_link"], "robot_digest": robot.digest(),
          "n_triangles": int(len(TRIS)), "pattern": "ROTARY hemisphere grid: 91 elevations 0-90 deg x 360 azimuth ticks"}
app_utils.play()
app_utils.update_app(steps=5)
stats = []
for k, (pose, scene) in enumerate(POSES):
    terrain.SetActive(scene != "flat")
    plane.GetPrim().SetActive(scene == "flat")
    lo, lR = sensor_pose_R(pose, MOUNT_XYZ, MOUNT_R)
    lq = quat_wxyz(lR)
    lidar.set_world_poses(positions=lo.reshape(1, 3), orientations=lq.reshape(1, 4))
    out[f"p{k}_origin"], out[f"p{k}_R"] = lo, lR
    for case in ("with", "without"):
        if case == "with":
            add_robot(pose)
        else:
            stage.RemovePrim(Sdf.Path(ROBOT_PATH))
        app_utils.update_app(steps=6)
        r, info = collect(lo, lq, args.scans)
        out[f"p{k}_r_{case}"] = r
        stats.append({"pose": list(pose), "scene": scene, "case": case, "returns": int(np.isfinite(r).sum()), **info})
        print(f"[selfmask_l1] pose {k} {scene} {case} robot: {np.isfinite(r).sum()}/{N} returns, "
              f"{int(np.sum(r < 1.2))} under 1.2 m, {info}", flush=True)
        show(pose, lo, np.nan_to_num(D_S) @ lR.T, r, f"{scene} {case} robot")

if DEBUG:
    np.savez(args.debug_scan, **{k_: np.asarray(v_) for k_, v_ in DEBUG.items()})
out["d_s"] = D_S
os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
out["mounts"] = np.array(json.dumps(mounts))
np.savez_compressed(args.out, **out)
with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
    json.dump({"usd": os.path.relpath(args.usd, HERE), "mounts": mounts, "n_rays": N, "scans_per_capture": args.scans,
               "captures": stats}, f, indent=1, default=float)
print(f"[selfmask_l1] wrote {args.out}", flush=True)
app.close()
