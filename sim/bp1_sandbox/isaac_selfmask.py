"""Robot self-occlusion in Isaac Sim: RTX LiDAR (spin16) and D455 depth at the team robot's mounts, each pose
captured with the robot body present and with its prim removed.

Usage (Isaac Sim's Python; run_isaac_selfmask.sh sets the environment):
  isaac_selfmask.py [--out isaac_out/selfmask.npz] [--headless]
Poses: (3.0, 2.5, 0) on a flat plane at z = 0 (the arena terrain deactivated) and (1.0, 2.5, 0) in the seed-1 arena.
Robot: robot.py's collision triangles (lidar3d and d455 housings excluded) as one UsdGeom.Mesh in the robot frame, placed with
base_link at robot.base_height over the pose; sensors with sensors.sensor_pose(pose, mount, pitch) and robot.py
mounts. LiDAR and depth settings are those of isaac_capture.py (spin16 pattern, noise off, nearRangeM 0.01, 4 merged
scans; depth rendered 5x with square pixels and resampled to the sandbox 212 x 120 grid); see that file for sources.
After every scene change (robot added or removed, terrain switched) the first scans are discarded so RTX has rebuilt
its acceleration structure before anything is kept.
Output keys: poses, scenes, mounts (json), p{k}_lidar_r_with / _without, p{k}_lidar_dirs_isaac (sandbox
spinning_dirs order), p{k}_lidar_normals_with (raw hitNormals), p{k}_depth_with / _without (212 x 120, row 0 at top).
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--usd", default=os.path.join(HERE, "usd", "arena_seed1.usda"))
parser.add_argument("--out", default=os.path.join(HERE, "isaac_out", "selfmask.npz"))
parser.add_argument("--scans", type=int, default=4)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gui-hold", type=float, default=1.5)
parser.add_argument("--min-echo-dist", type=float, default=0.01,
                    help="omni:sensor:Core:minDistBetweenEchosM (schema default 0.4 m drops every return closer than "
                         "0.40 m, measured 2026-10-12)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt  # noqa: E402

sys.path.insert(0, HERE)
import params as pr  # noqa: E402
from params import v  # noqa: E402
from robot import Robot  # noqa: E402
from sensors import sensor_pose  # noqa: E402

PROF = pr.LIDAR_PROFILES["spin16"]
N_CH, N_AZ, SCAN_HZ = PROF["channels"], int(round(360.0 / PROF["hres"])), 10
EL = np.linspace(PROF["vmin"], PROF["vmax"], N_CH)
POSES = [((3.0, 2.5, 0.0), "flat"), ((1.0, 2.5, 0.0), "arena_seed1")]


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


# ---------------------------------------------------------------- stage: arena terrain plus a flat plane
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
# Housings of the two sensors under test are left out: both sensor origins sit inside their own housing meshes
# (with d455_link included every D455 pixel hit the housing at 0.6-28 mm in the first run, 2026-10-12)
EXCLUDE = ("lidar3d_link", "d455_link")
TRIS, _ = robot.triangles(exclude=EXCLUDE)
lidar_mount, lidar_pitch = robot.mount_pitch("lidar3d_link")   # spin16 check: forward mount only
cam_mount, cam_pitch = robot.mount_pitch("d455_link")
print(f"[selfmask] robot {len(TRIS)} triangles, lidar3d {lidar_mount} pitch {lidar_pitch:.1f}, "
      f"d455 {cam_mount} pitch {cam_pitch:.1f}", flush=True)
ROBOT_PATH = "/World/bp1_robot"


def add_robot(pose):
    """Robot body as one mesh, its points already in the arena frame for this pose."""
    x, y, yaw = pose
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    P = TRIS.reshape(-1, 3) @ Rz.T + [x, y, 0.0]
    m = UsdGeom.Mesh.Define(stage, ROBOT_PATH)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRIS), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(P), dtype=np.int32)))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateDoubleSidedAttr(True)
    m.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.9)])


def remove_robot():
    stage.RemovePrim(Sdf.Path(ROBOT_PATH))


# ---------------------------------------------------------------- sensors (as isaac_capture.py)
LIDAR_ATTRS = {
    "omni:sensor:Core:scanType": "ROTARY",
    "omni:sensor:Core:numberOfEmitters": N_CH, "omni:sensor:Core:numberOfChannels": N_CH,
    "omni:sensor:Core:scanRateBaseHz": SCAN_HZ, "omni:sensor:Core:patternFiringRateHz": SCAN_HZ * N_AZ,
    "omni:sensor:Core:farRangeM": 8.0, "omni:sensor:Core:nearRangeM": 0.01, "omni:sensor:Core:maxReturns": 1,
    "omni:sensor:Core:instantLidar": True, "omni:sensor:Core:skipDroppingInvalidPoints": True,
    "omni:sensor:Core:rayType": "IDEALIZED", "omni:sensor:Core:rangeAccuracyM": 0.0,
    "omni:sensor:Core:azimuthErrorMean": 0.0, "omni:sensor:Core:azimuthErrorStd": 0.0,
    "omni:sensor:Core:elevationErrorMean": 0.0, "omni:sensor:Core:elevationErrorStd": 0.0,
    "omni:sensor:Core:originErrorMean": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:originErrorStd": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:rangeResolutionM": 0.001,
    "omni:sensor:Core:minDistBetweenEchosM": args.min_echo_dist,
    "omni:sensor:Core:emitterState:s001:azimuthDeg": Vt.FloatArray([0.0] * N_CH),
    "omni:sensor:Core:emitterState:s001:elevationDeg": Vt.FloatArray(EL.tolist()),
    "omni:sensor:Core:emitterState:s001:fireTimeNs": Vt.UIntArray([0] * N_CH),
    "omni:sensor:Core:emitterState:s001:channelId": Vt.UIntArray(list(range(1, N_CH + 1))),
}
lidar = Lidar("/World/bp1_lidar", attributes=LIDAR_ATTRS, tick_rate=SCAN_HZ, aux_output_level="FULL")
lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])

CAM_W, CAM_H = v(pr.D455, "sim_size")
HF, VF = np.radians(v(pr.D455, "hfov")), np.radians(v(pr.D455, "vfov"))
K = 5
HI_W = CAM_W * K
PITCH = 2 * np.tan(HF / 2) / HI_W
HI_H = 2 * int(np.ceil(np.tan(VF / 2) / PITCH)) + 2
cam = UsdGeom.Camera.Define(stage, "/World/bp1_depth")
cam.CreateFocalLengthAttr(10.0)
cam.CreateHorizontalApertureAttr(float(20.0 * np.tan(HF / 2)))
cam.CreateVerticalApertureAttr(float(20.0 * np.tan(HF / 2) * HI_H / HI_W))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
cam_xf = UsdGeom.Xformable(cam.GetPrim())
cam_t = cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
cam_q = cam_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
depth_ann = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
depth_ann.attach(rep.create.render_product(cam.GetPath().pathString, (HI_W, HI_H)))
USD_CAM_IN_SANDBOX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def resample_depth(hi):
    cols = K * np.arange(CAM_W) + K // 2
    vs = ((np.arange(CAM_H) + 0.5) / CAM_H * 2 - 1) * np.tan(VF / 2)
    fr = vs / PITCH + HI_H / 2 - 0.5
    r0 = np.floor(fr).astype(int)
    a = (fr - r0)[:, None]
    z0, z1 = hi[r0][:, cols], hi[r0 + 1][:, cols]
    near = np.where(a < 0.5, z0, z1)
    smooth = np.isfinite(z0) & np.isfinite(z1) & (np.abs(z1 - z0) < 0.05)
    return np.where(smooth, z0 * (1 - a) + z1 * a, near)


def collect(origin, q, n_scans, skip=3, max_updates=600):
    """n_scans distinct complete scans at the pose, after discarding the first `skip` (scene just changed)."""
    scans, last_ts, seen = [], None, 0
    for _ in range(max_updates):
        app_utils.update_app()
        data, _ = lidar_sensor.get_data("generic-model-output")
        if data is None:
            continue
        g = parse_generic_model_output_data(data)
        if g.numElements != N_CH * N_AZ or g.timestampNs == last_ts:
            continue
        last_ts = g.timestampNs
        qs = np.array(g.frameEnd.orientation)[[3, 0, 1, 2]]
        if not (np.allclose(g.frameEnd.posM, origin, atol=1e-4) and abs(np.dot(qs, q)) > 1 - 1e-6):
            continue
        seen += 1
        if seen <= skip:
            continue
        scans.append({"az": np.asarray(g.x, float).copy(), "el": np.asarray(g.y, float).copy(),
                      "r": np.asarray(g.z, float).copy(), "valid": (np.asarray(g.flags) & 64) > 0,
                      "normals": np.asarray(g.hitNormals, float).reshape(-1, 3).copy()})
        if len(scans) == n_scans:
            return scans
    raise RuntimeError("not enough LiDAR scans")


def merge(scans, lR):
    r = np.full((len(scans), N_CH * N_AZ), np.nan)
    dirs = np.full((N_CH * N_AZ, 3), np.nan)
    nrm = np.full((N_CH * N_AZ, 3), np.nan)
    for si, s in enumerate(scans):
        ch = np.rint((s["el"] - PROF["vmin"]) / ((PROF["vmax"] - PROF["vmin"]) / (N_CH - 1))).astype(int)
        idx = ch * N_AZ + np.rint((s["az"] + 180.0) / PROF["hres"]).astype(int) % N_AZ
        r[si, idx] = np.where(s["valid"], s["r"], np.nan)
        el, az = np.radians(s["el"]), np.radians(s["az"])
        dirs[idx] = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1) @ lR.T
        if len(s["normals"]) == len(idx):
            take = s["valid"] & ~np.isfinite(nrm[idx, 0])
            nrm[idx[take]] = s["normals"][take]
    hit = np.isfinite(r).any(0)
    spread = np.nanmax(r[:, hit], 0) - np.nanmin(r[:, hit], 0)
    merged = np.where(hit, np.nanmax(np.where(np.isfinite(r), r, -np.inf), 0), np.nan)
    return merged, dirs, nrm, float(spread.max()) if spread.size else 0.0


def depth_now():
    app_utils.update_app(steps=4)
    hi = np.asarray(depth_ann.get_data(), dtype=np.float64)
    return resample_depth(np.where(np.isfinite(hi) & (hi > 0), hi, np.nan))


# ---------------------------------------------------------------- GUI (view only)
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
    chase_xf = UsdGeom.Xformable(chase.GetPrim())
    chase_t = chase_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    chase_q = chase_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    get_active_viewport().camera_path = chase.GetPath().pathString


def show(pose, origin, dirs, r, label):
    if args.headless:
        return
    x, y, yaw = pose
    hv = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), 0.0])
    side = np.array([-hv[1], hv[0], 0.0])
    eye = np.array([x, y, 0.0]) - 1.6 * hv + 1.2 * side + np.array([0, 0, 1.4])
    f = np.array([x, y, 0.3]) - eye
    f /= np.linalg.norm(f)
    rr = np.cross(f, [0, 0, 1.0])
    rr /= np.linalg.norm(rr)
    chase_t.Set(Gf.Vec3d(*eye.tolist()))
    chase_q.Set(Gf.Quatd(*quat_wxyz(np.stack([rr, np.cross(rr, f), -f], 1)).tolist()))
    draw.clear_points()
    ok = np.isfinite(r)
    pts = origin + r[ok, None] * dirs[ok]
    near = r[ok] < 0.8             # returns this close come from the robot body
    cols = [(1.0, 0.25, 0.2, 1.0) if n else (0.2, 1.0, 0.4, 1.0) for n in near]
    draw.draw_points([tuple(map(float, p)) for p in pts], cols, [5.0] * len(pts))
    print(f"[selfmask] showing {label}", flush=True)
    t_end = time.time() + args.gui_hold
    while time.time() < t_end:
        app.update()


# ---------------------------------------------------------------- capture
out = {"poses": np.array([p for p, _ in POSES]), "scenes": np.array([s for _, s in POSES])}
mounts = {"lidar3d": {"xyz": list(lidar_mount), "pitch_down_deg": lidar_pitch},
          "d455": {"xyz": list(cam_mount), "pitch_down_deg": cam_pitch},
          "base_height": robot.base_height, "excluded_links": list(EXCLUDE), "robot_digest": robot.digest(), "n_triangles": int(len(TRIS))}
app_utils.play()
app_utils.update_app(steps=5)
stats = []
for k, (pose, scene) in enumerate(POSES):
    terrain.SetActive(scene != "flat")
    plane.GetPrim().SetActive(scene == "flat")
    lo, lR = sensor_pose(pose, lidar_mount, lidar_pitch)
    co, cR = sensor_pose(pose, cam_mount, cam_pitch)
    lq = quat_wxyz(lR)
    lidar.set_world_poses(positions=lo.reshape(1, 3), orientations=lq.reshape(1, 4))
    cam_t.Set(Gf.Vec3d(*co.tolist()))
    cam_q.Set(Gf.Quatd(*quat_wxyz(cR @ USD_CAM_IN_SANDBOX).tolist()))
    for case in ("with", "without"):
        if case == "with":
            add_robot(pose)
        else:
            remove_robot()
        app_utils.update_app(steps=6)
        r, dirs, nrm, spread = merge(collect(lo, lq, args.scans), lR)
        out[f"p{k}_lidar_r_{case}"] = r
        out[f"p{k}_depth_{case}"] = depth_now()
        if case == "with":
            out[f"p{k}_lidar_dirs_isaac"] = dirs
            out[f"p{k}_lidar_normals_with"] = nrm
        body = int(np.sum(r < 0.8))
        stats.append({"pose": list(pose), "scene": scene, "case": case, "lidar_returns": int(np.isfinite(r).sum()),
                      "lidar_returns_under_0.8m": body, "scan_range_spread_m": spread,
                      "depth_valid": int(np.isfinite(out[f"p{k}_depth_{case}"]).sum())})
        print(f"[selfmask] pose {k} {pose} {scene} {case} robot: {np.isfinite(r).sum()} returns, {body} under 0.8 m, "
              f"depth {stats[-1]['depth_valid']} px", flush=True)
        show(pose, lo, np.nan_to_num(dirs), r, f"{scene} {case} robot")
    a, b = out[f"p{k}_lidar_r_with"], out[f"p{k}_lidar_r_without"]
    changed = (np.isfinite(a) != np.isfinite(b)) | (np.abs(np.nan_to_num(a, nan=-1) - np.nan_to_num(b, nan=-1)) > 0.01)
    print(f"[selfmask] pose {k}: {int(changed.sum())} beams change when the robot is removed", flush=True)

os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
out["mounts"] = np.array(json.dumps(mounts))
np.savez_compressed(args.out, **out)
with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
    json.dump({"usd": os.path.relpath(args.usd, HERE), "mounts": mounts, "scans_per_capture": args.scans,
               "lidar_attrs": {k_: (list(v_) if hasattr(v_, "__len__") and not isinstance(v_, str) else v_)
                               for k_, v_ in LIDAR_ATTRS.items()},
               "depth_render": [HI_W, HI_H], "captures": stats}, f, indent=1, default=float)
print(f"[selfmask] wrote {args.out}", flush=True)
app.close()
