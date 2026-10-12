"""Capture ideal LiDAR ranges and ideal depth images of a sandbox arena in Isaac Sim (headless).

Runs under Isaac Sim's own Python (~/isaacsim-env, Isaac Sim 6.0.1); start it with run_isaac_capture.sh.
For each robot pose (x, y, yaw) it places an RTX LiDAR and a pinhole depth camera exactly as sensors.sensor_pose()
does with params.MOUNTS, renders, and writes one .npz that isaac_validate.py feeds through the BP-1 layer.

Sources (Isaac Sim 6.0.1 as installed in ~/isaacsim-env; paths below are relative to site-packages/isaacsim):
- RTX Lidar overview, Isaac Sim 6.0: https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_lidar.html
  (OmniLidar prims with OmniSensorGenericLidarCoreAPI; Lidar / LidarSensor / parse_generic_model_output_data).
- Lidar authoring class: exts/isaacsim.sensors.experimental.rtx/.../impl/lidar.py (Lidar(path, attributes=...,
  tick_rate=..., aux_output_level=...), which calls rep.functional.create.omni_lidar).
- Emitter-state attributes: extscache/omni.replicator.core-*/.../functional/create_batch.py, omni_lidar(): any
  "omni:sensor:Core:emitterState:<name>:*" attribute applies OmniSensorGenericLidarCoreEmitterStateAPI:<name>.
  Array values must be Vt arrays: modify.set_attributes() star-expands plain Python lists and fails.
- Attribute names, defaults and meanings: extscache/omni.usd.schema.omni_sensors-*/usd_plugins/generatedSchema.usda,
  class OmniSensorGenericLidarCoreAPI (rangeAccuracyM default 0.02 m, azimuthErrorStd 0.005 deg, instantLidar, ...).
- Runtime + parser: exts/isaacsim.sensors.experimental.rtx/.../impl/lidar_sensor.py, impl/utils.py, and the
  GenericModelOutput fields in generic_model_output/_rtx_sensors_gmo.pyi (x, y, z, flags, channelId, tickId,
  hitNormals, frameStart/frameEnd). With elementsCoordsType SPHERICAL, x = azimuth deg, y = elevation deg,
  z = range m; flags bit 64 (ElementFlags.VALID) marks a return. Checked with a probe run on 2026-10-11.
- Depth: the 'distance_to_image_plane' Replicator annotator on a UsdGeom.Camera render product, as in
  sim/isaac_lunar_sim.py. Isaac renders square pixels: exts/isaacsim.sensors.experimental.rtx/.../camera_sensor.py
  calls camera.enforce_square_pixels(), and the probe showed the verticalAperture attribute has no effect.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--usd", default=os.path.join(HERE, "usd", "arena_seed1.usda"))
parser.add_argument("--poses", default="1.0,2.5,0;1.25,2.5,0;1.5,2.5,0;1.75,2.5,0;"
                                       "0.8,1.0,20;1.035,1.086,20;1.27,1.171,20;1.505,1.257,20",
                    help="robot poses 'x,y,yaw_deg;...' in the arena frame (metres, degrees)")
parser.add_argument("--out", default=os.path.join(HERE, "isaac_out", "capture.npz"))
parser.add_argument("--scans", type=int, default=4,
                    help="LiDAR scans merged per pose: RTX drops ~3%% of returns at random in each scan (see wait_for_scans)")
parser.add_argument("--upsample", type=int, default=5,
                    help="odd factor: depth is rendered at upsample x 212 wide and resampled to the sandbox pixels")
parser.add_argument("--gui", action="store_true",
                    help="open the Isaac Sim window: chase camera on the robot, LiDAR hits and footprint drawn, lights on")
parser.add_argument("--gui-hold", type=float, default=0.6, help="seconds each pose stays on screen with --gui")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from pxr import Gf, UsdGeom, Vt  # noqa: E402

sys.path.insert(0, HERE)
import params as pr  # noqa: E402
from params import v  # noqa: E402
from sensors import sensor_pose  # noqa: E402

PROF = pr.LIDAR_PROFILES["spin16"]
N_CH = PROF["channels"]
N_AZ = int(round(360.0 / PROF["hres"]))
SCAN_HZ = 10
LIDAR_RMAX = 8.0                      # sensors.Lidar.rmax
EL = np.linspace(PROF["vmin"], PROF["vmax"], N_CH)   # same elevations as sensors.spinning_dirs


def quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method: stable for any rotation)."""
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


# ---------------------------------------------------------------- stage
ok, stage = stage_utils.open_stage(args.usd)
assert ok, f"could not open {args.usd}"
assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z and UsdGeom.GetStageMetersPerUnit(stage) == 1.0
app_utils.update_app(steps=10)

# ---------------------------------------------------------------- RTX LiDAR, spin16 pattern
# A rotary pattern fires every emitter once per azimuth tick; ticks per revolution = patternFiringRateHz / scanRateBaseHz,
# so 0.4 deg steps at 10 Hz need 9000 Hz (Inferred from the schema docs; the probe returned 16 x 900 = 14400 rays).
LIDAR_ATTRS = {
    "omni:sensor:Core:scanType": "ROTARY",
    "omni:sensor:Core:numberOfEmitters": N_CH,
    "omni:sensor:Core:numberOfChannels": N_CH,
    "omni:sensor:Core:scanRateBaseHz": SCAN_HZ,
    "omni:sensor:Core:patternFiringRateHz": SCAN_HZ * N_AZ,
    "omni:sensor:Core:farRangeM": LIDAR_RMAX,
    "omni:sensor:Core:nearRangeM": 0.01,        # the BP-1 layer applies the 0.3 m blind zone itself
    "omni:sensor:Core:maxReturns": 1,
    "omni:sensor:Core:instantLidar": True,      # one full scan at frame end, so a teleported pose never smears
    "omni:sensor:Core:skipDroppingInvalidPoints": True,   # keep no-return rays so every ray is accounted for
    "omni:sensor:Core:rayType": "IDEALIZED",
    # Built-in noise off (schema defaults: rangeAccuracyM 0.02 m, azimuthErrorStd 0.005 deg; the others are already 0)
    "omni:sensor:Core:rangeAccuracyM": 0.0,
    "omni:sensor:Core:azimuthErrorMean": 0.0, "omni:sensor:Core:azimuthErrorStd": 0.0,
    "omni:sensor:Core:elevationErrorMean": 0.0, "omni:sensor:Core:elevationErrorStd": 0.0,
    "omni:sensor:Core:originErrorMean": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:originErrorStd": Vt.FloatArray([0.0, 0.0, 0.0]),
    "omni:sensor:Core:rangeResolutionM": 0.001,  # schema default 0.004 m; quantisation, not noise
    "omni:sensor:Core:emitterState:s001:azimuthDeg": Vt.FloatArray([0.0] * N_CH),
    "omni:sensor:Core:emitterState:s001:elevationDeg": Vt.FloatArray(EL.tolist()),
    "omni:sensor:Core:emitterState:s001:fireTimeNs": Vt.UIntArray([0] * N_CH),
    "omni:sensor:Core:emitterState:s001:channelId": Vt.UIntArray(list(range(1, N_CH + 1))),
}
lidar = Lidar("/World/bp1_lidar", attributes=LIDAR_ATTRS, tick_rate=SCAN_HZ, aux_output_level="FULL")
lidar_sensor = LidarSensor(lidar, annotators=["generic-model-output"])

# ---------------------------------------------------------------- depth camera
CAM_W, CAM_H = v(pr.D455, "sim_size")
HF, VF = np.radians(v(pr.D455, "hfov")), np.radians(v(pr.D455, "vfov"))
K = args.upsample
assert K % 2 == 1, "odd upsample keeps the sandbox column centres on rendered column centres"
HI_W = CAM_W * K
PITCH = 2 * np.tan(HF / 2) / HI_W                  # square pixel size on the unit image plane
HI_H = 2 * int(np.ceil(np.tan(VF / 2) / PITCH)) + 2
FOCAL = 10.0                                         # mm; only the aperture / focal ratio matters
cam = UsdGeom.Camera.Define(stage, "/World/bp1_depth")
cam.CreateFocalLengthAttr(FOCAL)
cam.CreateHorizontalApertureAttr(float(2 * FOCAL * np.tan(HF / 2)))
cam.CreateVerticalApertureAttr(float(2 * FOCAL * np.tan(HF / 2) * HI_H / HI_W))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
cam_xf = UsdGeom.Xformable(cam.GetPrim())
cam_t = cam_xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
cam_q = cam_xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
depth_ann = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
depth_ann.attach(rep.create.render_product(cam.GetPath().pathString, (HI_W, HI_H)))
# USD cameras look along -Z with +Y up and +X right; the sandbox camera frame is x forward, y left, z up.
USD_CAM_IN_SANDBOX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def resample_depth(hi):
    """Hi-res square-pixel depth -> sandbox 212 x 120 grid (87 x 58 deg, row 0 at the top).

    Columns land exactly on rendered columns (odd upsample). Rows fall between rendered rows; interpolate
    linearly when both neighbours hit the same surface (within 5 cm), else take the nearest row.
    """
    cols = K * np.arange(CAM_W) + K // 2
    vs = ((np.arange(CAM_H) + 0.5) / CAM_H * 2 - 1) * np.tan(VF / 2)   # +down on the image plane
    fr = vs / PITCH + HI_H / 2 - 0.5
    r0 = np.floor(fr).astype(int)
    a = (fr - r0)[:, None]
    z0, z1 = hi[r0][:, cols], hi[r0 + 1][:, cols]
    near = np.where(a < 0.5, z0, z1)
    smooth = np.isfinite(z0) & np.isfinite(z1) & (np.abs(z1 - z0) < 0.05)
    return np.where(smooth, z0 * (1 - a) + z1 * a, near)


# ---------------------------------------------------------------- GUI (watching only; nothing here reaches the sensors)
if args.gui:
    import time
    from pxr import UsdLux
    from omni.kit.viewport.utility import get_active_viewport
    app_utils.enable_extension("isaacsim.util.debug_draw")
    from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
    # Lights only change what the viewport shows: RTX LiDAR ranges and distance_to_image_plane ignore lighting
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

    def show(pose, lo, dirs, rng_m):
        """Chase camera 2.2 m behind and 1.6 m above the robot, looking at a point 1.2 m ahead; hits and footprint."""
        x, y, yaw = pose
        hv = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), 0.0])
        eye = np.array([x, y, 0.0]) - 2.2 * hv + np.array([0, 0, 1.6])
        f = (np.array([x, y, 0.0]) + 1.2 * hv) - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        R = np.stack([r, u, -f], 1)               # USD camera: +X right, +Y up, looks along -Z
        chase_t.Set(Gf.Vec3d(*eye.tolist()))
        chase_q.Set(Gf.Quatd(*quat_wxyz(R).tolist()))
        draw.clear_points()
        draw.clear_lines()
        ok = np.isfinite(rng_m)
        pts = lo + rng_m[ok, None] * dirs[ok]
        draw.draw_points([tuple(map(float, q)) for q in pts], [(0.2, 1.0, 0.4, 1.0)] * len(pts), [4.0] * len(pts))
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        corners = [(x + c * a - s * b, y + s * a + c * b, 0.02) for a, b in ((0.53, 0.35), (0.53, -0.35), (-0.41, -0.35), (-0.41, 0.35))]
        draw.draw_lines(corners, corners[1:] + corners[:1], [(0.1, 0.85, 1.0, 1.0)] * 4, [3.0] * 4)
        t_end = time.time() + args.gui_hold
        while time.time() < t_end:
            app.update()


# ---------------------------------------------------------------- capture
def parse_poses(s):
    return [tuple(float(t) for t in p.split(",")) for p in s.split(";") if p.strip()]


def wait_for_scans(origin, q, n_scans, max_updates=400):
    """Step the app until n_scans distinct complete scans were taken at the requested pose; return them all.

    Each RTX scan here drops about 3% of its returns at random (measured 2026-10-11: three scans at one pose
    differed in ~420 of 14400 hit/no-hit flags, while every range that was returned agreed to 0.0 cm), and no
    attribute in OmniSensorGenericLidarCoreAPI documents a dropout. So the caller merges several scans.
    """
    scans, last_ts = [], None
    for _ in range(max_updates):
        app_utils.update_app()
        data, _ = lidar_sensor.get_data("generic-model-output")
        if data is None:
            continue
        g = parse_generic_model_output_data(data)
        if g.numElements != N_CH * N_AZ or g.timestampNs == last_ts:
            continue
        # frameStart/End orientation is (x, y, z, w): the probe returned [0, 0, 0, 1] for identity
        qs = np.array(g.frameEnd.orientation)[[3, 0, 1, 2]]
        at_pose = (np.allclose(g.frameStart.posM, origin, atol=1e-4) and np.allclose(g.frameEnd.posM, origin, atol=1e-4)
                   and abs(np.dot(qs, q)) > 1 - 1e-6)
        if not at_pose:
            scans = []
            continue
        last_ts = g.timestampNs
        scans.append({"az": np.asarray(g.x, float).copy(), "el": np.asarray(g.y, float).copy(),
                      "r": np.asarray(g.z, float).copy(), "valid": (np.asarray(g.flags) & 64) > 0,
                      "normals": np.asarray(g.hitNormals, float).reshape(-1, 3).copy()})
        if len(scans) > n_scans:   # the first scan at a new pose may have been rendered before the move
            return scans[1:]
    raise RuntimeError(f"only {len(scans)} complete LiDAR scans at {origin} after {max_updates} updates")


poses = parse_poses(args.poses)
app_utils.play()
app_utils.update_app(steps=5)
out = {"poses": np.array(poses), "lidar_profile": "spin16", "depth_hi_size": np.array([HI_H, HI_W])}
lidar_mount, lidar_pitch = v(pr.MOUNTS, "lidar_xyz"), v(pr.MOUNTS, "lidar_pitch_down")
cam_mount, cam_pitch = v(pr.MOUNTS, "d455_xyz"), v(pr.MOUNTS, "d455_pitch_down")
for k, pose in enumerate(poses):
    lo, lR = sensor_pose(pose, lidar_mount, lidar_pitch)
    co, cR = sensor_pose(pose, cam_mount, cam_pitch)
    lq = quat_wxyz(lR)
    lidar.set_world_poses(positions=lo.reshape(1, 3), orientations=lq.reshape(1, 4))
    cam_t.Set(Gf.Vec3d(*co.tolist()))
    cam_q.Set(Gf.Quatd(*quat_wxyz(cR @ USD_CAM_IN_SANDBOX).tolist()))
    scans = wait_for_scans(lo, lq, args.scans)
    app_utils.update_app(steps=4)                    # let the camera render product catch up with the new pose
    hi = np.asarray(depth_ann.get_data(), dtype=np.float64)
    hi = np.where(np.isfinite(hi) & (hi > 0), hi, np.nan)

    # Put each Isaac ray in sensors.spinning_dirs order: index = channel * N_AZ + azimuth step from -180 deg
    r_full = np.full((len(scans), N_CH * N_AZ), np.nan)
    dirs_isaac = np.full((N_CH * N_AZ, 3), np.nan)
    n_full = np.full((N_CH * N_AZ, 3), np.nan)
    for si, s in enumerate(scans):
        az, el = s["az"], s["el"]
        ch = np.rint((el - PROF["vmin"]) / ((PROF["vmax"] - PROF["vmin"]) / (N_CH - 1))).astype(int)
        ai = np.rint((az + 180.0) / PROF["hres"]).astype(int) % N_AZ
        idx = ch * N_AZ + ai
        assert len(np.unique(idx)) == len(idx), "two Isaac rays mapped to one sandbox ray"
        r_full[si, idx] = np.where(s["valid"], s["r"], np.nan)
        # Sensor-frame unit vectors (x forward, y left, z up; azimuth counter-clockwise from +x), rotated to the arena
        d_s = np.stack([np.cos(np.radians(el)) * np.cos(np.radians(az)),
                        np.cos(np.radians(el)) * np.sin(np.radians(az)), np.sin(np.radians(el))], -1)
        dirs_isaac[idx] = d_s @ lR.T
        if len(s["normals"]) == len(idx):
            take = s["valid"] & ~np.isfinite(n_full[idx, 0])
            n_full[idx[take]] = s["normals"][take]
    hit_any = np.isfinite(r_full).any(0)
    spread = np.nanmax(r_full[:, hit_any], 0) - np.nanmin(r_full[:, hit_any], 0)
    assert not np.any(spread > 1e-3), f"returned ranges differ between scans by up to {np.nanmax(spread)} m"
    valid = np.isfinite(r_full).any(0)
    r_merged = np.nanmax(np.where(np.isfinite(r_full), r_full, -np.inf), 0)
    r_merged = np.where(valid, r_merged, np.nan)
    if args.gui:
        show(pose, lo, np.nan_to_num(dirs_isaac), r_merged)
    out[f"p{k}_lidar_origin"], out[f"p{k}_lidar_R"] = lo, lR
    out[f"p{k}_lidar_range"], out[f"p{k}_lidar_dirs_isaac"] = r_merged, dirs_isaac
    out[f"p{k}_lidar_range_per_scan"] = r_full.astype(np.float32)
    out[f"p{k}_lidar_hit_normals_raw"] = n_full
    out[f"p{k}_cam_origin"], out[f"p{k}_cam_R"] = co, cR
    out[f"p{k}_depth"] = resample_depth(hi)
    out[f"p{k}_depth_hi"] = hi.astype(np.float32)
    print(f"[capture] pose {k} {pose}: lidar {valid.sum()}/{len(valid)} returns "
          f"(single scans {', '.join(str(int(c)) for c in np.isfinite(r_full).sum(1))}), "
          f"depth {np.isfinite(out[f'p{k}_depth']).sum()}/{CAM_W * CAM_H} px", flush=True)

os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
np.savez_compressed(args.out, **out)
meta = {"usd": os.path.abspath(args.usd), "poses": poses, "lidar_attrs": {k: (list(v_) if hasattr(v_, "__len__") and
        not isinstance(v_, str) else v_) for k, v_ in LIDAR_ATTRS.items()}, "depth_render": [HI_W, HI_H],
        "upsample": K, "scans_per_pose": args.scans}
with open(os.path.splitext(args.out)[0] + ".json", "w") as f:
    json.dump(meta, f, indent=1, default=float)
print(f"[capture] wrote {args.out}", flush=True)
# No timeline stop() here: in one run it hung for ~9 min and ended in "[Fatal] [omni.rtx] Out of resource descriptors!"
app.close()
