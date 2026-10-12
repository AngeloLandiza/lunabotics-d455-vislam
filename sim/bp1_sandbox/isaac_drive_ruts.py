"""The team robot drives an approach path in Isaac Sim while its wheels press ruts into the arena; the Unitree L1 and
the D455 capture ideal hits every frame for the BP-1 layer.

Usage (Isaac Sim's Python; run_isaac_drive.sh sets the environment):
  isaac_drive_ruts.py [--seed 1] [--rut-depth 0.032] [--earlier-trips 0] [--remesh 0.3] [--headless]
                      [--out isaac_out/drive_seed1.npz]
Path: approach.trial(seed)'s straight line, one frame every approach.DS (0.15 m); isaac_approach.approach_poses gives it.
Ruts: ruts.Ruts with the given depth (default ruts.rut_depth(robot) = 3.2 cm from Wong 2012 with Lunar Sourcebook
soil values; BP-1 Unknown) and the robot's wheel width. At each frame every wheel track is stamped from where the rear
wheels started to where the front wheels are now, so the ground under and behind the wheels is rutted and the ground
ahead is not. --earlier-trips N first adds ruts.add_earlier_ruts trips (rng seeded with the arena seed + 1).
Terrain in Isaac: the arena height field as one mesh at its full 1 cm resolution (isaac_bridge exports 2 cm), rebuilt
in place from arena.h whenever the robot has driven --remesh metres (and at the start).
Robot attitude (quasi-static, Inferred): each wheel rests on the rutted terrain under its centre; a plane through the
four contact points gives the base height, pitch and roll, so the sensors tilt with ruts and bumps. No PhysX: BP-1 is
not a rigid surface, and re-cooking a collision mesh every 0.3 m would reset contacts anyway; dynamics (bounce, slip)
are not modelled.
Sensors: the L1 at robot.mount("lidar3d_link") fires its hemisphere as the rotary grid of isaac_selfmask_l1.py
(91 elevations 0-90 deg x 360 azimuths, sensor frame z = scanner axis); the D455 depth at robot.mount("d455_link")
as in isaac_capture.py (rendered 5x with square pixels, resampled to 212 x 120). The robot body is in the scene, so
self-occlusion is real. Output per frame k: p{k}_pose (x, y, yaw), p{k}_base (world-from-robot 4x4), p{k}_l1_origin,
p{k}_l1_R, p{k}_l1_r (ranges per grid ray, nan = none), p{k}_d455_origin, p{k}_d455_R, p{k}_depth; once: l1_d_s,
rut_depth, wheel tracks, final arena height field (h_final) and the height-field grid (x0, y0, res).
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--rut-depth", type=float, default=None, help="m (default ruts.rut_depth(robot))")
parser.add_argument("--earlier-trips", type=int, default=0)
parser.add_argument("--remesh", type=float, default=0.3, help="m driven between terrain mesh updates")
parser.add_argument("--scans", type=int, default=2, help="L1 scans merged per frame (RTX drops ~2%% per scan)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gui-hold", type=float, default=0.3)
parser.add_argument("--out", default=None)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, Vt  # noqa: E402

sys.path.insert(0, HERE)
import params as pr  # noqa: E402
from params import v  # noqa: E402
from approach import DS  # noqa: E402
from arena import Arena  # noqa: E402
from isaac_approach import approach_poses  # noqa: E402
from robot import Robot  # noqa: E402
from ruts import Ruts, add_earlier_ruts, rut_depth, wheel_offsets  # noqa: E402

out_path = args.out or os.path.join(HERE, "isaac_out", f"drive_seed{args.seed}.npz")
TELEMETRY = os.path.join(HERE, "isaac_out", "telemetry", f"drive_seed{args.seed}.jsonl")


def telemetry(**kw):
    """One JSON line per event (frames done, pose, GPU memory, ETA) for the run dashboard."""
    os.makedirs(os.path.dirname(TELEMETRY), exist_ok=True)
    with open(TELEMETRY, "a") as f:
        f.write(json.dumps({"t": time.time(), "run": f"drive_seed{args.seed}", **kw}) + "\n")


def gpu_mib():
    try:
        import subprocess
        return int(subprocess.check_output(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                                           text=True).split()[0])
    except Exception:
        return None


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


# ---------------------------------------------------------------- arena, robot, ruts (plain numpy)
arena = Arena(np.random.default_rng(args.seed))
robot = Robot()
depth = rut_depth(robot) if args.rut_depth is None else args.rut_depth
trips = add_earlier_ruts(arena, np.random.default_rng(args.seed + 1), args.earlier_trips, depth, robot) \
    if args.earlier_trips else []
ruts = Ruts(arena, depth, width=robot.wheels[0][2])
LAT = wheel_offsets(robot)
WHEELS = np.array([c for c, _, _ in robot.wheels])          # wheel centres in the robot frame
X_FRONT, X_REAR = WHEELS[:, 0].max(), WHEELS[:, 0].min()
poses = approach_poses(args.seed)
print(f"[drive] seed {args.seed}: {len(poses)} frames, rut depth {100 * depth:.2f} cm, earlier trips {len(trips)}",
      flush=True)


def stamp_to(k):
    """Ruts under every wheel track from the rear wheels' start to the front wheels at frame k."""
    x0, y0, yaw = poses[0]
    xk, yk, _ = poses[k]
    h = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
    a, b = (np.array([x0, y0]) + X_REAR * h), (np.array([xk, yk]) + X_FRONT * h)
    ruts.path([(a[0], a[1], yaw), (b[0], b[1], yaw)], LAT)
    ruts.apply()


def base_pose(pose):
    """World-from-robot transform: wheels resting on the terrain (plane through the four contact points)."""
    x, y, yaw = pose
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    wxy = WHEELS[:, :2] @ Rz[:2, :2].T + [x, y]
    gz = arena.height(wxy[:, 0], wxy[:, 1])                   # ground under each wheel centre
    A = np.c_[WHEELS[:, 0], WHEELS[:, 1], np.ones(4)]         # z = z0 + sx * xr + sy * yr in the yawed frame
    sx, sy, z0 = np.linalg.lstsq(A, gz, rcond=None)[0]
    pitch, roll = np.arctan(sx), np.arctan(sy)   # ground rising ahead (sx > 0) lifts the nose; rising left lifts the left side
    Ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)], [0, 1, 0], [np.sin(pitch), 0, np.cos(pitch)]])   # x -> (c, 0, s)
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])     # y -> (0, c, s)
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [x, y, z0]
    return T, gz


# ---------------------------------------------------------------- Isaac stage: terrain mesh at 1 cm, robot, sensors
stage_utils.create_new_stage()
stage = stage_utils.get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
nx, ny = arena.h.shape
ii, jj = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
a_ = (ii * ny + jj).ravel()
TRI_IDX = np.stack([a_, a_ + ny, a_ + ny + 1, a_, a_ + ny + 1, a_ + 1], 1).reshape(-1).astype(np.int32)
terrain = UsdGeom.Mesh.Define(stage, "/World/Terrain")
terrain.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRI_IDX) // 3, 3, np.int32)))
terrain.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(TRI_IDX))
terrain.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
terrain.CreateDisplayColorAttr([Gf.Vec3f(0.42, 0.40, 0.37)])
terrain_pts = terrain.CreatePointsAttr()


def remesh():
    P = np.stack([arena.X.ravel(), arena.Y.ravel(), arena.h.ravel()], 1).astype(np.float32)
    terrain_pts.Set(Vt.Vec3fArray.FromNumpy(P))


TRIS, _ = robot.triangles(exclude=("lidar3d_link", "d455_link"))   # sensor housings left out (see isaac_selfmask.py)
rob_xf = UsdGeom.Xform.Define(stage, "/World/bp1_robot")
rob_mesh = UsdGeom.Mesh.Define(stage, "/World/bp1_robot/body")
rob_mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(TRIS.reshape(-1, 3).astype(np.float32)))
rob_mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRIS), 3, np.int32)))
rob_mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * len(TRIS), dtype=np.int32)))
rob_mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
rob_mesh.CreateDoubleSidedAttr(True)
rob_mesh.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.85, 0.9)])
rx = UsdGeom.Xformable(rob_xf.GetPrim())
rob_t, rob_q = rx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), rx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

L1_XYZ, L1_R = robot.mount("lidar3d_link")
D455_XYZ, D455_R = robot.mount("d455_link")
EL_GRID = np.arange(0.0, 90.0 + 1e-9, 1.0)
N_EM, N_TICK = len(EL_GRID), 360
N_L1 = N_EM * N_TICK
L1_ATTRS = {
    "omni:sensor:Core:scanType": "ROTARY",
    "omni:sensor:Core:numberOfEmitters": N_EM, "omni:sensor:Core:numberOfChannels": N_EM,
    "omni:sensor:Core:scanRateBaseHz": 10, "omni:sensor:Core:patternFiringRateHz": 10 * N_TICK,
    "omni:sensor:Core:farRangeM": float(pr.LIDAR_PROFILES["l1"]["rmax"]), "omni:sensor:Core:nearRangeM": 0.01,
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
lidar = Lidar("/World/bp1_l1", attributes=L1_ATTRS, tick_rate=10, aux_output_level="FULL")
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
cxf = UsdGeom.Xformable(cam.GetPrim())
cam_t, cam_q = cxf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), cxf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
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


D_S = None


def l1_scans(origin, q, n_scans, skip=2, max_updates=600):
    """Merged L1 grid ranges at the pose (skipping `skip` scans after the scene changed) and the grid directions."""
    global D_S
    rs, last_ts, seen = [], None, 0
    for _ in range(max_updates):
        app_utils.update_app()
        data, _ = lidar_sensor.get_data("generic-model-output")
        if data is None:
            continue
        g = parse_generic_model_output_data(data)
        if g.numElements != N_L1 or g.timestampNs == last_ts:
            continue
        last_ts = g.timestampNs
        qs = np.array(g.frameEnd.orientation)[[3, 0, 1, 2]]
        if not (np.allclose(g.frameEnd.posM, origin, atol=1e-4) and abs(np.dot(qs, q)) > 1 - 1e-6):
            continue
        seen += 1
        if seen <= skip:
            continue
        idx = np.asarray(g.tickId).astype(int) * N_EM + np.asarray(g.emitterId).astype(int)
        valid = (np.asarray(g.flags) & 64) > 0
        r = np.full(N_L1, np.nan)
        r[idx[valid]] = np.asarray(g.z, float)[valid]
        if D_S is None:
            az, el = np.radians(np.asarray(g.x, float)), np.radians(np.asarray(g.y, float))
            D_S = np.full((N_L1, 3), np.nan)
            D_S[idx] = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
        rs.append(r)
        if len(rs) == n_scans:
            R = np.stack(rs)
            hit = np.isfinite(R).any(0)
            return np.where(hit, np.nanmax(np.where(np.isfinite(R), R, -np.inf), 0), np.nan)
    raise RuntimeError("not enough L1 scans")


# ---------------------------------------------------------------- view (GUI only)
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    app_utils.enable_extension("isaacsim.util.debug_draw")
    from isaacsim.util.debug_draw import _debug_draw
    draw = _debug_draw.acquire_debug_draw_interface()
    UsdLux.DomeLight.Define(stage, "/World/view_dome").CreateIntensityAttr(350.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/view_sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.5)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(70.0, 0.0, 120.0))   # low sun: ruts cast shadows
    chase = UsdGeom.Camera.Define(stage, "/World/chase_cam")
    chase.CreateFocalLengthAttr(14.0)
    chase.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    chx = UsdGeom.Xformable(chase.GetPrim())
    chase_t, chase_q = chx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), chx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    get_active_viewport().camera_path = chase.GetPath().pathString


def show(pose, origin, R, r):
    if args.headless:
        return
    x, y, yaw = pose
    hv = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), 0.0])
    side = np.array([-hv[1], hv[0], 0.0])
    eye = np.array([x, y, 0.0]) - 1.9 * hv + 1.1 * side + np.array([0, 0, 1.5])   # behind-left: ruts in view
    f = np.array([x, y, 0.0]) + 0.6 * hv - eye
    f /= np.linalg.norm(f)
    rr = np.cross(f, [0, 0, 1.0])
    rr /= np.linalg.norm(rr)
    chase_t.Set(Gf.Vec3d(*eye.tolist()))
    chase_q.Set(Gf.Quatd(*quat_wxyz(np.stack([rr, np.cross(rr, f), -f], 1)).tolist()))
    draw.clear_points()
    ok = np.isfinite(r)
    pts = origin + r[ok, None] * (np.nan_to_num(D_S[ok]) @ R.T)
    pts = pts[::3]                                           # every third return keeps the view responsive
    draw.draw_points([tuple(map(float, p)) for p in pts], [(0.2, 1.0, 0.4, 0.9)] * len(pts), [3.0] * len(pts))
    t_end = time.time() + args.gui_hold
    while time.time() < t_end:
        app.update()


# ---------------------------------------------------------------- drive
out = {"poses": np.array(poses), "rut_depth": depth, "x0": arena.x0, "y0": arena.y0, "res": arena.res,
       "seed": args.seed}
telemetry(event="start", frames=len(poses), rut_depth=depth, gpu_mib=gpu_mib())
app_utils.play()
app_utils.update_app(steps=5)
last_mesh_s, t0 = None, time.time()
for k, pose in enumerate(poses):
    s = k * DS
    stamp_to(k)
    if last_mesh_s is None or s - last_mesh_s >= args.remesh - 1e-9:
        remesh()
        last_mesh_s = s
    T, gz = base_pose(pose)
    Rb, tb = T[:3, :3], T[:3, 3]
    rob_t.Set(Gf.Vec3d(*tb.tolist()))
    rob_q.Set(Gf.Quatd(*quat_wxyz(Rb).tolist()))
    lo, lR = tb + Rb @ np.asarray(L1_XYZ), Rb @ L1_R
    co, cR = tb + Rb @ np.asarray(D455_XYZ), Rb @ D455_R
    lq = quat_wxyz(lR)
    lidar.set_world_poses(positions=lo.reshape(1, 3), orientations=lq.reshape(1, 4))
    cam_t.Set(Gf.Vec3d(*co.tolist()))
    cam_q.Set(Gf.Quatd(*quat_wxyz(cR @ USD_CAM_IN_SANDBOX).tolist()))
    r = l1_scans(lo, lq, args.scans)
    app_utils.update_app(steps=4)
    hi = np.asarray(depth_ann.get_data(), dtype=np.float64)
    hi = np.where(np.isfinite(hi) & (hi > 0), hi, np.nan)
    out.update({f"p{k}_base": T, f"p{k}_wheel_ground_z": gz, f"p{k}_l1_origin": lo, f"p{k}_l1_R": lR,
                f"p{k}_l1_r": r.astype(np.float32), f"p{k}_d455_origin": co, f"p{k}_d455_R": cR,
                f"p{k}_depth": resample_depth(hi).astype(np.float32)})
    pitch = np.degrees(np.arcsin(Rb[2, 0]))           # nose up positive
    roll = np.degrees(np.arctan2(Rb[2, 1], Rb[2, 2]))
    eta = (time.time() - t0) / (k + 1) * (len(poses) - k - 1)
    print(f"[drive] frame {k + 1}/{len(poses)} s={s:.2f} m base z {tb[2]:.4f} pitch(nose up) {pitch:+.2f} roll(left up) {roll:+.2f} deg, "
          f"L1 {np.isfinite(r).sum()} returns, ETA {eta:.0f} s", flush=True)
    telemetry(event="frame", frame=k + 1, frames=len(poses), s=s, pose=list(pose), base_z=float(tb[2]),
              pitch_deg=float(pitch), roll_deg=float(roll), l1_returns=int(np.isfinite(r).sum()), gpu_mib=gpu_mib(),
              eta_s=eta)
    show(pose, lo, lR, r)

out.update({"l1_d_s": D_S, "h_final": arena.h.astype(np.float32), "trench_final": ruts.trench.astype(np.float32),
            "earlier_trips": json.dumps([np.asarray(t).tolist() for t in trips]),
            "mounts": json.dumps({"l1": {"xyz": list(L1_XYZ), "R": L1_R.tolist()},
                                  "d455": {"xyz": list(D455_XYZ), "R": D455_R.tolist()},
                                  "base_height": robot.base_height, "robot_digest": robot.digest()})})
os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
np.savez_compressed(out_path, **out)
telemetry(event="done", frames=len(poses), seconds=time.time() - t0, out=out_path, gpu_mib=gpu_mib())
print(f"[drive] wrote {out_path} ({time.time() - t0:.0f} s)", flush=True)
app.close()
