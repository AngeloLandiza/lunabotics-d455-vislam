"""D455 stereo images in Isaac Sim along the drift drives, for visual odometry (vo_eval.py).

Usage (Isaac Sim's Python; run_isaac_vo.sh): isaac_vo.py --seeds 3000,3002,3003,3004 [--low 3000] [--every 2]
                                             [--test] [--out isaac_out/vo_v1] [--headless]
Drives: results/traj_v1/traj_<seed>_nominal.npz (export_traj.py: the drift.py drive, 30 Hz). Every --every-th pose
is rendered (15 Hz at 2), plus every leg end. Terrain: the sandbox arena of the same seed (bp1_sandbox Arena, the
arena drift.py's L1 scans) as one 1 cm mesh with the BP-1 look of bp1_look.json, as in bp1_sandbox/isaac_landmarks.py;
the robot body is in the scene and sits settled on the height field (four wheels, least-squares plane), so the camera
rolls and pitches with the terrain. Camera: D455 at robot.mount("d455_link") (2027 front mount, 40 deg down); left and
right imagers 95 mm apart (bp1_sandbox/params.py D455 baseline, Verified from the datasheet) about that point, 848 x 480,
87 deg horizontal (params.py, Verified), square pixels, mono from the rendered colour. Light: isaac_landmarks
"diffuse" for every drive; --low drives are rendered again under its "low" light (sun 15 deg up + weak dome).
Output: <out>/<seed>_<light>.npz with left, right (n, 480, 848) uint8, idx (n,) pose indices into the drive file,
cam_T (n, 4, 4) world-from-left-camera (x forward, y left, z up), base_T (n, 4, 4) world-from-robot; <out>/meta.json.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SANDBOX = os.path.join(HERE, "..", "bp1_sandbox")
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--seeds", default="3000,3002,3003,3004")
parser.add_argument("--low", default="3000", help="drives also rendered under the low light")
parser.add_argument("--traj", default=os.path.join(HERE, "results", "traj_v1"))
parser.add_argument("--every", type=int, default=2)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test", action="store_true", help="first 20 rendered poses of the first seed only")
parser.add_argument("--settle", type=int, default=3, help="app updates after each pose change before averaging")
parser.add_argument("--average", type=int, default=4, help="renders averaged per image (RTX light-sampling noise)")
parser.add_argument("--out", default=os.path.join(HERE, "isaac_out", "vo_v1"))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt  # noqa: E402

sys.path.insert(0, SANDBOX)
import params as pr  # noqa: E402
from arena import Arena  # noqa: E402
from robot import Robot  # noqa: E402
from telemetry import Telemetry  # noqa: E402

LOOK = json.load(open(os.path.join(SANDBOX, "bp1_look.json")))["values"]
W_IMG, H_IMG = 848, 480
HFOV = pr.v(pr.D455, "hfov")
BASELINE = pr.v(pr.D455, "baseline")
os.makedirs(args.out, exist_ok=True)

# Camera-like rendering as in isaac_landmarks.py: no temporal smearing between poses, fixed exposure
for key, val in [("/rtx/post/aa/op", 0), ("/rtx/directLighting/domeLight/denoisingTechnique", 0),
                 ("/rtx/post/motionblur/enabled", False), ("/rtx/directLighting/sampledLighting/enabled", False),
                 ("/rtx/ambientOcclusion/enabled", False), ("/rtx/indirectDiffuse/enabled", False),
                 ("/rtx/reflections/enabled", False), ("/rtx/translucency/enabled", False),
                 ("/rtx/post/histogram/enabled", False)]:
    carb.settings.get_settings().set(key, val)


def quat_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method), as in isaac_landmarks.py."""
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


robot = Robot()
WHEELS = np.array([c for c, _, _ in robot.wheels])
CAM_XYZ, CAM_R = robot.mount("d455_link")
USD_CAM_IN_SANDBOX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # as isaac_landmarks.py


def settled(arena, x, y, yaw):
    """World-from-robot transform with the wheels resting on the height field (isaac_landmarks.settled, yaw in rad)."""
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    wxy = WHEELS[:, :2] @ Rz[:2, :2].T + [x, y]
    gz = arena.height(wxy[:, 0], wxy[:, 1])
    sx, sy, z0 = np.linalg.lstsq(np.c_[WHEELS[:, 0], WHEELS[:, 1], np.ones(4)], gz, rcond=None)[0]
    p, r = np.arctan(sx), np.arctan(sy)
    Ry = np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rz @ Ry @ Rx, [x, y, z0]
    return T


# ---------------------------------------------------------------- stage: terrain, robot, stereo pair, lights
stage_utils.create_new_stage()
stage = stage_utils.get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
terrain = UsdGeom.Mesh.Define(stage, "/World/Terrain")
terrain.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
mat = UsdShade.Material.Define(stage, "/World/Looks/bp1")
pbr = UsdShade.Shader.Define(stage, "/World/Looks/bp1/pbr")
pbr.CreateIdAttr("UsdPreviewSurface")
pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
pbr.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
pbr.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
reader = UsdShade.Shader.Define(stage, "/World/Looks/bp1/albedo")
reader.CreateIdAttr("UsdPrimvarReader_float3")
reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("bp1Albedo")
pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(), "result")
mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(terrain.GetPrim()).Bind(mat)
diffuse = np.array(LOOK["diffuse_rgb_linear"]["value"])


def build_terrain(arena, seed):
    """The arena height field as one 1 cm mesh, albedo per vertex: diffuse_rgb x (1 + 0.15 u) (isaac_landmarks.py)."""
    nx, ny = arena.X.shape
    ii, jj = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
    a_ = (ii * ny + jj).ravel()
    terrain.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(2 * len(a_), 3, np.int32)))
    terrain.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
        np.stack([a_, a_ + ny, a_ + ny + 1, a_, a_ + ny + 1, a_ + 1], 1).reshape(-1).astype(np.int32)))
    terrain.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(
        np.stack([arena.X.ravel(), arena.Y.ravel(), arena.h.ravel()], 1).astype(np.float32)))
    factor = 1 + 0.15 * np.random.default_rng(1000 + seed).uniform(-1, 1, arena.h.shape)
    pv = UsdGeom.PrimvarsAPI(terrain.GetPrim()).CreatePrimvar("bp1Albedo", Sdf.ValueTypeNames.Color3fArray,
                                                               UsdGeom.Tokens.vertex)
    pv.Set(Vt.Vec3fArray.FromNumpy((factor.reshape(-1, 1) * diffuse).astype(np.float32)))


TRIS, _ = robot.triangles(exclude=("lidar3d_link", "d455_link"))
rob_xf = UsdGeom.Xform.Define(stage, "/World/robot")
rm = UsdGeom.Mesh.Define(stage, "/World/robot/body")
rm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(TRIS.reshape(-1, 3).astype(np.float32)))
rm.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRIS), 3, np.int32)))
rm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * len(TRIS), dtype=np.int32)))
rm.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
rm.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.62)])
rx = UsdGeom.Xformable(rob_xf.GetPrim())
rob_t, rob_q = rx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), rx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

FOCAL = 10.0
cams = {}
for side, dy in (("left", BASELINE / 2), ("right", -BASELINE / 2)):      # camera y points left
    cam = UsdGeom.Camera.Define(stage, f"/World/d455_{side}")
    cam.CreateFocalLengthAttr(FOCAL)
    cam.CreateHorizontalApertureAttr(float(2 * FOCAL * np.tan(np.radians(HFOV / 2))))
    cam.CreateVerticalApertureAttr(float(2 * FOCAL * np.tan(np.radians(HFOV / 2)) * H_IMG / W_IMG))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
    cx = UsdGeom.Xformable(cam.GetPrim())
    rp = rep.create.render_product(cam.GetPath().pathString, (W_IMG, H_IMG))
    ann = rep.AnnotatorRegistry.get_annotator("LdrColor")
    ann.attach(rp)
    cams[side] = {"dy": dy, "t": cx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble),
                  "q": cx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble), "ann": ann, "path": cam.GetPath().pathString}

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateAngleAttr(0.53)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(75.0, 0.0, 135.0))   # 15 deg above the horizon
LIGHTS = {"diffuse": {"dome": 1500.0, "sun": 0.0}, "low": {"dome": 300.0, "sun": 250000.0}}   # isaac_landmarks.py
dome.CreateIntensityAttr(LIGHTS["diffuse"]["dome"])
sun.CreateIntensityAttr(0.0)
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = cams["left"]["path"]          # the window shows the left imager


def render(Tb):
    rob_t.Set(Gf.Vec3d(*Tb[:3, 3].tolist()))
    rob_q.Set(Gf.Quatd(*quat_wxyz(Tb[:3, :3]).tolist()))
    Tc = np.eye(4)
    Tc[:3, :3] = Tb[:3, :3] @ CAM_R
    Tc[:3, 3] = Tb[:3, 3] + Tb[:3, :3] @ np.asarray(CAM_XYZ)
    q = Gf.Quatd(*quat_wxyz(Tc[:3, :3] @ USD_CAM_IN_SANDBOX).tolist())
    for c in cams.values():
        c["t"].Set(Gf.Vec3d(*(Tc[:3, 3] + Tc[:3, :3] @ [0.0, c["dy"], 0.0]).tolist()))
        c["q"].Set(q)
    app_utils.update_app(steps=args.settle)
    acc = {s: np.zeros((H_IMG, W_IMG, 3), np.float32) for s in cams}
    for _ in range(args.average):
        app_utils.update_app()
        for s, c in cams.items():
            acc[s] += np.asarray(c["ann"].get_data())[..., :3].astype(np.float32)
    TcL = Tc.copy()
    TcL[:3, 3] = Tc[:3, 3] + Tc[:3, :3] @ [0.0, cams["left"]["dy"], 0.0]
    return ({s: np.clip((a / args.average) @ np.array([0.299, 0.587, 0.114], np.float32), 0, 255).astype(np.uint8)
             for s, a in acc.items()}, TcL)


seeds = [int(s) for s in args.seeds.split(",") if s]
low = {int(s) for s in args.low.split(",") if s}
jobs = [(s, "diffuse") for s in seeds] + [(s, "low") for s in seeds if s in low]
if args.test:
    jobs = jobs[:1]
plan = []
for seed, light in jobs:
    d = np.load(os.path.join(args.traj, f"traj_{seed}_nominal.npz"))
    ends = np.minimum(d["leg_end_idx"], len(d["t"]) - 1)        # export_traj's last leg end can be one past the end
    idx = np.unique(np.r_[np.arange(0, len(d["t"]), args.every), ends])
    plan.append((seed, light, d, idx[:20] if args.test else idx))
tel = Telemetry(kind="isaac", total=int(sum(len(p[3]) for p in plan)), args=vars(args),
                label=f"Isaac D455 stereo for VO, seeds {args.seeds}")
app_utils.play()
app_utils.update_app(steps=10)
meta = {"seeds": seeds, "low": sorted(low), "every": args.every, "image": [W_IMG, H_IMG], "hfov_deg": HFOV,
        "baseline_m": BASELINE, "cam_mount": [list(CAM_XYZ), np.asarray(CAM_R).tolist()], "lights": LIGHTS,
        "average": args.average, "traj": os.path.relpath(args.traj, HERE), "drives": {}}
done, t0, built = 0, time.time(), None
for seed, light, d, idx in plan:
    if built != seed:
        arena = Arena(np.random.default_rng(seed))
        build_terrain(arena, seed)
        built = seed
        app_utils.update_app(steps=10)
    dome.GetIntensityAttr().Set(LIGHTS[light]["dome"])
    sun.GetIntensityAttr().Set(LIGHTS[light]["sun"])
    L, R, camT, baseT = [], [], [], []
    for n, i in enumerate(idx):
        Tb = settled(arena, float(d["x"][i]), float(d["y"][i]), float(d["yaw"][i]))
        imgs, TcL = render(Tb)
        L.append(imgs["left"])
        R.append(imgs["right"])
        camT.append(TcL)
        baseT.append(Tb)
        if n in (0, len(idx) // 2):
            Image.fromarray(np.hstack([imgs["left"], imgs["right"]])).save(
                os.path.join(args.out, f"{seed}_{light}_f{n:03d}.png"))
        done += 1
        tel.progress(done, seed=seed, light=light, frame=n, mean_gray=float(imgs["left"].mean()))
    np.savez_compressed(os.path.join(args.out, f"{seed}_{light}.npz"), left=np.stack(L), right=np.stack(R), idx=idx,
                        cam_T=np.stack(camT), base_T=np.stack(baseT))
    meta["drives"][f"{seed}_{light}"] = {"frames": len(idx), "mean_gray": float(np.mean([g.mean() for g in L]))}
    print(f"[vo] {seed} {light}: {len(idx)} stereo pairs, {time.time() - t0:.0f} s so far", flush=True)
with open(os.path.join(args.out, "meta.json"), "w") as f:
    json.dump(meta, f, indent=1, default=float)
tel.end(seconds=round(time.time() - t0, 1))
print(f"[vo] wrote {args.out} ({time.time() - t0:.0f} s)", flush=True)
app.close()
