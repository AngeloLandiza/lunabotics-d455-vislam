"""Do wheel ruts break visual landmarks? Camera images of the same trip on BP-1-looking terrain, before and after ruts.

Usage (Isaac Sim's Python; run_isaac_landmarks.sh): isaac_landmarks.py [--seed 1] [--trip-seed 2] [--headless]
                                                     [--test] [--out isaac_out/landmarks_seed1]
Terrain: the seed-1 arena height field as one 1 cm mesh with the BP-1-like material of bp1_look.json (matte,
albedo 0.11, tint 1 : 0.66 : 0.31 from guidebook Figure 7, +-15 % 1 cm variation). Camera: a D455-like infrared
imager at robot.mount("d455_link") (848 x 480, 87 deg horizontal, square pixels; mono from the rendered colour), with
a distance_to_image_plane image from the same camera for each pixel's world position. The robot body is in the scene.
Passes over the same trip poses (drive_path.trip_path, the obstacle-avoiding trip of isaac_drive_eval.py):
  A        no ruts;
  B32      after one earlier drive of the same route left 3.2 cm ruts (ruts.rut_depth(robot)), robot settled in them;
  B32cam   the B32 terrain seen from pass A's camera poses (appearance change only, no camera shift);
  B50      the same with 5 cm ruts;
  Acam32   control: no ruts, seen from B32's camera poses (the camera shift alone);
  Arep     control: pass A rendered again (what render noise alone costs).
Each image is the mean of --average renders at the same pose (RTX's per-frame light sampling noise).
Every pass is rendered under two lights (bp1_look.json): "diffuse" (dome only) and "low" (sun 15 deg up + weak dome).
Output: <out>/<pass>_<light>.npz with gray (n, 480, 848) uint8, depth (n, 480, 848) float16, cam_T (n, 4, 4) world-from-
camera (x forward, y left, z up); <out>/meta.json. landmark_eval.py measures features and A-vs-B matches.
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--trip-seed", type=int, default=None)
parser.add_argument("--passes", default="A,Arep,Acam32,B32,B32cam,B50")
parser.add_argument("--lights", default="diffuse,low")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test", action="store_true", help="render one frame per light of pass A only (look check)")
parser.add_argument("--settle", type=int, default=6, help="app updates after each pose change before averaging")
parser.add_argument("--average", type=int, default=16,
                    help="renders averaged per image: RTX samples the lights randomly every frame (no denoiser here), "
                         "and one render carries ~3.6 grey levels of noise in diffuse light (measured 2026-10-12)")
parser.add_argument("--out", default=None)
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

sys.path.insert(0, HERE)
from arena import Arena  # noqa: E402
from drive_path import trip_path, wheel_track_points  # noqa: E402
from robot import Robot  # noqa: E402
from ruts import Ruts, rut_depth, wheel_offsets  # noqa: E402
from telemetry import Telemetry  # noqa: E402

LOOK = json.load(open(os.path.join(HERE, "bp1_look.json")))["values"]
W_IMG, H_IMG, HFOV = 848, 480, 87.0
out_dir = args.out or os.path.join(HERE, "isaac_out", f"landmarks_seed{args.seed}")
os.makedirs(out_dir, exist_ok=True)

# Camera-like rendering, as in sim/isaac_lunar_sim.py: no temporal reconstruction that would smear between poses
# and fixed exposure (auto exposure off) so every pass and frame is exposed the same way
RENDER = [("/rtx/post/aa/op", 0), ("/rtx/directLighting/domeLight/denoisingTechnique", 0),
          ("/rtx/post/motionblur/enabled", False), ("/rtx/directLighting/sampledLighting/enabled", False),
          ("/rtx/ambientOcclusion/enabled", False), ("/rtx/indirectDiffuse/enabled", False),
          ("/rtx/reflections/enabled", False), ("/rtx/translucency/enabled", False),
          ("/rtx/post/histogram/enabled", False)]
for key, val in RENDER:
    carb.settings.get_settings().set(key, val)


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


# ---------------------------------------------------------------- arena, path, ruts of the earlier drive
robot = Robot()
base_arena = Arena(np.random.default_rng(args.seed))
trip_seed = args.trip_seed if args.trip_seed is not None else args.seed + 1
poses = trip_path(base_arena, np.random.default_rng(trip_seed))
WHEELS = np.array([c for c, _, _ in robot.wheels])
X_FRONT, X_REAR = WHEELS[:, 0].max(), WHEELS[:, 0].min()
LAT = wheel_offsets(robot)
H0 = base_arena.h.copy()


def terrain_for(depth):
    """Height field after one earlier drive of the whole route left ruts of this depth (0: none)."""
    base_arena.h = H0.copy()
    if depth > 0:
        r = Ruts(base_arena, depth, width=robot.wheels[0][2])
        r.path(wheel_track_points(poses, len(poses) - 1, X_REAR, X_FRONT), LAT)
        r.apply()
    return base_arena.h.copy()


def settled(pose, h):
    """World-from-robot transform with the wheels resting on height field h (as isaac_drive_ruts.base_pose)."""
    base_arena.h = h
    x, y, yaw = pose
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    wxy = WHEELS[:, :2] @ Rz[:2, :2].T + [x, y]
    gz = base_arena.height(wxy[:, 0], wxy[:, 1])
    sx, sy, z0 = np.linalg.lstsq(np.c_[WHEELS[:, 0], WHEELS[:, 1], np.ones(4)], gz, rcond=None)[0]
    p, r = np.arctan(sx), np.arctan(sy)
    Ry = np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rz @ Ry @ Rx, [x, y, z0]
    return T


# ---------------------------------------------------------------- stage: textured terrain, robot, camera, lights
stage_utils.create_new_stage()
stage = stage_utils.get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
nx, ny = H0.shape
ii, jj = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
a_ = (ii * ny + jj).ravel()
terrain = UsdGeom.Mesh.Define(stage, "/World/Terrain")
terrain.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(2 * len(a_), 3, np.int32)))
terrain.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
    np.stack([a_, a_ + ny, a_ + ny + 1, a_, a_ + ny + 1, a_ + 1], 1).reshape(-1).astype(np.int32)))
terrain.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
terrain_pts = terrain.CreatePointsAttr()
# Albedo per height-field vertex (1 cm apart): diffuse_rgb x (1 + 0.15 u), u uniform in [-1, 1] (bp1_look.json),
# as a vertex primvar read by the shader. (A world-mapped 1 cm UsdUVTexture was tried first, 2026-10-12: the tint
# showed but the fine variation did not reach the image, 0.4 % high-pass contrast, so per-vertex colour is used.)
rng_tex = np.random.default_rng(1000 + args.seed)
factor = 1 + 0.15 * rng_tex.uniform(-1, 1, H0.shape)
diffuse = np.array(LOOK["diffuse_rgb_linear"]["value"])
albedo = (factor.reshape(-1, 1) * diffuse).astype(np.float32)
pv = UsdGeom.PrimvarsAPI(terrain.GetPrim()).CreatePrimvar("bp1Albedo", Sdf.ValueTypeNames.Color3fArray,
                                                           UsdGeom.Tokens.vertex)
pv.Set(Vt.Vec3fArray.FromNumpy(albedo))
tex_path = os.path.join(out_dir, "bp1_albedo_1cm.png")      # the same pattern as an image, for the record
Image.fromarray(np.clip(np.rint(factor.T[::-1] / 1.15 * 255), 0, 255).astype(np.uint8), "L").save(tex_path)
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

CAM_XYZ, CAM_R = robot.mount("d455_link")
FOCAL = 10.0
cam = UsdGeom.Camera.Define(stage, "/World/d455_ir")
cam.CreateFocalLengthAttr(FOCAL)
cam.CreateHorizontalApertureAttr(float(2 * FOCAL * np.tan(np.radians(HFOV / 2))))
cam.CreateVerticalApertureAttr(float(2 * FOCAL * np.tan(np.radians(HFOV / 2)) * H_IMG / W_IMG))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
cx = UsdGeom.Xformable(cam.GetPrim())
cam_t, cam_q = cx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), cx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
rp = rep.create.render_product(cam.GetPath().pathString, (W_IMG, H_IMG))
ann_rgb = rep.AnnotatorRegistry.get_annotator("LdrColor")
ann_rgb.attach(rp)
ann_d = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
ann_d.attach(rp)
USD_CAM_IN_SANDBOX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateAngleAttr(0.53)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(75.0, 0.0, 135.0))   # 15 deg above the horizon
LIGHTS = {"diffuse": {"dome": float(os.environ.get("LM_DOME", 1500)), "sun": 0.0},
          "low": {"dome": float(os.environ.get("LM_LOW_DOME", 300)), "sun": float(os.environ.get("LM_SUN", 250000))}}


def set_light(name):
    dome.GetIntensityAttr().Set(LIGHTS[name]["dome"])
    sun.GetIntensityAttr().Set(LIGHTS[name]["sun"])


dome.CreateIntensityAttr(LIGHTS["diffuse"]["dome"])
sun.CreateIntensityAttr(0.0)

if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = cam.GetPath().pathString       # the window shows what the camera sees


def capture(T_cam):
    cam_t.Set(Gf.Vec3d(*T_cam[:3, 3].tolist()))
    cam_q.Set(Gf.Quatd(*quat_wxyz(T_cam[:3, :3] @ USD_CAM_IN_SANDBOX).tolist()))
    app_utils.update_app(steps=args.settle)
    rgb = np.zeros((H_IMG, W_IMG, 3), np.float32)
    for _ in range(args.average):
        app_utils.update_app()
        rgb += np.asarray(ann_rgb.get_data())[..., :3].astype(np.float32)
    rgb /= args.average
    gray = np.clip(rgb @ np.array([0.299, 0.587, 0.114], np.float32), 0, 255).astype(np.uint8)
    d = np.asarray(ann_d.get_data(), dtype=np.float32)
    return gray, np.where(np.isfinite(d) & (d > 0), d, np.nan).astype(np.float16), rgb.astype(np.uint8)


# ---------------------------------------------------------------- passes
depth32 = rut_depth(robot)
# pass: (rut depth of the terrain shown, rut depth of the terrain the robot settles on for its camera pose)
PASSES = {"A": (0.0, 0.0), "B32": (depth32, depth32), "B32cam": (depth32, 0.0), "B50": (0.05, 0.05),
          "Acam32": (0.0, depth32),     # control: no ruts, camera where B32 had it (the robot sunk in the ruts)
          "Arep": (0.0, 0.0)}           # control: pass A rendered again (the render-noise floor)
todo = [p for p in args.passes.split(",") if p]
lights = [l_ for l_ in args.lights.split(",") if l_]
if args.test:
    todo, poses_run = ["A"], poses[:1]
else:
    poses_run = poses
tel = Telemetry(kind="isaac", total=len(todo) * len(lights) * len(poses_run), args=vars(args),
                label=f"Isaac landmark images, seed {args.seed}, trip {trip_seed}")
app_utils.play()
app_utils.update_app(steps=5)
cam_T_by_pass, done, t0 = {}, 0, time.time()
meta = {"seed": args.seed, "trip_seed": trip_seed, "poses": [list(p) for p in poses_run], "image": [W_IMG, H_IMG],
        "hfov_deg": HFOV, "lights": LIGHTS, "passes": {}, "look": "bp1_look.json", "texture": os.path.basename(tex_path)}
for pname in todo:
    depth, cam_depth = PASSES[pname]
    h_cam = terrain_for(cam_depth)
    h = terrain_for(depth)
    terrain_pts.Set(Vt.Vec3fArray.FromNumpy(np.stack([base_arena.X.ravel(), base_arena.Y.ravel(), h.ravel()], 1)
                                            .astype(np.float32)))
    Ts = []
    for pose in poses_run:
        Tb = settled(pose, h_cam)                          # robot (and camera) settled on the camera's terrain
        Tc = np.eye(4)
        Tc[:3, :3], Tc[:3, 3] = Tb[:3, :3] @ CAM_R, Tb[:3, 3] + Tb[:3, :3] @ np.asarray(CAM_XYZ)
        Ts.append((Tb, Tc))
    cam_T_by_pass[pname] = [t[1] for t in Ts]
    meta["passes"][pname] = {"rut_depth_m": depth, "camera_settled_on_rut_depth_m": cam_depth}
    for lname in lights:
        set_light(lname)
        grays, depths = [], []
        for k, (Tb, Tc) in enumerate(Ts):
            rob_t.Set(Gf.Vec3d(*Tb[:3, 3].tolist()))
            rob_q.Set(Gf.Quatd(*quat_wxyz(Tb[:3, :3]).tolist()))
            g, d, rgb = capture(Tc)
            grays.append(g)
            depths.append(d)
            if k in (0, len(Ts) // 2):
                Image.fromarray(rgb).save(os.path.join(out_dir, f"{pname}_{lname}_f{k:02d}.png"))
            done += 1
            tel.progress(done, pass_=pname, light=lname, frame=k, mean_gray=float(g.mean()))
        np.savez_compressed(os.path.join(out_dir, f"{pname}_{lname}.npz"), gray=np.stack(grays),
                            depth=np.stack(depths), cam_T=np.stack([t[1] for t in Ts]))
        print(f"[landmarks] {pname} {lname}: {len(grays)} images, mean gray {np.mean([g.mean() for g in grays]):.1f}",
              flush=True)
with open(os.path.join(out_dir, "meta.json"), "w") as f:
    json.dump(meta, f, indent=1, default=float)
tel.end(seconds=round(time.time() - t0, 1))
print(f"[landmarks] wrote {out_dir} ({time.time() - t0:.0f} s)", flush=True)
app.close()
