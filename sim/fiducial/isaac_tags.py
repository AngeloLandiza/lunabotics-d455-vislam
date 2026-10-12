"""Start-zone tag test in Isaac Sim: render the rear camera's view of the `four` tag layout from random start poses.

Usage (Isaac Sim's Python; run_isaac_tags.sh): isaac_tags.py [--poses 50] [--seed 0] [--layout two_perpendicular] [--test]
                                               [--out isaac_out/tags_v1] [--headless]
Scene: flat ground (BP-1 albedo and tint from bp1_sandbox/bp1_look.json), the two start-zone walls x = 0 and y = 0 as
grey boxes, tag_start.layouts()[--layout] as real tag36h11 tags (ids 0.. in layout order, black square edge
--tag-size, default the team's 0.1651 m from config_pkg/params/apriltag/tag_params.yaml, Verified) with a one-cell
white margin, and the robot body (team URDF meshes) at each pose. Poses: tag_start's sampler (same seed, the first
--poses of its draws). Two rear cameras at the URDF rear_cam pose, both 1280 x 720 pinhole:
  wide   2.0 rad horizontal field of view, pitched 15 deg down (tag_start's best setup);
  urdf   1.2 rad, 15 deg up (the URDF camera as-is; control).
At each pose the robot then turns in place (5 deg steps, alternating sides, up to 180 deg) until the detector finds a
tag in that camera's image: the Isaac counterpart of tag_start.turn_to_see (2027 16.5 lets the robot move to localize).
Output: <out>/<camera>.npz with gray (n, 720, 1280) uint8 at the drawn heading, poses (n, 3) (x, y, yaw rad) and turn
(n,) in deg (nan: no tag at any heading), <out>/meta.json, sample frames <out>/<camera>_fNN.png.
isaac_tags_eval.py detects the tags and estimates the pose.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SANDBOX = os.path.join(HERE, "..", "bp1_sandbox")
parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--poses", type=int, default=50)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--layout", default="two_perpendicular")
parser.add_argument("--tag-size", type=float, default=0.1651)
parser.add_argument("--tag-height", type=float, default=0.30)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--test", action="store_true", help="render the first pose only (look check)")
parser.add_argument("--settle", type=int, default=4, help="app updates after each pose change before averaging")
parser.add_argument("--average", type=int, default=4, help="renders averaged per image (RTX light-sampling noise)")
parser.add_argument("--turn-step", type=float, default=5.0, help="in-place turn search step, deg (tag_start: 5)")
parser.add_argument("--out", default=os.path.join(HERE, "isaac_out", "tags_v1"))
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": args.headless, "width": 1600, "height": 900})

import carb  # noqa: E402
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt  # noqa: E402

sys.path.insert(0, HERE)
sys.path.insert(0, SANDBOX)
import tag_start as ts  # noqa: E402
from robot import Robot  # noqa: E402
from telemetry import Telemetry  # noqa: E402

ts.PARAMS["tag_size"]["value"] = args.tag_size
ts.PARAMS["tag_height"]["value"] = args.tag_height
LOOK = json.load(open(os.path.join(SANDBOX, "bp1_look.json")))["values"]
W_IMG, H_IMG = ts.W_PX, ts.H_PX
CAMERAS = {"wide": (2.0, -15.0), "urdf": (1.2, 15.0)}       # (hfov rad, pitch deg up)
os.makedirs(args.out, exist_ok=True)

# Camera-like rendering as in bp1_sandbox/isaac_landmarks.py: no temporal smearing between poses, fixed exposure
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


def material(path, rgb=None, texture=None):
    mat = UsdShade.Material.Define(stage, path)
    pbr = UsdShade.Shader.Define(stage, path + "/pbr")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    pbr.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
    pbr.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    if texture:
        st = UsdShade.Shader.Define(stage, path + "/st")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tex = UsdShade.Shader.Define(stage, path + "/tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0.8, 0.8, 0.8, 1.0))   # white paper ~0.8
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    else:
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    return mat


def quad(path, corners, mat, uv=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(corners, np.float32)))
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                           UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray.FromNumpy(np.asarray(uv, np.float32)))
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    return m


def box(path, lo, hi, mat):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(c.GetPrim())
    x.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2).tolist()))
    x.AddScaleOp().Set(Gf.Vec3d(*(hi - lo).tolist()))
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)


# ---------------------------------------------------------------- stage
stage_utils.create_new_stage()
stage = stage_utils.get_current_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

ground = material("/World/Looks/ground", rgb=LOOK["diffuse_rgb_linear"]["value"])
quad("/World/ground", [[-1, -1, 0], [4, -1, 0], [4, 4, 0], [-1, 4, 0]], ground)
wall = material("/World/Looks/wall", rgb=(0.35, 0.35, 0.37))
box("/World/wall_y0", (-0.05, -0.05, 0.0), (4.0, 0.0, 0.8), wall)       # wall y = 0, face at y = 0
box("/World/wall_x0", (-0.05, -0.05, 0.0), (0.0, 4.0, 0.8), wall)       # wall x = 0, face at x = 0

# Tags: tag36h11 image (8 cells incl. the black border) with a one-cell white margin; the black square edge is
# --tag-size, so the textured quad is 10/8 of it. Texture row 0 is the tag's top (UV v = 1).
ARUCO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
CELL_PX = 80
TAGS = ts.layouts()[args.layout]
tag_meta = []
for k, (corners, normal) in enumerate(TAGS):
    img = cv2.aruco.generateImageMarker(ARUCO, k, 8 * CELL_PX, borderBits=1)
    img = cv2.copyMakeBorder(img, CELL_PX, CELL_PX, CELL_PX, CELL_PX, cv2.BORDER_CONSTANT, value=255)
    png = os.path.join(args.out, f"tag36h11_id{k}.png")
    cv2.imwrite(png, img)
    c = corners.mean(0) + normal * 0.001                                   # 1 mm proud of the wall
    full = (corners - corners.mean(0)) * 10 / 8 + c                        # TL, TR, BR, BL seen from the front
    quad(f"/World/tag{k}", full, material(f"/World/Looks/tag{k}", texture=png), uv=[[0, 1], [1, 1], [1, 0], [0, 0]])
    tag_meta.append({"id": k, "centre": corners.mean(0).tolist(), "normal": normal.tolist()})

robot = Robot()
TRIS, _ = robot.triangles(exclude=("lidar3d_link", "d455_link", "rear_cam_link"))
rob_xf = UsdGeom.Xform.Define(stage, "/World/robot")
rm = UsdGeom.Mesh.Define(stage, "/World/robot/body")
rm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(TRIS.reshape(-1, 3).astype(np.float32)))
rm.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(TRIS), 3, np.int32)))
rm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * len(TRIS), dtype=np.int32)))
rm.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
rm.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.62)])
rx = UsdGeom.Xformable(rob_xf.GetPrim())
rob_t, rob_q = rx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble), rx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

dome = UsdLux.DomeLight.Define(stage, "/World/dome")
dome.CreateIntensityAttr(1500.0)                                          # bp1_sandbox "diffuse" light

FOCAL = 10.0
OPT_TO_USD = np.diag([1.0, -1.0, -1.0])           # optical (z forward, y down) -> USD camera (-z forward, y up)
cams = {}
for name, (hfov, pitch) in CAMERAS.items():
    cam = UsdGeom.Camera.Define(stage, f"/World/rear_cam_{name}")
    cam.CreateFocalLengthAttr(FOCAL)
    cam.CreateHorizontalApertureAttr(float(2 * FOCAL * math.tan(hfov / 2)))
    cam.CreateVerticalApertureAttr(float(2 * FOCAL * math.tan(hfov / 2) * H_IMG / W_IMG))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 50.0))
    cx = UsdGeom.Xformable(cam.GetPrim())
    rp = rep.create.render_product(cam.GetPath().pathString, (W_IMG, H_IMG))
    ann = rep.AnnotatorRegistry.get_annotator("LdrColor")
    ann.attach(rp)
    cams[name] = {"t": cx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble),
                  "q": cx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble), "ann": ann, "hfov": hfov, "pitch": pitch,
                  "path": cam.GetPath().pathString}
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = cams["wide"]["path"]          # the window shows the wide rear camera

# ---------------------------------------------------------------- poses: tag_start's sampler
rng = np.random.default_rng(args.seed)
x0, x1, y0, y1 = ts.v("start_zone")
r = ts.v("body_radius")
poses = [(rng.uniform(x0 + r, x1 - r), rng.uniform(y0 + r, y1 - r), rng.uniform(-math.pi, math.pi))
         for _ in range(args.poses)]
if args.test:
    poses = [(1.0, 1.0, math.pi / 2)]           # look check: rear cameras face wall y = 0 and its tag


def place_cameras(pose):
    for c in cams.values():
        ts.set_hfov(c["hfov"])
        ts.set_pitch(c["pitch"])
        R_wc, t_wc = ts.cam_pose(pose)
        c["t"].Set(Gf.Vec3d(*t_wc.tolist()))
        c["q"].Set(Gf.Quatd(*quat_wxyz(R_wc @ OPT_TO_USD).tolist()))


def render(pose):
    """Mean of --average renders per camera at this robot pose: {camera: (rgb float32, gray uint8)}."""
    x, y, yaw = pose
    rob_t.Set(Gf.Vec3d(x, y, 0.0))
    rob_q.Set(Gf.Quatd(*quat_wxyz(ts.rot_z(yaw)).tolist()))
    place_cameras(pose)
    app_utils.update_app(steps=args.settle)
    acc = {n: np.zeros((H_IMG, W_IMG, 3), np.float32) for n in cams}
    for _ in range(args.average):
        app_utils.update_app()
        for n, c in cams.items():
            acc[n] += np.asarray(c["ann"].get_data())[..., :3].astype(np.float32)
    out = {}
    for n in cams:
        rgb = acc[n] / args.average
        out[n] = (rgb, np.clip(rgb @ np.array([0.299, 0.587, 0.114], np.float32), 0, 255).astype(np.uint8))
    return out


DETECTOR = cv2.aruco.ArucoDetector(ARUCO, cv2.aruco.DetectorParameters())


def sees_tag(gray):
    _, ids, _ = DETECTOR.detectMarkers(gray)
    return ids is not None and any(0 <= int(i) < len(TAGS) for i in np.ravel(ids))


tel = Telemetry(kind="isaac", total=len(poses), args=vars(args),
                label=f"Isaac start-zone tags ({args.layout}, {len(poses)} poses)")
app_utils.play()
app_utils.update_app(steps=60)                  # let the tag textures load before the first frame
t0 = time.time()
grays = {n: [] for n in cams}
turns = {n: [] for n in cams}
for k, pose in enumerate(poses):
    imgs = render(pose)
    for n in cams:
        grays[n].append(imgs[n][1])
        if k < 4 or args.test:
            Image.fromarray(imgs[n][0].astype(np.uint8)).save(os.path.join(args.out, f"{n}_f{k:02d}.png"))
    turn = {n: (0.0 if sees_tag(imgs[n][1]) else None) for n in cams}
    for d in np.arange(args.turn_step, 180 + args.turn_step / 2, args.turn_step):
        if all(t is not None for t in turn.values()):
            break
        for sgn in (1, -1):
            imgs = render((pose[0], pose[1], pose[2] + sgn * math.radians(d)))
            for n in cams:
                if turn[n] is None and sees_tag(imgs[n][1]):
                    turn[n] = float(d)
    for n in cams:
        turns[n].append(np.nan if turn[n] is None else turn[n])
    tel.progress(k + 1, pose=k, turn_wide=turn["wide"], turn_urdf=turn["urdf"])
    print(f"[tags] pose {k}: turn wide {turn['wide']}, urdf {turn['urdf']}", flush=True)
for n in cams:
    np.savez_compressed(os.path.join(args.out, f"{n}.npz"), gray=np.stack(grays[n]), poses=np.array(poses),
                        turn=np.array(turns[n], float))
json.dump({"seed": args.seed, "layout": args.layout, "tag_size": args.tag_size, "tag_height": args.tag_height,
           "tags": tag_meta, "cameras": {n: {"hfov": c["hfov"], "pitch_deg_up": c["pitch"]} for n, c in cams.items()},
           "image": [W_IMG, H_IMG], "average": args.average, "light": "dome 1500 (bp1_sandbox diffuse)",
           "poses": [list(p) for p in poses]}, open(os.path.join(args.out, "meta.json"), "w"), indent=1)
tel.end(seconds=round(time.time() - t0, 1))
print(f"[tags] wrote {args.out} ({len(poses)} poses, {time.time() - t0:.0f} s)", flush=True)
app.close()
