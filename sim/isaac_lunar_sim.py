"""Isaac Sim 6.0: tank-drive rover with a down-pitched RealSense D455.

Robots (--robot): team = the EDT Lunabotics rover from the team URDF (sim/team_robot.py, D455 at its 2027 front
mount; default in the arena), jackal = Clearpath Jackal with the D455 on a front mast (default on lunar terrain).

Worlds (--world):
  arena   the Lunabotics Artemis Arena (2027 guidebook; sim/arena.py, sim/arena_scene.py),
          Earth gravity, indoor lighting, random obstacles and start pose per round
  lunar   procedural lunar terrain, lunar gravity

Publishes what the real D455 driver does, so the SLAM stack runs unchanged (sim:=true):
  /camera/infra1|infra2/image_rect_raw + camera_info   stereo pair (with stereo baseline)
  /camera/depth/image_rect_raw + camera_info           depth (32FC1, m)
  /camera/imu (200 Hz, gravity included)   /clock   /ground_truth/odom
  /ground_truth/camera_pose (camera_link in the world frame, for checking SLAM)
  /arena/heightmap (true terrain, 32FC1 rows = x)   /arena/volume ([cut, fill, berm] m^3)
  /l1/points (team robot: Unitree L1 PointCloud2, frame lidar3d_link, ~10 Hz; body returns left in)
  /rear_camera/image_raw + camera_info (team robot: wide rear camera for the start-zone tags, rgb8 1280 x 720)
  /wheel/odom (encoder odometry, frame wheel_odom: over-reads by 1/(1 - slip) like real encoders on loose BP-1)
--realism nominal|heavy (default nominal; off = ideal sensors, rigid ground) adds the BP-1 layers of sim/realism.py:
D455 depth noise / dropouts / dust and IR noise, wheel slip, ruts, dust plumes after digs, the L1 BP-1 layer, and
IMU noise (the sim then publishes the ideal IMU on /sim/imu_ideal; sim/imu_realism_relay.py adds the noise and
republishes /camera/imu, started by run/sim/arena/arena.sh).
drives from /cmd_vel, and digs/dumps where told on /arena/dig ([x, y, radius, depth]).

Run via run/sim/procedural/lunar_terrain.sh or run/sim/arena/arena.sh (Isaac Sim's own Python + internal ROS 2 libraries):  python sim/isaac_lunar_sim.py [--headless]
  --dig-at T     at sim time T, dig a hole at --dig-xy (tests terrain re-mapping)
"""
import argparse
import array
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--world', choices=['arena', 'lunar'], default='lunar')
parser.add_argument('--seed', type=int, default=0, help='terrain / obstacle layout / start pose')
parser.add_argument('--start-heading', default='random', help='arena: N, E, S, W or random')
parser.add_argument('--no-lidar', action='store_true', help='team robot: leave out the Unitree L1')
parser.add_argument('--light', choices=['arena', 'low'], default='arena',
                    help='arena: the hall lights; low: hall lights at 1/4 and a sun 15 deg above the horizon (the VO '
                         'test\'s low light: long shadows, glare)')
parser.add_argument('--no-tags', action='store_true', help='arena: leave out the two start-zone AprilTags')
parser.add_argument('--earlier-trips', type=int, default=0,
                    help='arena, team robot: start with the ruts of N earlier drives across the bed (a used bed)')
parser.add_argument('--realism', choices=['off', 'nominal', 'heavy'], default='nominal',
                    help='BP-1 realism layers (sim/realism.py, sim/bp1_sandbox): off = ideal sensors')
parser.add_argument('--robot', choices=['team', 'jackal'], default=None,
                    help='team: the EDT Lunabotics rover (sim/team_robot.py; default in the arena); '
                         'jackal: Clearpath Jackal (default on lunar terrain)')
parser.add_argument('--view', default='', help='viewport camera prim (arena default: overview)')
parser.add_argument('--dig-at', type=float, default=-1.0, help='sim time (s) of the dig event; <0 = never')
parser.add_argument('--dig-xy', type=float, nargs=2, default=[1.5, 0.8])  # lunar: beside the survey loop's first side
parser.add_argument('--dig-radius', type=float, default=0.4)
parser.add_argument('--dig-depth', type=float, default=0.25)
parser.add_argument('--profile', action='store_true', help='print loop timings')
parser.add_argument('--rt-subframes', type=int, default=2,
                    help='renders per camera frame at the same pose; >1 lets RTX history converge (no ghosting)')
parser.add_argument('--debug-poses', action='store_true', help='print rover vs camera world poses')
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': args.headless, 'width': 1024, 'height': 576,
                     'renderer': 'RaytracedLighting', 'anti_aliasing': 0})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.storage.native import get_assets_root_path  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402

enable_extension('isaacsim.ros2.bridge')
# Camera-like rendering: RTX real-time accumulates across frames (motion blur, temporally
# denoised lighting), which smears a moving image into streaks and breaks visual
# odometry. A global-shutter D455 at 5 ms exposure shows none of that, so turn it off.
import carb  # noqa: E402

RENDER_SETTINGS = [('/rtx/post/aa/op', 0),  # DLSS/TAA reconstruct from past frames; on by default
                   ('/rtx/directLighting/domeLight/denoisingTechnique', 0),
                   ('/rtx/post/motionblur/enabled', False),
                   ('/rtx/directLighting/sampledLighting/enabled', False),
                   ('/rtx/ambientOcclusion/enabled', False),
                   ('/rtx/indirectDiffuse/enabled', False),
                   ('/rtx/reflections/enabled', False),
                   ('/rtx/translucency/enabled', False)]
for key, value in RENDER_SETTINGS:
    carb.settings.get_settings().set(key, value)
app.update()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as A  # noqa: E402
import lunar_terrain  # noqa: E402
from terrain_tiles import TerrainTiles  # noqa: E402

# ---------------------------------------------------------------- config
SIZE, RES = 20.0, 0.05            # lunar terrain extent / mesh spacing (m)
CAM_RES = (640, 400)              # IR render resolution (D455 IR is 16:10)
DEPTH_RES = (640, 400)            # same 16:10 aperture as the IR cameras
ROBOT = args.robot or ('team' if args.world == 'arena' else 'jackal')
D455_USD = '/Isaac/Sensors/RealSense/D455/rsd455.usd'
if ROBOT == 'team':
    import team_robot  # noqa: E402
    ROVER_USD = None                  # imported from the team URDF below (needs the URDF importer extension)
    CAM_MOUNT = team_robot.D455_MOUNT  # 2027 front mount (sim/robot/uic_bot_2027.xacro)
    CAM_PITCH_DEG = team_robot.D455_PITCH_DEG
    WHEEL_RADIUS, TRACK = team_robot.WHEEL_RADIUS, team_robot.TRACK
    SPAWN_DZ = team_robot.BASE_HEIGHT + 0.02   # base_link above the wheel bottoms, plus a 2 cm drop
    SKID_TRACK = 1.5 * TRACK          # effective turning track: ICR factor 1.5 (sim/planner/arena2d.py, Assumed)
else:
    ROVER_USD = '/Isaac/Robots/Clearpath/Jackal/jackal.usd'
    CAM_MOUNT = (0.30, 0.0, 0.42)     # D455 on a front mast: 5 cm ahead of the bumper, ~0.49 m up
    CAM_PITCH_DEG = 40.0              # pitched down so the rover stays out of view
    WHEEL_RADIUS, TRACK = 0.098, 0.37559
    SPAWN_DZ = 0.08                   # base ~6.5 cm above wheel bottoms
    SKID_TRACK = 1.8 * TRACK          # skid steer turns about a wider effective track (wheels scrub sideways)
CACHE = os.path.expanduser('~/.cache/lunabotics_sim')
os.makedirs(CACHE, exist_ok=True)

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0, rendering_dt=1.0 / 200.0)  # 1 step = 1 physics step
stage = world.stage
assets = get_assets_root_path()
scene = UsdPhysics.Scene.Get(stage, world.get_physics_context().prim_path)
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81 if args.world == 'arena' else 1.62)


def place(path, translate, rotate_xyz=(0.0, 0.0, 0.0)):
    """Replace a prim's transform (referenced assets often carry their own xform ops)."""
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translate))
    xf.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*rotate_xyz))


if args.world == 'arena':
    import arena_scene  # noqa: E402
    terrain, H0, _ = arena_scene.build(stage, args.seed)
    xs, ys, H = terrain.xs, terrain.ys, terrain.H
    start_x, start_y, start_yaw = A.start_pose(args.seed, args.start_heading)
    WORLD_FRAME = 'arena'
    if not args.no_tags:
        # Start-zone tags (sim/fiducial, results/isaac_tags_v1: the two-tag layout): tag36h11 ids 0 and 1, 16.51 cm
        # black square (the team's apriltag params, Verified) with a one-cell white margin, centre 0.30 m above the
        # bed, 1.0 m along the two bin walls of the start-zone corner. The walls stand behind the 17 cm ducts, so the
        # tag faces are at y = -0.17 and x = -0.17 (the Python test put the walls at 0).
        import cv2  # noqa: E402
        from pxr import Vt  # noqa: E402
        _aruco = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        TAGS = [(0, (1.0, -A.DUCT_D + 0.001), 90.0), (1, (-A.DUCT_D + 0.001, 1.0), 0.0)]   # id, centre xy, facing
        for tid, (tx, ty), face in TAGS:
            img = cv2.aruco.generateImageMarker(_aruco, tid, 640, borderBits=1)
            img = cv2.copyMakeBorder(img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
            png = os.path.join(CACHE, f'tag36h11_id{tid}.png')
            cv2.imwrite(png, img)
            n = np.array([np.cos(np.radians(face)), np.sin(np.radians(face)), 0.0])
            right = np.array([-n[1], n[0], 0.0])
            c, h = np.array([tx, ty, 0.30]), 0.1651 / 2 * 10 / 8
            up = np.array([0.0, 0.0, 1.0])
            corners = [c - right * h + up * h, c + right * h + up * h, c + right * h - up * h, c - right * h - up * h]
            mat = UsdShade.Material.Define(stage, f'/World/Looks/tag{tid}')
            pbr_ = UsdShade.Shader.Define(stage, f'/World/Looks/tag{tid}/pbr')
            pbr_.CreateIdAttr('UsdPreviewSurface')
            pbr_.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.9)
            st_ = UsdShade.Shader.Define(stage, f'/World/Looks/tag{tid}/st')
            st_.CreateIdAttr('UsdPrimvarReader_float2')
            st_.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
            tex = UsdShade.Shader.Define(stage, f'/World/Looks/tag{tid}/tex')
            tex.CreateIdAttr('UsdUVTexture')
            tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(png)
            tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(0.8, 0.8, 0.8, 1.0))   # paper white
            tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st_.ConnectableAPI(), 'result')
            pbr_.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
            mat.CreateSurfaceOutput().ConnectToSource(pbr_.ConnectableAPI(), 'surface')
            q = UsdGeom.Mesh.Define(stage, f'/World/start_tags/tag{tid}')
            q.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(corners, np.float32)))
            q.CreateFaceVertexCountsAttr([4])
            q.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            q.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            UsdGeom.PrimvarsAPI(q.GetPrim()).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                                         UsdGeom.Tokens.vertex).Set(
                Vt.Vec2fArray.FromNumpy(np.array([[0, 1], [1, 1], [1, 0], [0, 0]], np.float32)))
            UsdShade.MaterialBindingAPI.Apply(q.GetPrim()).Bind(mat)
        print(f'[sim] start-zone tags: ids 0, 1 (36h11, 16.51 cm) at {[t[1] for t in TAGS]}, 0.30 m high', flush=True)
    if args.light == 'low':
        # The VO test's low light (sim/odometry/isaac_vo.py): a sun 15 deg above the horizon, plus the hall lights
        # dimmed to 1/4 so its long shadows and glare dominate
        for prim in stage.Traverse():
            if prim.GetTypeName() in ('RectLight', 'DomeLight', 'SphereLight', 'DiskLight'):
                a = prim.GetAttribute('inputs:intensity')
                if a and a.Get() is not None:
                    a.Set(a.Get() * 0.25)
        sun = UsdLux.DistantLight.Define(stage, '/World/low_sun')
        sun.CreateIntensityAttr(250000.0)
        sun.CreateAngleAttr(0.53)
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(75.0, 0.0, 135.0))
        print('[sim] light: low sun (15 deg up), hall lights at 1/4', flush=True)
else:
    terrain = None
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    WORLD_FRAME = 'world'
print(f'[sim] world={args.world} start=({start_x:.2f}, {start_y:.2f}) heading={np.degrees(start_yaw):.0f} deg',
      flush=True)

# ---------------------------------------------------------------- lunar terrain
if args.world == 'lunar':
    X, Y, H_lunar = lunar_terrain.heightmap(SIZE, RES, args.seed)
    tex_path = os.path.join(CACHE, f'regolith_v2_{args.seed}.png')
    if not os.path.exists(tex_path):
        from PIL import Image
        Image.fromarray(lunar_terrain.albedo(4096, args.seed)).save(tex_path)

    # Regolith-like friction so the tank drive has traction in lunar gravity.
    ground = UsdShade.Material.Define(stage, '/World/Looks/regolith_physics')
    ground_phys = UsdPhysics.MaterialAPI.Apply(ground.GetPrim())
    ground_phys.CreateStaticFrictionAttr(0.8)
    ground_phys.CreateDynamicFrictionAttr(0.7)
    ground_phys.CreateRestitutionAttr(0.0)


    def regolith_material(path, texture, tint=1.0):
        mat = UsdShade.Material.Define(stage, path)
        pbr = UsdShade.Shader.Define(stage, path + '/pbr')
        pbr.CreateIdAttr('UsdPreviewSurface')
        pbr.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.95)
        reader = UsdShade.Shader.Define(stage, path + '/st')
        reader.CreateIdAttr('UsdPrimvarReader_float2')
        reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
        tex = UsdShade.Shader.Define(stage, path + '/tex')
        tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(texture)
        tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
        tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
        tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(tint, tint, tint, 1))
        tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
        pbr.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
        mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), 'surface')
        return mat


    terrain = TerrainTiles(stage, '/World/terrain', X[:, 0], Y[0, :], H_lunar, tile=40, uv_size=SIZE / 4,
                           material=regolith_material('/World/Looks/regolith', tex_path), physics_material=ground)
    xs, ys, H = terrain.xs, terrain.ys, terrain.H
    H0 = H.copy()
    rock_mat = regolith_material('/World/Looks/rock', tex_path, tint=0.7)


    height_at = terrain.height_at
    rng = np.random.default_rng(args.seed)
    for k, (x, y, r) in enumerate(lunar_terrain.rocks(SIZE, args.seed)):
        rock = UsdGeom.Sphere.Define(stage, f'/World/rocks/rock_{k}')
        rock.CreateRadiusAttr(1.0)
        xf = UsdGeom.Xformable(rock)
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, height_at(x, y) + 0.2 * r))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rng.uniform(0, 360, 3)))
        xf.AddScaleOp().Set(Gf.Vec3f(r * rng.uniform(0.8, 1.3), r * rng.uniform(0.8, 1.3), r * rng.uniform(0.5, 0.8)))
        UsdPhysics.CollisionAPI.Apply(rock.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(rock.GetPrim()).Bind(rock_mat)

    # Sun: low (35 deg) so craters and rocks cast shadows; faint fill (no atmosphere).
    sun = UsdLux.DistantLight.Define(stage, '/World/sun')
    sun.CreateIntensityAttr(700.0)
    sun.CreateAngleAttr(0.53)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, 30.0))
    fill = UsdLux.DomeLight.Define(stage, '/World/fill')
    fill.CreateIntensityAttr(60.0)
    fill.CreateColorAttr(Gf.Vec3f(0.8, 0.85, 1.0))

# ---------------------------------------------------------------- rover + D455
ROVER = '/World/rover'
add_reference_to_stage(team_robot.usd_path() if ROBOT == 'team' else assets + ROVER_USD, ROVER)
ground_z = terrain.height_at(start_x, start_y) if terrain is not None else 0.0
place(ROVER, (start_x, start_y, ground_z + SPAWN_DZ), (0.0, 0.0, float(np.degrees(start_yaw))))
if ROBOT == 'team':
    team_robot.fix_physics(stage, ROVER)
print(f'[sim] robot={ROBOT} d455 mount={CAM_MOUNT} pitch={CAM_PITCH_DEG} deg', flush=True)
app.update()


def find_prim(root, name_contains):
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if p.startswith(root) and name_contains.lower() in prim.GetName().lower():
            return p
    raise RuntimeError(f'no prim containing "{name_contains}" under {root}')


BASE = find_prim(ROVER, 'base_link')
sensor_prims = [p.GetPath() for p in stage.Traverse() if str(p.GetPath()).startswith(ROVER)
                and any(k in p.GetName().lower() for k in ('lidar', 'laser', 'sick', 'lms1'))]
for path in sensor_prims:
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsActive():
        prim.SetActive(False)
CAM = BASE + '/d455'
add_reference_to_stage(assets + D455_USD, CAM)
place(CAM, CAM_MOUNT, (0.0, CAM_PITCH_DEG, 0.0))  # +pitch about Y tilts +X (forward) down
# The camera rides on the rover's base_link rigid body: drop any physics on the asset
# itself (a nested rigid body would break the articulation; the IMU reads base_link).
for prim in stage.Traverse():
    if str(prim.GetPath()).startswith(CAM):
        for api in (UsdPhysics.RigidBodyAPI, UsdPhysics.CollisionAPI, UsdPhysics.MassAPI):
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
app.update()

REAR = None
if ROBOT == 'team':
    # Wide rear camera for the start-zone tags (sim/fiducial: 2.0 rad horizontal field of view, 1280 x 720, pitched
    # 15 deg down at the xacro rear_cam position; the xacro has 1.2 rad and 15 deg up, which saw no tag from the zone)
    REAR = BASE + '/rear_cam'
    REAR_RES, REAR_HFOV = (1280, 720), 2.0
    rc = UsdGeom.Camera.Define(stage, REAR)
    rc.CreateFocalLengthAttr(10.0)
    rc.CreateHorizontalApertureAttr(float(2 * 10.0 * np.tan(REAR_HFOV / 2)))
    rc.CreateVerticalApertureAttr(float(2 * 10.0 * np.tan(REAR_HFOV / 2) * REAR_RES[1] / REAR_RES[0]))
    rc.CreateClippingRangeAttr(Gf.Vec2f(0.02, 30.0))
    _c, _s = np.cos(np.radians(15.0)), np.sin(np.radians(15.0))
    R_bl = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1.0]]) @ np.array([[_c, 0, _s], [0, 1, 0], [-_s, 0, _c]])  # yaw pi, 15 deg down
    R_usd = R_bl @ np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # USD camera: -Z forward, +Y up
    m_ = np.eye(4)
    m_[:3, :3], m_[3, :3] = R_usd.T, team_robot.REAR_CAM_MOUNT[0]
    rxf = UsdGeom.Xformable(rc.GetPrim())
    rxf.ClearXformOpOrder()
    rxf.AddTransformOp().Set(Gf.Matrix4d(m_.tolist()))

LEFT = find_prim(CAM, 'Left')
RIGHT = find_prim(CAM, 'Right')
DEPTH = find_prim(CAM, 'Depth')
# Isaac's IMU reads its parent rigid body, and the asset's IMU sits under the (now
# non-rigid) D455 prim, so its gyro would read zero. Recreate it directly on base_link
# at the same pose and deactivate the asset's copy.
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema  # noqa: E402

asset_imu = find_prim(CAM, 'Imu')
xcache = UsdGeom.XformCache()
imu_in_base = (xcache.GetLocalToWorldTransform(stage.GetPrimAtPath(asset_imu))
               * xcache.GetLocalToWorldTransform(stage.GetPrimAtPath(BASE)).GetInverse())
IMU = BASE + '/d455_imu'
IsaacSensorSchema.IsaacImuSensor.Define(stage, IMU)
imu_xf = UsdGeom.Xformable(stage.GetPrimAtPath(IMU))
imu_xf.ClearXformOpOrder()
imu_xf.AddTransformOp().Set(imu_in_base)
stage.GetPrimAtPath(asset_imu).SetActive(False)
WHEEL_PATHS = {str(p.GetPath()).split('/')[-1]: str(p.GetPath()) for p in stage.Traverse()
               if str(p.GetPath()).startswith(ROVER) and p.IsA(UsdPhysics.RevoluteJoint) and 'wheel' in p.GetName()}
WHEELS = sorted(WHEEL_PATHS)
LEFT_WHEELS = [w for w in WHEELS if 'left' in w]
RIGHT_WHEELS = [w for w in WHEELS if 'right' in w]
for w in WHEELS:
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(WHEEL_PATHS[w]), 'angular')
    print(f'[sim] drive {w}: stiffness={drive.GetStiffnessAttr().Get()} damping={drive.GetDampingAttr().Get()} '
          f'maxForce={drive.GetMaxForceAttr().Get()}', flush=True)
print(f'[sim] base={BASE}\n[sim] left={LEFT}\n[sim] right={RIGHT}\n[sim] depth={DEPTH}\n'
      f'[sim] imu={IMU}\n[sim] wheels L={LEFT_WHEELS} R={RIGHT_WHEELS}', flush=True)

# Unitree L1 (team robot): RTX LiDAR at the xacro's front mount, upside down with its axis 45 deg forward of straight
# down. Its 360 x 90 deg field (a hemisphere about the sensor's +z) is sampled as a fixed rotary grid with the L1's
# point rate, 43200/s (Verified, L1 manual; the real pattern is non-repetitive): 18 elevation rings x 240 azimuths per
# 0.1 s frame. minDistBetweenEchosM: the schema default 0.4 m silently drops every return under 0.4 m (sandbox,
# 2026-10-12). Isaac gives ideal hits; the sandbox's BP-1 LiDAR layer (bp1_sandbox/sensors.Lidar('l1').apply) turns
# them into returns: 0.10 reflectivity at 905 nm vs the L1's return-power threshold, the 0.05 m blind zone, 2 cm range
# noise and dust attenuation (realism.DustPlumes optical depth).
LIDAR = None
if ROBOT == 'team' and not args.no_lidar:
    from pxr import Vt  # noqa: E402,F811
    from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor  # noqa: E402
    L1_HZ, L1_EL, L1_AZ = 10, np.arange(2.5, 90.0, 5.0), 240
    L1_ATTRS = {
        'omni:sensor:Core:scanType': 'ROTARY',
        'omni:sensor:Core:numberOfEmitters': len(L1_EL), 'omni:sensor:Core:numberOfChannels': len(L1_EL),
        'omni:sensor:Core:scanRateBaseHz': L1_HZ, 'omni:sensor:Core:patternFiringRateHz': L1_HZ * L1_AZ,
        'omni:sensor:Core:farRangeM': 30.0, 'omni:sensor:Core:nearRangeM': 0.01,
        'omni:sensor:Core:minDistBetweenEchosM': 0.01, 'omni:sensor:Core:maxReturns': 1,
        'omni:sensor:Core:rayType': 'IDEALIZED', 'omni:sensor:Core:rangeAccuracyM': 0.0,
        'omni:sensor:Core:azimuthErrorStd': 0.0, 'omni:sensor:Core:elevationErrorStd': 0.0,
        'omni:sensor:Core:rangeResolutionM': 0.001,
        'omni:sensor:Core:emitterState:s001:azimuthDeg': Vt.FloatArray([0.0] * len(L1_EL)),
        'omni:sensor:Core:emitterState:s001:elevationDeg': Vt.FloatArray(L1_EL.tolist()),
        'omni:sensor:Core:emitterState:s001:fireTimeNs': Vt.UIntArray([0] * len(L1_EL)),
        'omni:sensor:Core:emitterState:s001:channelId': Vt.UIntArray(list(range(1, len(L1_EL) + 1))),
    }
    LIDAR = BASE + '/l1'
    l1 = Lidar(LIDAR, attributes=L1_ATTRS, tick_rate=L1_HZ, aux_output_level='FULL')
    (lx, ly, lz), (lr, lp, lyaw) = team_robot.LIDAR_MOUNT
    place(LIDAR, (lx, ly, lz), tuple(float(np.degrees(a)) for a in (lr, lp, lyaw)))   # URDF rpy = rotateXYZ
    l1_sensor = LidarSensor(l1, annotators=['generic-model-output'])
    print(f'[sim] lidar={LIDAR} (Unitree L1, {len(L1_EL) * L1_AZ} points per {1000 // L1_HZ} ms frame)', flush=True)

# ---------------------------------------------------------------- ROS 2 graphs
annotators = {}
rear_ann = None
if REAR:
    rear_ann = rep.AnnotatorRegistry.get_annotator('LdrColor')
    rear_ann.attach(rep.create.render_product(REAR, REAR_RES))
for name, prim, res, kind in [('infra1', LEFT, CAM_RES, 'LdrColor'), ('infra2', RIGHT, CAM_RES, 'LdrColor'),
                              ('depth', DEPTH, DEPTH_RES, 'distance_to_image_plane')]:
    ann = rep.AnnotatorRegistry.get_annotator(kind)
    ann.attach(rep.create.render_product(prim, res))
    annotators[name] = ann

keys = og.Controller.Keys
# /clock once per app update. Camera streams are published from the main loop below,
# once per rendered frame with one shared stamp (Isaac's camera helpers re-publish each
# frame per render product with fresh camera_info stamps, which breaks exact sync).
og.Controller.edit(
    {'graph_path': '/ROS/clock', 'evaluator_name': 'execution'},
    {keys.CREATE_NODES: [
        ('tick', 'omni.graph.action.OnPlaybackTick'),
        ('ctx', 'isaacsim.ros2.bridge.ROS2Context'),
        ('sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
    ],
     keys.CONNECT: [
        ('tick.outputs:tick', 'clock.inputs:execIn'),
        ('ctx.outputs:context', 'clock.inputs:context'),
        ('sim_time.outputs:simulationTime', 'clock.inputs:timeStamp'),
    ]})

# OnPhysicsStep graphs must run on demand (ticked by each 200 Hz physics step).
og.Controller.edit(
    {'graph_path': '/ROS/physics', 'evaluator_name': 'execution',
     'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {keys.CREATE_NODES: [
        ('step', 'isaacsim.core.nodes.OnPhysicsStep'),
        ('ctx', 'isaacsim.ros2.bridge.ROS2Context'),
        ('sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('read_imu', 'isaacsim.sensors.physics.IsaacReadIMU'),
        ('imu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
        ('twist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
        ('odom', 'isaacsim.core.nodes.IsaacComputeOdometry'),
        ('gt', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
    ],
     keys.SET_VALUES: [
        ('read_imu.inputs:imuPrim', [Sdf.Path(IMU)]), ('read_imu.inputs:readGravity', True),
        ('imu.inputs:topicName', '/camera/imu' if args.realism == 'off' else '/sim/imu_ideal'),
        ('imu.inputs:frameId', 'camera_imu_optical_frame'),
        ('imu.inputs:publishOrientation', False),
        ('twist.inputs:topicName', '/cmd_vel'),
        ('odom.inputs:chassisPrim', [Sdf.Path(BASE)]),
        ('gt.inputs:topicName', '/ground_truth/odom'),
        ('gt.inputs:odomFrameId', 'gt_world'), ('gt.inputs:chassisFrameId', 'gt_base'),
    ],
     keys.CONNECT: [
        ('step.outputs:step', 'read_imu.inputs:execIn'),
        ('read_imu.outputs:execOut', 'imu.inputs:execIn'),
        ('ctx.outputs:context', 'imu.inputs:context'),
        ('sim_time.outputs:simulationTime', 'imu.inputs:timeStamp'),
        ('read_imu.outputs:angVel', 'imu.inputs:angularVelocity'),
        ('read_imu.outputs:linAcc', 'imu.inputs:linearAcceleration'),
        ('read_imu.outputs:orientation', 'imu.inputs:orientation'),
        ('step.outputs:step', 'twist.inputs:execIn'),
        ('ctx.outputs:context', 'twist.inputs:context'),
        ('step.outputs:step', 'odom.inputs:execIn'),
        ('odom.outputs:execOut', 'gt.inputs:execIn'),
        ('ctx.outputs:context', 'gt.inputs:context'),
        ('sim_time.outputs:simulationTime', 'gt.inputs:timeStamp'),
        ('odom.outputs:position', 'gt.inputs:position'),
        ('odom.outputs:orientation', 'gt.inputs:orientation'),
        ('odom.outputs:linearVelocity', 'gt.inputs:linearVelocity'),
        ('odom.outputs:angularVelocity', 'gt.inputs:angularVelocity'),
    ]})


# Static camera TFs from the asset geometry. camera_link sits at the left IR camera
# like the real D4xx: x = view direction, y = left, z = up. Optical frames: z = view,
# x = right, y = down. The IMU frame uses the IMU prim's own axes.
def world_frame(path):
    m = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0))
    axes = m[:3, :3] / np.linalg.norm(m[:3, :3], axis=1, keepdims=True)  # rows = local x,y,z in world
    return axes, m[3, :3]


def quat_xyzw(R):
    """Rotation matrix -> quaternion (x, y, z, w), stable for 180-degree rotations (Shepperd)."""
    t = np.trace(R)
    if t > 0:
        k = 2 * np.sqrt(1 + t)
        return [(R[2, 1] - R[1, 2]) / k, (R[0, 2] - R[2, 0]) / k, (R[1, 0] - R[0, 1]) / k, k / 4]
    i = int(np.argmax(np.diag(R)))
    j, m = (i + 1) % 3, (i + 2) % 3
    k = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[m, m])
    q = np.zeros(4)
    q[i], q[j], q[m], q[3] = k / 4, (R[j, i] + R[i, j]) / k, (R[m, i] + R[i, m]) / k, (R[m, j] - R[j, m]) / k
    return list(q)


def camera_axes(path, optical):
    ax, pos = world_frame(path)
    x, y, z = ax  # USD camera: looks along -Z, +Y up
    cols = [x, -y, -z] if optical else [-z, -x, y]
    return np.stack(cols, 1), pos


R_link, p_link = camera_axes(LEFT, optical=False)
frames = {'camera_infra1_optical_frame': camera_axes(LEFT, True),
          'camera_infra2_optical_frame': camera_axes(RIGHT, True),
          'camera_depth_optical_frame': camera_axes(DEPTH, True)}
imu_ax, imu_pos = world_frame(IMU)
frames['camera_imu_optical_frame'] = (imu_ax.T, imu_pos)
if REAR:
    frames['rear_camera_optical_frame'] = camera_axes(REAR, True)
if ROBOT == 'team':
    # base_link as a CHILD of camera_link (static), so it does not clash with the SLAM launch's odom -> camera_link
    # (base_frame camera_link); the autonomy nodes reach the other sensor frames through it
    base_ax, base_pos = world_frame(BASE)
    frames['base_link'] = (base_ax.T, base_pos)
if LIDAR:
    l1_ax, l1_pos = world_frame(LIDAR)
    frames['lidar3d_link'] = (l1_ax.T, l1_pos)       # the L1's own axes (columns = sensor x, y, z in the world)
static_tfs = []  # (child, translation, quaternion xyzw); published via rclpy below
for child, (R, pos) in frames.items():
    rel_R = R_link.T @ R
    rel_t = R_link.T @ (pos - p_link)
    static_tfs.append((child, rel_t, quat_xyzw(rel_R)))
    print(f'[sim] tf camera_link -> {child}: t={np.round(rel_t, 4)}', flush=True)

# Lock every camera against viewport navigation: looking through the robot's D455 in the
# GUI and orbiting would otherwise move the sensor itself (and corrupt SLAM's input).
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Camera) and prim.GetName() != 'overview':   # overview stays free to orbit
        prim.CreateAttribute('omni:kit:cameraLock', Sdf.ValueTypeNames.Bool).Set(True)

# ---------------------------------------------------------------- run
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport  # noqa: E402
    view = args.view or ('/World/arena/cameras/overview' if args.world == 'arena' else '')
    if view:
        get_active_viewport().camera_path = view
world.reset()
for key, value in RENDER_SETTINGS:  # re-apply: render products / reset can restore defaults
    carb.settings.get_settings().set(key, value)
print(f"[sim] /rtx/post/aa/op = {carb.settings.get_settings().get('/rtx/post/aa/op')}", flush=True)
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

art_root = next(str(p.GetPath()) for p in stage.Traverse()
                if str(p.GetPath()).startswith(ROVER) and p.HasAPI(UsdPhysics.ArticulationRootAPI))
rover = SingleArticulation(art_root)
rover.initialize()
left_idx = [rover.get_dof_index(w) for w in LEFT_WHEELS]
right_idx = [rover.get_dof_index(w) for w in RIGHT_WHEELS]
twist_lin = og.Controller.attribute('/ROS/physics/twist.outputs:linearVelocity')
twist_ang = og.Controller.attribute('/ROS/physics/twist.outputs:angularVelocity')

import rclpy  # noqa: E402  (Isaac's bundled Humble rclpy, loaded by the ROS 2 bridge extension)
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

rclpy.init()
ros = rclpy.create_node('isaac_d455')
tf_static = ros.create_publisher(TFMessage, '/tf_static',
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
tf_msg = TFMessage()
for child, t, q in static_tfs:
    ts = TransformStamped()
    ts.header.frame_id, ts.child_frame_id = 'camera_link', child
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = map(float, t)
    ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = map(float, q)
    tf_msg.transforms.append(ts)
tf_static.publish(tf_msg)
pubs = {n: (ros.create_publisher(Image, f'/camera/{n}/image_rect_raw', 5),
            ros.create_publisher(CameraInfo, f'/camera/{n}/camera_info', 5)) for n in annotators}
frame_ids = {'infra1': 'camera_infra1_optical_frame', 'infra2': 'camera_infra2_optical_frame',
             'depth': 'camera_depth_optical_frame', 'rear': 'rear_camera_optical_frame'}
if REAR:
    rear_pubs = (ros.create_publisher(Image, '/rear_camera/image_raw', 2),
                 ros.create_publisher(CameraInfo, '/rear_camera/camera_info', 2))
baseline = float(np.linalg.norm(world_frame(RIGHT)[1] - world_frame(LEFT)[1]))
import realism  # noqa: E402
dust = realism.DustPlumes(args.realism) if args.realism != 'off' else None   # plumes after each dig / dump
d455r = realism.D455Realism(args.realism, np.random.default_rng(args.seed + 303))
slip = realism.Slip(args.realism, np.random.default_rng(args.seed + 404))
wheel_odom_pub = ros.create_publisher(Odometry, '/wheel/odom', 20)
wheel_odom = np.array([start_x, start_y, start_yaw], float)   # encoder dead reckoning, starts at the true pose
# depth-camera pixel rays in its optical frame (for the dust optical depth per pixel)
_u, _v = np.meshgrid(np.arange(DEPTH_RES[0]) + 0.5, np.arange(DEPTH_RES[1]) + 0.5)
_fd = UsdGeom.Camera(stage.GetPrimAtPath(DEPTH)).GetFocalLengthAttr().Get() / \
    UsdGeom.Camera(stage.GetPrimAtPath(DEPTH)).GetHorizontalApertureAttr().Get() * DEPTH_RES[0]
DEPTH_RAYS = np.stack([(_u - DEPTH_RES[0] / 2) / _fd, (_v - DEPTH_RES[1] / 2) / _fd, np.ones_like(_u)], -1).reshape(-1, 3)

# Wheel ruts (arena, team robot): realism.Ruts presses each wheel's 0.1 m patch into the bed to the Bekker static
# sinkage of the 74.1 kg rover on 4 wheels of 0.3302 m (about 3.2 cm, lunar-soil values), with side berms, whenever a
# wheel has moved 2 cm; touched tiles are re-meshed (render and collision), so the rover sits in its own tracks.
ruts = None
if args.world == 'arena' and ROBOT == 'team':
    WHEEL_XY = [(0.275, 0.27), (0.275, -0.27), (-0.18, 0.27), (-0.18, -0.27)]     # xacro wheel joints
    RUT_DEPTH = realism.rut_depth(65.0 + 4 * 2.27, 4, 0.1, 2 * WHEEL_RADIUS)
    ruts = realism.Ruts(args.realism, xs, ys, H0, RUT_DEPTH, 0.1, WHEEL_XY)
    print(f'[sim] ruts: {"on" if ruts.on else "off"}, depth {RUT_DEPTH * 100:.1f} cm', flush=True)
    if ruts.on and args.earlier_trips > 0:
        # earlier drives from the start side across the obstacle zone to the far side (sandbox ruts.earlier_trips
        # pattern, Assumed), each leg clear of rocks and craters (|H0| > 5 cm) by the body half-width + 5 cm
        rng_t = np.random.default_rng(args.seed + 202)

        def corridor_clear(a, b, half=0.41):
            n = int(np.hypot(*(b - a)) / 0.05) + 2
            for f in np.linspace(0, 1, n):
                p = a + f * (b - a)
                i0, i1 = np.searchsorted(xs, [p[0] - half, p[0] + half])
                j0, j1 = np.searchsorted(ys, [p[1] - half, p[1] + half])
                if np.abs(H0[i0:i1, j0:j1]).max(initial=0.0) > 0.05:
                    return False
            return True
        trips, tries = [], 0
        while len(trips) < args.earlier_trips and tries < 20000:
            tries += 1
            W = ys[-1]
            pts = ([np.array([rng_t.uniform(0.5, 1.8), rng_t.uniform(0.6, W - 0.6)])]
                   + [np.array([x, rng_t.uniform(0.6, W - 0.6)]) for x in np.sort(rng_t.uniform(2.5, 4.4, rng_t.integers(1, 4)))]
                   + [np.array([rng_t.uniform(4.6, xs[-1] - 0.6), rng_t.uniform(0.6, W - 0.6)])])
            if all(corridor_clear(pts[k], pts[k + 1]) for k in range(len(pts) - 1)):
                trips.append(pts)
        box = [10 ** 9, -1, 10 ** 9, -1]
        for pts in trips:
            ruts.last = None
            for k in range(len(pts) - 1):
                a, b = pts[k], pts[k + 1]
                yaw = float(np.arctan2(*(b - a)[::-1]))
                for f in np.linspace(0, 1, int(np.hypot(*(b - a)) / 0.02) + 2):
                    bb = ruts.stamp(H, *(a + f * (b - a)), yaw)
                    if bb:
                        box = [min(box[0], bb[0]), max(box[1], bb[1]), min(box[2], bb[2]), max(box[3], bb[3])]
        ruts.last = None
        if box[1] > 0:
            terrain.update(*box)
        print(f'[sim] ruts of {len(trips)} earlier trips stamped', flush=True)
if LIDAR:
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data  # noqa: E402
    cloud_pub = ros.create_publisher(PointCloud2, '/l1/points', 5)
    l1_rng = np.random.default_rng(args.seed + 101)
    l1_last = [None]
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bp1_sandbox'))
    import sensors as sb_sensors  # noqa: E402  (numpy only)
    l1_model = sb_sensors.Lidar('l1')
    L1_R = None                     # base-from-sensor rotation, set at the first frame
    (L1_T, _) = team_robot.LIDAR_MOUNT
    BODY_BOX = ((-0.45, 0.57), (-0.37, 0.37), (-0.28, 0.32))   # base_link frame, body.stl extents + 1-2 cm


def ground_normals(P):
    """Terrain normals at world points P (N, 3) from the heightmap (H rows = x, cols = y), nan off the bed."""
    i = np.clip(np.rint((P[:, 0] - xs[0]) / (xs[1] - xs[0])).astype(int), 1, len(xs) - 2)
    j = np.clip(np.rint((P[:, 1] - ys[0]) / (ys[1] - ys[0])).astype(int), 1, len(ys) - 2)
    gx = (H[i + 1, j] - H[i - 1, j]) / (xs[2] - xs[0])
    gy = (H[i, j + 1] - H[i, j - 1]) / (ys[2] - ys[0])
    n = np.stack([-gx, -gy, np.ones_like(gx)], -1)
    return n / np.linalg.norm(n, axis=1, keepdims=True), np.abs(P[:, 2] - H[i, j]) < 0.03


def publish_wheel_odom(stamp):
    """Encoder odometry: mean measured wheel speed per side / (1 - slip), integrated with the calibrated effective
    track (SKID_TRACK) over this camera frame."""
    qd = np.asarray(rover.get_joint_velocities(), float)
    s_ = slip.ratio(world.current_time)
    vl = float(np.mean(qd[left_idx])) * WHEEL_RADIUS / (1 - s_)
    vr = float(np.mean(qd[right_idx])) * WHEEL_RADIUS / (1 - s_)
    v_, w_ = (vl + vr) / 2, (vr - vl) / SKID_TRACK
    dt = SUBSTEPS / 200.0
    wheel_odom[2] += w_ * dt / 2
    wheel_odom[0] += v_ * dt * np.cos(wheel_odom[2])
    wheel_odom[1] += v_ * dt * np.sin(wheel_odom[2])
    wheel_odom[2] += w_ * dt / 2
    msg = Odometry()
    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'wheel_odom', 'base_link'
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(wheel_odom[0]), float(wheel_odom[1])
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = float(np.sin(wheel_odom[2] / 2)), float(np.cos(wheel_odom[2] / 2))
    msg.twist.twist.linear.x, msg.twist.twist.angular.z = v_, w_
    wheel_odom_pub.publish(msg)


def publish_lidar(stamp):
    """The newest complete L1 frame as PointCloud2 (x, y, z float32 in lidar3d_link), noise and blind zone applied."""
    data, _ = l1_sensor.get_data('generic-model-output')
    if data is None:
        return
    g = parse_generic_model_output_data(data)
    if g.numElements == 0 or g.timestampNs == l1_last[0]:
        return
    l1_last[0] = g.timestampNs
    hit = (np.asarray(g.flags) & 64) > 0
    az, el = np.radians(np.asarray(g.x, float)), np.radians(np.asarray(g.y, float))
    d_s = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)    # sensor frame
    t = np.where(hit, np.asarray(g.z, float), np.nan)
    if args.realism == 'off':
        ok = np.isfinite(t)
        pts = (d_s[ok] * t[ok, None]).astype(np.float32)
    else:
        q = np.asarray(g.frameEnd.orientation, float)                         # x, y, z, w
        R_ws = quat_wxyz_to_R(q[[3, 0, 1, 2]])
        origin = np.asarray(g.frameEnd.posM, float)
        dirs = d_s @ R_ws.T
        P = origin + np.nan_to_num(t)[:, None] * dirs
        n_g, on_ground = ground_normals(P)
        normals = np.where(on_ground[:, None], n_g, -dirs)                   # off the bed: normal incidence
        global L1_R
        if L1_R is None:
            pos, qb = rover.get_world_pose()
            L1_R = quat_wxyz_to_R(np.asarray(qb, float)).T @ R_ws
        Pb = d_s * np.nan_to_num(t)[:, None] @ L1_R.T + L1_T                  # hits in base_link
        is_body = np.isfinite(t) & np.all([(Pb[:, k] > lo) & (Pb[:, k] < hi) for k, (lo, hi) in enumerate(BODY_BOX)], 0)
        tau = dust.optical_depth(origin, dirs, np.nan_to_num(t), world.current_time) if dust is not None else 0.0
        m = {'tau_hit': np.where(np.isfinite(t), tau, 0.0), 'samples': None, 'step': 0.02}
        out = l1_model.apply(origin, dirs, t, normals, m, l1_rng, dust_on=False, cos_override=np.ones(len(t)),
                             is_body=is_body)
        ok = out['valid']
        pts = (d_s[ok] * out['r'][ok, None]).astype(np.float32)
    msg = PointCloud2()
    msg.header.stamp, msg.header.frame_id = stamp, 'lidar3d_link'
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
    msg.data = array.array('B', pts.tobytes())
    cloud_pub.publish(msg)

# Ground truth + terrain editing (test harness only; SLAM never subscribes to these).
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
gt_cam_pub = ros.create_publisher(PoseStamped, '/ground_truth/camera_pose', 10)
gt_base_pub = ros.create_publisher(PoseStamped, '/ground_truth/base_pose', 10)
height_pub = ros.create_publisher(Image, '/arena/heightmap', latched)
volume_pub = ros.create_publisher(Float32MultiArray, '/arena/volume', latched)
dig_queue = []
ros.create_subscription(Float32MultiArray, '/arena/dig', lambda m: dig_queue.append(list(m.data)[:4]), 50)
# camera_link pose in base_link (fixed): lets the camera's GT pose follow the physics base pose.
base_m = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(BASE)).ComputeLocalToWorldTransform(0)).T
cam_m = np.eye(4)
cam_m[:3, :3], cam_m[:3, 3] = R_link, p_link
T_base_cam = np.linalg.inv(base_m) @ cam_m


def quat_wxyz_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_msg(T, stamp):
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = stamp, WORLD_FRAME
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, T[:3, 3])
    qx, qy, qz, qw = quat_xyzw(T[:3, :3])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
        float(qx), float(qy), float(qz), float(qw)
    return msg


def publish_gt_camera(stamp):
    pos, q = rover.get_world_pose()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = quat_wxyz_to_R(np.asarray(q, float)), np.asarray(pos, float)
    gt_base_pub.publish(pose_msg(T, stamp))
    gt_cam_pub.publish(pose_msg(T @ T_base_cam, stamp))


def publish_terrain(stamp):
    img = Image()
    img.header.stamp, img.header.frame_id = stamp, WORLD_FRAME
    img.height, img.width = H.shape                     # rows = x index, cols = y index
    img.encoding, img.step = '32FC1', H.shape[1] * 4
    img.data = array.array('B', np.ascontiguousarray(H, np.float32).tobytes())
    height_pub.publish(img)
    vol = Float32MultiArray()
    vol.data = [float(v) for v in A.volumes(H, H0)] if args.world == 'arena' else \
        [float(np.clip(H0 - H, 0, None).sum() * RES * RES), float(np.clip(H - H0, 0, None).sum() * RES * RES), 0.0]
    volume_pub.publish(vol)


def apply_digs(requests):
    """Edit the terrain: [x, y, radius, depth] (+depth digs, -depth dumps); re-mesh touched tiles."""
    lo, hi = [10 ** 9, 10 ** 9], [-1, -1]
    for x, y, r, d in requests:
        if dust is not None:
            dust.add(x, y, world.current_time)        # a plume over every dig and dump
        (i0, i1, j0, j1), moved = A.dig(H, xs, ys, x, y, max(r, 0.03), d)
        lo, hi = [min(lo[0], i0), min(lo[1], j0)], [max(hi[0], i1), max(hi[1], j1)]
    tiles = terrain.update(lo[0], hi[0], lo[1], hi[1])
    print(f'[sim] terrain edit x{len(requests)} (last {requests[-1]}) -> {tiles} tiles re-meshed', flush=True)


def camera_info(name, prim, res):
    cam = UsdGeom.Camera(stage.GetPrimAtPath(prim))
    w, h = res
    f = cam.GetFocalLengthAttr().Get() / cam.GetHorizontalApertureAttr().Get() * w  # square pixels
    info = CameraInfo()
    info.header.frame_id = frame_ids[name]
    info.width, info.height, info.distortion_model = w, h, 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    tx = -f * baseline if name == 'infra2' else 0.0  # stereo: right camera carries -fx * baseline
    info.p = [f, 0.0, w / 2, tx, 0.0, f, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


infos = {'infra1': camera_info('infra1', LEFT, CAM_RES), 'infra2': camera_info('infra2', RIGHT, CAM_RES),
         'depth': camera_info('depth', DEPTH, DEPTH_RES)}
if REAR:
    infos['rear'] = camera_info('rear', REAR, REAR_RES)


def publish_rear(stamp):
    data = rear_ann.get_data()
    if data is None or np.size(data) == 0:
        return
    rgb = np.ascontiguousarray(np.asarray(data)[..., :3])
    msg = Image()
    msg.header.stamp, msg.header.frame_id = stamp, 'rear_camera_optical_frame'
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding, msg.step = 'rgb8', rgb.shape[1] * 3
    msg.data = array.array('B', rgb.tobytes())
    infos['rear'].header.stamp = stamp
    rear_pubs[1].publish(infos['rear'])
    rear_pubs[0].publish(msg)
print(f'[sim] stereo baseline {baseline:.4f} m, fx {infos["infra1"].k[0]:.1f}', flush=True)


def publish_cameras(t):
    stamp = Time(sec=int(t), nanosec=int((t % 1) * 1e9))
    for name, ann in annotators.items():
        data = ann.get_data()
        if data is None or np.size(data) == 0:
            return
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, frame_ids[name]
        if name == 'depth':
            depth = np.nan_to_num(np.asarray(data, np.float32), nan=0.0, posinf=0.0)
            depth[depth > 20.0] = 0.0
            if args.realism != 'off':
                tau = None
                if dust is not None and dust.active(t):
                    R_d, p_d = camera_axes(DEPTH, optical=True)
                    norm = np.linalg.norm(DEPTH_RAYS, axis=1)
                    tau = dust.optical_depth(p_d, (DEPTH_RAYS / norm[:, None]) @ R_d.T,
                                             depth.reshape(-1) * norm, t).reshape(depth.shape)
                depth = d455r.depth(depth, tau)
            msg.height, msg.width = depth.shape
            msg.encoding, msg.step = '32FC1', depth.shape[1] * 4
            msg.data = array.array('B', depth.tobytes())  # array: rclpy's fast path (bytes are checked per element)
        else:  # D455 IR cameras are monochrome (Y8)
            rgb = np.asarray(data)[..., :3].astype(np.float32)
            mono = d455r.ir((rgb @ np.array([0.299, 0.587, 0.114], np.float32)).astype(np.uint8))
            msg.height, msg.width = mono.shape
            msg.encoding, msg.step = 'mono8', mono.shape[1]
            msg.data = array.array('B', mono.tobytes())
        infos[name].header.stamp = stamp
        pubs[name][1].publish(infos[name])
        pubs[name][0].publish(msg)


SUBSTEPS = int(round((1.0 / 30.0) / (1.0 / 200.0)))  # physics steps per camera frame (~29 Hz)
dug = False
yaw_i = 0.0
terrain_dirty, last_terrain_pub = True, -1e9
prof, n_prof = np.zeros(3), 0
n_frame = 0
# --profile: per-piece wall time over the last 60 frames (ms per frame) and the tiles ever re-meshed
PIECES = ('physics', 'render', 'cameras', 'rear', 'lidar', 'ruts', 'remesh', 'dig', 'rest', 'dust')
pt = dict.fromkeys(PIECES, 0.0)
tiles_touched = set()
if args.profile and dust is not None:
    _od = dust.optical_depth

    def _timed_od(*a, **k):
        t_ = time.perf_counter()
        out = _od(*a, **k)
        pt['dust'] += time.perf_counter() - t_
        return out
    dust.optical_depth = _timed_od
if args.profile:
    _upd = terrain.update

    def _timed_update(i0, i1, j0, j1):
        t_ = time.perf_counter()
        n = _upd(i0, i1, j0, j1)
        t = terrain.tile
        for ti in range((max(i0 - 1, 0) // t) * t, min(i1 + 1, len(terrain.xs) - 1), t):
            for tj in range((max(j0 - 1, 0) // t) * t, min(j1 + 1, len(terrain.ys) - 1), t):
                tiles_touched.add((ti, tj))
        pt['remesh'] += time.perf_counter() - t_
        return n
    terrain.update = _timed_update
print('[sim] running', flush=True)
while app.is_running():
    # Skid-steer (tank) drive: same speed for every wheel on a side. Skid turns slip by an
    # amount that depends on gravity and ground, so a PI loop on the measured yaw rate
    # (like a motor controller with a gyro) makes the rover turn at the commanded rate.
    v, w = float(twist_lin.get()[0]), float(twist_ang.get()[2])
    w_meas = float(np.asarray(rover.get_angular_velocity())[2])
    yaw_i = float(np.clip(yaw_i + (w - w_meas) * SUBSTEPS * 0.005, -1.5, 1.5)) if abs(w) > 1e-3 else 0.0
    w_cmd = w + 0.4 * (w - w_meas) + 1.0 * yaw_i if abs(w) > 1e-3 else 0.0
    wl, wr = (v - w_cmd * SKID_TRACK / 2) / WHEEL_RADIUS, (v + w_cmd * SKID_TRACK / 2) / WHEEL_RADIUS
    # Slip (realism.Slip): on loose BP-1 the body moves (1 - s) of the wheel surface speed. The rigid physics wheels
    # are driven at (1 - s) of the command, and the encoders report the command-level speed, so odometry over-reads.
    s_slip = slip.ratio(world.current_time)
    rover.apply_action(ArticulationAction(
        joint_velocities=np.array([wl * (1 - s_slip)] * len(left_idx) + [wr * (1 - s_slip)] * len(right_idx)),
        joint_indices=np.array(left_idx + right_idx)))
    t0 = time.perf_counter()
    for _ in range(SUBSTEPS - 1):
        world.step(render=False)  # 200 Hz physics + IMU between camera frames
    t1 = time.perf_counter()
    world.step(render=True)
    for _ in range(args.rt_subframes - 1):
        world.render()  # same pose, no physics: lets temporal history settle
    t2 = time.perf_counter()
    publish_cameras(world.current_time)
    tc = time.perf_counter()
    now = world.current_time
    stamp = Time(sec=int(now), nanosec=int((now % 1) * 1e9))
    publish_gt_camera(stamp)
    publish_wheel_odom(stamp)
    tr0 = time.perf_counter()
    if REAR and n_frame % 3 == 0:                 # ~10 Hz is plenty for the tags
        publish_rear(stamp)
    n_frame += 1
    tr1 = time.perf_counter()
    if ruts is not None and ruts.on:
        pos, q = rover.get_world_pose()
        Rb = quat_wxyz_to_R(np.asarray(q, float))
        bb = ruts.stamp(H, float(pos[0]), float(pos[1]), float(np.arctan2(Rb[1, 0], Rb[0, 0])))
        if bb:
            terrain.update(*bb)
            terrain_dirty = True
    tr2 = time.perf_counter()
    if LIDAR:
        publish_lidar(stamp)
    tl = time.perf_counter()
    for _ in range(20):                       # non-blocking: handle queued /arena/dig requests
        rclpy.spin_once(ros, timeout_sec=0.0)
    if not dug and 0 <= args.dig_at <= now:
        dig_queue.append([*args.dig_xy, args.dig_radius, args.dig_depth])
        print(f'[sim] dug {args.dig_depth} m hole at {args.dig_xy} (t={now:.1f}s)', flush=True)
        dug = True
    td0 = time.perf_counter()
    if dig_queue:
        apply_digs(dig_queue)
        dig_queue.clear()
        terrain_dirty = True
    td1 = time.perf_counter()
    if (terrain_dirty and now - last_terrain_pub > 0.3) or now - last_terrain_pub > 5.0:
        publish_terrain(stamp)
        terrain_dirty, last_terrain_pub = False, now
    t3 = time.perf_counter()
    if args.debug_poses and n_prof % 30 == 0:
        from isaacsim.core.prims import XFormPrim
        cam_pos = XFormPrim(LEFT).get_world_poses()[0][0]
        print(f'[sim] t={world.current_time:.2f} base={np.round(rover.get_world_pose()[0], 3)} '
              f'left_cam={np.round(np.asarray(cam_pos), 3)}', flush=True)
    if args.debug_poses and not args.profile:
        n_prof += 1
    if args.profile:
        prof += [t1 - t0, t2 - t1, t3 - t2]
        n_prof += 1
        pt['physics'] += t1 - t0
        pt['render'] += t2 - t1
        pt['cameras'] += tc - t2
        pt['rear'] += tr1 - tr0
        pt['ruts'] += tr2 - tr1          # includes the re-mesh of rut-touched tiles (also counted in remesh)
        pt['lidar'] += tl - tr2
        pt['dig'] += td1 - td0
        pt['rest'] += (t3 - t2) - (tc - t2) - (tr1 - tr0) - (tr2 - tr1) - (tl - tr2) - (td1 - td0)
        if n_prof % 60 == 0:
            wall = time.perf_counter()
            rtf = (now - prof_t[1]) / (wall - prof_t[0]) if 'prof_t' in globals() else float('nan')
            prof_t = (wall, now)
            print(f'[sim] prof t={now:.1f} RTF {rtf:.3f} ms/frame ' + ' '.join(f'{k} {pt[k] / 60 * 1000:.1f}' for k in PIECES)
                  + f' | tiles re-meshed so far {len(tiles_touched)} of {len(terrain.meshes)}', flush=True)
            pt = dict.fromkeys(PIECES, 0.0)
ros.destroy_node()
rclpy.try_shutdown()
app.close()
