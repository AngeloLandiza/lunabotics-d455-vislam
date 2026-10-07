"""Isaac Sim 6.0: tank-drive rover with a down-pitched RealSense D455.

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
drives from /cmd_vel, and digs/dumps where told on /arena/dig ([x, y, radius, depth]).

Run via ./run_sim.sh (Isaac Sim's own Python + internal ROS 2 libraries):  python sim/isaac_lunar_sim.py [--headless]
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
ROVER_USD = '/Isaac/Robots/Clearpath/Jackal/jackal.usd'
D455_USD = '/Isaac/Sensors/RealSense/D455/rsd455.usd'
CAM_MOUNT = (0.30, 0.0, 0.42)     # D455 on a front mast: 5 cm ahead of the bumper, ~0.49 m up
CAM_PITCH_DEG = 40.0              # pitched down so the rover stays out of view
WHEEL_RADIUS, TRACK = 0.098, 0.37559
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
add_reference_to_stage(assets + ROVER_USD, ROVER)
place(ROVER, (start_x, start_y, terrain.height_at(start_x, start_y) + 0.08),  # base ~6.5 cm above wheel bottoms
      (0.0, 0.0, float(np.degrees(start_yaw))))
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
WHEELS = sorted(str(p.GetPath()).split('/')[-1] for p in stage.Traverse()
                if str(p.GetPath()).startswith(ROVER) and p.IsA(UsdPhysics.RevoluteJoint)
                and 'wheel' in p.GetName())
LEFT_WHEELS = [w for w in WHEELS if 'left' in w]
RIGHT_WHEELS = [w for w in WHEELS if 'right' in w]
for w in WHEELS:
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f'{ROVER}/{w}'), 'angular')
    print(f'[sim] drive {w}: stiffness={drive.GetStiffnessAttr().Get()} damping={drive.GetDampingAttr().Get()} '
          f'maxForce={drive.GetMaxForceAttr().Get()}', flush=True)
print(f'[sim] base={BASE}\n[sim] left={LEFT}\n[sim] right={RIGHT}\n[sim] depth={DEPTH}\n'
      f'[sim] imu={IMU}\n[sim] wheels L={LEFT_WHEELS} R={RIGHT_WHEELS}', flush=True)

# ---------------------------------------------------------------- ROS 2 graphs
annotators = {}
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
        ('imu.inputs:topicName', '/camera/imu'), ('imu.inputs:frameId', 'camera_imu_optical_frame'),
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
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
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
             'depth': 'camera_depth_optical_frame'}
baseline = float(np.linalg.norm(world_frame(RIGHT)[1] - world_frame(LEFT)[1]))

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
            msg.height, msg.width = depth.shape
            msg.encoding, msg.step = '32FC1', depth.shape[1] * 4
            msg.data = array.array('B', depth.tobytes())  # array: rclpy's fast path (bytes are checked per element)
        else:  # D455 IR cameras are monochrome (Y8)
            rgb = np.asarray(data)[..., :3].astype(np.float32)
            mono = (rgb @ np.array([0.299, 0.587, 0.114], np.float32)).astype(np.uint8)
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
    rover.apply_action(ArticulationAction(joint_velocities=np.array([wl] * len(left_idx) + [wr] * len(right_idx)),
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
    now = world.current_time
    stamp = Time(sec=int(now), nanosec=int((now % 1) * 1e9))
    publish_gt_camera(stamp)
    for _ in range(20):                       # non-blocking: handle queued /arena/dig requests
        rclpy.spin_once(ros, timeout_sec=0.0)
    if not dug and 0 <= args.dig_at <= now:
        dig_queue.append([*args.dig_xy, args.dig_radius, args.dig_depth])
        print(f'[sim] dug {args.dig_depth} m hole at {args.dig_xy} (t={now:.1f}s)', flush=True)
        dug = True
    if dig_queue:
        apply_digs(dig_queue)
        dig_queue.clear()
        terrain_dirty = True
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
        if n_prof % 60 == 0:
            ms = prof / n_prof * 1000
            print(f'[sim] per frame: physics-only steps {ms[0]:.1f} ms, render step {ms[1]:.1f} ms, publish {ms[2]:.1f} ms', flush=True)
ros.destroy_node()
rclpy.try_shutdown()
app.close()
