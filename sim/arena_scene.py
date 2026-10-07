"""Isaac Sim (USD) build of the Lunabotics Artemis Arena from sim/arena.py.

BP-1 regolith bed (deformable tiles), bin walls, 17 cm perimeter ducts, central
column, 80/20 rail in the starting zone, overhead box truss with the arena's
static/PTZ camera positions (as viewport cameras), boulders, and indoor lighting.
"""
import math
import os

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

import arena as A
from terrain_tiles import TerrainTiles

# Appearance. Dimensions: guidebook. Looks: NASA photos and 2024-2026 run footage, and the
# Purdue Lunabotics arena model (BSD-3) for wall / duct / glass heights.
STYLE = {
    'bp1_rgb': (0.25, 0.225, 0.19),     # linear; ~sRGB (138,131,120): light tan-grey under hall light
    'bp1_contrast': 0.9,                # clods, footprints and grain read clearly in footage
    'rock_rgb': (0.47, 0.37, 0.27),     # boulders: real rocks, light tan, lighter than the BP-1
    'wall_rgb': (0.55, 0.52, 0.45),     # beige bin walls (~0.45 m above the regolith)
    'wall_height': 0.45,
    'band_rgb': (0.02, 0.02, 0.02),     # black lower band outside / enclosure frames
    'glass_top': 1.95,                  # enclosure glazing from the wall top to here
    'duct_rgb': (0.62, 0.62, 0.60),     # light-grey PVC lying at the wall base
    'duct_z': 0.0,
    'column_rgb': (0.45, 0.45, 0.45),   # grey box around the central truss leg
    'truss_rgb': (0.85, 0.85, 0.85),    # white box truss
    'truss_height': 3.05,
    'cyclone_rgb': (0.85, 0.85, 0.83),
    'room_rgb': (0.12, 0.12, 0.13),     # dark hall walls beyond the glass
    'floor_rgb': (0.18, 0.18, 0.19),    # grey carpet
    'light_height': 8.0,                # hall ceiling, linear LED strips seen through the clear roof
    'light_rows': 4, 'light_strip': (6.0, 0.25),
    'light_intensity': 40000.0,
    'light_temp': 5000.0,               # K, neutral
    'ambient': 700.0,                   # diffuse fill (white ceiling bounce)
    'rake_period': 0.025,               # faint raking left after the reset
    'rake_strength': 0.25,
    'rake_axis': 'y',
}
CACHE = os.path.expanduser('~/.cache/lunabotics_sim')


# ---------------------------------------------------------------- textures
def bp1_textures(seed, px=4096, size_m=None):
    """BP-1 albedo + normal map (cached PNGs) spanning the whole bed without repeats."""
    import cv2
    size_m = size_m or (A.LENGTH + 2 * A.DUCT_D)
    tag = f'bp1_{seed}_{px}_{STYLE["rake_strength"]}_{STYLE["rake_axis"]}_{STYLE["bp1_rgb"][0]}_{STYLE["bp1_contrast"]}_v5'
    alb_path, nrm_path = os.path.join(CACHE, tag + '_albedo.png'), os.path.join(CACHE, tag + '_normal.png')
    if os.path.exists(alb_path) and os.path.exists(nrm_path):
        return alb_path, nrm_path
    os.makedirs(CACHE, exist_ok=True)
    rng = np.random.default_rng(seed + 3)
    ppm = px / size_m
    tone = cv2.resize(A.fractal_noise((256, 256), rng, beta=2.0).astype(np.float32), (px, px))
    grain = A.fractal_noise((px, px), rng, beta=0.6).astype(np.float32)
    relief = np.zeros((px, px), np.float32)
    shade = np.zeros((px, px), np.float32)

    def blobs(count, rmin, rmax, light_frac, gain):
        xs, ys = rng.integers(0, px, count), rng.integers(0, px, count)
        rs = np.maximum((rng.uniform(rmin, rmax, count) * ppm).astype(int), 1)
        vs = np.where(rng.random(count) < light_frac, 1.0, -1.0) * rng.uniform(0.5, 1.2, count)
        for x, y, r, v in zip(xs, ys, rs, vs):
            ax = (int(r * rng.uniform(0.7, 1.3)), int(r * rng.uniform(0.7, 1.3)))
            cv2.ellipse(shade, (int(x), int(y)), ax, float(rng.uniform(0, 180)), 0, 360, gain * v, -1, cv2.LINE_AA)
            cv2.ellipse(relief, (int(x), int(y)), ax, 0, 0, 360, 1.0, -1, cv2.LINE_AA)

    area = size_m ** 2
    blobs(int(1400 * area), 0.003, 0.012, 0.45, 0.8)   # clods / fine aggregate
    blobs(int(60 * area), 0.006, 0.012, 0.6, 1.0)      # ~2 cm gravel mixed into the BP-1
    blobs(int(25 * area), 0.04, 0.09, 0.35, 0.35)      # boot prints / scuffs from the reset crew
    relief = cv2.GaussianBlur(relief, (0, 0), 1.5)
    # Rake furrows: tines every ~2.5 cm, slightly wavy, in the bed's long direction.
    u = np.arange(px, dtype=np.float32) / ppm
    U, V = np.meshgrid(u, u, indexing='ij')           # texture (s, t) = (x, y)
    across = V if STYLE['rake_axis'] == 'x' else U
    along = U if STYLE['rake_axis'] == 'x' else V
    wobble = 0.004 * np.sin(2 * np.pi * along / 0.9) + 0.002 * cv2.resize(
        A.fractal_noise((64, 64), rng, beta=2.5).astype(np.float32), (px, px))
    rake = np.sin(2 * np.pi * (across + wobble) / STYLE['rake_period'])
    relief += 0.6 * STYLE['rake_strength'] * rake
    shade += 0.25 * STYLE['rake_strength'] * rake
    alb = 1.0 + STYLE['bp1_contrast'] * (0.6 * tone + 0.5 * grain + shade) / 1.6
    rgb = np.clip(np.array(STYLE['bp1_rgb'], np.float32)[None, None, :] * alb[..., None], 0, 1)
    # Image row = t (y), column = s (x); flip rows so t=0 is the bottom (USD UV convention).
    img = (rgb.transpose(1, 0, 2)[::-1] ** (1 / 2.2) * 255).astype(np.uint8)
    cv2.imwrite(alb_path, img[..., ::-1])
    gx, gy = np.gradient(relief.T[::-1] * 0.004, 1 / ppm)   # ~4 mm relief amplitude
    n = np.stack([-gx, gy, np.ones_like(gx)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    cv2.imwrite(nrm_path, ((n * 0.5 + 0.5) * 255).astype(np.uint8)[..., ::-1])
    return alb_path, nrm_path


# ---------------------------------------------------------------- materials / prims
def pbr(stage, path, rgb, roughness=0.9, metallic=0.0, albedo=None, normal=None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/pbr')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    if albedo or normal:
        reader = UsdShade.Shader.Define(stage, path + '/st')
        reader.CreateIdAttr('UsdPrimvarReader_float2')
        reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')

    def texture(name, file, raw):
        tex = UsdShade.Shader.Define(stage, f'{path}/{name}')
        tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(file)
        tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
        tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
        if raw:
            tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('raw')
        tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
        return tex
    if albedo:
        tex = texture('albedo', albedo, False)
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    else:
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    if normal:
        tex = texture('normal', normal, True)
        tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
        tex.CreateInput('bias', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
        sh.CreateInput('normal', Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return mat


def box(stage, path, center, size, mat, collide=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def rod(stage, path, p0, p1, radius, mat, collide=False):
    """Cylinder between two points."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(length)
    cyl.CreateAxisAttr('Z')
    z = d / length
    axis = np.cross([0, 0, 1], z)
    angle = math.degrees(math.acos(np.clip(z[2], -1, 1)))
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(*((p0 + p1) / 2)))
    if np.linalg.norm(axis) > 1e-9:
        xf.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(*axis), angle).GetQuat()))
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl


def rock_mesh(stage, path, b, ground_z, mat):
    """Irregular boulder: noisy ellipsoid, partly buried, convex-hull collider."""
    rng = np.random.default_rng(b.seed)
    nu, nv = 36, 20
    u = np.linspace(0, 2 * np.pi, nu, endpoint=False)
    v = np.linspace(0.04, np.pi - 0.04, nv)
    U, V = np.meshgrid(u, v, indexing='ij')
    d = np.stack([np.cos(U) * np.sin(V), np.sin(U) * np.sin(V), np.cos(V)], -1)
    r = np.ones(U.shape)
    for _ in range(7):                                  # low-frequency lumps + facets
        k = rng.normal(size=3)
        k /= np.linalg.norm(k)
        r += rng.uniform(0.04, 0.10) * np.cos(rng.uniform(1.5, 4.0) * (d @ k) + rng.uniform(0, 6.3))
    r = np.clip(r, 0.75, 1.3)
    a, bb = b.diameter / 2, b.diameter / 2 * rng.uniform(0.75, 1.0)
    c = b.height * 1.15
    P = d * r[..., None] * np.array([a, bb, c])
    yaw = rng.uniform(0, 2 * np.pi)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    P = P.reshape(-1, 3) @ Rz.T + np.array([b.x, b.y, ground_z + b.height - c])
    top, bot = len(P), len(P) + 1
    P = np.vstack([P, [b.x, b.y, ground_z + b.height - c + c * 1.0], [b.x, b.y, ground_z + b.height - 2 * c]])
    faces, counts = [], []
    for i in range(nu):
        i2 = (i + 1) % nu
        for j in range(nv - 1):
            faces += [i * nv + j, i * nv + j + 1, i2 * nv + j + 1, i2 * nv + j]
            counts.append(4)
        faces += [top, i2 * nv, i * nv]
        faces += [bot, i * nv + nv - 1, i2 * nv + nv - 1]
        counts += [3, 3]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    return mesh


def look_at_camera(stage, path, eye, target, focal=14.0):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)
    return cam


# ---------------------------------------------------------------- build
def build(stage, seed=0):
    """Create the arena under /World/arena. Returns (terrain tiles, H0 copy, layout)."""
    root = '/World/arena'
    UsdGeom.Xform.Define(stage, root)
    boulders, craters = A.layout(seed)
    xs, ys, H = A.heightmap(boulders, craters, seed)
    alb, nrm = bp1_textures(seed)
    bp1 = pbr(stage, f'{root}/Looks/bp1', STYLE['bp1_rgb'], roughness=0.97, albedo=alb, normal=nrm)
    phys = UsdShade.Material.Define(stage, f'{root}/Looks/bp1_physics')
    pm = UsdPhysics.MaterialAPI.Apply(phys.GetPrim())
    pm.CreateStaticFrictionAttr(0.55)   # BP-1 vs. wheel is unmeasured (guidebook 11.1); Purdue's sim uses 0.4
    pm.CreateDynamicFrictionAttr(0.5)
    pm.CreateRestitutionAttr(0.0)
    terrain = TerrainTiles(stage, f'{root}/regolith', xs, ys, H, tile=25,
                           uv_size=A.LENGTH + 2 * A.DUCT_D, material=bp1, physics_material=phys)

    wall = pbr(stage, f'{root}/Looks/wall', STYLE['wall_rgb'], roughness=0.8)
    black = pbr(stage, f'{root}/Looks/black', STYLE['band_rgb'], roughness=0.6)
    duct = pbr(stage, f'{root}/Looks/pvc', STYLE['duct_rgb'], roughness=0.5)
    alu = pbr(stage, f'{root}/Looks/truss', STYLE['truss_rgb'], roughness=0.4, metallic=0.6)
    column = pbr(stage, f'{root}/Looks/column', STYLE['column_rgb'], roughness=0.85)
    room = pbr(stage, f'{root}/Looks/room', STYLE['room_rgb'], roughness=0.95)
    floor = pbr(stage, f'{root}/Looks/carpet', STYLE['floor_rgb'], roughness=1.0)
    white = pbr(stage, f'{root}/Looks/cyclone', STYLE['cyclone_rgb'], roughness=0.5, metallic=0.3)
    rock = pbr(stage, f'{root}/Looks/rock', STYLE['rock_rgb'], roughness=0.9)

    # Bin: walls outside the ducts, from the bin floor to wall_height above the surface.
    d, L, W, hw, t = A.DUCT_D, A.LENGTH, A.WIDTH, STYLE['wall_height'], 0.08
    z0, z1 = -A.BED_DEPTH - 0.05, hw
    zc, zs = (z0 + z1) / 2, z1 - z0
    for name, c, sz in [('south', (L / 2, -d - t / 2, zc), (L + 2 * d + 2 * t, t, zs)),
                        ('north', (L / 2, W + d + t / 2, zc), (L + 2 * d + 2 * t, t, zs)),
                        ('west', (-d - t / 2, W / 2, zc), (t, W + 2 * d, zs)),
                        ('east', (L + d + t / 2, W / 2, zc), (t, W + 2 * d, zs))]:
        box(stage, f'{root}/bin/wall_{name}', c, sz, wall)
    box(stage, f'{root}/bin/floor', (L / 2, W / 2, -A.BED_DEPTH - 0.025), (L + 2 * d, W + 2 * d, 0.05), wall)
    # 17 cm PVC ducts on the inside of the walls (interior dims are measured between them).
    r, zd = d / 2, STYLE['duct_z'] + d / 2
    for name, p0, p1 in [('south', (-d, -r, zd), (L + d, -r, zd)), ('north', (-d, W + r, zd), (L + d, W + r, zd)),
                         ('west', (-r, -d, zd), (-r, W + d, zd)), ('east', (L + r, -d, zd), (L + r, W + d, zd))]:
        rod(stage, f'{root}/ducts/{name}', p0, p1, r, duct, collide=True)

    # Glass enclosure on top of the bin walls: black frames (the glass itself is left out:
    # translucency is off for camera realism and real glass barely shows in footage).
    o = d + t
    gt = STYLE['glass_top']
    ring = [(-o, -o), (L + o, -o), (L + o, W + o), (-o, W + o)]
    for k, (a, b) in enumerate(zip(ring, ring[1:] + ring[:1])):
        a, b = np.array(a), np.array(b)
        n = max(int(np.linalg.norm(b - a) / 1.2), 1)
        for m in range(n):
            p = a + (b - a) * m / n
            box(stage, f'{root}/enclosure/mullion_{k}_{m}', (p[0], p[1], (hw + gt) / 2), (0.05, 0.05, gt - hw),
                black, collide=False)
        mid = (a + b) / 2
        size = (abs(b[0] - a[0]) + 0.05, abs(b[1] - a[1]) + 0.05)
        box(stage, f'{root}/enclosure/top_{k}', (mid[0], mid[1], gt), (*size, 0.05), black, collide=False)
        # Black skirt on the outside of the bin wall, down to the hall floor.
        outward = np.sign(mid - np.array([L / 2, W / 2])) * (np.array(size) < 0.1) * 0.03
        box(stage, f'{root}/enclosure/skirt_{k}', (mid[0] + outward[0], mid[1] + outward[1], hw - 0.45),
            (*size, 0.9), black, collide=False)
    # 80/20 rail on the bin walls along the starting zone (guidebook 11.2: beacon / fiducial mounting).
    sx0, sy0, sx1, sy1 = A.ZONES['start']
    box(stage, f'{root}/rail_8020_south', ((sx0 + sx1) / 2, -d - t / 2, hw + 0.02), (sx1 - sx0, 0.04, 0.04), alu)
    box(stage, f'{root}/rail_8020_west', (-d - t / 2, (sy0 + sy1) / 2, hw + 0.02), (0.04, sy1 - sy0, 0.04), alu)

    # Hall floor (grey carpet) and dark far walls.
    fz = hw - 0.9
    box(stage, f'{root}/hall/floor', (L / 2, W / 2, fz - 0.05), (L + 14, W + 14, 0.1), floor, collide=False)
    for name, c, sz in [('n', (L / 2, W + 7, 4), (L + 14, 0.1, 9)), ('s', (L / 2, -7, 4), (L + 14, 0.1, 9)),
                        ('e', (L + 7, W / 2, 4), (0.1, W + 14, 9)), ('w', (-7, W / 2, 4), (0.1, W + 14, 9))]:
        box(stage, f'{root}/hall/wall_{name}', c, sz, room, collide=False)

    # White box truss (Purdue model: legs on a grid around the arena centre, 3.05 m tall).
    # The central leg stands in the bin inside a grey box: the guidebook's support column.
    th, ts = STYLE['truss_height'], 0.15
    cx, cy, side = A.COLUMN
    box(stage, f'{root}/column', (cx, cy, (1.0 - A.BED_DEPTH) / 2), (side, side, 1.0 + A.BED_DEPTH), column)
    k = [0]

    def truss(p0, p1):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        axis = (p1 - p0) / np.linalg.norm(p1 - p0)
        side1 = np.cross(axis, [0, 0, 1]) if abs(axis[2]) < 0.9 else np.array([1.0, 0, 0])
        side1 /= np.linalg.norm(side1)
        side2 = np.cross(axis, side1)
        offs = [ts * (a * side1 + b * side2) for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        for q in offs:
            rod(stage, f'{root}/truss/c{k[0]}', p0 + q, p1 + q, 0.024, alu)
            k[0] += 1
        n = max(int(np.linalg.norm(p1 - p0) / 0.6), 1)
        for s_ in range(n):
            a0, a1 = p0 + (p1 - p0) * s_ / n, p0 + (p1 - p0) * (s_ + 1) / n
            for f in range(4):
                rod(stage, f'{root}/truss/l{k[0]}', a0 + offs[f], a1 + offs[(f + 1) % 4], 0.009, alu)
                k[0] += 1
    xs_leg = [cx - 4.0, cx, cx + 4.0]
    ys_leg = [-o - 0.25, cy, W + o + 0.25]
    legs = {(x, y): (fz if not (x == cx and y == cy) else 1.0) for x in xs_leg for y in ys_leg}
    for (x, y), zb in legs.items():
        truss((x, y, zb), (x, y, th))
    for y in ys_leg:
        truss((xs_leg[0], y, th), (xs_leg[-1], y, th))
    for x in xs_leg:
        truss((x, ys_leg[0], th), (x, ys_leg[-1], th))

    # Cyclone dust collectors with flex ducts at the corners outside the bin.
    for k3, (x, y) in enumerate([(-o - 0.6, -o - 0.6), (L + o + 0.6, W + o + 0.6)]):
        rod(stage, f'{root}/dust/cyclone_body_{k3}', (x, y, fz + 1.4), (x, y, fz + 2.6), 0.30, white)
        cone = UsdGeom.Cone.Define(stage, f'{root}/dust/cyclone_cone_{k3}')
        cone.CreateRadiusAttr(0.30)
        cone.CreateHeightAttr(1.0)
        UsdGeom.Xformable(cone).AddTranslateOp().Set(Gf.Vec3d(x, y, fz + 0.9))
        UsdGeom.Xformable(cone).AddRotateXYZOp().Set(Gf.Vec3f(180, 0, 0))
        UsdShade.MaterialBindingAPI.Apply(cone.GetPrim()).Bind(white)
        rod(stage, f'{root}/dust/flex_{k3}', (x, y, fz + 2.6), (x + np.sign(L / 2 - x) * 0.9, y, hw + 0.25), 0.09,
            alu)

    # Arena cameras (guidebook Fig. 10) as viewport cameras: static cams 1,2,5,6 ~2.5 ft and
    # 3,4 ~10 ft above the bin, PTZ 1-4 ~8 ft on the middle span, plus the overhead fisheye.
    centre = (L / 2, W / 2, 0)
    for name, eye in [('static_cam_1', (-o, -o, 0.76)), ('static_cam_2', (0.6, W + o, 0.76)),
                      ('static_cam_3', (L * 0.3, W + o, 3.05)), ('static_cam_4', (L * 0.7, W + o, 3.05)),
                      ('static_cam_5', (L - 0.6, W + o, 0.76)), ('static_cam_6', (L + o, -o, 0.76)),
                      ('ptz_cam_1', (cx, W - 0.2, 2.44)), ('ptz_cam_2', (cx, W * 0.62, 2.44)),
                      ('ptz_cam_3', (cx, W * 0.38, 2.44)), ('ptz_cam_4', (cx, 0.2, 2.44))]:
        look_at_camera(stage, f'{root}/cameras/{name}', eye, centre, focal=10.0 if 'static' in name else 14.0)
    look_at_camera(stage, f'{root}/cameras/overhead', (L / 2, W / 2, 6.5), (L / 2, W / 2 + 0.01, 0), focal=9.0)
    look_at_camera(stage, f'{root}/cameras/overview', (L * 0.45, -3.6, 3.6), (L * 0.5, W * 0.45, 0), focal=12.0)

    for k2, b in enumerate(boulders):
        rock_mesh(stage, f'{root}/boulders/boulder_{k2}', b, terrain.height_at(b.x, b.y), rock)

    # Hall lighting through the enclosure's clear roof: rows of long LED strips high up
    # (diffuse, soft shadows) plus white-ceiling bounce.
    lw, lh = STYLE['light_strip']
    for i in range(STYLE['light_rows']):
        light = UsdLux.RectLight.Define(stage, f'{root}/lights/strip_{i}')
        light.CreateWidthAttr(lw)
        light.CreateHeightAttr(lh)
        light.CreateIntensityAttr(STYLE['light_intensity'])
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(STYLE['light_temp'])
        y = W / 2 + (i - (STYLE['light_rows'] - 1) / 2) * 2.2
        UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(L / 2, y, STYLE['light_height']))  # emits -Z
    dome = UsdLux.DomeLight.Define(stage, f'{root}/lights/ambient')
    dome.CreateIntensityAttr(STYLE['ambient'])
    dome.CreateEnableColorTemperatureAttr(True)
    dome.CreateColorTemperatureAttr(STYLE['light_temp'])
    return terrain, H.copy(), (boulders, craters)
