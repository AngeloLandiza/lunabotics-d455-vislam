"""Isaac Sim (USD) build of the Lunabotics Artemis Arena from sim/arena.py.

BP-1 regolith bed (deformable tiles), bin walls, 17 cm perimeter ducts, central
column, 80/20 rail in the starting zone, overhead box truss with the arena's
static/PTZ camera positions (as viewport cameras), boulders, range poles, projected
zone laser lines, the printed lunar mural, and indoor lighting.
"""
import math
import os

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

import arena as A
from terrain_tiles import TerrainTiles

# Appearance. Dimensions: guidebook. Looks: the KSC arena cameras in the Lunabotics 2026 Day 1
# stream (youtube.com/watch?v=6TjQcgX_XBo, eight runs), NASA photos, and the Purdue Lunabotics
# arena model (BSD-3) for wall / duct / glass heights.
STYLE = {
    # BP-1 in the footage is a neutral light grey with a faint warm cast: sRGB R ~ G, B ~8% lower
    # (medians 173/169/157 and 137/138/131 from two cameras). Same luminance as before, less brown.
    'bp1_rgb': (0.235, 0.232, 0.205),   # linear
    'bp1_contrast': 0.5,                # albedo is even; the texture reads through relief and shading
    # Reset surface (footage): swaths raked in sweeping arcs, shovel-smoothed patches of flat
    # overlapping plates with crisp edges, boot prints everywhere, a little ~2 cm gravel.
    'rake_frac': 0.4,                   # share of the bed in raked swaths
    'rake_period': 0.028,               # tine spacing (m)
    'rake_depth': 0.002,                # furrow amplitude (m)
    'plate_size': 0.2,                  # shovel-smoothed plates (m)
    'plate_step': 0.004,                # height steps at plate edges (m)
    # Rocks: angular, faceted pale grey to cream stones with dark pits. Their median matches the
    # bed's despite self-shadowing and their lit faces are much brighter (p90 ~200 vs 179), so the
    # albedo is ~1.5x the BP-1's.
    'rock_rgb': [(0.36, 0.355, 0.33), (0.40, 0.38, 0.345), (0.33, 0.33, 0.31)],
    # Orange-red projected zone lines: a speckled band of laser dots ~4-6 cm wide.
    'laser_rgb': (1.0, 0.22, 0.06), 'laser_width': 0.035, 'laser_emission': 1.6,
    'pole_bands': [('w', 0.2), ('r', 0.2), ('w', 0.2), ('r', 1.0)],   # top down; red/white range pole
    # Long wall opposite the start zone and berm (y = WIDTH): black panels printed with a grey
    # lunar landscape in a band at mid-height, framed by black rails, up to the truss.
    'mural_top': 2.9,
    'wall_rgb': (0.33, 0.33, 0.32),     # grey bin walls (~0.45 m above the regolith)
    'wall_height': 0.45,
    'band_rgb': (0.02, 0.02, 0.02),     # black lower band outside / enclosure frames
    'glass_top': 1.95,                  # enclosure glazing from the wall top to here
    'duct_rgb': (0.45, 0.45, 0.44),     # grey PVC lying at the wall base
    'duct_z': 0.0,
    'column_rgb': (0.45, 0.45, 0.45),   # grey box around the central truss leg
    'truss_rgb': (0.85, 0.85, 0.85),    # white box truss
    'truss_height': 3.05,
    'cyclone_rgb': (0.85, 0.85, 0.83),
    'room_rgb': (0.40, 0.40, 0.40),     # light grey hall beyond the glass
    'floor_rgb': (0.18, 0.18, 0.19),    # grey carpet
    'light_height': 8.0,                # hall ceiling, linear LED strips seen through the clear roof
    'light_rows': 4, 'light_strip': (6.0, 0.25),
    'light_intensity': 40000.0,
    'light_temp': 6500.0,               # K: the footage is white-balanced to neutral grey
    'ambient': 1100.0,                  # diffuse fill: ceiling / hall bounce (indirect diffuse is off)
}
CACHE = os.path.expanduser('~/.cache/lunabotics_sim')


# ---------------------------------------------------------------- textures
def _style_tag(*keys):
    return '_'.join(str(STYLE[k]) for k in keys).replace(' ', '').replace('(', '').replace(')', '').replace(',', '-')


def _write_rgb(path, rgb):
    """Save a linear [i (s), j (t), 3] array as an sRGB PNG in USD UV layout (t = 0 at the bottom)."""
    import cv2
    img = rgb.transpose(1, 0, 2)[::-1]
    img = np.where(img <= 0.0031308, 12.92 * img, 1.055 * np.power(np.clip(img, 0, 1), 1 / 2.4) - 0.055)
    cv2.imwrite(path, (np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)[..., ::-1])


def _write_normal(path, relief, ppm):
    """Tangent-space normal map from relief (m) on an [i (s), j (t)] grid; +t is up in the image."""
    import cv2
    dx, dy = np.gradient(relief, 1 / ppm)
    n = np.stack([-dx, -dy, np.ones_like(dx)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    cv2.imwrite(path, ((n.transpose(1, 0, 2)[::-1] * 0.5 + 0.5) * 255 + 0.5).astype(np.uint8)[..., ::-1])


def _nearest_seed(X, Y, sx, sy, chunk=256):
    """Index of the nearest seed for every point (brute force, row chunks)."""
    out = np.empty(X.shape, np.int32)
    for r in range(0, X.shape[0], chunk):
        d = (X[r:r + chunk, :, None] - sx) ** 2 + (Y[r:r + chunk, :, None] - sy) ** 2
        out[r:r + chunk] = d.argmin(-1)
    return out


def _plates(X, Y, cell, step, rng):
    """Shovel-smoothed surface: Voronoi plates (jittered grid), each a slightly tilted plane at its
    own height, so plate edges show as crisp small steps."""
    nx, ny = int(X.max() / cell) + 3, int(Y.max() / cell) + 3
    sx = (np.arange(nx)[:, None] - 1 + rng.random((nx, ny))) * cell
    sy = (np.arange(ny)[None, :] - 1 + rng.random((nx, ny))) * cell
    off, tx, ty = rng.uniform(-1, 1, (3, nx, ny)) * np.array([step, 0.05, 0.05])[:, None, None]
    ci = np.clip((X / cell).astype(int) + 1, 1, nx - 2)
    cj = np.clip((Y / cell).astype(int) + 1, 1, ny - 2)
    best, bi, bj = np.full(X.shape, np.inf, np.float32), ci.copy(), cj.copy()
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            i, j = ci + di, cj + dj
            d = (X - sx[i, j]) ** 2 + (Y - sy[i, j]) ** 2
            better = d < best
            best[better], bi[better], bj[better] = d[better], i[better], j[better]
    return off[bi, bj] + tx[bi, bj] * (X - sx[bi, bj]) + ty[bi, bj] * (Y - sy[bi, bj])


def bp1_textures(seed, px=4096, size_m=None):
    """BP-1 albedo, normal and laser-emission maps (cached PNGs) spanning the whole bed without repeats.

    Texture (s, t) = arena (x, y) + DUCT_D, scaled by 1 / size_m (TerrainTiles uv_size).
    """
    import cv2
    size_m = size_m or (A.LENGTH + 2 * A.DUCT_D)
    tag = (f'bp1_{seed}_{px}_' + _style_tag('bp1_rgb', 'bp1_contrast', 'rake_frac', 'rake_period', 'rake_depth',
                                           'plate_size', 'plate_step', 'laser_rgb', 'laser_width') + '_v10')
    paths = [os.path.join(CACHE, f'{tag}_{k}.png') for k in ('albedo', 'normal', 'emission')]
    if all(os.path.exists(p) for p in paths):
        return paths
    os.makedirs(CACHE, exist_ok=True)
    rng = np.random.default_rng(seed + 3)
    ppm = px / size_m
    c = (np.arange(px, dtype=np.float32) + 0.5) / ppm - A.DUCT_D
    X, Y = np.meshgrid(c, c, indexing='ij')                     # arena coordinates of each texel
    tone = cv2.resize(A.fractal_noise((256, 256), rng, beta=2.0).astype(np.float32), (px, px))
    grain = A.fractal_noise((px, px), rng, beta=0.6).astype(np.float32)

    # Reset patchwork: Voronoi swaths, raked (arcs around a far centre) or shovel-smoothed.
    n = int(size_m ** 2 / 0.5)
    sx, sy = rng.uniform(-A.DUCT_D, size_m - A.DUCT_D, (2, n))
    raked = rng.random(n) < STYLE['rake_frac']
    ang, rad = rng.uniform(0, 2 * np.pi, n), rng.uniform(1.5, 4.0, n)   # arc centres well outside the swath
    cx, cy = sx + rad * np.cos(ang), sy + rad * np.sin(ang)
    lo = cv2.resize(X, (512, 512), interpolation=cv2.INTER_AREA), cv2.resize(Y, (512, 512), interpolation=cv2.INTER_AREA)
    label = cv2.resize(_nearest_seed(*lo, sx, sy).astype(np.float32), (px, px), interpolation=cv2.INTER_NEAREST)
    label = label.astype(np.int32)
    del lo
    rake_mask = cv2.GaussianBlur(raked[label].astype(np.float32), (0, 0), 4)
    wob = 0.004 * cv2.resize(A.fractal_noise((128, 128), rng, beta=2.5).astype(np.float32), (px, px))
    phase = 2 * np.pi * (np.hypot(X - cx[label], Y - cy[label]) + wob) / STYLE['rake_period']
    del label
    furrow = np.sin(phase) + 0.25 * np.sin(2 * phase)               # tines leave sharper troughs
    del phase
    half = px // 2
    plates = _plates(cv2.resize(X, (half, half)), cv2.resize(Y, (half, half)), STYLE['plate_size'],
                     STYLE['plate_step'], rng).astype(np.float32)
    plates = cv2.resize(cv2.GaussianBlur(plates, (0, 0), 1.0), (px, px), interpolation=cv2.INTER_LINEAR)
    relief = rake_mask * STYLE['rake_depth'] * furrow + (1 - rake_mask) * plates
    shade = -0.06 * rake_mask * np.clip(-furrow, 0, None) + 0.03 * (1 - rake_mask)   # furrow troughs darker
    del furrow, plates

    def blobs(count, rmin, rmax, light_frac, gain, height):
        xs, ys = rng.integers(0, px, count), rng.integers(0, px, count)
        rs = np.maximum((rng.uniform(rmin, rmax, count) * ppm).astype(int), 1)
        vs = np.where(rng.random(count) < light_frac, 1.0, -1.0) * rng.uniform(0.5, 1.2, count)
        bump = np.zeros((px, px), np.float32)
        for x, y, r, v in zip(xs, ys, rs, vs):
            ax = (int(r * rng.uniform(0.7, 1.3)), int(r * rng.uniform(0.7, 1.3)))
            a = float(rng.uniform(0, 180))
            cv2.ellipse(shade, (int(y), int(x)), ax, a, 0, 360, gain * v, -1, cv2.LINE_AA)
            cv2.ellipse(bump, (int(y), int(x)), ax, a, 0, 360, height, -1, cv2.LINE_AA)
        return cv2.GaussianBlur(bump, (0, 0), 1.2)

    area = size_m ** 2
    relief += blobs(int(900 * area), 0.003, 0.010, 0.45, 0.10, 0.0015)   # clods / fine aggregate
    relief += blobs(int(40 * area), 0.005, 0.011, 0.85, 0.25, 0.004)     # ~2 cm gravel: pale chips
    # Boot prints (same list as the heightmap, which carries most of their depth): shoe covers
    # press a smooth sole with soft, slumped walls and leave it a touch darker.
    sole = np.zeros((px, px), np.float32)
    for x, y, hd, ln, wd, dp in A.boot_prints(seed):
        ctr = (int((y + A.DUCT_D) * ppm), int((x + A.DUCT_D) * ppm))
        axes = (int(ln / 2 * ppm), int(wd / 2 * ppm))
        cv2.ellipse(sole, ctr, axes, float(np.degrees(np.pi / 2 - hd)), 0, 360, 0.5 * dp, -1, cv2.LINE_AA)
    sole = cv2.GaussianBlur(sole, (0, 0), 5)
    pressed = np.clip(sole / 0.002, 0, 1)
    relief = relief * (1 - pressed) - sole
    shade -= 0.04 * pressed
    del sole

    alb = 1.0 + STYLE['bp1_contrast'] * (0.2 * tone + 0.12 * grain) + shade
    rgb = np.array(STYLE['bp1_rgb'], np.float32)[None, None, :] * alb[..., None]
    del tone, grain, shade
    # Projected laser lines: a band of speckle dots; emissive, plus a light orange tint so they
    # also read in renders where the emission is weak.
    d = np.full((px, px), np.inf, np.float32)
    for x0, y0, x1, y1 in A.LASER_LINES:
        ex, ey = x1 - x0, y1 - y0
        t = np.clip(((X - x0) * ex + (Y - y0) * ey) / (ex * ex + ey * ey), 0, 1)
        d = np.minimum(d, np.hypot(X - x0 - t * ex, Y - y0 - t * ey))
    band = np.exp(-(d / STYLE['laser_width']) ** 2)
    del d
    # Laser speckle: separate ~5 mm dots, denser at the band centre.
    q = px // 3
    dots = (rng.random((q, q), np.float32) > 0.86).astype(np.float32) * rng.uniform(0.4, 1.0, (q, q))
    speckle = cv2.GaussianBlur(cv2.resize(dots.astype(np.float32), (px, px), interpolation=cv2.INTER_NEAREST),
                               (0, 0), 1.2)
    laser = np.clip(band * (0.06 + 1.6 * speckle), 0, 1)
    del speckle, band
    tint = np.array(STYLE['laser_rgb'], np.float32)
    rgb = rgb * (1 - 0.4 * laser[..., None]) + 0.4 * laser[..., None] * tint * 0.6
    _write_rgb(paths[0], rgb)
    del rgb
    _write_normal(paths[1], relief, ppm)
    _write_rgb(paths[2], laser[..., None] * tint)
    return paths


def rock_textures(px=1024):
    """Rock albedo multiplier (x0.5, tinted per material via the texture scale) and normal map:
    mottled stone with dark pits and pale crystalline specks. Periodic in s and t."""
    import cv2
    tag = os.path.join(CACHE, f'rock_{px}_v2')
    paths = [tag + '_albedo.png', tag + '_normal.png']
    if all(os.path.exists(p) for p in paths):
        return paths
    os.makedirs(CACHE, exist_ok=True)
    rng = np.random.default_rng(101)
    tone = A.fractal_noise((px, px), rng, beta=2.4).astype(np.float32)
    grain = A.fractal_noise((px, px), rng, beta=0.8).astype(np.float32)
    pits = np.zeros((px, px), np.float32)
    specks = np.zeros((px, px), np.float32)
    for img, count, rmax in ((pits, 2500, 5), (specks, 1500, 2)):
        for x, y, r in zip(rng.integers(0, px, count), rng.integers(0, px, count), rng.integers(1, rmax + 1, count)):
            for ox in (-px, 0, px):                     # wrap so the texture stays periodic
                cv2.circle(img, (int(x + ox), int(y)), int(r), 1.0, -1, cv2.LINE_AA)
    mult = np.clip(1 + 0.16 * tone + 0.05 * grain - 0.25 * pits + 0.06 * specks, 0.2, 1.9)
    _write_rgb(paths[0], np.repeat(0.5 * mult[..., None], 3, -1))
    relief = 0.0015 * tone + 0.0003 * grain - 0.0015 * cv2.GaussianBlur(pits, (0, 0), 1.0)
    _write_normal(paths[1], relief, px / 0.4)          # the texture spans ~0.4 m of rock surface
    return paths


def mural_texture(width_m, height_m, ppm=500):
    """The printed backdrop: black panels with grey lunar ridgelines in a band at mid-height,
    each ridge lit along its crest and fading to black below (footage)."""
    import cv2
    path = os.path.join(CACHE, f'mural_{width_m:.2f}_{height_m:.2f}_{ppm}_v2.png')
    if os.path.exists(path):
        return path
    os.makedirs(CACHE, exist_ok=True)
    rng = np.random.default_rng(29)
    w, h = int(width_m * ppm), int(height_m * ppm)
    z = (np.arange(h, dtype=np.float32)[None, :] + 0.5) / ppm       # height above the mural bottom
    xs = np.arange(w, dtype=np.float32) / ppm
    img = np.full((w, h), 0.006, np.float32)                        # near-black print (linear)
    for k in range(9):                                              # far ridges first, near ones over them
        frac = k / 8
        base = height_m * (0.72 - 0.5 * frac) + rng.uniform(-0.05, 0.05)
        ridge = (base + 0.04 * np.sin(2 * np.pi * xs / rng.uniform(1.5, 4.0) + rng.uniform(0, 6.3))
                 + 0.025 * np.interp(xs, np.linspace(0, width_m, 64), rng.normal(size=64)))
        below = ridge[:, None] - z
        lit = rng.uniform(0.05, 0.16) * np.exp(-np.clip(below, 0, None) / rng.uniform(0.03, 0.08))
        segment = (np.sin(2 * np.pi * xs / rng.uniform(0.8, 2.0) + rng.uniform(0, 6.3)) > rng.uniform(-0.6, 0.2))
        cover = (below >= 0) & segment[:, None]
        img = np.where(cover, np.maximum(lit, 0.006), img)
    img = cv2.GaussianBlur(img, (0, 0), 1.5)
    _write_rgb(path, np.repeat(img[..., None], 3, -1))
    return path


# ---------------------------------------------------------------- materials / prims
def pbr(stage, path, rgb, roughness=0.9, metallic=0.0, albedo=None, normal=None, emission=None,
        emission_gain=1.0, albedo_scale=None):
    """UsdPreviewSurface; `albedo_scale` (rgb) multiplies the albedo texture (tints a shared texture)."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + '/pbr')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
    if albedo or normal or emission:
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
        if albedo_scale is not None:
            tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*albedo_scale, 1))
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    else:
        sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    if emission:
        tex = texture('emission', emission, False)
        tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(emission_gain, emission_gain, emission_gain, 1))
        sh.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
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


def rock_mesh(stage, path, stone, ground_z, mat):
    """Faceted stone: intersection of random half-spaces with softly rounded edges (angular,
    flat-faced blocks as in the footage), set into the bed with its top at the visible height.
    Convex-hull collider."""
    x, y, a, bw, h, yaw, tilt, seed = stone
    rng = np.random.default_rng(seed)
    nu, nv = 56, 30
    u = np.linspace(0, 2 * np.pi, nu, endpoint=False)
    v = np.linspace(0.03, np.pi - 0.03, nv)
    U, V = np.meshgrid(u, v, indexing='ij')
    d = np.stack([np.cos(U) * np.sin(V), np.sin(U) * np.sin(V), np.cos(V)], -1)
    normals = rng.normal(size=(11, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.vstack([normals, [[0, 0, 1]], [[0, 0, -1]]])      # flat-ish top and bed
    offsets = np.concatenate([rng.uniform(0.82, 1.0, 11), [rng.uniform(0.8, 0.95)], [0.8]])
    dn = d @ normals.T
    t = np.where(dn > 1e-3, offsets / np.maximum(dn, 1e-3), 10.0)  # ray distance to each plane
    tau = 0.07                                                      # edge rounding (soft-min): worn edges
    r = -tau * np.log(np.exp(-(t - t.min(-1, keepdims=True)) / tau).sum(-1)) + t.min(-1)
    for _ in range(3):                                              # gentle lumps on the faces
        k = rng.normal(size=3)
        r *= 1 + 0.025 * np.cos(rng.uniform(2.0, 4.0) * (d @ (k / np.linalg.norm(k))) + rng.uniform(0, 6.3))
    r = np.clip(r, 0.45, 1.6)
    c = h * 0.75                                                    # ~1/3 of the stone below grade
    P = (d * r[..., None]).reshape(-1, 3)
    P = P / np.abs(P).max(0) * np.array([a, bw, c])                 # fit the requested extents
    cy, sy_, ct, st_ = math.cos(yaw), math.sin(yaw), math.cos(tilt), math.sin(tilt)
    R = np.array([[cy, -sy_, 0], [sy_, cy, 0], [0, 0, 1]]) @ np.array([[1, 0, 0], [0, ct, -st_], [0, st_, ct]])
    P = P @ R.T
    P += np.array([x, y, ground_z + h - P[:, 2].max()])
    # Poles of the parameterisation: centroid of the first / last ring.
    ring0, ring1 = P.reshape(nu, nv, 3)[:, 0].mean(0), P.reshape(nu, nv, 3)[:, -1].mean(0)
    top, bot = len(P), len(P) + 1
    P = np.vstack([P, ring0, ring1])
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
    # Spherical UVs (the rock texture is periodic), offset per stone so no two look alike.
    s0, t0 = rng.random(2)
    st = np.concatenate([np.stack([U.ravel() / (2 * np.pi) * 2 + s0, V.ravel() / np.pi + t0], 1),
                         [[s0, t0], [s0, 1 + t0]]]).astype(np.float32)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(st))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    return mesh


def range_pole(stage, path, x, y, ground_z, mats):
    """Red/white survey pole pushed into the bed (2.5 cm, POLE_HEIGHT above grade)."""
    z = ground_z + A.POLE_HEIGHT
    for k, (colour, length) in enumerate(STYLE['pole_bands']):
        z1 = max(z - length, ground_z - 0.3)
        rod(stage, f'{path}/band_{k}', (x, y, z1), (x, y, z), A.POLE_R, mats[colour], collide=True)
        z = z1


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
    alb, nrm, emit = bp1_textures(seed)
    bp1 = pbr(stage, f'{root}/Looks/bp1', STYLE['bp1_rgb'], roughness=0.97, albedo=alb, normal=nrm,
              emission=emit, emission_gain=STYLE['laser_emission'])
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
    rock_alb, rock_nrm = rock_textures()
    rocks = [pbr(stage, f'{root}/Looks/rock_{k}', rgb, roughness=0.85, albedo=rock_alb, normal=rock_nrm,
                 albedo_scale=tuple(2 * v for v in rgb)) for k, rgb in enumerate(STYLE['rock_rgb'])]
    pole_mats = {'r': pbr(stage, f'{root}/Looks/pole_red', (0.55, 0.02, 0.02), roughness=0.4),
                 'w': pbr(stage, f'{root}/Looks/pole_white', (0.8, 0.8, 0.8), roughness=0.4)}

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
    # Printed lunar mural on the long side opposite the start zone and berm, from the bin wall up
    # to the truss, behind the frames (which continue up to its top rail).
    mt = STYLE['mural_top']
    mural = UsdGeom.Mesh.Define(stage, f'{root}/enclosure/mural')
    ym = W + o + 0.03
    mural.CreatePointsAttr([(-o, ym, hw), (L + o, ym, hw), (L + o, ym, mt), (-o, ym, mt)])
    mural.CreateFaceVertexCountsAttr([4])
    mural.CreateFaceVertexIndicesAttr([0, 1, 2, 3])                   # faces -y, into the arena
    mural.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(mural).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.vertex).Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    UsdShade.MaterialBindingAPI.Apply(mural.GetPrim()).Bind(
        pbr(stage, f'{root}/Looks/mural', (0.01, 0.01, 0.01), roughness=0.55,
            albedo=mural_texture(L + 2 * o, mt - hw)))
    n = max(int((L + 2 * o) / 1.2), 1)
    for m in range(n + 1):
        box(stage, f'{root}/enclosure/mural_frame_{m}', (-o + (L + 2 * o) * m / n, W + o, (gt + mt) / 2),
            (0.05, 0.05, mt - gt), black, collide=False)
    box(stage, f'{root}/enclosure/mural_top', (L / 2, W + o, mt), (L + 2 * o + 0.05, 0.05, 0.05), black, collide=False)
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
        for k3, s in enumerate(A.stones(b)):
            rock_mesh(stage, f'{root}/boulders/boulder_{k2}/stone_{k3}', s, terrain.height_at(s[0], s[1]),
                      rocks[s[7] % len(rocks)])
    for k2, (x, y) in enumerate(A.POLES):
        range_pole(stage, f'{root}/poles/pole_{k2}', x, y, terrain.height_at(x, y), pole_mats)

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
