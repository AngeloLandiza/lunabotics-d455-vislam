"""Bridge to Isaac Sim: export a sandbox arena as USD, and add the BP-1 sensor layer to ideal sensor output.

The pages read in step 2 document no LiDAR range noise, no D455 stereo noise, no non-repetitive (Livox-style)
LiDAR pattern, and no dust effect on RTX LiDAR (one forum reply, Isaac Sim 4.2.0). So the plan is a hybrid:
Isaac Sim supplies geometry and ideal hits, and this module adds BP-1 reflectivity, noise and dust with the same
code paths as sensors.py. tests/test_bridge.py checks that both paths give identical output for identical input.

Nothing here imports Isaac Sim. The export needs the Pixar USD bindings (`pip install usd-core`; Isaac Sim ships
its own). How to read ideal hits out of Isaac Sim is not covered here, because no Isaac Sim was available to test it.
"""
import json

import numpy as np

from dust import NoDust


# ---------------------------------------------------------------- USD export

def export_arena_usd(arena, path, stride=2, color=(0.25, 0.25, 0.25)):
    """Write the arena height field (terrain, boulders, craters, column, walls) as one triangle mesh.

    Units are metres with Z up, matching the sandbox frame: origin at the start-zone corner, x along the 6.8 m side.
    stride=2 samples the 1 cm height map every 2 cm. The obstacle ground truth goes to <path>.truth.json.
    The display color is cosmetic; LiDAR reflectivity is applied by this module, not by the USD material.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    h = arena.h[::stride, ::stride]
    nx, ny = h.shape
    xs = arena.x0 + np.arange(nx) * arena.res * stride
    ys = arena.y0 + np.arange(ny) * arena.res * stride
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel(), h.ravel()], 1)

    i, j = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
    a = (i * ny + j).ravel()
    b, c, d = a + ny, a + ny + 1, a + 1
    tris = np.stack([a, b, c, a, c, d], 1).reshape(-1, 3)   # counter-clockwise seen from +z

    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Arena")
    stage.SetDefaultPrim(root.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, "/Arena/Terrain")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(tris), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(tris.ravel().astype(np.int32)))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0).tolist()), Gf.Vec3f(*pts.max(0).tolist())])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    # Static collider so a robot can drive on it (UsdPhysics schema; not yet tried in Isaac Sim)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)

    truth = [{"kind": o.kind, "x": float(o.x), "y": float(o.y), "radius": float(o.radius), "size": float(o.size)}
             for o in arena.obstacles]
    root.GetPrim().SetCustomDataByKey("sandbox:obstacles", json.dumps(truth))
    root.GetPrim().SetCustomDataByKey("sandbox:arena_size_m", Gf.Vec2d(arena.L, arena.W))
    stage.GetRootLayer().Save()
    with open(path + ".truth.json", "w") as f:
        json.dump({"arena_size_m": [arena.L, arena.W], "obstacles": truth}, f, indent=1)
    return len(pts), len(tris)


# ---------------------------------------------------------------- BP-1 layer on ideal hits

def dust_march(origin, dirs, t_stop, dust, rmax, step=0.02):
    """Free-space samples and optical depth along each ray, stopping at the ideal hit (or rmax when no hit).

    Same sample grid and sums as raycast.march, without the terrain, so it can follow hits that came from Isaac Sim.
    """
    dust = dust or NoDust()
    origin = np.asarray(origin, float)
    ts = np.arange(step, rmax + step, step)
    stop = np.where(np.isfinite(t_stop), t_stop, np.inf)
    keep = ts[None, :] < stop[:, None]
    # raycast.march keeps every sample before the first one at or below the ground, i.e. ts < t_hit
    r_i, t_i = np.nonzero(keep)
    P = origin[None, :] + ts[t_i, None] * dirs[r_i]
    alpha = dust.alpha(P[:, 0], P[:, 1], P[:, 2])
    tau_full = np.zeros((len(dirs), len(ts)))
    tau_full[r_i, t_i] = alpha
    tau_full = np.cumsum(tau_full, axis=1) * step
    n_keep = keep.sum(1)
    tau_hit = np.where(np.isfinite(t_stop) & (n_keep > 0), tau_full[np.arange(len(dirs)), np.maximum(n_keep - 1, 0)], 0.0)
    samples = {"ray": r_i, "t": ts[t_i], "x": P[:, 0], "y": P[:, 1], "z": P[:, 2],
               "alpha": alpha, "tau": tau_full[r_i, t_i]}
    return {"t_hit": np.asarray(t_stop, float), "tau_hit": tau_hit, "samples": samples, "step": step}


def lidar_from_ideal(lidar, origin, dirs, t_ideal, normals, rng, dust=None):
    """Apply the BP-1 LiDAR layer to ideal ranges.

    origin (3,), dirs (N,3) unit vectors in the arena frame, t_ideal (N,) ideal ranges with nan for no return,
    normals (N,3) surface normals at the hits (pass None to assume normal incidence, which overstates return power).
    """
    dirs = np.asarray(dirs, float)
    if normals is None:
        normals = -dirs
    m = dust_march(origin, dirs, t_ideal, dust, lidar.rmax)
    return lidar.apply(np.asarray(origin, float), dirs, np.asarray(t_ideal, float), np.asarray(normals, float),
                       m, rng, dust_on=dust is not None)


def d455_from_ideal(cam, origin, R, depth_img, rng, dust=None):
    """Apply the BP-1 D455 layer to an ideal depth image.

    depth_img (h, w): distance to the image plane along the optical axis in metres, nan where nothing was hit,
    row 0 at the top and column 0 at the left. Its shape must equal (cam.h, cam.w) from params.D455 sim_size.
    R is the world-from-camera rotation with x forward, y left, z up (sensors.rot gives it for a yaw and pitch).
    """
    if depth_img.shape != (cam.h, cam.w):
        raise ValueError(f"depth image is {depth_img.shape}, the camera model expects {(cam.h, cam.w)}; "
                         "set D455 sim_size in params.py or resample the image")
    dirs = cam.d_s @ np.asarray(R).T
    z = depth_img.reshape(-1).astype(float)
    m = dust_march(origin, dirs, z / cam.d_s[:, 0], dust, cam.zmax / cam.d_s[:, 0].min())
    return cam.apply(np.asarray(origin, float), dirs, z, m, rng)


if __name__ == "__main__":
    # python3 isaac_bridge.py [seed] [out.usda]  ->  one random arena as USD, plus <out>.truth.json
    import sys
    from arena import Arena
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    out = sys.argv[2] if len(sys.argv) > 2 else f"usd/arena_seed{seed}.usda"
    import os
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    n, t = export_arena_usd(Arena(np.random.default_rng(seed)), out)
    print(f"{out}: {n} vertices, {t} triangles")
