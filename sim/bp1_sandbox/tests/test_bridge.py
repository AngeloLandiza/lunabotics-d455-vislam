"""Checks for isaac_bridge.py. Run from the sandbox folder: python3 tests/test_bridge.py

1. The BP-1 layer gives identical output whether the ideal hits come from our ray marcher or from outside
   (the path Isaac Sim output will take), in clear air and in dust, for the LiDAR and the D455.
2. The USD export reopens, and its mesh heights match the sandbox height field.
"""
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arena import Arena                                    # noqa: E402
from dust import Plume                                     # noqa: E402
from isaac_bridge import export_arena_usd, lidar_from_ideal, d455_from_ideal   # noqa: E402
from raycast import march                                  # noqa: E402
from sensors import Lidar, D455Depth, sensor_pose          # noqa: E402


def same(a, b):
    return np.array_equal(np.nan_to_num(a, nan=-9.0), np.nan_to_num(b, nan=-9.0))


def check_lidar(arena, robot, dust):
    for name in ("spin16", "rosette70"):
        lid = Lidar(name)
        ref = lid.scan(arena, robot, np.random.default_rng(7), dust=dust)
        # ideal hits computed separately, as Isaac Sim would provide them; same rng stream for the layer
        rng = np.random.default_rng(7)
        if name == "rosette70":
            from sensors import rosette_dirs
            rosette_dirs(lid.prof, rng)    # consume the draw the scan used for the pattern phase
        dirs = ref["dirs"]
        t = march(arena, ref["origin"], dirs, lid.rmax)["t_hit"]
        P = ref["origin"] + np.nan_to_num(t)[:, None] * dirs
        out = lidar_from_ideal(lid, ref["origin"], dirs, t, arena.normal(P[:, 0], P[:, 1]), rng, dust=dust)
        assert same(ref["r"], out["r"]), f"{name} ranges differ (dust={dust is not None})"
        assert np.array_equal(ref["is_dust"], out["is_dust"]), f"{name} dust flags differ"
        print(f"ok  {name:9s} dust={dust is not None}: {ref['valid'].sum()} returns, {ref['is_dust'].sum()} dust echoes, identical")


def check_d455(arena, robot, dust):
    cam = D455Depth()
    ref = cam.scan(arena, robot, np.random.default_rng(9), dust=dust)
    origin, R = sensor_pose(robot, cam.mount, cam.pitch)
    dirs = cam.d_s @ R.T
    t = march(arena, origin, dirs, cam.zmax / cam.d_s[:, 0].min())["t_hit"]
    ideal = (t * cam.d_s[:, 0]).reshape(cam.h, cam.w)
    out = d455_from_ideal(cam, origin, R, ideal, np.random.default_rng(9), dust=dust)
    assert same(ref["depth_img"], out["depth_img"]), f"D455 depth differs (dust={dust is not None})"
    print(f"ok  d455      dust={dust is not None}: {ref['valid'].sum()} valid pixels, identical")


def check_usd(arena):
    from pxr import Usd, UsdGeom
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "arena.usda")
        n_pts, n_tri = export_arena_usd(arena, path, stride=2)
        stage = Usd.Stage.Open(path)
        mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/Arena/Terrain"))
        pts = np.array(mesh.GetPointsAttr().Get())
        assert len(pts) == n_pts and len(mesh.GetFaceVertexCountsAttr().Get()) == n_tri
        assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z and UsdGeom.GetStageMetersPerUnit(stage) == 1.0
        inside = (pts[:, 0] > 0.05) & (pts[:, 0] < arena.L - 0.05) & (pts[:, 1] > 0.05) & (pts[:, 1] < arena.W - 0.05)
        err = np.abs(pts[inside, 2] - arena.height(pts[inside, 0], pts[inside, 1]))
        assert err.max() < 1e-4, err.max()
        size = os.path.getsize(path) / 1e6
        print(f"ok  usd export: {n_pts} vertices, {n_tri} triangles, {size:.1f} MB, max height error {err.max():.1e} m")


if __name__ == "__main__":
    A = Arena(np.random.default_rng(1))
    robot = (1.0, 2.5, 0.0)
    for dust in (None, Plume(1.8, 2.5)):
        check_lidar(A, robot, dust)
        check_d455(A, robot, dust)
    check_usd(A)
    print("all bridge checks passed")
