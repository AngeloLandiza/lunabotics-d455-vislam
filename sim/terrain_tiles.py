"""Deformable heightmap terrain for Isaac Sim, split into tile meshes.

A dig only rewrites (and re-cooks the collision of) the few tiles it touches, so
terrain edits stay cheap even with a fine 2 cm grid.
"""
import numpy as np
from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade, Vt


class TerrainTiles:
    def __init__(self, stage, root, xs, ys, H, tile=32, uv_size=None, material=None,
                 physics_material=None, collision=True):
        self.stage, self.root, self.xs, self.ys, self.H = stage, root, xs, ys, H
        self.res = float(xs[1] - xs[0])
        self.tile = tile
        self.uv_size = uv_size or max(xs[-1] - xs[0], ys[-1] - ys[0])
        self.meshes = {}
        UsdGeom.Xform.Define(stage, root)
        for ti in range(0, len(xs) - 1, tile):
            for tj in range(0, len(ys) - 1, tile):
                mesh = self._make(ti, tj)
                if material is not None:
                    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
                if collision:
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
                    if physics_material is not None:
                        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
                            physics_material, UsdShade.Tokens.weakerThanDescendants, 'physics')
                self.meshes[(ti, tj)] = mesh

    def _span(self, ti, tj):
        return ti, min(ti + self.tile, len(self.xs) - 1), tj, min(tj + self.tile, len(self.ys) - 1)

    def _points_normals(self, ti, tj):
        i0, i1, j0, j1 = self._span(ti, tj)
        X, Y = np.meshgrid(self.xs[i0:i1 + 1], self.ys[j0:j1 + 1], indexing='ij')
        Z = self.H[i0:i1 + 1, j0:j1 + 1]
        # Normals from the global map (one cell of context) so tile seams shade smoothly.
        a0, a1 = max(i0 - 1, 0), min(i1 + 2, len(self.xs))
        b0, b1 = max(j0 - 1, 0), min(j1 + 2, len(self.ys))
        gx, gy = np.gradient(self.H[a0:a1, b0:b1].astype(np.float64), self.res)
        gx, gy = gx[i0 - a0:i0 - a0 + Z.shape[0], j0 - b0:j0 - b0 + Z.shape[1]], \
            gy[i0 - a0:i0 - a0 + Z.shape[0], j0 - b0:j0 - b0 + Z.shape[1]]
        n = np.stack([-gx.ravel(), -gy.ravel(), np.ones(Z.size)], 1)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        return pts.astype(np.float32), n.astype(np.float32), X, Y

    def _make(self, ti, tj):
        i0, i1, j0, j1 = self._span(ti, tj)
        mesh = UsdGeom.Mesh.Define(self.stage, f'{self.root}/tile_{ti}_{tj}')
        pts, nrm, X, Y = self._points_normals(ti, tj)
        ni, nj = i1 - i0 + 1, j1 - j0 + 1
        a, b = np.meshgrid(np.arange(ni - 1), np.arange(nj - 1), indexing='ij')
        v0 = (a * nj + b).ravel()
        quads = np.stack([v0, v0 + nj, v0 + nj + 1, v0 + 1], 1)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(quads), 4, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(quads.ravel().astype(np.int32)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(nrm))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateSubdivisionSchemeAttr('none')
        uv = np.stack([(X.ravel() - self.xs[0]) / self.uv_size, (Y.ravel() - self.ys[0]) / self.uv_size], 1)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                                     UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        lo, hi = pts.min(0), pts.max(0)
        mesh.CreateExtentAttr([tuple(map(float, lo)), tuple(map(float, hi))])
        return mesh

    def update(self, i0, i1, j0, j1):
        """Rewrite every tile overlapping the changed index window [i0, i1) x [j0, j1)."""
        t = self.tile
        n = 0
        for ti in range((max(i0 - 1, 0) // t) * t, min(i1 + 1, len(self.xs) - 1), t):
            for tj in range((max(j0 - 1, 0) // t) * t, min(j1 + 1, len(self.ys) - 1), t):
                mesh = self.meshes.get((ti, tj))
                if mesh is None:
                    continue
                pts, nrm, _, _ = self._points_normals(ti, tj)
                mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts))
                mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(nrm))
                lo, hi = pts.min(0), pts.max(0)
                mesh.GetExtentAttr().Set([tuple(map(float, lo)), tuple(map(float, hi))])
                n += 1
        return n

    def height_at(self, x, y):
        i = int(np.clip(round((x - self.xs[0]) / self.res), 0, len(self.xs) - 1))
        j = int(np.clip(round((y - self.ys[0]) / self.res), 0, len(self.ys) - 1))
        return float(self.H[i, j])
