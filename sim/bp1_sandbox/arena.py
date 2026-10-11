"""Synthetic 2027 arena as a height field: rough BP-1 surface, gravel, boulders, craters, column, walls."""
from dataclasses import dataclass
import numpy as np
import params as pr
from params import v


@dataclass
class Obstacle:
    kind: str      # "boulder", "crater" or "column"
    x: float
    y: float
    radius: float
    size: float    # boulder height or crater depth (positive), column height


class Arena:
    WALL_H = 1.2   # walls drawn as tall cells so rays stop at the arena edge

    def __init__(self, rng, n_boulders=None, n_craters=None):
        self.res = v(pr.ARENA, "grid_res")
        self.L = v(pr.ARENA, "length_x")
        self.W = v(pr.ARENA, "width_y")
        pad = 0.10
        self.x0, self.y0 = -pad, -pad
        nx = int(round((self.L + 2 * pad) / self.res)) + 1
        ny = int(round((self.W + 2 * pad) / self.res)) + 1
        xs = self.x0 + np.arange(nx) * self.res
        ys = self.y0 + np.arange(ny) * self.res
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        self.X, self.Y = X, Y
        h = self._rough_surface(rng, X, Y)
        h += self._gravel(rng, X, Y)
        self.label = np.zeros_like(h, dtype=np.int16)   # 0 ground, k>0 obstacle index + 1
        self.obstacles = []
        nb = n_boulders if n_boulders is not None else v(pr.OBSTACLES, "n_boulders")
        nc = n_craters if n_craters is not None else v(pr.OBSTACLES, "n_craters")
        cx, cy, cr = v(pr.ARENA, "column_xy_r")
        self._add(h, Obstacle("column", cx, cy, cr, self.WALL_H))
        for kind in ["boulder"] * nb + ["crater"] * nc:
            self._add(h, self._sample(rng, kind))
        inside = (X >= 0) & (X <= self.L) & (Y >= 0) & (Y <= self.W)
        h[~inside] = self.WALL_H
        self.h = h

    def _rough_surface(self, rng, X, Y):
        rms = v(pr.ARENA, "surface_rms")
        corr = v(pr.ARENA, "surface_corr_len")
        n = 40
        k = rng.normal(0, 1 / corr, size=(n, 2))
        ph = rng.uniform(0, 2 * np.pi, n)
        s = sum(np.cos(k[i, 0] * X + k[i, 1] * Y + ph[i]) for i in range(n))
        return s * rms / np.sqrt(n / 2)

    def _gravel(self, rng, X, Y):
        d = v(pr.ARENA, "gravel_diam")
        n = rng.poisson(v(pr.ARENA, "gravel_per_m2") * self.L * self.W)
        g = np.zeros_like(X)
        for gx, gy in zip(rng.uniform(0, self.L, n), rng.uniform(0, self.W, n)):
            r2 = (X - gx) ** 2 + (Y - gy) ** 2
            g = np.maximum(g, np.sqrt(np.clip((d / 2) ** 2 - r2, 0, None)))
        return g

    def _sample(self, rng, kind):
        x0, x1 = v(pr.ARENA, "obstacle_zone_x")
        sep = v(pr.OBSTACLES, "min_separation")
        for _ in range(1000):
            x, y = rng.uniform(x0, x1), rng.uniform(0.4, self.W - 0.4)
            if all(np.hypot(x - o.x, y - o.y) > sep for o in self.obstacles):
                break
        if kind == "boulder":
            r = rng.uniform(*v(pr.OBSTACLES, "boulder_diam")) / 2
            s = rng.uniform(*v(pr.OBSTACLES, "boulder_height"))
        else:
            r = rng.uniform(*v(pr.OBSTACLES, "crater_diam")) / 2
            s = rng.uniform(*v(pr.OBSTACLES, "crater_depth"))
        return Obstacle(kind, x, y, r, s)

    def _add(self, h, o):
        r = np.hypot(self.X - o.x, self.Y - o.y)
        idx = len(self.obstacles) + 1
        if o.kind == "boulder":        # half-ellipsoid
            bump = o.size * np.sqrt(np.clip(1 - (r / o.radius) ** 2, 0, None))
            h[:] = np.maximum(h, bump)
            self.label[r < o.radius] = idx
        elif o.kind == "column":
            h[r < o.radius] = o.size
            self.label[r < o.radius] = idx
        else:                          # paraboloid bowl plus a small raised rim
            bowl = -o.size * np.clip(1 - (r / o.radius) ** 2, 0, None)
            rim = v(pr.OBSTACLES, "crater_rim_frac") * o.size * np.exp(-((r - o.radius) / (0.3 * o.radius)) ** 2)
            h[:] = np.where(r < o.radius, bowl, h + rim)
            self.label[r < o.radius] = idx
        self.obstacles.append(o)

    def height(self, x, y):
        """Bilinear height lookup; points outside the grid get wall height."""
        fx = (x - self.x0) / self.res
        fy = (y - self.y0) / self.res
        nx, ny = self.h.shape
        out = (fx < 0) | (fy < 0) | (fx > nx - 2) | (fy > ny - 2)
        fx = np.clip(fx, 0, nx - 2.0001)
        fy = np.clip(fy, 0, ny - 2.0001)
        i, j = fx.astype(np.int64), fy.astype(np.int64)
        ax, ay = fx - i, fy - j
        h = self.h
        z = (h[i, j] * (1 - ax) * (1 - ay) + h[i + 1, j] * ax * (1 - ay)
             + h[i, j + 1] * (1 - ax) * ay + h[i + 1, j + 1] * ax * ay)
        return np.where(out, self.WALL_H, z)

    def label_at(self, x, y):
        i = np.clip(np.round((x - self.x0) / self.res).astype(np.int64), 0, self.h.shape[0] - 1)
        j = np.clip(np.round((y - self.y0) / self.res).astype(np.int64), 0, self.h.shape[1] - 1)
        return self.label[i, j]

    def normal(self, x, y, eps=0.01):
        dzdx = (self.height(x + eps, y) - self.height(x - eps, y)) / (2 * eps)
        dzdy = (self.height(x, y + eps) - self.height(x, y - eps)) / (2 * eps)
        n = np.stack([-dzdx, -dzdy, np.ones_like(dzdx)], axis=-1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)
