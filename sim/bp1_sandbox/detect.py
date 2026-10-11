"""Boulder and crater detectors on a 2.5D grid.

baseline: fixed height thresholds on the highest and lowest point in each cell.
carve:    baseline plus three additions (all Inferred, ideas from the step 2 research):
  1. thresholds scale with each point's range-dependent noise (3 sigma), a CFAR-style normalisation;
  2. free-space carving: every ray that returned a hit proves the terrain under its path is lower than the ray,
     which gives an upper bound on cell height and exposes crater floors no ray reached;
  3. solidity check: a high return in a cell that some other ray passed beneath cannot be a solid boulder,
     so it is dropped as dust or noise.
logodds:  per-cell evidence counted ray by ray and frame by frame, with Pairet et al.'s log-odds steps and clamps.
  Boulders: +l_occ per return above the ground by max(h_pos, 3 vertical sigma); l_free per distinct ray that
  passed beneath the cell's highest return. Craters: +l_occ per return, or distinct ray in free space, below the
  ground by max(d_neg, 3 vertical sigma); l_free per return on the ground. Free space below the ground plane is the
  height-map form of Hines et al.'s virtual surfaces: it bounds hidden ground from above (Inferred adaptation).
"""
import numpy as np
from scipy import ndimage
import params as pr
from params import v


class Grid:
    def __init__(self, L, W, cell):
        self.cell = cell
        self.nx, self.ny = int(np.ceil(L / cell)), int(np.ceil(W / cell))
        self.L, self.W = L, W

    def idx(self, x, y):
        i = np.floor(x / self.cell).astype(np.int64)
        j = np.floor(y / self.cell).astype(np.int64)
        ok = (i >= 1) & (j >= 1) & (i < self.nx - 1) & (j < self.ny - 1)   # drop the wall-adjacent ring
        return i, j, ok

    def centers(self):
        xs = (np.arange(self.nx) + 0.5) * self.cell
        ys = (np.arange(self.ny) + 0.5) * self.cell
        return np.meshgrid(xs, ys, indexing="ij")


def accumulate(scans, grid, carve=True):
    shape = (grid.nx, grid.ny)
    L = {
        "count": np.zeros(shape), "zmax": np.full(shape, -np.inf), "zmin": np.full(shape, np.inf),
        "zsum": np.zeros(shape), "sig": np.full(shape, np.inf), "carve": np.full(shape, np.inf),
        "expected": np.zeros(shape),
    }
    for s in scans:
        P, ok = s["pts"], s["valid"]
        i, j, inside = grid.idx(P[:, 0], P[:, 1])
        m = ok & inside
        np.add.at(L["count"], (i[m], j[m]), 1)
        np.maximum.at(L["zmax"], (i[m], j[m]), P[m, 2])
        np.minimum.at(L["zmin"], (i[m], j[m]), P[m, 2])
        np.add.at(L["zsum"], (i[m], j[m]), P[m, 2])
        np.minimum.at(L["sig"], (i[m], j[m]), s["sigma"][m])
        # cells a flat floor would have put returns in (for "in view" bookkeeping)
        rf = s["r_flat"]
        okf = np.isfinite(rf) & (rf < 8.0)
        F = s["origin"] + np.nan_to_num(rf)[:, None] * s["dirs"]
        fi, fj, fin = grid.idx(F[:, 0], F[:, 1])
        mf = okf & fin
        np.add.at(L["expected"], (fi[mf], fj[mf]), 1)
        if carve:
            sm = s["samples"]
            r = s["r"][sm["ray"]]
            sig = np.nan_to_num(s["sigma"][sm["ray"]], nan=1.0)
            free = np.isfinite(r) & (sm["t"] < r - (v(pr.DETECT, "carve_margin") + 3 * sig))
            ci, cj, cin = grid.idx(sm["x"], sm["y"])
            mc = free & cin
            np.minimum.at(L["carve"], (ci[mc], cj[mc]), sm["z"][mc])
    return L


def fit_ground(L, grid):
    X, Y = grid.centers()
    has = L["count"] > 0
    z = np.where(has, L["zsum"] / np.maximum(L["count"], 1), np.nan)
    m = has & (np.abs(z) < 0.15)
    A = np.c_[X[m], Y[m], np.ones(m.sum())]
    b = z[m]
    keep = np.ones(len(b), bool)
    coef = np.zeros(3)
    for _ in range(4):
        if keep.sum() < 10:
            break
        coef, *_ = np.linalg.lstsq(A[keep], b[keep], rcond=None)
        res = b - A @ coef
        mad = np.median(np.abs(res[keep] - np.median(res[keep]))) + 1e-4
        keep = np.abs(res) < 3 * 1.4826 * mad
    return coef[0] * X + coef[1] * Y + coef[2]


def _ray_cells(grid, rays, ci, cj, mask):
    """Distinct (cell, ray) pairs among masked samples, returned as cell indices (one entry per ray per cell)."""
    if not mask.any():
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    key = (ci[mask] * grid.ny + cj[mask]) * (rays.max() + 1) + rays[mask]
    u = np.unique(key) // (rays.max() + 1)
    return u // grid.ny, u % grid.ny


def logodds_layers(scans, grid, L, g):
    """Per-cell log-odds for boulders (lp) and craters (ln), updated frame by frame and clamped."""
    lo, lf = v(pr.DETECT, "l_occ"), v(pr.DETECT, "l_free")
    lmin, lmax = v(pr.DETECT, "l_min"), v(pr.DETECT, "l_max")
    h_pos, d_neg, margin = v(pr.DETECT, "h_pos"), v(pr.DETECT, "d_neg"), v(pr.DETECT, "carve_margin")
    shape = (grid.nx, grid.ny)
    lp, ln = np.zeros(shape), np.zeros(shape)
    zref = np.where(np.isfinite(L["zmax"]) & (L["zmax"] - g > h_pos), L["zmax"] - h_pos, g + h_pos)
    for s in scans:
        sig_r = np.nan_to_num(s["sigma_r"], nan=1.0)
        sv = sig_r * np.abs(s["dirs"][:, 2])                # vertical part of the range noise
        P, ok = s["pts"], s["valid"]
        i, j, inside = grid.idx(P[:, 0], P[:, 1])
        m = ok & inside
        i, j, dz, svm = i[m], j[m], P[m, 2] - g[i[m], j[m]], sv[m]
        tp, tn = np.maximum(h_pos, 3 * svm), np.maximum(d_neg, 3 * svm)
        hit, low, ground = dz > tp, dz < -tn, (dz >= -tn) & (dz <= tp)
        dp, dn = np.zeros(shape), np.zeros(shape)
        np.add.at(dp, (i[hit], j[hit]), lo)
        np.add.at(dn, (i[low], j[low]), lo)
        np.add.at(dn, (i[ground], j[ground]), lf)
        sm = s["samples"]
        r = s["r"][sm["ray"]]
        free = np.isfinite(r) & (sm["t"] < r - (margin + 3 * sig_r[sm["ray"]]))
        ci, cj, cin = grid.idx(sm["x"], sm["y"])
        ci, cj = np.clip(ci, 0, grid.nx - 1), np.clip(cj, 0, grid.ny - 1)
        free &= cin
        bi, bj = _ray_cells(grid, sm["ray"], ci, cj, free & (sm["z"] < zref[ci, cj]))
        np.add.at(dp, (bi, bj), lf)
        deep = free & (sm["z"] < g[ci, cj] - np.maximum(d_neg, 3 * sv[sm["ray"]]))
        di, dj = _ray_cells(grid, sm["ray"], ci, cj, deep)
        np.add.at(dn, (di, dj), lo)
        lp = np.clip(lp + dp, lmin, lmax)
        ln = np.clip(ln + dn, lmin, lmax)
    return lp, ln


def detect(L, grid, method="carve", scans=None):
    g = fit_ground(L, grid)
    h_pos, h_neg = v(pr.DETECT, "h_pos"), v(pr.DETECT, "h_neg")
    has = L["count"] > 0
    if method == "baseline":
        pos = has & (L["zmax"] - g > h_pos)
        neg = has & (L["zmin"] - g < -h_neg)
    elif method == "logodds":
        lp, ln = logodds_layers(scans, grid, L, g)
        pos, neg = lp > 0, ln > 0          # cells where support outweighs contradiction; clusters are judged below
        l_dec = v(pr.DETECT, "l_decide_d455" if scans[0]["sensor"] == "d455" else "l_decide_lidar")
    else:
        sig = np.where(np.isfinite(L["sig"]), L["sig"], 0.0)
        tp, tn = np.maximum(h_pos, 3 * sig), np.maximum(h_neg, 3 * sig)
        pos = has & (L["zmax"] - g > tp)
        hollow = L["carve"] < L["zmax"] - tp            # some ray passed beneath the high return
        pos &= ~hollow
        neg = (has & (L["zmin"] - g < -tn)) | (np.isfinite(L["carve"]) & (L["carve"] - g < -h_neg))
    out = []
    for kind, mask, mn in (("pos", pos, v(pr.DETECT, "min_cells_pos")), ("neg", neg, v(pr.DETECT, "min_cells_neg"))):
        lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
        for k in range(1, n + 1):
            ii, jj = np.nonzero(lab == k)
            if len(ii) < mn:
                continue
            if method == "logodds" and (lp if kind == "pos" else ln)[ii, jj].sum() < l_dec:
                continue                   # not enough pooled evidence over the whole object
            out.append({"kind": kind, "x": (ii.mean() + 0.5) * grid.cell, "y": (jj.mean() + 0.5) * grid.cell,
                        "cells": len(ii)})
    return out, {"pos": pos, "neg": neg, "ground": g}


def score(dets, arena, L, grid, sensor_xy):
    """Per-obstacle detection records, false alarms, and wall detections for one frame.

    Detections centred within wall_band of the arena edge are real wall returns, not obstacle false alarms,
    so they are counted separately.
    """
    margin = v(pr.DETECT, "match_margin")
    band = v(pr.DETECT, "wall_band")
    X, Y = grid.centers()
    records, used = [], set()
    for o in arena.obstacles:
        disk = np.hypot(X - o.x, Y - o.y) < o.radius
        in_view = bool((L["expected"][disk] > 0).any())
        want = "pos" if o.kind in ("boulder", "column") else "neg"
        hit = False
        for k, d in enumerate(dets):
            if d["kind"] == want and np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin:
                hit = True
                used.add(k)
        if o.kind != "column":
            records.append({"kind": o.kind, "range": float(np.hypot(o.x - sensor_xy[0], o.y - sensor_xy[1])),
                            "in_view": in_view, "detected": hit, "size": o.size, "radius": o.radius})
    fa, wall = 0, 0
    for k, d in enumerate(dets):
        if k in used:
            continue
        near = any(np.hypot(d["x"] - o.x, d["y"] - o.y) < o.radius + margin for o in arena.obstacles)
        if near:
            continue
        if min(d["x"], d["y"], arena.L - d["x"], arena.W - d["y"]) < band:
            wall += 1
        else:
            fa += 1
    return records, fa, wall
