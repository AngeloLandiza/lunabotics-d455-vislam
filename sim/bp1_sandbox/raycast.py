"""Ray marching against the arena height field, with dust optical depth and free-space samples."""
import numpy as np


def march(arena, origin, dirs, rmax, step=0.02, dust=None, chunk=3000):
    """Cast rays from one origin.

    Returns a dict with, per ray:
      t_hit    range to the terrain (nan if no hit within rmax)
      tau_hit  dust optical depth from the sensor to the hit (0 if no dust)
    and the march grid (ts) plus per-chunk callbacks are avoided by returning
      samples: list of (ray_index, t, x, y, z, tau) arrays for free-space carving and clutter, only for t < t_hit.
    """
    origin = np.asarray(origin, float)
    n = len(dirs)
    ts = np.arange(step, rmax + step, step)
    T = len(ts)
    t_hit = np.full(n, np.nan)
    tau_hit = np.zeros(n)
    samples = []
    for s in range(0, n, chunk):
        d = dirs[s:s + chunk]
        m = len(d)
        P = origin[None, None, :] + ts[None, :, None] * d[:, None, :]
        zt = arena.height(P[..., 0], P[..., 1])
        above = P[..., 2] - zt
        below = above <= 0
        has = below.any(axis=1)
        idx = np.where(has, below.argmax(axis=1), T)
        # linear refine between the last sample above and the first below
        th = np.full(m, np.nan)
        ok = has & (idx > 0)
        i1 = idx[ok]
        a0 = above[ok, i1 - 1]
        a1 = above[ok, i1]
        th[ok] = ts[i1 - 1] + step * a0 / (a0 - a1)
        t_hit[s:s + m] = th
        if dust is not None:
            alpha = dust.alpha(P[..., 0], P[..., 1], P[..., 2])
            tau = np.cumsum(alpha, axis=1) * step
            tau_hit[s:s + m] = np.where(ok, tau[np.arange(m), np.clip(idx - 1, 0, T - 1)], 0.0)
        else:
            alpha = None
            tau = None
        keep = np.arange(T)[None, :] < idx[:, None]
        r_i, t_i = np.nonzero(keep)
        rec = {
            "ray": r_i + s, "t": ts[t_i],
            "x": P[r_i, t_i, 0], "y": P[r_i, t_i, 1], "z": P[r_i, t_i, 2],
        }
        if dust is not None:
            rec["alpha"] = alpha[r_i, t_i]
            rec["tau"] = tau[r_i, t_i]
        samples.append(rec)
    merged = {k: np.concatenate([r[k] for r in samples]) for k in samples[0]}
    return {"t_hit": t_hit, "tau_hit": tau_hit, "samples": merged, "step": step}


def flat_ground_range(origin, dirs, z0=0.0):
    """Range at which each ray would meet a flat plane z = z0 (nan if it never does)."""
    dz = dirs[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (z0 - origin[2]) / dz
    return np.where((dz < -1e-6) & (t > 0), t, np.nan)
