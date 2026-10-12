"""Lunabotics Artemis Arena (2027 guidebook, sec. 11) as numpy data.

Shared by the Isaac Sim scene, the dig tool and the test driver, so all three agree.

Frame "arena": origin at the starting-zone corner on the inner face of the perimeter
ducts, x along the 6.88 m length toward the construction zone, y across the 5.0 m
width, z up, z = 0 at the raked regolith surface (guidebook Fig. 9).
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

LENGTH, WIDTH = 6.88, 5.0      # interior, between the 17 cm perimeter ducts
DUCT_D = 0.17
BED_DEPTH = 0.45               # BP-1 depth; the bin floor is below
RES = 0.02                     # heightmap / dig resolution (m)
REPOSE_DEG = 38.0              # loose BP-1 angle of repose (slump limit after digging)
CUT_WALL_DEG = 45.0            # hand-cut crater walls: BP-1 friction angle 39-51 deg plus 0-2 kPa cohesion

# Zones as (x0, y0, x1, y1) in the arena frame.
ZONES = {
    'start': (0.0, 0.0, 2.0, 2.0),
    'excavation': (0.0, 0.0, 2.5, WIDTH),
    'obstacle': (2.5, 0.0, LENGTH, WIDTH),
    'construction': (3.88, 0.0, LENGTH, 1.5),
    'berm': (5.38 - 1.1, 0.9 - 0.45, 5.38 + 1.1, 0.9 + 0.45),   # 2.2 x 0.9 m target
}
COLUMN = (3.44, 2.5, 0.46)     # central truss leg in a grey box at the arena centre: x, y, side (m)
# Seen in every run of the 2026 KSC footage (Lunabotics 2026 Day 1 stream), not in the guidebook:
# red/white range poles standing at the four corners of the berm target box, and orange-red
# laser lines projected on the bed along the zone boundaries (start zone box, the
# excavation / obstacle boundary, the construction zone corner).
POLE_R, POLE_HEIGHT = 0.0125, 1.2
POLES = [(x, y) for x in (ZONES['berm'][0], ZONES['berm'][2]) for y in (ZONES['berm'][1], ZONES['berm'][3])]
LASER_LINES = [(2.0, 0.0, 2.0, 2.0), (0.0, 2.0, 2.0, 2.0), (2.5, 0.0, 2.5, WIDTH),   # (x0, y0, x1, y1)
               (3.88, 0.0, 3.88, 1.5), (3.88, 1.5, LENGTH, 1.5)]


@dataclass
class Boulder:
    """One rock obstacle: a single stone or a group of 2-3 stones (footage); see stones()."""
    x: float
    y: float
    diameter: float            # footprint of the whole group (m)
    height: float              # visible height above the surface (max)
    seed: int
    size: float = 0.0          # guidebook boulder size, 30-40 cm: the main stone's length


@dataclass
class Crater:
    x: float
    y: float
    diameter: float            # 40-50 cm (guidebook: <= ~40-50 cm wide and deep)
    depth: float


def stones(b):
    """The stones of one boulder obstacle. Footage: a single block 30-45 cm long, or 2-3 stones
    of 20-40 cm resting against each other (sometimes a slab leaning on the main stone), with
    15-25 cm showing above the bed.

    Returns [(x, y, half_len, half_wid, visible_height, yaw, tilt, seed)], centred on (b.x, b.y).
    """
    rng = np.random.default_rng(b.seed)
    size = b.size or b.diameter
    kind = rng.choice(3, p=[0.45, 0.35, 0.20])
    yaw0 = rng.uniform(0, 2 * np.pi)
    if kind == 0:
        a = size / 2 * rng.uniform(0.95, 1.15)
        return [(b.x, b.y, a, a * rng.uniform(0.6, 0.85), min(b.height, 0.6 * 2 * a), yaw0,
                 rng.uniform(-0.15, 0.15), int(rng.integers(1 << 30)))]
    sizes = [size * rng.uniform(0.85, 1.0)] + list(size * rng.uniform(0.55, 0.85 - 0.15 * (kind - 1), kind))
    pos = [np.zeros(2)]
    for k, s in enumerate(sizes[1:], 1):                 # each rests against the main stone
        ang = yaw0 + 2 * np.pi * k / (kind + 1) + rng.uniform(-0.4, 0.4)
        pos.append((sizes[0] + s) / 2 * rng.uniform(0.6, 0.8) * np.array([np.cos(ang), np.sin(ang)]))
    pos = np.array(pos) - np.mean(pos, 0)
    out = []
    for k, (s, p) in enumerate(zip(sizes, pos)):
        slab = k == 1 and rng.random() < 0.5             # thin slab leaning on the main stone
        h = min(b.height, (1.1 if slab else 0.75) * s) * (1.0 if k == 0 or slab else rng.uniform(0.6, 0.85))
        wid = s / 2 * (rng.uniform(0.3, 0.4) if slab else rng.uniform(0.6, 0.85))
        toward = math.atan2(-p[1], -p[0]) if np.any(p) else yaw0
        # Slabs: long side facing the main stone, top tipped toward it (tilt about the local x axis).
        yaw = toward - np.pi / 2 + rng.uniform(-0.3, 0.3) if slab else rng.uniform(0, 2 * np.pi)
        out.append((b.x + p[0], b.y + p[1], s / 2, wid, h, yaw,
                    -rng.uniform(0.2, 0.35) if slab else rng.uniform(-0.15, 0.15), int(rng.integers(1 << 30))))
    return out


def grid():
    """Heightmap axes: covers the interior plus the strip under the ducts."""
    xs = np.arange(-DUCT_D, LENGTH + DUCT_D + 1e-9, RES)
    ys = np.arange(-DUCT_D, WIDTH + DUCT_D + 1e-9, RES)
    return xs, ys


def in_box(x, y, box, margin=0.0):
    x0, y0, x1, y1 = box
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin


def layout(seed=0, obstacle_boulders=5, obstacle_craters=3, excavation_boulders=2,
           excavation_craters=1, robot_radius=0.38, max_tries=200):
    """Random obstacle field per the guidebook (re-drawn every round).

    Obstacle zone: >= 3 boulders and >= 3 craters. Excavation zone may also hold some.
    None in the starting zone or construction zone; a robot-sized path from the start
    zone to the berm always exists.
    """
    rng = np.random.default_rng(seed)
    for _ in range(max_tries):
        placed = []

        def place(box, size, keepout):
            for _ in range(500):
                x = rng.uniform(box[0] + 0.3, box[2] - 0.3)
                y = rng.uniform(box[1] + 0.3, box[3] - 0.3)
                if any(in_box(x, y, ZONES[z], size / 2 + 0.1) for z in keepout):
                    continue
                cx, cy, side = COLUMN
                if abs(x - cx) < side / 2 + size / 2 + 0.3 and abs(y - cy) < side / 2 + size / 2 + 0.3:
                    continue
                if all(math.hypot(x - px, y - py) > (size + ps) / 2 + 0.55 for px, py, ps in placed):
                    placed.append((x, y, size))
                    return x, y
            return None

        boulders, craters, ok = [], [], True
        for box, nb, nc, keep in [(ZONES['obstacle'], obstacle_boulders, obstacle_craters, ['construction']),
                                  (ZONES['excavation'], excavation_boulders, excavation_craters, ['start'])]:
            for _ in range(nc):
                d = rng.uniform(0.40, 0.50)
                p = place(box, d + 0.15, keep)
                ok &= p is not None
                if p:
                    # Footage: craters read as shallow, soft-edged dishes (well inside the guidebook's
                    # ~40-50 cm depth limit); none showed as a deep pit.
                    craters.append(Crater(*p, d, rng.uniform(0.07, 0.12)))
            for _ in range(nb):
                d = rng.uniform(0.30, 0.40)
                p = place(box, d, keep)
                ok &= p is not None
                if p:
                    b = Boulder(*p, d, d * rng.uniform(0.45, 0.75), int(rng.integers(1 << 30)), size=d)
                    b.diameter = 2 * max(math.hypot(s[0] - b.x, s[1] - b.y) + s[2] for s in stones(b))
                    boulders.append(b)
        if ok and plan(occupancy(boulders, craters, robot_radius), (1.0, 1.0),
                       (ZONES['berm'][0] + 0.3, 0.9)) is not None:
            return boulders, craters
    raise RuntimeError('could not place a traversable obstacle field')


def fractal_noise(shape, rng, beta=2.2):
    """Zero-mean noise with a 1/f^beta spectrum, unit std."""
    fx = np.fft.fftfreq(shape[0])[:, None]
    fy = np.fft.fftfreq(shape[1])[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2)
    f[0, 0] = 1.0
    spec = (rng.normal(size=shape) + 1j * rng.normal(size=shape)) / f ** (beta / 2)
    spec[0, 0] = 0
    out = np.real(np.fft.ifft2(spec))
    return out / out.std()


def bandpass_noise(shape, rng, wavelength, res=RES):
    """Unit-std noise concentrated around one wavelength (m): mounds and dimples."""
    f0 = res / wavelength
    fx = np.fft.fftfreq(shape[0])[:, None]
    fy = np.fft.fftfreq(shape[1])[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2)
    spec = (rng.normal(size=shape) + 1j * rng.normal(size=shape)) * np.exp(-((f - f0) / (0.5 * f0)) ** 2)
    out = np.real(np.fft.ifft2(spec))
    return out / out.std()


def track_prints(X, Y, rng, n_tracks, gauge=0.40, width=0.14, depth=0.005):
    """Old wheel/track ruts from earlier runs: pairs of shallow grooves along smooth curves."""
    H = np.zeros(X.shape, np.float32)
    for _ in range(n_tracks):
        p = np.array([rng.uniform(0.3, LENGTH - 0.3), rng.uniform(0.3, WIDTH - 0.3)])
        heading = rng.uniform(0, 2 * np.pi)
        for _ in range(int(rng.uniform(2.0, 5.0) / 0.05)):
            heading += rng.normal(0, 0.06)
            p = p + 0.05 * np.array([np.cos(heading), np.sin(heading)])
            if not (0 < p[0] < LENGTH and 0 < p[1] < WIDTH):
                break
            n = np.array([-np.sin(heading), np.cos(heading)])
            for side in (-1, 1):
                c = p + side * gauge / 2 * n
                i = int((c[0] - X[0, 0]) / RES)
                j = int((c[1] - Y[0, 0]) / RES)
                r = int(width / RES)
                a0, a1, b0, b1 = max(i - r, 0), min(i + r + 1, X.shape[0]), max(j - r, 0), min(j + r + 1, X.shape[1])
                d = np.hypot(X[a0:a1, b0:b1] - c[0], Y[a0:a1, b0:b1] - c[1]) / (width / 2)
                groove = depth * np.clip(1 - d ** 2, 0, None)
                H[a0:a1, b0:b1] = np.minimum(H[a0:a1, b0:b1], -groove)
    return H


def boot_prints(seed=0, n_walks=28):
    """Shoe-cover prints of the reset crew (footage: all over the bed between runs).

    Returns [(x, y, heading, length, width, depth)]: strides along wandering walks.
    """
    rng = np.random.default_rng(seed + 13)
    prints = []
    for _ in range(n_walks):
        p = np.array([rng.uniform(0.2, LENGTH - 0.2), rng.uniform(0.2, WIDTH - 0.2)])
        heading = rng.uniform(0, 2 * np.pi)
        for k in range(int(rng.integers(3, 14))):
            heading += rng.normal(0, 0.25)
            p = p + 0.32 * np.array([np.cos(heading), np.sin(heading)])
            if not (0.1 < p[0] < LENGTH - 0.1 and 0.1 < p[1] < WIDTH - 0.1):
                break
            side = 0.09 * (1 if k % 2 else -1)
            c = p + side * np.array([-np.sin(heading), np.cos(heading)])
            prints.append((c[0], c[1], heading + rng.normal(0, 0.12), rng.uniform(0.27, 0.31),
                           rng.uniform(0.10, 0.12), rng.uniform(0.004, 0.012)))
    return prints


def stamp_prints(H, xs, ys, prints):
    """Press boot prints into H: flat-soled ovals with a soft edge."""
    for x, y, hd, ln, wd, dp in prints:
        i0, i1 = np.searchsorted(xs, [x - ln, x + ln])
        j0, j1 = np.searchsorted(ys, [y - ln, y + ln])
        X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
        u = ((X - x) * np.cos(hd) + (Y - y) * np.sin(hd)) / (ln / 2)
        v = (-(X - x) * np.sin(hd) + (Y - y) * np.cos(hd)) / (wd / 2)
        d = np.hypot(u, v * (1 + 0.15 * u))        # wider at the toe than the heel
        H[i0:i1, j0:j1] -= dp * np.clip((1.15 - d) / 0.3, 0, 1)


def heightmap(boulders, craters, seed=0, n_tracks=8):
    """BP-1 bed after a reset and some traffic, with the craters carved in.

    Boulders are separate meshes. Returns (xs, ys, H), H[i, j] = height at (xs[i], ys[j]).
    """
    rng = np.random.default_rng(seed + 7)
    xs, ys = grid()
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    # Hand-levelled bed: ~1 cm long-wavelength unevenness, 10-20 cm mounds/dimples
    # (footage), a few mm of roughness, old track prints and the reset crew's boot prints.
    H = (0.010 * fractal_noise(X.shape, rng, beta=3.0) + 0.004 * bandpass_noise(X.shape, rng, 0.15)
         + 0.002 * fractal_noise(X.shape, rng, beta=1.2) + track_prints(X, Y, rng, n_tracks))
    stamp_prints(H, xs, ys, boot_prints(seed))
    for c in craters:
        # Footage (pre-run, berm-end camera): a shallow dish with a soft, scalloped, slightly
        # irregular edge and a low lip on part of the rim. Most spoil is raked away.
        cr = np.random.default_rng(int(c.x * 1000) * 7919 + int(c.y * 1000))
        th = np.arctan2(Y - c.y, X - c.x)
        wobble = 1 + 0.07 * np.sin(2 * th + cr.uniform(0, 6.3)) + 0.05 * np.sin(3 * th + cr.uniform(0, 6.3))
        r = np.hypot(X - c.x, Y - c.y) / (c.diameter / 2 * wobble)
        t = np.clip((r - 0.15) / 0.85, 0, 1)
        bowl = c.depth * (1 - t * t * (3 - 2 * t))
        lip = np.exp(-((r - 1.1) / 0.15) ** 2) * (1 + 0.7 * np.cos(th - cr.uniform(0, 6.3)))
        lip *= 0.25 * bowl.sum() / max(lip.sum(), 1e-9)
        H += lip - bowl
    H = H.astype(np.float32)
    slump(H, 0, H.shape[0], 0, H.shape[1], repose_deg=CUT_WALL_DEG)   # cohesive BP-1 holds ~45 deg cuts
    return xs, ys, H


def start_pose(seed=0, heading='random', footprint=(0.51, 0.43)):
    """Random start in the starting zone; heading N/E/S/W from the direction wheel."""
    rng = np.random.default_rng(seed + 11)
    half = max(footprint) / 2 + 0.1
    x0, y0, x1, y1 = ZONES['start']
    x, y = rng.uniform(x0 + half, x1 - half), rng.uniform(y0 + half, y1 - half)
    choices = {'E': 0.0, 'N': 90.0, 'W': 180.0, 'S': 270.0}
    if heading == 'random':
        heading = str(rng.choice(list(choices)))
    yaw = choices[heading] if heading in choices else float(heading)
    return x, y, math.radians(yaw)


# ---------------------------------------------------------------- digging
def slump(H, i0, i1, j0, j1, repose_deg=REPOSE_DEG, iters=400, res=RES):
    """Thermal erosion in a window: move material downhill until slopes <= angle of repose."""
    i0, j0 = max(i0, 1), max(j0, 1)
    i1, j1 = min(i1, H.shape[0] - 1), min(j1, H.shape[1] - 1)
    talus = math.tan(math.radians(repose_deg)) * res
    for _ in range(iters):
        h = H[i0 - 1:i1 + 1, j0 - 1:j1 + 1]
        moved = False
        for axis in (0, 1):
            d = np.diff(h, axis=axis)                     # h[k+1] - h[k]
            excess = np.where(np.abs(d) > talus, (np.abs(d) - talus) * np.sign(d) * 0.22, 0)
            if np.abs(excess).max() < 2e-4:
                continue
            moved = True
            if axis == 0:
                h[1:, :] -= excess
                h[:-1, :] += excess
            else:
                h[:, 1:] -= excess
                h[:, :-1] += excess
        if not moved:
            break


def dig(H, xs, ys, x, y, radius, depth, floor=-BED_DEPTH):
    """Remove (depth > 0) or add (depth < 0) regolith with a smooth brush, then slump.

    Returns the changed index window (i0, i1, j0, j1) and the volume moved (m^3, + = removed).
    """
    res = float(xs[1] - xs[0])
    i0 = max(int((x - radius - xs[0]) / res) - 2, 0)
    i1 = min(int((x + radius - xs[0]) / res) + 3, len(xs))
    j0 = max(int((y - radius - ys[0]) / res) - 2, 0)
    j1 = min(int((y + radius - ys[0]) / res) + 3, len(ys))
    X, Y = np.meshgrid(xs[i0:i1], ys[j0:j1], indexing='ij')
    d = np.hypot(X - x, Y - y) / radius
    brush = np.where(d < 1, 0.5 * (1 + np.cos(np.pi * np.clip(d, 0, 1))), 0.0)
    before = H[i0:i1, j0:j1].sum()
    H[i0:i1, j0:j1] = np.maximum(H[i0:i1, j0:j1] - depth * brush, floor)
    moved = float(before - H[i0:i1, j0:j1].sum()) * res * res
    pad = int(0.25 / res)
    w = (max(i0 - pad, 0), min(i1 + pad, len(xs)), max(j0 - pad, 0), min(j1 + pad, len(ys)))
    slump(H, *w, res=res)
    return w, moved


def volumes(H, H0, min_change=0.005):
    """True cut / fill (m^3) vs the pristine arena, and fill inside the berm target box."""
    dh = H - H0
    cell = RES * RES
    cut = float(np.where(dh < -min_change, -dh, 0).sum() * cell)
    fill = float(np.where(dh > min_change, dh, 0).sum() * cell)
    xs, ys = grid()
    x0, y0, x1, y1 = ZONES['berm']
    bi = (xs >= x0) & (xs <= x1)
    bj = (ys >= y0) & (ys <= y1)
    berm = float(np.where(dh > min_change, dh, 0)[np.ix_(bi, bj)].sum() * cell)
    return cut, fill, berm


# ---------------------------------------------------------------- planning (test driver only)
PLAN_RES = 0.05


def occupancy(boulders, craters, inflate=0.38):
    """Blocked cells for a robot of radius `inflate`: obstacles, range poles, column, walls."""
    nx, ny = int(LENGTH / PLAN_RES) + 1, int(WIDTH / PLAN_RES) + 1
    X, Y = np.meshgrid(np.arange(nx) * PLAN_RES, np.arange(ny) * PLAN_RES, indexing='ij')
    occ = (X < inflate) | (Y < inflate) | (X > LENGTH - inflate) | (Y > WIDTH - inflate)
    for o in [*boulders, *craters]:
        occ |= np.hypot(X - o.x, Y - o.y) < o.diameter / 2 + inflate + (0.08 if isinstance(o, Crater) else 0)
    for px, py in POLES:
        occ |= np.hypot(X - px, Y - py) < POLE_R + inflate
    # The column box is ~1 m tall: the D455 mast sticking out front hits it, so keep extra room.
    cx, cy, side = COLUMN
    occ |= (np.abs(X - cx) < side / 2 + inflate + 0.15) & (np.abs(Y - cy) < side / 2 + inflate + 0.15)
    return occ


def nearest_free(occ, p, max_dist=1.0):
    """Closest unblocked point to p (a goal inside an inflated obstacle), or None."""
    free = np.argwhere(~occ)
    if len(free) == 0:
        return None
    d = np.hypot(free[:, 0] * PLAN_RES - p[0], free[:, 1] * PLAN_RES - p[1])
    k = int(np.argmin(d))
    return (free[k, 0] * PLAN_RES, free[k, 1] * PLAN_RES) if d[k] <= max_dist else None


def plan(occ, start, goal):
    """8-connected A* on the occupancy grid; returns [(x, y), ...] or None."""
    def cell(p):
        return (min(max(int(round(p[0] / PLAN_RES)), 0), occ.shape[0] - 1),
                min(max(int(round(p[1] / PLAN_RES)), 0), occ.shape[1] - 1))
    s, g = cell(start), cell(goal)
    if occ[g]:
        return None
    occ = occ.copy()
    occ[s] = False
    openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
    while openq:
        _, c = heapq.heappop(openq)
        if c == g:
            path = []
            while c is not None:
                path.append((c[0] * PLAN_RES, c[1] * PLAN_RES))
                c = came[c]
            return path[::-1]
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                n = (c[0] + di, c[1] + dj)
                if (di or dj) and 0 <= n[0] < occ.shape[0] and 0 <= n[1] < occ.shape[1] and not occ[n]:
                    nc = cost[c] + math.hypot(di, dj)
                    if nc < cost.get(n, 1e18):
                        cost[n], came[n] = nc, c
                        heapq.heappush(openq, (nc + math.hypot(n[0] - g[0], n[1] - g[1]), n))
    return None
