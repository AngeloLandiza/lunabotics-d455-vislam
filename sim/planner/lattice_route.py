"""Fallback route for tight arenas: a search over (x, y, heading) on a grid, where the robot drives straight along
one of 16 grid directions or turns in place (custom planner, used when the tangent route finds nothing).

The tangent route only makes smooth paths, which keep the body's sides clear but not always its corners; in tight
arenas the only way through can need a stop and a turn in place. This search covers that case: the free
configurations of the 1.0 x 0.7 m body (CHECK_MARGIN from every obstacle, wall and the berm box) are found on a 5 cm
grid for 16 headings (the 16 grid directions, so a straight move lands on a cell), plus the headings in between to
check the sweep of every turn in place. Dijkstra runs over moves forward along the heading and turns to the next
heading; the result is then shortened by straight cuts the footprint check allows (any angle), so the robot stops
and turns only where it has to. A skid-steer that can turn in place reaches every configuration that is connected in
free space, so the search fails only when the grid finds no connection (up to its 5 cm and the heading steps).
"""
import time

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

import arena2d as a2

RES = 0.05                    # m, grid cell
NH = 72                       # headings, 5 deg apart
MAX_MOVE = 10                 # cells: a straight move is the shortest grid vector within 2.5 deg of the heading
HEAD = np.arange(NH) * 2 * np.pi / NH
REVERSE_COST = 2.0            # backing up counts double (Assumed: the robot is slower and blind that way)
GRID_SLACK = 0.01             # m, the grid is built with this much more margin: its moves are checked at cell centres
DIRECT_PENALTY = 1.0          # m, cost of ending without a straight run in (the goal turn may be blocked; Assumed)
_CACHE = {}


def _moves():
    """For each heading, the shortest integer cell vector pointing within half a heading step of it (or None)."""
    cand = [(i, j) for i in range(-MAX_MOVE, MAX_MOVE + 1) for j in range(-MAX_MOVE, MAX_MOVE + 1)
            if 0 < np.hypot(i, j) <= MAX_MOVE]
    out = []
    for th in HEAD:
        ok = [(np.hypot(i, j), i, j) for i, j in cand
              if abs(np.angle(np.exp(1j * (np.arctan2(j, i) - th)))) <= np.pi / NH + 1e-9]
        out.append(min(ok)[1:] if ok else None)
    return out


MOVES = _moves()


def _free(w, angles, margin):
    """Free cells (len(angles), ny, nx) for the footprint at each heading."""
    x0, x1, y0, y1 = a2.v(a2.PARAMS, "footprint")
    xs, ys = np.arange(RES / 2, w.L, RES), np.arange(RES / 2, w.W, RES)
    X, Y = np.meshgrid(xs, ys)
    Q = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    bx0, bx1, by0, by1 = w.berm
    B = np.array([[bx0, by0], [bx1, by0], [bx1, by1], [bx0, by1]])
    out = np.ones((len(angles), len(ys), len(xs)), bool)
    for k, t in enumerate(angles):
        c, s = np.cos(t), np.sin(t)
        ok = np.ones_like(X, bool)
        for o in w.obstacles:
            dx, dy = o.x - X, o.y - Y
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            d = np.hypot(lx - np.clip(lx, x0, x1), ly - np.clip(ly, y0, y1))
            ok &= d - o.r >= margin              # d == 0 when the centre is inside the body
        cx = X[..., None] + c * Q[:, 0] - s * Q[:, 1]
        cy = Y[..., None] + s * Q[:, 0] + c * Q[:, 1]
        ok &= (cx.min(-1) >= margin) & (cx.max(-1) <= w.L - margin) & (cy.min(-1) >= margin) & (cy.max(-1) <= w.W - margin)
        # berm box grown by the margin (square corners, slightly cautious): separating axes of both rectangles
        sep = (cx.max(-1) < bx0 - margin) | (cx.min(-1) > bx1 + margin) | (cy.max(-1) < by0 - margin) | (cy.min(-1) > by1 + margin)
        for ax in (np.array([c, s]), np.array([-s, c])):
            p = cx * ax[0] + cy * ax[1]
            q = B @ ax
            sep |= (p.max(-1) < q.min() - margin) | (p.min(-1) > q.max() + margin)
        out[k] = ok & sep
    return out


class Lattice:
    """Free configurations and the move graph for one arena."""

    def __init__(self, w, margin):
        from tangent_route import TURN_COST
        self.w = w
        half = np.arange(2 * NH) * np.pi / NH                 # every 2.5 deg: the headings and the halfway points
        F = _free(w, half, margin)
        free = F[::2]
        self.free = free
        nh, ny, nx = free.shape
        self.shape = free.shape
        idx = np.arange(free.size).reshape(free.shape)
        rows, cols, cost = [], [], []
        step = 2 * np.pi / NH
        for h in range(nh):
            # turn in place to the next heading, either way (the halfway heading must be free too)
            h2 = (h + 1) % nh
            ok = free[h] & F[2 * h + 1] & free[h2]
            for a, b in ((h, h2), (h2, h)):
                rows.append(idx[a][ok]), cols.append(idx[b][ok]), cost.append(np.full(ok.sum(), TURN_COST * step))
            # one straight move along the heading's grid vector; every cell it passes must be free at the heading and
            # at the halfway heading on the vector's side (the vector is up to 2.5 deg off the heading)
            if MOVES[h] is None:
                continue
            di, dj = MOVES[h]
            side = np.angle(np.exp(1j * (np.arctan2(dj, di) - HEAD[h])))
            Fs = F[(2 * h + (1 if side > 0 else -1)) % (2 * nh)] if abs(side) > 1e-9 else free[h]
            J, I = np.mgrid[0:ny, 0:nx]
            J2, I2 = J + dj, I + di
            ok = free[h] & (J2 >= 0) & (J2 < ny) & (I2 >= 0) & (I2 < nx)
            n = int(np.ceil(np.hypot(di, dj) * 2))
            for f in np.linspace(0, 1, n + 1)[1:]:
                Jm = np.clip(np.round(J + f * dj).astype(int), 0, ny - 1)
                Im = np.clip(np.round(I + f * di).astype(int), 0, nx - 1)
                ok &= free[h][Jm, Im] & Fs[Jm, Im]
            rows.append(idx[h][ok]), cols.append(idx[h][J2[ok], I2[ok]])
            cost.append(np.full(ok.sum(), RES * np.hypot(di, dj)))
            # the same move backwards (the cells checked are the same), dearer
            rows.append(idx[h][J2[ok], I2[ok]]), cols.append(idx[h][ok])
            cost.append(np.full(ok.sum(), REVERSE_COST * RES * np.hypot(di, dj)))
        r, c, d = np.concatenate(rows), np.concatenate(cols), np.concatenate(cost)
        self.G = coo_matrix((d, (r, c)), shape=(free.size, free.size)).tocsr()

    def cell(self, xy):
        nh, ny, nx = self.shape
        i = int(np.clip(np.floor(xy[0] / RES), 0, nx - 1))
        j = int(np.clip(np.floor(xy[1] / RES), 0, ny - 1))
        return j, i

    def xy(self, j, i):
        return np.array([(i + 0.5) * RES, (j + 0.5) * RES])

    def nearest_free(self, pose, max_cells=2):
        """Free lattice states near a pose (the two nearest headings), nearest first."""
        nh, ny, nx = self.shape
        j0, i0 = self.cell(pose[:2])
        dh = np.abs(np.angle(np.exp(1j * (HEAD - pose[2]))))
        out = []
        for h in np.argsort(dh)[:2]:
            for dj in range(-max_cells, max_cells + 1):
                for di in range(-max_cells, max_cells + 1):
                    j, i = j0 + dj, i0 + di
                    if 0 <= j < ny and 0 <= i < nx and self.free[h, j, i]:
                        out.append((np.hypot(*(self.xy(j, i) - pose[:2])) + 0.05 * dh[h], (int(h), j, i)))
        return [s for _, s in sorted(out)]


def get(w, margin=None):
    from execute import CHECK_MARGIN
    margin = CHECK_MARGIN if margin is None else margin
    key = (w.seed, margin)
    if key not in _CACHE:
        _CACHE.clear()                                   # one arena at a time per process
        _CACHE[key] = Lattice(w, margin)
    return _CACHE[key]


def _heading(a, b, sgn):
    return float(np.arctan2(b[1] - a[1], b[0] - a[0]) + (0.0 if sgn > 0 else np.pi))


def _straight_ok(w, a, b, margin, sgn=1, step=0.03):
    th = _heading(a, b, sgn)
    n = max(int(np.ceil(np.hypot(*(b - a)) / step)), 1)
    return all(a2.clearance(w, (*p, th)) >= margin for p in np.linspace(a, b, n + 1))


def _turn_ok(w, xy, th0, th1, margin):
    e = a2.wrap(th1 - th0)
    n = max(int(np.ceil(abs(e) / np.radians(4.0))), 1)
    return all(a2.clearance(w, (xy[0], xy[1], th0 + e * f)) >= margin for f in np.linspace(0, 1, n + 1))


def shortcut(w, runs, th0, margin, end_th):
    """Fewer stops. runs: [(+1 forwards / -1 backwards, points, leave headings)], each run starting where the last one
    ended; a point's leave heading is the body heading the grid route drives off from it with (end_th after the last
    point). From each kept point, jump to the farthest later point of the run that one straight move reaches, with a
    clear turn before the move and a clear turn after it back to the grid route's heading, so the rest of the grid
    route stays valid. Returns segments [(a, b, direction)]."""
    segs, th = [], th0
    for sgn, pts, leave in runs:
        i = 0
        while i < len(pts) - 1:
            for j in range(len(pts) - 1, i, -1):
                thn = _heading(pts[i], pts[j], sgn)
                if j == i + 1:
                    break
                if (_turn_ok(w, pts[j], thn, leave[j], margin) and _turn_ok(w, pts[i], th, thn, margin)
                        and _straight_ok(w, pts[i], pts[j], margin, sgn)):
                    break
            segs.append((pts[i], pts[j], sgn))
            th = _heading(pts[i], pts[j], sgn)
            i = j
    return segs


def plan(w, start, goal, margin=None, approaches=(0.6, 0.3)):
    """Executor actions from start to goal: turns in place and straight moves forwards or backwards, then a straight
    run in along the goal heading where there is room (approaches, m). Returns (actions or None, info)."""
    import execute as ex
    margin = ex.CHECK_MARGIN if margin is None else margin
    t0 = time.time()
    lat = get(w, margin + GRID_SLACK)
    info = {"length": None, "fail": None, "build_s": time.time() - t0}
    g = np.asarray(goal, float)
    h = np.array([np.cos(g[2]), np.sin(g[2])])
    targets = []                                        # (point, run-in length)
    for d in approaches:
        A = g[:2] - d * h
        if _straight_ok(w, A, g[:2], margin):
            targets.append((A, d))
    targets.append((g[:2], 0.0))
    S = lat.nearest_free(start)
    if not S:
        info.update(fail="start not free on the grid", solve_s=time.time() - t0)
        return None, info
    flat = lambda st: int(np.ravel_multi_index(st, lat.shape))  # noqa: E731
    s_idx = flat(S[0])
    dist, pred = dijkstra(lat.G, indices=s_idx, return_predecessors=True)
    best = None
    for A, d in targets:
        extra = {0.0: DIRECT_PENALTY}.get(d, 0.6 - d)  # prefer the longest run in; none leaves a turn at the goal
        for st in lat.nearest_free((A[0], A[1], g[2]), max_cells=1):
            k = flat(st)
            q = lat.xy(*st[1:]) - g[:2]
            lateral = abs(q[0] * h[1] - q[1] * h[0])     # off the run-in line: becomes a heading error at the goal
            if d > 0 and lateral > 0.04:
                continue
            c = dist[k] + extra + 10.0 * lateral
            if np.isfinite(c) and (best is None or c < best[0]):
                best = (c, k, A, d)
    if best is None:
        info.update(fail="no route on the grid", solve_s=time.time() - t0)
        return None, info
    _, k, A, d = best
    states = []
    while k >= 0:
        states.append(np.unravel_index(k, lat.shape))
        if k == s_idx:
            break
        k = pred[k]
    states.reverse()
    # the moves that change position: (from, to, direction, body heading); turns in place only change the heading
    moves = []
    for (h0, j0, i0), (h1, j1, i1) in zip(states[:-1], states[1:]):
        if (j0, i0) != (j1, i1):
            sgn = 1 if (i1 - i0, j1 - j0) == MOVES[h0] else -1
            moves.append((lat.xy(j0, i0), lat.xy(j1, i1), sgn, float(HEAD[h0])))
    S0 = np.asarray(start[:2], float)
    end = moves[-1][1] if moves else lat.xy(*states[-1][1:])
    end_th = _heading(end, g[:2], 1) if np.hypot(*(g[:2] - end)) > 0.05 else g[2]
    # runs of one direction; the robot starts where it is (a hop to the first cell centre, straight)
    runs = [(1, [S0, moves[0][0] if moves else end], [moves[0][3] if moves else end_th, None])]
    for m, (p0, p1, sgn, hd) in enumerate(moves):
        if runs[-1][0] != sgn:
            runs.append((sgn, [p0], [hd]))
        runs[-1][1].append(p1)
        runs[-1][2].append(moves[m + 1][3] if m + 1 < len(moves) else end_th)
    for r in range(len(runs) - 1):                        # a run's last point is the next run's first
        runs[r][2][-1] = runs[r + 1][2][0]
    runs[-1][2][-1] = end_th
    runs = [(sg, [p for p in pts], hd) for sg, pts, hd in runs]
    segs = [sg for sg in shortcut(w, runs, start[2], margin, end_th) if np.hypot(*(sg[1] - sg[0])) > 1e-3]
    actions, th = [], start[2]
    for a, b, sgn in segs:
        thn = _heading(a, b, sgn)
        if not _turn_ok(w, a, th, thn, margin) or not _straight_ok(w, a, b, margin, sgn):
            info.update(fail="grid route touches", solve_s=time.time() - t0)
            return None, info
        actions += [("turn", thn), ("line", (np.asarray(b, float), sgn, 0.6 if sgn > 0 else 0.3))]
        th = thn
    last = segs[-1][1] if segs else np.asarray(start[:2], float)
    if np.hypot(*(g[:2] - last)) > 1e-3:
        aim = _heading(last, g[:2], 1)
        if not (_turn_ok(w, last, th, aim, margin) and _straight_ok(w, last, g[:2], margin)):
            info.update(fail="grid route touches", solve_s=time.time() - t0)
            return None, info
        actions.append(("line", (g[:2].copy(), 1, 0.4)))
        th = aim
    if not _turn_ok(w, g[:2], th, g[2], margin):
        info.update(fail="turn at goal", solve_s=time.time() - t0)
        return None, info
    actions.append(("final", g))
    info.update(length=float(sum(np.hypot(*(b - a)) for a, b, _ in segs) + np.hypot(*(g[:2] - last))),
                solve_s=time.time() - t0, approach=d,
                reverse=float(sum(np.hypot(*(b - a)) for a, b, sg in segs if sg < 0)))
    return actions, info
