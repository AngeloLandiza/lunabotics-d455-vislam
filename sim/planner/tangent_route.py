"""Exact shortest route among the grown obstacles: the tangent graph (custom planner candidate B).

With every obstacle grown into a disc (the berm box into a rounded rectangle), a shortest route for base_link is made
of straight segments tangent to the discs and arcs along them. This is the classic structure of shortest paths among
circles; here the graph holds every tangent point with the direction of travel (clockwise or counter-clockwise), the
collision-free tangent segments between them, and the free arcs between neighbouring tangent points on each circle,
and Dijkstra finds the shortest route. On an arc the robot drives concentric with the obstacle, so its inner side
stays (grown radius - half-width) from it, which a polyline that cuts corners cannot promise.

The goal heading: the route either reaches the goal and turns in place there (when the robot has room to turn), or
ends with a straight approach of APPROACHES metres along the goal heading. It reaches the approach start along one of
two virtual circles (radius VIRTUAL_R, tangent to the approach line, so the robot arrives already facing the goal
heading) or by stopping there and turning in place. Turning in place costs TURN_COST metres per radian (about the
distance driven in the time a turn takes).
"""
import heapq
import time

import numpy as np

import arena2d as a2

APPROACHES = (0.30, 0.60)  # m, straight final approaches along the goal heading (Assumed)
VIRTUAL_R = 0.40          # m, radius of the virtual arrival circles (Assumed)
TURN_COST = 0.35          # m per rad of turn in place (Assumed: ~0.5 m/s driving vs ~1.4 rad/s turning)
ARC_STEP = 0.01           # m, sampling of arcs in the output


class Circle:
    def __init__(self, c, R, kind, virtual=False):
        self.c, self.R, self.kind, self.virtual = np.asarray(c, float), float(R), kind, virtual


class Scene:
    """Grown obstacles for one inflation: discs, the berm's rounded rectangle and the shrunken walls."""

    def __init__(self, w, inflate):
        self.w, self.inf = w, inflate
        self.circles = [Circle((o.x, o.y), o.r + inflate, o.kind) for o in w.obstacles]
        b = w.berm
        self.circles += [Circle((x, y), inflate, "berm") for x in b[:2] for y in b[2:]]
        self.lo = np.array([inflate, inflate])
        self.hi = np.array([w.L - inflate, w.W - inflate])

    def clear(self, P, skip=()):
        """Clearance of points P (N, 2) from the grown obstacles and the walls (negative = blocked)."""
        P = np.atleast_2d(P)
        c = np.full(len(P), np.inf)
        for k, ci in enumerate(self.circles):
            if ci.virtual or ci.kind == "berm" or k in skip:
                continue
            c = np.minimum(c, np.hypot(P[:, 0] - ci.c[0], P[:, 1] - ci.c[1]) - ci.R)
        b = self.w.berm
        dx = np.maximum.reduce([b[0] - P[:, 0], P[:, 0] - b[1], np.zeros(len(P))])
        dy = np.maximum.reduce([b[2] - P[:, 1], P[:, 1] - b[3], np.zeros(len(P))])
        inside = (dx == 0) & (dy == 0)
        depth = np.minimum.reduce([P[:, 0] - b[0], b[1] - P[:, 0], P[:, 1] - b[2], b[3] - P[:, 1]])
        c = np.minimum(c, np.where(inside, -depth, np.hypot(dx, dy)) - self.inf)
        c = np.minimum.reduce([c, P[:, 0] - self.lo[0], self.hi[0] - P[:, 0], P[:, 1] - self.lo[1], self.hi[1] - P[:, 1]])
        return c

    def seg_free(self, a, b, tol=1e-6):
        n = max(int(np.hypot(*(b - a)) / 0.01), 2)
        return bool(self.clear(np.linspace(a, b, n)).min() > -tol)


def _tangents_point(p, ci):
    """Tangent points on circle ci for lines through p, with the travel orientation at the touch point when going
    from p to the circle (+1 counter-clockwise, -1 clockwise)."""
    d = p - ci.c
    L = np.hypot(*d)
    if L <= ci.R + 1e-9:
        return []
    phi, alpha = np.arctan2(d[1], d[0]), np.arccos(ci.R / L)
    out = []
    for s in (+1, -1):
        t = ci.c + ci.R * np.array([np.cos(phi + s * alpha), np.sin(phi + s * alpha)])
        u = t - p
        o = np.sign((t - ci.c)[0] * u[1] - (t - ci.c)[1] * u[0])
        out.append((t, o))
    return out


def _bitangents(ci, cj):
    """Common tangents between two circles: (t_i, t_j, orientation at i, orientation at j) travelling i -> j."""
    d = cj.c - ci.c
    L = np.hypot(*d)
    out = []
    if L < 1e-9:
        return out
    base = np.arctan2(d[1], d[0])
    for sj in (+1, -1):                    # +1: outer tangents (same side), -1: inner (crossing)
        rr = (ci.R - sj * cj.R) / L
        if abs(rr) > 1:
            continue
        a = np.arccos(rr)
        for s in (+1, -1):
            n = np.array([np.cos(base + s * a), np.sin(base + s * a)])
            ti, tj = ci.c + ci.R * n, cj.c + sj * cj.R * n
            u = tj - ti
            if np.hypot(*u) < 1e-9:
                continue
            oi = np.sign((ti - ci.c)[0] * u[1] - (ti - ci.c)[1] * u[0])
            oj = np.sign((tj - cj.c)[0] * u[1] - (tj - cj.c)[1] * u[0])
            out.append((ti, tj, oi, oj))
    return out


def edge_clear(w, geom, step=0.05):
    """Smallest footprint clearance along one route edge, the robot heading along the edge."""
    if geom[0] == "seg":
        a, b = geom[1], geom[2]
        n = max(int(np.ceil(np.hypot(*(b - a)) / step)), 1)
        th = np.arctan2(b[1] - a[1], b[0] - a[0])
        P = np.linspace(a, b, n + 1)
        poses = np.column_stack([P, np.full(len(P), th)])
    elif geom[0] == "arc":
        _, c, R, a1, da = geom
        n = max(int(np.ceil(abs(da) * R / step)), 1)
        ang = a1 + da * np.linspace(0, 1, n + 1)
        poses = np.column_stack([c[0] + R * np.cos(ang), c[1] + R * np.sin(ang), ang + np.sign(da) * np.pi / 2])
    else:
        return np.inf
    return min(a2.clearance(w, q) for q in poses)


def route(w, start, goal, inflate):
    """Shortest route for base_link from start (x, y, yaw) to goal (x, y, yaw). Returns (points (N, 2), info)."""
    from execute import CHECK_MARGIN
    t0 = time.time()
    sc = Scene(w, inflate)
    S, G = np.asarray(start[:2], float), np.asarray(goal[:2], float)
    h = np.array([np.cos(goal[2]), np.sin(goal[2])])
    nrm = np.array([-h[1], h[0]])
    info = {"length": None, "solve_s": None, "fail": None}
    if sc.clear(np.array([S, G])).min() < -1e-6:
        info.update(fail="start or goal blocked", solve_s=time.time() - t0)
        return None, info
    # arrival options: (point, approach length or None for "turn at the goal", turning there allowed)
    arrivals = []
    if a2.turn_clearance(w, G) >= CHECK_MARGIN:
        arrivals.append((G, None, True))
    for d in APPROACHES:
        A = G - d * h
        if sc.clear(A[None])[0] > -1e-6 and sc.seg_free(A, G):
            arrivals.append((A, d, a2.turn_clearance(w, A) >= CHECK_MARGIN))
    if not arrivals:
        info.update(fail="no arrival", solve_s=time.time() - t0)
        return None, info
    circles = list(sc.circles)
    exits = []                                   # (virtual circle index, arrival index)
    for m, (A, d, _) in enumerate(arrivals):
        if d is None:
            continue
        for side in (+1, -1):                    # virtual circles tangent to the approach line at A
            circles.append(Circle(A + side * VIRTUAL_R * nrm, VIRTUAL_R, "virtual", virtual=True))
            exits.append((len(circles) - 1, m))
    nodes, node_of, edges = [], {}, {}

    def node(ci, p, o):
        key = (ci, round(float(p[0]), 6), round(float(p[1]), 6), int(o))
        if key not in node_of:
            node_of[key] = len(nodes)
            nodes.append((ci, np.asarray(p, float), int(o)))
        return node_of[key]

    def add(u, v_, cost, geom):
        if cost < edges.get((u, v_), (np.inf,))[0]:
            edges[(u, v_)] = (cost, geom)

    def usable(ci, p):
        return sc.clear(p[None], skip=(ci,) if not circles[ci].virtual else ())[0] > -1e-6

    def heading(a, b):
        return float(np.arctan2(b[1] - a[1], b[0] - a[0]))

    s_node = node(-1, S, 0)
    t_node = node(-3, G, 0)
    arr_nodes = [node(-10 - m, A, 0) for m, (A, _, _) in enumerate(arrivals)]
    on_circle = {i: [] for i in range(len(circles))}
    arr_heading = [goal[2] if d is None else goal[2] for (A, d, _) in arrivals]
    # start -> circles, circles -> arrivals (with a turn in place at the arrival), start -> arrivals
    for i, ci in enumerate(circles):
        for t, o in _tangents_point(S, ci):
            if usable(i, t) and sc.seg_free(S, t):
                k = node(i, t, o)
                on_circle[i].append(k)
                add(s_node, k, np.hypot(*(t - S)) + TURN_COST * abs(a2.wrap(heading(S, t) - start[2])), ("seg", S, t))
        if ci.virtual:
            continue
        for m, (A, d, can_turn) in enumerate(arrivals):
            if not can_turn:
                continue
            for t, o in _tangents_point(A, ci):
                if usable(i, t) and sc.seg_free(t, A):
                    k = node(i, t, -o)              # travelling circle -> A reverses the orientation
                    on_circle[i].append(k)
                    add(k, arr_nodes[m], np.hypot(*(A - t)) + TURN_COST * abs(a2.wrap(arr_heading[m] - heading(t, A))),
                        ("seg", t, A))
    for m, (A, d, can_turn) in enumerate(arrivals):
        if can_turn and sc.seg_free(S, A):
            cost = np.hypot(*(A - S)) + TURN_COST * (abs(a2.wrap(heading(S, A) - start[2])) +
                                                     abs(a2.wrap(arr_heading[m] - heading(S, A))))
            add(s_node, arr_nodes[m], cost, ("seg", S, A))
        add(arr_nodes[m], t_node, 0.0 if d is None else d, ("seg", A, G) if d is not None else ("none",))
    # circle -> circle (nothing leaves a virtual circle except its exit at the approach start)
    for i in range(len(circles)):
        if circles[i].virtual:
            continue
        for j in range(len(circles)):
            if i == j:
                continue
            for ti, tj, oi, oj in _bitangents(circles[i], circles[j]):
                if usable(i, ti) and usable(j, tj) and sc.seg_free(ti, tj):
                    ki, kj = node(i, ti, oi), node(j, tj, oj)
                    on_circle[i].append(ki)
                    on_circle[j].append(kj)
                    add(ki, kj, np.hypot(*(tj - ti)), ("seg", ti, tj))
    for i, m in exits:
        c, A = circles[i], arrivals[m][0]
        o = np.sign((A - c.c)[0] * h[1] - (A - c.c)[1] * h[0])
        k = node(i, A, o)
        on_circle[i].append(k)
        add(k, arr_nodes[m], 0.0, ("none",))
    # arcs between neighbouring tangent points of the same orientation, in that orientation
    for i, ks in on_circle.items():
        c = circles[i]
        for o in (+1, -1):
            pts = sorted({k for k in ks if nodes[k][2] == o}, key=lambda k: np.arctan2(*(nodes[k][1] - c.c)[::-1]))
            if len(pts) < 2:
                continue
            angs = [np.arctan2(*(nodes[k][1] - c.c)[::-1]) for k in pts]
            for m in range(len(pts)):
                n2 = (m + 1) % len(pts)
                if o > 0:
                    k1, k2, a1, da = pts[m], pts[n2], angs[m], (angs[n2] - angs[m]) % (2 * np.pi)
                else:
                    k1, k2, a1, da = pts[n2], pts[m], angs[n2], -((angs[n2] - angs[m]) % (2 * np.pi))
                if abs(da) < 1e-9:
                    continue
                n = max(int(abs(da) * c.R / 0.01), 3)
                ang = a1 + da * np.linspace(0, 1, n)
                P = c.c + c.R * np.stack([np.cos(ang), np.sin(ang)], 1)
                if sc.clear(P, skip=(i,) if not c.virtual else ()).min() > -1e-6:
                    add(k1, k2, abs(da) * c.R, ("arc", c.c, c.R, a1, da))
    # Dijkstra, then check the footprint along the route's edges (heading on the tangent, as the executor drives them);
    # edges where the body would touch something are dropped and the search runs again (lazy collision checking).
    # The grown circles only make sure the body's sides clear an obstacle the route wraps around; the front and rear
    # corners swing wider on arcs and can reach a neighbouring obstacle.
    adj = {}
    for (u, v_), (cost, geom) in edges.items():
        adj.setdefault(u, {})[v_] = (cost, geom)
    checked, rounds = {}, 0
    while True:
        rounds += 1
        dist, prev, pq = {s_node: 0.0}, {}, [(0.0, s_node)]
        while pq:
            dcur, u = heapq.heappop(pq)
            if dcur > dist.get(u, np.inf) + 1e-12:
                continue
            if u == t_node:
                break
            for v_, (cost, geom) in adj.get(u, {}).items():
                nd = dcur + cost
                if nd < dist.get(v_, np.inf) - 1e-12:
                    dist[v_], prev[v_] = nd, (u, geom)
                    heapq.heappush(pq, (nd, v_))
        if t_node not in dist:
            break
        bad, k = [], t_node
        while k != s_node:
            u, g = prev[k]
            if (u, k) not in checked:
                checked[(u, k)] = edge_clear(w, g) >= CHECK_MARGIN
            if not checked[(u, k)]:
                bad.append((u, k))
            k = u
        if not bad:
            break
        for u, k in bad:
            del adj[u][k]
    info["rounds"] = rounds
    if t_node not in dist:
        info.update(fail="no route", solve_s=time.time() - t0)
        return None, info
    geoms, k = [], t_node
    while k != s_node:
        k, g = prev[k]
        geoms.append(g)
    geoms.reverse()
    pts = [S]
    for g in geoms:
        if g[0] == "seg":
            n = max(int(np.hypot(*(g[2] - g[1])) / ARC_STEP), 1)
            pts.extend(np.linspace(g[1], g[2], n + 1)[1:])
        elif g[0] == "arc":
            _, c, R, a1, da = g
            n = max(int(abs(da) * R / ARC_STEP), 2)
            ang = a1 + da * np.linspace(0, 1, n + 1)[1:]
            pts.extend(c + R * np.stack([np.cos(ang), np.sin(ang)], 1))
    P = [pts[0]]
    for p in pts[1:]:
        if np.hypot(*(p - P[-1])) > 1e-4:
            P.append(p)
    P = np.array(P)
    info.update(length=float(np.hypot(*np.diff(P, axis=0).T).sum()), solve_s=time.time() - t0,
                cost=float(dist[t_node]), nodes=len(nodes), edges=len(edges))
    return P, info


class TangentController:
    """Plans one leg with the tangent graph (the next inflation if there is no route or the footprint check fails),
    then drives it with the shared executor."""

    INFLATIONS = (0.45, 0.40, 0.37)     # half-width 0.353 + 0.1 m margin, then less where passages are narrow

    def __init__(self, w, robot, goal, cache=None, fallback=True):
        import execute as ex
        self.goal = np.asarray(goal, float)
        t0 = time.time()
        self.info = {"inflate": None, "solve_s": 0.0, "fail": None, "pieces": 0, "length": None, "route": None}
        backup, p = ex.backup_distance(w, robot.pose)
        actions, why = None, []
        for inf in self.INFLATIONS:
            P, ri = route(w, p, self.goal, inf)
            self.info["solve_s"] += ri["solve_s"] or 0.0
            if P is None:
                why.append(f"{inf}: {ri['fail']}")
                continue
            actions, reason = ex.build(w, robot, P, self.goal, start_heading=p[2])
            why.append(f"{inf}: {reason}")
            if actions is not None:
                self.info.update(inflate=inf, length=ri["length"], route="tangent")
                break
        if actions is None and fallback:
            # tight arena: stop-and-turn route on the (x, y, heading) grid
            import lattice_route
            actions, ri = lattice_route.plan(w, p, self.goal)
            self.info["solve_s"] += ri["solve_s"] or 0.0
            why.append(f"grid: {ri['fail']}")
            if actions is not None:
                self.info.update(length=ri["length"], route="grid", reverse=ri["reverse"])
        if actions is not None:
            self.info["pieces"] = sum(a[0] in ("follow", "line") for a in actions)
        self.info["fail"] = None if actions is not None else "; ".join(why)
        self.info["plan_s"] = time.time() - t0
        self.run = ex.Executor(robot, actions, backup) if actions is not None else None

    def __call__(self, pose, t):
        if self.run is None:
            return 0.0, 0.0, True
        return self.run(pose, t)
