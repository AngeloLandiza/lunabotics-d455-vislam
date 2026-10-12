"""Wheel ruts: the robot's wheels press tracks into loose BP-1, and later drives see them.

Rut depth: static sinkage of a rigid wheel, J.Y. Wong, "Predicting the performances of rigid rover wheels on
extraterrestrial surfaces based on test results obtained on earth", Journal of Terramechanics 49 (2012) 49-61, Eq. (6)
with Eqs. (4)-(5) (Kc = kc/b, K_phi = k_phi/g), which gives Bekker's form
    z = [3 W / ((3 - n) (kc + b k_phi) sqrt(D))] ** (2 / (2n + 1))
W wheel load (N), b wheel width (m), D wheel diameter (m). No Bekker parameters for BP-1 were found, so params.RUTS uses
the Lunar Sourcebook's recommended lunar-soil values (via Carrier, LPI) and treats the result as one point of a sweep.

Rut shape (Assumed): a flat-bottomed trench as wide as the wheel with 1 cm rounded edges, and berms on both sides
holding berm_frac of the trench volume (the rest is compaction). Front and rear wheels on one side share a track; a
second pass in the same track does not deepen it (multi-pass sinkage is not modelled). Ruts are stamped into the
arena height field, so the sandbox ray caster and Isaac Sim (through isaac_bridge.export_arena_usd) both see them.
"""
import numpy as np

import params as pr
from params import v

EDGE = 0.01     # m, rounded trench edge (Assumed)


def bekker_sinkage(load_n, width_m, diam_m, kc, kphi, n):
    """Static sinkage (m) of a rigid wheel; kc in N/m^(n+1), kphi in N/m^(n+2)."""
    return (3 * load_n / ((3 - n) * (kc + width_m * kphi) * np.sqrt(diam_m))) ** (2 / (2 * n + 1))


def rut_depth(robot=None):
    """Sinkage for params.RUTS soil values and the robot's mass and wheels (robot.py), in metres."""
    n = v(pr.RUTS, "n")
    kc = v(pr.RUTS, "kc") * 100 ** (n + 1)        # N/cm^(n+1) -> N/m^(n+1)
    kphi = v(pr.RUTS, "kphi") * 100 ** (n + 2)    # N/cm^(n+2) -> N/m^(n+2)
    radius, width = (robot.wheels[0][1], robot.wheels[0][2]) if robot is not None else (0.1651, 0.1)
    n_wheels = len(robot.wheels) if robot is not None else 4
    load = v(pr.RUTS, "robot_mass") * 9.81 / n_wheels
    return bekker_sinkage(load, width, 2 * radius, kc, kphi, n)


class Ruts:
    """Accumulates trench depth and berm height over an arena's grid; apply() writes them into arena.h."""

    def __init__(self, arena, depth, width=0.10, berm_frac=None, berm_width=None):
        self.arena = arena
        self.h0 = arena.h.copy()
        self.trench = np.zeros_like(arena.h)
        self.berm = np.zeros_like(arena.h)
        self.depth, self.width = depth, width
        self.berm_frac = v(pr.RUTS, "berm_frac") if berm_frac is None else berm_frac
        self.berm_width = v(pr.RUTS, "berm_width") if berm_width is None else berm_width
        inside = (arena.X >= 0) & (arena.X <= arena.L) & (arena.Y >= 0) & (arena.Y <= arena.W)
        self.ground = inside & (arena.label == 0)          # no ruts on obstacles, the column or the walls

    def segment(self, p0, p1):
        """Stamp one wheel's track from p0 to p1 (x, y), clipped to a bounding box for speed."""
        a = self.arena
        half = self.width / 2
        reach = half + self.berm_width + 0.02
        x0, x1 = min(p0[0], p1[0]) - reach, max(p0[0], p1[0]) + reach
        y0, y1 = min(p0[1], p1[1]) - reach, max(p0[1], p1[1]) + reach
        i0, i1 = np.searchsorted(a.X[:, 0], [x0, x1])
        j0, j1 = np.searchsorted(a.Y[0, :], [y0, y1])
        if i1 <= i0 or j1 <= j0:
            return
        X, Y = a.X[i0:i1, j0:j1], a.Y[i0:i1, j0:j1]
        d = np.asarray(p1, float) - np.asarray(p0, float)
        L2 = max(float(d @ d), 1e-12)
        s = np.clip(((X - p0[0]) * d[0] + (Y - p0[1]) * d[1]) / L2, 0, 1)
        u = np.hypot(X - (p0[0] + s * d[0]), Y - (p0[1] + s * d[1]))      # distance to the track centre line
        # trench: full depth to half - EDGE, then a half-cosine to zero at half
        tr = self.depth * np.where(u <= half - EDGE, 1.0,
                                   np.where(u < half, 0.5 * (1 + np.cos(np.pi * (u - half + EDGE) / EDGE)), 0.0))
        # berms: a sin^2 hump over berm_width on each side; volume per side = berm_frac/2 of the trench volume
        h_b = self.berm_frac * self.depth * self.width / (2 * self.berm_width * 0.5)   # mean of sin^2 is 0.5
        w = (u - half) / self.berm_width
        bm = np.where((w > 0) & (w < 1), h_b * np.sin(np.pi * w) ** 2, 0.0)
        sl = (slice(i0, i1), slice(j0, j1))
        self.trench[sl] = np.maximum(self.trench[sl], tr)
        self.berm[sl] = np.maximum(self.berm[sl], bm)

    def path(self, poses, wheel_offsets):
        """Stamp every wheel's track along a robot path; poses are (x, y, yaw_deg), wheel_offsets (lateral m)."""
        P = np.asarray(poses, float)
        for k in range(len(P) - 1):
            for lat in wheel_offsets:
                ends = []
                for x, y, yaw in (P[k], P[k + 1]):
                    a = np.radians(yaw)
                    ends.append((x - lat * np.sin(a), y + lat * np.cos(a)))
                self.segment(*ends)

    def apply(self):
        """Write the ruts into arena.h: trenches win over berms where tracks cross."""
        dz = np.where(self.trench > 0, -self.trench, self.berm)
        self.arena.h = np.where(self.ground, self.h0 + dz, self.h0)
        return self.arena


def wheel_offsets(robot=None):
    """Distinct lateral wheel positions (front and rear wheels on one side share a track)."""
    if robot is None:
        return [-0.27, 0.27]
    return sorted({round(c[1], 4) for c, _, _ in robot.wheels})


def earlier_trips(arena, rng, n_trips, half_w=0.36, clear=0.05, tries=20000):
    """Drives across the obstacle zone, from the start side (x 0.5-1.8 m) through up to three waypoints in the zone
    (x 2.0-4.4 m, in order) to the far side (x 4.6-6.3 m), whose corridor (half_w plus clear) misses every obstacle, as
    a planner would weave through (Assumed traffic pattern). Each trip is a list of (x, y, yaw_deg) corners."""
    def clear_of_obstacles(a, b):
        d = b - a
        L2 = max(float(d @ d), 1e-12)
        for o in arena.obstacles:
            s = np.clip(np.dot([o.x - a[0], o.y - a[1]], d) / L2, 0, 1)
            if np.hypot(*(a + s * d - [o.x, o.y])) < o.radius + half_w + clear:
                return False
        return True

    trips = []
    for _ in range(tries):
        if len(trips) >= n_trips:
            break
        k = rng.integers(1, 4)
        xs = np.sort(rng.uniform(2.0, 4.4, k))
        pts = ([np.array([rng.uniform(0.5, 1.8), rng.uniform(0.6, arena.W - 0.6)])]
               + [np.array([x, rng.uniform(0.6, arena.W - 0.6)]) for x in xs]
               + [np.array([rng.uniform(4.6, 6.3), rng.uniform(0.6, arena.W - 0.6)])])
        if all(clear_of_obstacles(pts[i], pts[i + 1]) for i in range(len(pts) - 1)):
            trip = []
            for i in range(len(pts) - 1):
                d = pts[i + 1] - pts[i]
                yaw = float(np.degrees(np.arctan2(d[1], d[0])))
                trip += [(pts[i][0], pts[i][1], yaw), (pts[i + 1][0], pts[i + 1][1], yaw)]
            trips.append(trip)
    return trips


def add_earlier_ruts(arena, rng, n_trips, depth, robot=None):
    """Arena after n_trips earlier drives left ruts. Returns the trips (for plots and Isaac)."""
    r = Ruts(arena, depth, width=robot.wheels[0][2] if robot is not None else 0.10)
    trips = earlier_trips(arena, rng, n_trips)
    for t in trips:
        r.path(t, wheel_offsets(robot))
    r.apply()
    return trips


if __name__ == "__main__":
    from robot import Robot
    rb = Robot()
    print(f"rut depth from params.RUTS and the URDF ({v(pr.RUTS, 'robot_mass')} kg, 4 wheels "
          f"r {rb.wheels[0][1]} m, b {rb.wheels[0][2]} m): {100 * rut_depth(rb):.2f} cm")
