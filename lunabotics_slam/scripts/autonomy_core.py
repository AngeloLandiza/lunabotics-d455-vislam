"""Autonomy building blocks shared by the autonomy nodes (no ROS imports, so they run and test offline).

Each block is the step 2 research code the sim tests scored, called directly rather than copied, so the robot runs the
same code the numbers came from:

  HazardMap     log-odds boulder / crater detector with the robust local (LOESS) ground fit
                (sim/bp1_sandbox/detect.py; thresholds and evidence levels in its params.DETECT)
  WallFilter    wheel + gyro prediction, optional LiDAR-odometry window fusion, and the four-wall ICP fix with the
                chi-square gate and relock (sim/odometry/drift.py fused_walls; ICP from sim/bp1_sandbox/localize.py)
  TagFix        start-zone pose from the two AprilTag 36h11 tags, one per start-zone wall (sim/fiducial/tag_start.py)
  MissionPlan   dig -> dump -> dig legs, tangent-graph routes and the shared executor (sim/planner)

What is new here, and so not covered by those tests (Inferred unless measured in arena.sh): turning ROS point
clouds into the detector's ray form (free-space samples along each returned ray only; rays with no return carry no
evidence), the body box that removes the robot's own returns, timing from message stamps, and replanning when a new
hazard lands on the route.

The arena frame is Angelo's sim/arena.py frame: origin at the start-zone corner, x along the 6.88 m length, z = 0 at
the raked regolith surface.
"""
import importlib.util
import math
import os
import sys

import numpy as np


def _repo_root():
    env = os.environ.get('LUNABOTICS_REPO')
    if env:
        return env
    here = os.path.dirname(os.path.realpath(__file__))          # <repo>/lunabotics_slam/scripts
    return os.path.dirname(os.path.dirname(here))


REPO = _repo_root()
SIM = os.path.join(REPO, 'sim')
if not os.path.isdir(os.path.join(SIM, 'bp1_sandbox')):
    raise ImportError(f'autonomy_core needs the repo sim/ folder (looked in {SIM}); set LUNABOTICS_REPO')
for sub in ('fiducial', 'odometry', 'planner', 'bp1_sandbox'):    # bp1_sandbox ends up first, as the sim code expects
    p = os.path.join(SIM, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

import params as pr                     # noqa: E402  (sim/bp1_sandbox/params.py)
from params import v                    # noqa: E402
import detect                           # noqa: E402
import localize                         # noqa: E402
import arena2d as a2                    # noqa: E402
import execute as ex                    # noqa: E402
import tangent_route as tr              # noqa: E402
import drift                            # noqa: E402
import tag_start                        # noqa: E402


def _load_arena_2027():
    """sim/arena.py under its own name: sim/bp1_sandbox/arena.py already owns the module name 'arena'."""
    spec = importlib.util.spec_from_file_location('arena_2027', os.path.join(SIM, 'arena.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


A27 = _load_arena_2027()
ARENA_L, ARENA_W = A27.LENGTH, A27.WIDTH
_bz = A27.ZONES['berm']
BERM = (_bz[0], _bz[2], _bz[1], _bz[3])                         # (x0, x1, y0, y1), arena2d.World order
COLUMN_DISC = (A27.COLUMN[0], A27.COLUMN[1], A27.COLUMN[2] / math.sqrt(2))   # the box's circumscribed disc

PARAMS = {   # new numbers this file adds (P(value, unit, status, source), as elsewhere)
    'body_box': pr.P((-0.48, 0.60, -0.40, 0.39, -0.20, 0.80), 'm (x0,x1,y0,y1,z0,z1) in base_link', 'Inferred',
                     'arena2d footprint (uic_bot_2027.xacro body extents) grown by 4 cm; z from just above the wheel '
                     'contact (base_link is 0.265 m up) to above the L1. Returns inside are the robot itself'),
    'base_height': pr.P(0.2651, 'm', 'Verified', 'base_link height above the ground (team URDF; tag_start.py)'),
    'ray_step': pr.P(0.025, 'm', 'Assumed', 'free-space sample spacing along a ray (half a detector cell)'),
    'ray_zcap': pr.P(0.50, 'm', 'Assumed', 'free-space samples above this height are skipped: they sit over the '
                     'tallest boulder (0.40 m) plus h_pos, so they cannot change a cell'),
    'l1_sigma': pr.P(0.02, 'm', 'Inferred', 'L1 range accuracy 2.0 cm used as 1 sigma (params.LIDAR_GENERIC l1)'),
    'd455_k': pr.P(0.32 / ((848 / 2) / math.tan(math.radians(87 / 2)) * 0.095), '1/m', 'Inferred',
                   'D455 depth sigma = k z^2 (sim/realism.py: 0.32 px subpixel, 848 px, 87 deg, 95 mm baseline)'),
    'wall_margin': pr.P(0.40, 'm', 'Inferred', 'detections centred this close to the interior edge are the ducts or '
                        'the wall (DETECT.wall_band was 0.15 m in the sandbox, which had no ducts; arena.sh seed 0 '
                        'had false boulders along the ducts up to 0.26 m out). The planner keeps base_link 0.37-0.45 m '
                        'from the walls anyway'),
    'pole_margin': pr.P(0.15, 'm', 'Inferred', 'detections this close to a berm range pole are the pole (sim/arena.py '
                        'POLES; arena.sh seed 0 had false boulders at the poles)'),
    'max_range': pr.P(4.0, 'm', 'Assumed', 'returns farther than this from the sensor are not used for hazards: '
                      'arena.sh seed 0 had many false small boulders at 3-6.6 m (grazing L1 returns, far D455 depth with '
                      'sigma = k z^2, about 12 cm at 4 m). The sandbox speed runs need hazards seen by about 2 m ahead'),
    'max_radius': pr.P(0.5, 'm', 'Assumed', 'a blob wider than this is not one guidebook obstacle (boulders 30-40 '
                       'cm, craters up to about 50 cm): arena.sh seed 0 grew a 1.9 m crater over the rutted dig area'),
    'min_radius': pr.P(0.14, 'm', 'Assumed', 'objects smaller than this are dropped: guidebook boulders are 30-40 cm, '
                       'and detected discs come out a few cm larger than the true ones; arena.sh seed 0 false boulders '
                       'had a median radius of 9 cm, and false craters from the ruts 13 cm; true ones came out 18-22 cm'),
    'crater_depth': pr.P((0.05, 3), 'm, cells', 'Assumed',
                         'a crater needs at least this many cells inside its disc whose mean return height lies this far '
                         'below the fitted ground. arena.sh seed 0 start zone (true pose): 4 false craters just behind '
                         'boulders / the column were a 9-17 % tail of low returns (to -4..-9 cm, median -1 to -2 cm), the '
                         'true crater (11 cm deep) had 26 % of returns below -3 cm, down to -12 cm; guidebook-scale '
                         'craters in arena.sh are 7-12 cm deep'),
    'column_margin': pr.P(0.15, 'm', 'Assumed', 'detections this close to the known column box are the column'),
    'tag_ids': pr.P((0, 1), '', 'Assumed', 'tag on the y = 0 wall, tag on the x = 0 wall'),
    'tag_xy': pr.P(((1.0, -A27.DUCT_D, 90.0), (-A27.DUCT_D, 1.0, 0.0)), 'm, m, deg facing', 'Assumed',
                   'tag centres 1.0 m along the two start-zone bin walls (tag_start.layouts two_perpendicular, the '
                   'layout Angelo chose), on the wall faces behind the 17 cm ducts, as arena.sh places them'),
    'wall_offset': pr.P(A27.DUCT_D, 'm', 'Inferred', 'the bin walls stand behind the ducts: wall faces at x = -0.17, '
                        'L + 0.17 and the same in y (sim/arena_scene.py)'),
    'wall_band': pr.P((0.22, 0.44), 'm above the bed', 'Inferred', 'wall points for the fix: above the 17 cm ducts '
                      'and below the 0.45 m bin wall top (arena_scene STYLE wall_height, from the BSD-3 arena model); '
                      'the sandbox test used 0.30-1.10 m on taller walls'),
    'tag_size': pr.P(0.1651, 'm', 'Assumed', 'black-square edge; 16.51 cm as printed for arena.sh '
                     '(tag_start used 0.16 m; the team size is open)'),
    'dig_time': pr.P(8.0, 's', 'Assumed', 'time at a dig spot (no excavator model yet)'),
    'dump_time': pr.P(5.0, 's', 'Assumed', 'time at the dump spot'),
    'dig_stroke': pr.P((0.25, 0.05), 'm radius, m depth', 'Assumed', 'what one dig asks the sim to remove'),
    'replan_clear': pr.P(0.02, 'm', 'Assumed', 'replan when a hazard comes closer than this to the planned route '
                         'at the planning inflation'),
}


def val(name):
    return PARAMS[name].value


# --------------------------------------------------------------------------- geometry

def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def base_T(x, y, yaw, z=None):
    """4x4 pose of base_link in the arena (planar robot; base_link at base_height)."""
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[:3, 3] = (x, y, val('base_height') if z is None else z)
    return T


def apply_T(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def quat_to_R(q):
    """q = (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def body_mask(P_base):
    """True for points inside the robot's own body box (base_link frame)."""
    x0, x1, y0, y1, z0, z1 = val('body_box')
    return ((P_base[:, 0] > x0) & (P_base[:, 0] < x1) & (P_base[:, 1] > y0) & (P_base[:, 1] < y1)
            & (P_base[:, 2] > z0) & (P_base[:, 2] < z1))


# --------------------------------------------------------------------------- hazard detector

def make_scan(origin, P, sigma_r, sensor):
    """One sensor frame in the form detect.py works on: rays from origin through the returned points P (arena frame),
    with free-space samples every ray_step from the sensor up to (r - ray_step), below ray_zcap."""
    origin = np.asarray(origin, float)
    d = P - origin
    r = np.linalg.norm(d, axis=1)
    keep = (r > 0.05) & (r <= val('max_range'))
    P, d, r = P[keep], d[keep], r[keep]
    sigma_r = np.broadcast_to(np.asarray(sigma_r, float), keep.shape)[keep] if np.ndim(sigma_r) else \
        np.full(len(r), float(sigma_r))
    dirs = d / r[:, None]
    step, zcap = val('ray_step'), val('ray_zcap')
    # first sample index per ray: where the ray drops below zcap (rays rising above it from below are kept whole)
    dz = dirs[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        t0 = np.where(dz < -1e-6, (origin[2] - zcap) / -dz, 0.0)
    t0 = np.clip(np.nan_to_num(t0), 0.0, None)
    k0 = np.ceil(t0 / step).astype(np.int64)
    k0 = np.maximum(k0, 1)
    k1 = np.floor((r - step) / step).astype(np.int64)
    n = np.clip(k1 - k0 + 1, 0, None)
    ray = np.repeat(np.arange(len(r)), n)
    if len(ray):
        start = np.repeat(np.cumsum(n) - n, n)
        k = np.arange(len(ray)) - start + np.repeat(k0, n)
        t = k * step
        xyz = origin + t[:, None] * dirs[ray]
    else:
        t, xyz = np.zeros(0), np.zeros((0, 3))
    samples = {'ray': ray, 't': t, 'x': xyz[:, 0], 'y': xyz[:, 1], 'z': xyz[:, 2]}
    from raycast import flat_ground_range
    return {'sensor': sensor, 'origin': origin, 'dirs': dirs, 'r': r, 'valid': np.ones(len(r), bool), 'pts': P,
            'sigma': sigma_r, 'sigma_r': sigma_r, 'r_flat': flat_ground_range(origin, dirs), 'samples': samples}


class HazardMap:
    """detect.py's log-odds layers kept over the whole arena and updated frame by frame.

    The sim scored detect.detect(method='logodds') on a batch of frames with one ground fit. Here the ground is refit
    every refit_s from all returns so far, and each new frame's evidence is added with the latest fit, clamped as in
    detect.logodds_layers. Objects are clusters whose pooled evidence reaches the sensor's decision level
    (DETECT.l_decide_lidar 5.1, l_decide_d455 0.85)."""

    def __init__(self, cell=None, size=None):
        self.size = size or (ARENA_L, ARENA_W)
        self.grid = detect.Grid(*self.size, cell or v(pr.DETECT, 'cell'))
        self.L = detect.new_layers(self.grid)
        shape = (self.grid.nx, self.grid.ny)
        self.lp = {s: np.zeros(shape) for s in ('lidar', 'd455')}
        self.ln = {s: np.zeros(shape) for s in ('lidar', 'd455')}
        self.g = np.zeros(shape)
        self.frames = 0
        self.dropped = []                   # oversized blobs objects() split or dropped (for logs)
        self.shallow = []                   # crater blobs with no real depression under them (for logs)

    def refit(self):
        if (self.L['count'] > 0).sum() > 50:
            self.g = detect.fit_ground(self.L, self.grid)

    def add(self, scan):
        kind = 'd455' if scan['sensor'] == 'd455' else 'lidar'
        detect.add_scan(self.L, scan, self.grid, carve=False)
        if self.frames == 0:
            self.refit()
        zref = detect.logodds_zref(self.L, self.g)
        dp, dn = detect.logodds_delta(scan, self.grid, self.g, zref)
        lmin, lmax = v(pr.DETECT, 'l_min'), v(pr.DETECT, 'l_max')
        self.lp[kind] = np.clip(self.lp[kind] + dp, lmin, lmax)
        self.ln[kind] = np.clip(self.ln[kind] + dn, lmin, lmax)
        self.frames += 1

    def objects(self):
        """Hazards as dicts: kind ('boulder' / 'crater'), x, y, r (disc of the cluster's area), evidence, sensor.
        Wall returns and the known column are dropped (the planner already has both)."""
        out, big = [], []
        self.shallow = []
        cell = self.grid.cell
        cx, cy, cr = COLUMN_DISC
        for sensor in ('lidar', 'd455'):
            lp, ln = self.lp[sensor], self.ln[sensor]
            for d in detect.clusters(lp > 0, ln > 0, self.grid, lp, ln, detect.l_decide(sensor)):
                if min(d['x'], d['y'], self.size[0] - d['x'], self.size[1] - d['y']) < val('wall_margin'):
                    continue
                if math.hypot(d['x'] - cx, d['y'] - cy) < cr + val('column_margin'):
                    continue
                if any(math.hypot(d['x'] - px, d['y'] - py) < val('pole_margin') for px, py in A27.POLES):
                    continue
                r = math.sqrt(d['cells'] * cell * cell / math.pi) + cell / 2
                if r < val('min_radius'):
                    continue
                if r > val('max_radius'):
                    big.append((sensor, d))
                    continue
                if d['kind'] == 'neg' and not self.depression(d['x'], d['y'], r):
                    self.shallow.append((sensor, round(d['x'], 2), round(d['y'], 2), round(r, 2)))
                    continue
                out.append({'kind': 'boulder' if d['kind'] == 'pos' else 'crater', 'x': d['x'], 'y': d['y'], 'r': r,
                            'evidence': d['evidence'], 'sensor': sensor})
        # a blob wider than any guidebook obstacle (shallow ruts or dug ground joined to a crater): keep only its
        # saturated cells, split into clusters again; what is still too wide is dropped as terrain
        for sensor, d in big:
            lp, ln = self.lp[sensor], self.ln[sensor]
            strong = v(pr.DETECT, 'l_max') - 0.5
            pos, neg = (lp >= strong, ln * 0 > 1) if d['kind'] == 'pos' else (lp * 0 > 1, ln >= strong)
            for e in detect.clusters(pos, neg, self.grid, lp, ln, detect.l_decide(sensor)):
                r = math.sqrt(e['cells'] * cell * cell / math.pi) + cell / 2
                if e['kind'] == 'neg' and not self.depression(e['x'], e['y'], r):
                    continue
                if val('min_radius') <= r <= val('max_radius') and math.hypot(e['x'] - d['x'], e['y'] - d['y']) < 3:
                    out.append({'kind': 'boulder' if e['kind'] == 'pos' else 'crater', 'x': e['x'], 'y': e['y'],
                                'r': r, 'evidence': e['evidence'], 'sensor': sensor})
            self.dropped.append((d['kind'], round(d['x'], 2), round(d['y'], 2), d['cells']))
        return merge_discs(out)

    def depression(self, x, y, r):
        """True when the returns inside the disc show a real hole: crater_depth[1] cells (2+ returns each) whose mean
        height is crater_depth[0] or more below the fitted ground. Crater log-odds alone also fire on the scattered
        low returns just behind boulders and the column, where only grazing rays land."""
        depth, n_min = val('crater_depth')
        g, L, c = self.grid, self.L, self.grid.cell
        i0, i1 = max(int((x - r) / c), 0), min(int((x + r) / c) + 1, g.nx)
        j0, j1 = max(int((y - r) / c), 0), min(int((y + r) / c) + 1, g.ny)
        X, Y = np.meshgrid((np.arange(i0, i1) + 0.5) * c, (np.arange(j0, j1) + 0.5) * c, indexing='ij')
        inside = (X - x) ** 2 + (Y - y) ** 2 <= r * r
        n = L['count'][i0:i1, j0:j1]
        mean = L['zsum'][i0:i1, j0:j1] / np.maximum(n, 1)
        deep = inside & (n >= 2) & (self.g[i0:i1, j0:j1] - mean >= depth)
        return int(deep.sum()) >= n_min

    def occupancy(self):
        """int8 grid (nx, ny): 100 where either sensor's evidence is positive, 0 where seen, -1 unseen."""
        seen = self.L['count'] > 0
        occ = np.where(seen, 0, -1).astype(np.int8)
        bad = np.zeros(seen.shape, bool)
        for s in ('lidar', 'd455'):
            bad |= (self.lp[s] > 0) | (self.ln[s] > 0)
        occ[bad] = 100
        return occ


def merge_discs(objs):
    """The same object seen by both sensors: one disc covering both. Only discs whose centres lie within the larger
    radius merge, and a merged disc never grows past max_radius (chains of small blobs once merged into a 1.3 m disc
    in arena.sh)."""
    out = []
    for o in sorted(objs, key=lambda o: -o['evidence']):
        for m in out:
            d = math.hypot(m['x'] - o['x'], m['y'] - o['y'])
            if m['kind'] == o['kind'] and d < max(m['r'], o['r']) and d + o['r'] <= val('max_radius'):
                m['r'] = max(m['r'], d + o['r'])
                m['sensor'] = 'both' if m['sensor'] != o['sensor'] else m['sensor']
                break
        else:
            out.append(dict(o))
    return out


def depth_cloud(depth, K, stride=8, zmax=6.0):
    """Points (camera optical frame) and their ray-direction cos from a depth image (metres, 0 = none)."""
    h, w = depth.shape
    vv, uu = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[vv, uu].astype(float)
    ok = np.isfinite(z) & (z > 0.3) & (z < zmax)
    uu, vv, z = uu[ok], vv[ok], z[ok]
    x = (uu - K[0, 2]) / K[0, 0] * z
    y = (vv - K[1, 2]) / K[1, 1] * z
    P = np.stack([x, y, z], 1)
    cosang = z / np.linalg.norm(P, axis=1)
    return P, cosang


def d455_sigma_r(z, cosang):
    """Range sigma of a D455 depth pixel: the depth sigma k z^2 along the optical axis, spread along the ray."""
    return val('d455_k') * z * z / np.maximum(cosang, 0.2)


# --------------------------------------------------------------------------- wall-matching filter

def _P(name):
    return v(drift.PARAMS, name)


class WallFilter:
    """drift.py's fused_walls estimator, driven by message callbacks.

    predict()         wheel speed (calibrated scale) + gyro yaw rate, mid-point integration
    lidar_window()    optional: a LiDAR-odometry increment over the last fuse_every seconds, fused with the wheel+gyro
                      increment by inverse covariance behind the chi-square gate (KISS-ICP or Point-LIO)
    add_wall_points() L1 returns in the wall band, put in the arena with the filter's pose at the cloud's time
    wall_fix()        every wall_every s: four-wall ICP from the current estimate; gated update, relock after
                      relock_n refused fits that agree
    """

    def __init__(self, scale=1.0, size=None, wall_offset=None, band=None):
        """size: interior (L, W); wall_offset: how far outside the interior the wall faces stand; band: wall-point
        heights above the bed. Defaults: Angelo's arena (6.88 x 5.0, walls behind the ducts, 0.22-0.44 m)."""
        self.scale = scale
        self.L, self.W = size or (ARENA_L, ARENA_W)
        self.off = val('wall_offset') if wall_offset is None else wall_offset
        self.band = band or val('wall_band')
        self.x = None
        self.Pc = None
        self.t = None
        self.hist = []                      # (t, x, y, yaw), for placing clouds at their own time
        self.buf = []
        self.rejected = []
        self.win = None                     # (x_w0, P_w0, t_w0, dist, steps, dt_sum)
        self.stats = {'wall_tries': 0, 'wall_weak': 0, 'wall_fixes': 0, 'wall_gated': 0, 'wall_relocks': 0,
                      'windows': 0, 'win_gated': 0}

    # -- state
    def init(self, pose, t, sig=None):
        sx, sa = sig or _P('tag_sigma')
        self.x = np.array(pose, float)
        self.Pc = np.diag([sx ** 2, sx ** 2, math.radians(sa) ** 2])
        self.t = t
        self.t_q, self.d_q = t, 0.0
        self.hist = [(t, *self.x)]
        self.win = (self.x.copy(), self.Pc.copy(), t, 0.0, 0.0)
        self.buf, self.rejected = [], []

    @property
    def ready(self):
        return self.x is not None

    def pose_at(self, t):
        """Estimated pose at time t (nearest history entry at or before t)."""
        if not self.hist:
            return self.x
        ts = np.array([h[0] for h in self.hist])
        k = int(np.clip(np.searchsorted(ts, t, side='right') - 1, 0, len(ts) - 1))
        return np.array(self.hist[k][1:])

    # -- prediction
    def predict(self, v_wheel, w_gyro, t):
        if self.x is None:
            return
        dt = t - self.t
        if dt <= 0 or dt > 0.5:
            self.t = t
            return
        vv = self.scale * v_wheel
        am = self.x[2] + w_gyro * dt / 2
        self.x = self.x + [vv * math.cos(am) * dt, vv * math.sin(am) * dt, w_gyro * dt]
        self.x[2] = wrap(self.x[2])
        # covariance: drift.py's window form. Slip and the scale error are correlated over a window (fuse_every), so
        # the distance part grows with the square of the distance driven in the window, and the gyro bias part with
        # the square of the window's duration; spread per step so the window total matches drift.py
        sig_d = math.hypot(_P('sig_v_wheel'), _P('wheel_cal_err'))
        if self.t - self.t_q >= _P('fuse_every'):
            self.t_q, self.d_q = self.t, 0.0
        d0, span0 = self.d_q, self.t - self.t_q
        self.d_q += abs(vv) * dt
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        J1 = np.array([[1, 0, -s * vv * dt], [0, 1, c * vv * dt], [0, 0, 1]])
        dvar = sig_d ** 2 * (self.d_q ** 2 - d0 ** 2) + 1e-10
        span1 = span0 + dt
        q = np.diag([dvar, dvar, math.radians(_P('gyro_noise')) ** 2 * dt
                     + math.radians(_P('gyro_bias')) ** 2 * (span1 ** 2 - span0 ** 2)])
        self.Pc = J1 @ self.Pc @ J1.T + q
        x0, P0, t0, dist, _ = self.win
        self.win = (x0, P0, t0, dist + abs(vv) * dt, t - t0)
        self.t = t
        self.hist.append((t, *self.x))
        if len(self.hist) > 4000:
            self.hist = self.hist[-2000:]

    # -- optional LiDAR odometry window
    def lidar_window(self, inc_l):
        """inc_l: the LiDAR odometry's (dx, dy, dyaw) over the window since the last call, in the window-start body
        frame. Replaces the window's wheel+gyro increment by the fused one (drift.py)."""
        if self.x is None:
            return
        x_w0, P_w0, t0, dist, span = self.win
        S0 = _se2(*x_w0)
        inc = _from_se2(np.linalg.inv(S0) @ _se2(*self.x))
        sv = math.hypot(_P('sig_v_wheel'), _P('wheel_cal_err')) * dist + 1e-4
        syaw2 = (math.radians(_P('gyro_noise')) ** 2 * span + (math.radians(_P('gyro_bias')) * span) ** 2) + 1e-10
        S_w = np.diag([sv ** 2, sv ** 2, syaw2])
        sl = _P('sig_win_lidar')
        S_l = np.diag([sl[0] ** 2, sl[0] ** 2, math.radians(sl[1]) ** 2])
        r = np.asarray(inc_l, float) - inc
        r[2] = wrap(r[2])
        self.stats['windows'] += 1
        if r @ np.linalg.solve(S_w + S_l, r) < _P('gate_chi2'):
            Sf = np.linalg.inv(np.linalg.inv(S_w) + np.linalg.inv(S_l))
            incf = Sf @ (np.linalg.solve(S_w, inc) + np.linalg.solve(S_l, inc + r))
            c, s = math.cos(x_w0[2]), math.sin(x_w0[2])
            J1 = np.array([[1, 0, -s * incf[0] - c * incf[1]], [0, 1, c * incf[0] - s * incf[1]], [0, 0, 1]])
            J2 = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            self.x = _from_se2(S0 @ _se2(*incf))
            self.Pc = J1 @ P_w0 @ J1.T + J2 @ Sf @ J2.T
        else:
            self.stats['win_gated'] += 1
        self.win = (self.x.copy(), self.Pc.copy(), self.t, 0.0, 0.0)

    # -- absolute pose (start-zone tags)
    def pose_update(self, z_pose, sig=None):
        """An absolute (x, y, yaw) fix with per-axis sigmas (m, deg; default tag_sigma), chi-square gated."""
        if self.x is None:
            return False
        sx, sa = sig or _P('tag_sigma')
        R = np.diag([sx ** 2, sx ** 2, math.radians(sa) ** 2])
        z = np.asarray(z_pose, float) - self.x
        z[2] = wrap(z[2])
        S = self.Pc + R
        if z @ np.linalg.solve(S, z) >= _P('gate_chi2'):
            return False
        Kg = self.Pc @ np.linalg.inv(S)
        self.x = self.x + Kg @ z
        self.x[2] = wrap(self.x[2])
        self.Pc = (np.eye(3) - Kg) @ self.Pc
        return True

    # -- wall fix
    def add_wall_points(self, P_base, t):
        """P_base: L1 returns in base_link (body returns removed). Keeps the wall band, placed in the arena with the
        estimated pose at t (the filter's own track deskews motion between clouds)."""
        if self.x is None:
            return
        zlo, zhi = self.band
        hb = val('base_height')
        band = (P_base[:, 2] + hb > zlo) & (P_base[:, 2] + hb < zhi)
        q = P_base[band, :2]
        x, y, a = self.pose_at(t)
        c, s = math.cos(a), math.sin(a)
        self.buf.append(np.stack([x + c * q[:, 0] - s * q[:, 1], y + s * q[:, 0] + c * q[:, 1]], -1))

    def wall_fix(self):
        """Returns 'fix', 'relock', 'gated', 'weak', 'few' or None (nothing buffered)."""
        if self.x is None or not self.buf:
            return None
        pts = np.concatenate(self.buf)
        self.buf = []
        Xi = np.linalg.inv(_se2(*self.x))
        Pl = pts @ Xi[:2, :2].T + Xi[:2, 2]
        if len(Pl) < _P('wall_min_pts'):
            return 'few'
        self.stats['wall_tries'] += 1
        # localize.icp has the walls at 0 and (L, W): shift by the wall offset and back
        o = np.array([self.off, self.off, 0.0])
        Lw, Ww = self.L + 2 * self.off, self.W + 2 * self.off
        pose, lam, n_used = localize.icp(Pl, self.x + o, Lw, Ww)
        if not (lam is not None and lam > _P('wall_lam_min') and n_used >= _P('wall_min_pts')):
            self.stats['wall_weak'] += 1
            return 'weak'
        rr, Jw = localize.wall_residuals(Pl, pose, Lw, Ww)
        pose = pose - o
        near = np.abs(rr) < 0.1
        H = Jw[near].T @ Jw[near]
        Rw = 10 * _P('wall_sigma') ** 2 * np.linalg.inv(H + 1e-6 * np.eye(3))
        z = pose - self.x
        z[2] = wrap(z[2])
        S = self.Pc + Rw
        gate = _P('gate_chi2')
        relock = False
        if z @ np.linalg.solve(S, z) >= gate:
            self.rejected.append(z)
            zs = np.array(self.rejected[-_P('relock_n'):])
            tol_xy, tol_a = _P('relock_tol')
            if (len(zs) == _P('relock_n') and np.ptp(zs[:, 0]) < tol_xy and np.ptp(zs[:, 1]) < tol_xy
                    and np.ptp(zs[:, 2]) < math.radians(tol_a)):
                self.Pc = self.Pc + np.diag(z ** 2)
                S = self.Pc + Rw
                relock = True
                self.stats['wall_relocks'] += 1
        else:
            self.rejected = []
        if relock or z @ np.linalg.solve(S, z) < gate:
            self.rejected = []
            Kg = self.Pc @ np.linalg.inv(S)
            self.x = self.x + Kg @ z
            self.x[2] = wrap(self.x[2])
            self.Pc = (np.eye(3) - Kg) @ self.Pc
            self.stats['wall_fixes'] += 1
            if self.hist:
                self.hist[-1] = (self.hist[-1][0], *self.x)
            return 'relock' if relock else 'fix'
        self.stats['wall_gated'] += 1
        return 'gated'


def _se2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def _from_se2(M):
    return np.array([M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0])])


# --------------------------------------------------------------------------- start-zone tags

def tag_corners():
    """{id: 4x3 world corners in OpenCV IPPE_SQUARE order} for the two start-zone tags (tag_start.tag geometry)."""
    out = {}
    s = val('tag_size') / 2
    for tid, (x, y, facing) in zip(val('tag_ids'), val('tag_xy')):
        normal_yaw = math.radians(facing)                           # into the arena
        right = np.array([-math.sin(normal_yaw), math.cos(normal_yaw), 0.0])
        up = np.array([0, 0, 1.0])
        c = np.array([x, y, tag_start.v('tag_height')])
        out[tid] = np.array([c - right * s + up * s, c + right * s + up * s, c + right * s - up * s,
                             c - right * s - up * s])
    return out


class TagFix:
    """Robot pose in the arena from AprilTag 36h11 detections in one rear-camera image."""

    def __init__(self):
        import cv2
        self.cv2 = cv2
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'ArucoDetector'):       # OpenCV >= 4.7
            prm = cv2.aruco.DetectorParameters()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            det = cv2.aruco.ArucoDetector(dic, prm)
            self._detect = det.detectMarkers
        else:                                          # Ubuntu 22.04's OpenCV 4.5.4: the older function API
            prm = cv2.aruco.DetectorParameters_create()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda g: cv2.aruco.detectMarkers(g, dic, parameters=prm)
        self.world = tag_corners()

    def detect(self, gray):
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners) if int(i) in self.world}

    def solve(self, found, K, D, T_base_opt):
        """found: {id: 4x2 px}. T_base_opt: 4x4 camera optical frame in base_link. Returns (x, y, yaw, n_tags,
        reprojection rms px) or None. One tag: IPPE_SQUARE (both solutions tried, lowest error kept, as tag_start);
        two tags on perpendicular walls: SQPnP + LM refinement on all 8 corners."""
        cv2 = self.cv2
        if not found:
            return None
        obj = np.concatenate([self.world[i] for i in sorted(found)]).astype(np.float64)
        img = np.concatenate([found[i] for i in sorted(found)]).astype(np.float64)
        D = np.zeros(5) if D is None else np.asarray(D, np.float64)
        if len(found) == 1:
            s = val('tag_size') / 2
            local = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
            n, rvecs, tvecs, errs = cv2.solvePnPGeneric(local, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not n:
                return None
            k = int(np.argmin(np.ravel(errs)))
            R_ct, _ = cv2.Rodrigues(rvecs[k])
            t_ct = tvecs[k].ravel()
            # tag frame -> world: local x = right, y = up, z = out of the tag (towards the viewer)
            W = self.world[next(iter(found))]
            c = W.mean(0)
            right = (W[1] - W[0]) / np.linalg.norm(W[1] - W[0])
            up = (W[0] - W[3]) / np.linalg.norm(W[0] - W[3])
            R_wt = np.stack([right, up, np.cross(right, up)], 1)
            R_cw = R_ct @ R_wt.T
            t_cw = t_ct - R_cw @ c
            err = float(np.ravel(errs)[k])
        else:
            ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
            if not ok:
                return None
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
            R_cw, _ = cv2.Rodrigues(rvec)
            t_cw = tvec.ravel()
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, 1))))
        R_wc = R_cw.T
        t_wc = -R_wc @ t_cw
        R_bo, t_bo = T_base_opt[:3, :3], T_base_opt[:3, 3]
        R_wb = R_wc @ R_bo.T
        t_wb = t_wc - R_wb @ t_bo
        return float(t_wb[0]), float(t_wb[1]), math.atan2(R_wb[1, 0], R_wb[0, 0]), len(found), err


# --------------------------------------------------------------------------- mission planning and driving

class RobotModel:
    """What execute.Executor reads from the robot: commanded wheel speeds with the acceleration limit (the base
    driver's ramp), track B and the driver's track multiplier, and the speed limits (arena2d.SkidSteer numbers)."""

    def __init__(self, mult=None):
        self.B = v(a2.PARAMS, 'track')
        self.mult = mult if mult is not None else a2.SLIP_LEVELS['nominal'][2]
        self.vmax, self.amax = v(a2.PARAMS, 'v_wheel_max'), v(a2.PARAMS, 'a_wheel_max')
        self.vl = self.vr = 0.0
        self.pose = np.zeros(3)

    @property
    def w_max(self):
        return 2 * self.vmax / (self.B * self.mult)

    def command(self, cmd_v, cmd_w, dt):
        """Ramp the commanded wheel speeds as SkidSteer does; returns the (v, w) the ramp allows."""
        tl, trr = cmd_v - cmd_w * self.B * self.mult / 2, cmd_v + cmd_w * self.B * self.mult / 2
        k = max(1.0, abs(tl) / self.vmax, abs(trr) / self.vmax)
        tl, trr = tl / k, trr / k
        da = self.amax * dt
        self.vl += float(np.clip(tl - self.vl, -da, da))
        self.vr += float(np.clip(trr - self.vr, -da, da))
        return (self.vl + self.vr) / 2, (self.vr - self.vl) / (self.B * self.mult)


def make_world(hazards):
    """arena2d.World for Angelo's arena: hazard discs, the column box disc, the berm keep-out box."""
    obs = [a2.Disc(h['kind'], h['x'], h['y'], h['r']) for h in hazards]
    obs.append(a2.Disc('column', *COLUMN_DISC))
    return a2.World(seed=world_key(obs), L=ARENA_L, W=ARENA_W, obstacles=obs, berm=BERM)


def world_key(obs):
    """World.seed for a hazard set. lattice_route caches its grid per World.seed, so every different hazard set
    needs its own key (one shared key would reuse a grid built before the newest hazards were known)."""
    return -1 - (hash(tuple((o.kind, round(o.x, 3), round(o.y, 3), round(o.r, 3)) for o in obs)) & 0x7FFFFFFF)


def mission_goals(rng, cycles=None):
    """Dig and dump poses (arena2d.mission rules: dig box, three dump spots along the berm's start-side edge, facing
    it) on the known map (column only): hazards found later are handled by replanning."""
    w = make_world([])
    _, goals = a2.mission(w, rng)
    n = 2 * (cycles or v(a2.PARAMS, 'cycles'))
    return goals[:n]


def escape_move(w, pose, max_d, step=0.05):
    """Signed straight distance (+ forwards) to drive so the robot can turn in place, or has the most room to: the
    footprint must not get closer to anything along the way than it is now (it may start inside a grown hazard)."""
    pose = np.asarray(pose, float)
    u = np.array([math.cos(pose[2]), math.sin(pose[2]), 0.0])
    here = min(a2.clearance(w, pose), ex.CHECK_MARGIN)
    best, best_score = 0.0, a2.turn_clearance(w, pose[:2])
    for sgn in (-1.0, 1.0):
        for d in np.arange(step, max_d + 1e-9, step):
            q = pose + sgn * d * u
            if a2.clearance(w, q) < here - 0.005:
                break
            sc = a2.turn_clearance(w, q[:2])
            if sc > best_score + 0.01:
                best, best_score = sgn * d, sc
    return float(best)


def relocate_goal(kind, g, hazards, max_move=0.8, step=0.05):
    """A nearby goal the robot can reach and turn at: dig goals move to the nearest free pose in the dig box (same
    heading), dump goals to the other dump spots along the berm. None if there is none."""
    w = make_world(hazards)
    if kind == 'dump':
        fp = v(a2.PARAMS, 'footprint')
        y = w.berm[3] + v(a2.PARAMS, 'dump_clear') + fp[1]
        for x in sorted(v(a2.PARAMS, 'dump_x'), key=lambda x: abs(x - g[0])):
            c = (x, y, g[2])
            if abs(x - g[0]) > 1e-6 and a2.clearance(w, c, berm=False) > 0.10:
                return c
        return None
    x0, x1, y0, y1 = v(a2.PARAMS, 'dig_box')
    best = None
    for r in np.arange(step, max_move + 1e-9, step):
        for a in np.linspace(0, 2 * np.pi, max(int(2 * np.pi * r / step), 8), endpoint=False):
            c = (g[0] + r * math.cos(a), g[1] + r * math.sin(a), g[2])
            if not (x0 <= c[0] <= x1 and y0 <= c[1] <= y1):
                continue
            if a2.clearance(w, c) > 0.15 and a2.turn_clearance(w, c[:2]) > 0.05:
                best = c
                break
        if best:
            return best
    return None


class Leg:
    """One leg: tangent-graph route (TangentController's inflation sequence and grid fallback) and the executor."""

    def __init__(self, hazards, robot, goal):
        self.goal = np.asarray(goal, float)
        self.world = make_world(hazards)
        self.r_pose0 = np.array(robot.pose, float)
        self.ctl = tr.TangentController(self.world, robot, self.goal)
        self.info = self.ctl.info
        self.route = self._route_points()

    def _route_points(self):
        """The planned base_link path as points every 5 cm (follow profiles sampled, straight runs interpolated)."""
        run = self.ctl.run
        if run is None:
            return np.zeros((0, 2))
        P = [np.asarray(self.r_pose0[:2], float)[None]]
        for kind, arg in run.actions:
            if kind == 'follow':
                s = np.linspace(0, arg.length, max(int(arg.length / 0.05), 2))
                P.append(np.stack([arg.sx(s), arg.sy(s)], 1))
            elif kind in ('line', 'final'):
                b = np.asarray(arg[0] if kind == 'line' else arg, float)[:2]
                a = P[-1][-1]
                n = max(int(np.hypot(*(b - a)) / 0.05), 1)
                P.append(a + np.linspace(0, 1, n + 1)[1:, None] * (b - a))
        return np.concatenate(P)

    @property
    def ok(self):
        return self.ctl.run is not None

    def blocked_by(self, hazards, pose):
        """True when a hazard not in this leg's world now lies on the rest of the route (at the planning inflation)."""
        if not len(self.route):
            return False
        inf = self.info.get('inflate') or tr.TangentController.INFLATIONS[-1]     # grid routes: the tightest one
        k = int(np.argmin(np.hypot(*(self.route - pose[:2]).T)))
        rest = self.route[k:]
        known = [(o.x, o.y, o.r) for o in self.world.obstacles]
        for h in hazards:
            if any(math.hypot(h['x'] - x, h['y'] - y) < 0.05 and abs(h['r'] - r) < 0.05 for x, y, r in known):
                continue
            d = np.hypot(rest[:, 0] - h['x'], rest[:, 1] - h['y']).min() - h['r']
            if d < inf - val('replan_clear'):
                return True
        return False

    def __call__(self, pose, t):
        return self.ctl(pose, t)


class MissionRunner:
    """Dig -> dump legs as a state machine, called at the control rate with the estimated pose and current hazards.

    plan     plan a Leg to the next goal from rest; a failed plan is retried every retry_s (the hazard map may change)
             and the goal is skipped after plan_tries
    drive    run the Leg; when a new hazard lands on the rest of the route, stop, then plan again from rest
    stop     zero command until the wheels are still (RobotModel), then plan
    retreat  after a failed plan: a straight move forwards or backwards of up to retreat_m that ends where the robot
             has the most room to turn (escape_move), then plan again; a stop beside a hazard seen late, or the start
             pose by a wall, can leave no route from where the robot is
    move     after a second failure: the goal itself may sit too close to a detected hazard (detected discs come out a
             few cm larger than the true ones), so a dig goal moves to the nearest free pose in the dig box and a dump
             goal to the next dump spot (relocate_goal); then plan again
    work     at a dig or dump goal: hold still for dig_time / dump_time; the event is reported once on arrival
    done     all goals handled
    stuck    two goals in a row skipped, or the robot has not moved stuck_m in stuck_s while it should be driving:
             the mission ends there (Angelo, 2026-10-12: terminate a run when the rover gets stuck)
    step() returns (v, w, event) with event None or ('dig' | 'dump', goal) on arrival.
    """

    def __init__(self, goals, robot, retry_s=1.0, plan_tries=5, retreat_m=0.4, stuck_s=30.0, stuck_m=0.10):
        self.goals = list(goals)
        self.r = robot
        self.k = 0
        self.state = 'plan'
        self.leg = None
        self.t_state = 0.0
        self.tries = 0
        self.retry_s, self.plan_tries, self.retreat_m = retry_s, plan_tries, retreat_m
        self.retreat_from = None
        self.escape = None
        self.skips_in_row = 0
        self.stuck_s, self.stuck_m = stuck_s, stuck_m
        self.moved_at = None                # (t, xy) of the last time the robot had moved stuck_m
        self.why_stuck = None
        self.log = []                       # (t, message) for the node to print

    def _go(self, state, t):
        self.state, self.t_state = state, t

    def _still(self):
        return abs(self.r.vl) + abs(self.r.vr) < 0.02

    @property
    def goal(self):
        return self.goals[self.k] if self.k < len(self.goals) else None

    def step(self, pose, t, hazards, dt):
        self.r.pose = np.asarray(pose, float)
        if self.k >= len(self.goals):
            self.state = 'done'
            return self._cmd(0.0, 0.0, dt) + (None,)
        if self.state == 'stuck':
            return self._cmd(0.0, 0.0, dt) + (None,)
        kind, g = self.goals[self.k]
        xy = np.asarray(pose[:2], float)
        if self.moved_at is None or np.hypot(*(xy - self.moved_at[1])) >= self.stuck_m or self.state == 'work':
            self.moved_at = (t, xy.copy())
        elif t - self.moved_at[0] > self.stuck_s:
            return self._stuck(t, f'no progress for {self.stuck_s:.0f} s')
        if self.state == 'plan':
            if t - self.t_state < (self.retry_s if self.tries else 0.0):
                return self._cmd(0.0, 0.0, dt) + (None,)
            self.leg = Leg(hazards, self.r, g)
            self.tries += 1
            if self.leg.ok:
                self.log.append((t, f'goal {self.k + 1}/{len(self.goals)} {kind}: route {self.leg.info["route"]} '
                                    f'{self.leg.info["length"]:.2f} m, inflation {self.leg.info["inflate"]}'))
                self.tries = 0
                self._go('drive', t)
            else:
                self.log.append((t, f'goal {self.k + 1} {kind}: no route ({self.leg.info["fail"]})'))
                if self.tries == 2:
                    g2 = relocate_goal(kind, g, hazards)
                    if g2 is not None:
                        self.goals[self.k] = (kind, g2)
                        self.log.append((t, f'goal {self.k + 1} {kind}: moved to x {g2[0]:.2f} y {g2[1]:.2f}'))
                        self.t_state = t - self.retry_s
                        return self._cmd(0.0, 0.0, dt) + (None,)
                if self.tries == 1:
                    self.retreat_from = np.asarray(pose, float)[:2].copy()
                    self.log.append((t, f'goal {self.k + 1} {kind}: backing out to plan again'))
                    self._go('retreat', t)
                    return self._cmd(0.0, 0.0, dt) + (None,)
                if self.tries >= self.plan_tries:
                    self.log.append((t, f'goal {self.k + 1} {kind}: skipped'))
                    self.k, self.tries = self.k + 1, 0
                    self.skips_in_row += 1
                    if self.skips_in_row >= 2:
                        return self._stuck(t, 'two goals in a row skipped')
                self.t_state = t
                return self._cmd(0.0, 0.0, dt) + (None,)
        if self.state == 'drive':
            if self.leg.blocked_by(hazards, pose):
                self.log.append((t, f'goal {self.k + 1} {kind}: new hazard on the route, stopping to replan'))
                self._go('stop', t)
            else:
                cv, cw, done = self.leg(np.asarray(pose, float), t - self.t_state)
                if done:
                    self.skips_in_row = 0
                    self.log.append((t, f'goal {self.k + 1} {kind}: arrived'))
                    self._go('work', t)
                    return self._cmd(0.0, 0.0, dt) + ((kind, g),)
                return self._cmd(cv, cw, dt) + (None,)
        if self.state == 'stop':
            if self._still():
                self._go('plan', t)
            return self._cmd(0.0, 0.0, dt) + (None,)
        if self.state == 'retreat':
            if self.escape is None:
                self.escape = escape_move(make_world(hazards), pose, self.retreat_m)
                self.log.append((t, f'goal {self.k + 1} {kind}: escape move {self.escape:+.2f} m'))
            gone = math.hypot(*(np.asarray(pose[:2]) - self.retreat_from))
            left = abs(self.escape) - gone
            if left <= 0.01 or t - self.t_state > 8.0:
                if self._still():
                    self.escape = None
                    self._go('plan', t - self.retry_s)
                return self._cmd(0.0, 0.0, dt) + (None,)
            return self._cmd(math.copysign(min(0.15, 1.5 * left + 0.05), self.escape), 0.0, dt) + (None,)
        if self.state == 'work':
            hold = val('dig_time') if kind == 'dig' else val('dump_time')
            if t - self.t_state >= hold:
                self.k += 1
                self._go('plan', t)
            return self._cmd(0.0, 0.0, dt) + (None,)
        return self._cmd(0.0, 0.0, dt) + (None,)

    def _stuck(self, t, why):
        self.state, self.why_stuck = 'stuck', why
        self.log.append((t, f'STUCK: {why}; mission stopped'))
        return self._cmd(0.0, 0.0, 0.05) + (None,)

    def _cmd(self, cv, cw, dt):
        self.r.command(cv, cw, dt)
        return (float(cv), float(cw))
