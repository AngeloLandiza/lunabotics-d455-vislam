"""BP-1 realism layer for the Isaac Sim arena (sim/isaac_lunar_sim.py, run/sim/arena/arena.sh).

Isaac renders ideal sensors on a rigid-friction terrain. This module adds what the real arena does to the robot,
using the numbers worked out in the sandbox (sim/bp1_sandbox, sim/odometry) and the step 2 research
(/mnt/project-files/research/step2, 14-24 with their checks). Every number sits in PARAMS with a status:
Verified (read in a source), Inferred (derived from sources), Assumed (our choice, to be measured).

Layers (each switchable, all on at --realism nominal):
  depth   D455 stereo depth: disparity noise sigma_z = z^2 sigma_d / (f B) at the driver's 848 px width, correlated
          over the matcher's window, a dropout floor, flying-pixel loss at depth edges, range limits, and dust;
  ir      D455 IR images: read and shot noise;
  imu     gyro and accelerometer white noise and a per-run bias (the D455 IMU; BMI055 class);
  slip    longitudinal wheel slip on loose BP-1: the wheels turn faster than the body moves, so wheel odometry
          over-reads; /wheel/odom reports what encoders would;
  ruts    wheels press trenches into the bed as the rover drives (Bekker static sinkage, lunar-soil values);
  dust    a plume after each dig or dump that blinds stereo depth along rays through it.

Pure numpy, so it runs under Isaac Sim's Python and system Python alike. Levels: off, nominal, heavy.
"""
import math

import numpy as np

SB = "sim/bp1_sandbox/params.py"
R22, R23 = "research/step2/22-imu-noise.md", "research/step2/23-wheel-odometry-slip.md"


class P:
    def __init__(self, value, unit, status, source):
        self.value, self.unit, self.status, self.source = value, unit, status, source


PARAMS = {
    # D455 depth (Intel: sigma_z = z^2 * subpixel / (f * B))
    "baseline": P(0.095, "m", "Verified", "D455 product page (" + SB + " D455)"),
    "hfov": P(87.0, "deg", "Verified", "D455 product page"),
    "driver_width": P(848, "px", "Verified", "lunabotics_slam/config/d455_fusion.yaml depth_profile 848,480,30"),
    "subpixel": P({"nominal": 0.32, "heavy": 0.5}, "px", "Inferred",
                  "nominal from Intel's '<2% at 4 m' (" + SB + " subpixel_field); best case 0.08 px is Verified "
                  "(Intel tuning guide); heavy Assumed"),
    "noise_corr_px": P(6, "px", "Assumed", "disparity errors are shared over the stereo matcher's window"),
    "invalid_frac": P({"nominal": 0.02, "heavy": 0.06}, "", "Assumed", "dropout floor on low-texture regolith"),
    "edge_rel": P(0.06, "", "Assumed", "neighbouring depths differing by more than this share of depth = an edge"),
    "edge_drop": P(0.5, "", "Assumed", "share of edge pixels the matcher loses (flying pixels removed)"),
    "min_depth": P(0.52, "m", "Assumed", "D455 minimum depth at 848 px; check against the datasheet's min-Z"),
    "max_depth": P(6.0, "m", "Assumed", "useful range cap (" + SB + ")"),
    # D455 IR images
    "ir_read_dn": P({"nominal": 2.0, "heavy": 4.0}, "DN", "Assumed", "8-bit read noise"),
    "ir_shot_gain": P({"nominal": 0.25, "heavy": 0.5}, "DN per DN", "Assumed", "shot-noise variance per signal DN"),
    # IMU
    "gyro_nd": P(0.014, "deg/s/sqrt(Hz)", "Inferred", "BMI055 noise density, Bosch forum post (" + R22 + "); the "
                 "D455's IMU chip is unconfirmed"),
    "gyro_bias": P({"nominal": 0.1, "heavy": 0.3}, "deg/s (1 sigma per run)", "Assumed",
                   "turn-on bias; BMI055 offset drifts 0.015 deg/s/K (" + R22 + ")"),
    "accel_nd": P(150e-6 * 9.81, "m/s^2/sqrt(Hz)", "Inferred", "BMI055 150 ug/sqrt(Hz), Bosch forum post (" + R22 + ")"),
    "accel_bias": P({"nominal": 0.05, "heavy": 0.15}, "m/s^2 (1 sigma per run)", "Assumed",
                    "turn-on bias; BMI055 offset drifts 1 mg/K (" + R22 + ")"),
    "imu_hz": P(200.0, "Hz", "Verified", "rate the sim publishes /camera/imu (D455 gyro 200/400 Hz, " + R22 + ")"),
    # wheel slip
    "slip": P({"nominal": (0.05, 0.03), "heavy": (0.25, 0.08)}, "slip ratio mean, sd", "Inferred",
              "nominal: flat loose-soil values suggested in " + R23 + " (no BP-1 measurement); heavy Assumed "
              "(sim/planner SLIP_LEVELS)"),
    "slip_tau": P(1.0, "s", "Assumed", "how long one slip draw lasts (sim/planner)"),
    # ruts (Wong 2012 Eq. 6, Bekker form)
    "kc": P(0.14, "N/cm^(n+1)", "Verified", "Carrier / Lunar Sourcebook, lunar soil, not BP-1 (" + SB + " RUTS)"),
    "kphi": P(0.82, "N/cm^(n+2)", "Verified", "same"),
    "n": P(1.0, "", "Verified", "same"),
    "rut_min_move": P(0.02, "m", "Assumed", "stamp ruts after the wheel has moved this far"),
    "berm_frac": P(0.3, "", "Assumed", "share of the trench volume pushed into side berms; the rest is compaction of "
                   "fluffy BP-1 (" + SB + " RUTS)"),
    "berm_width": P(0.06, "m", "Assumed", "width of each side berm (" + SB + " RUTS)"),
    # dust plume after a dig or dump
    "dust_beta": P({"nominal": 1.5, "heavy": 3.0}, "1/m", "Assumed", "peak extinction (" + SB + " DUST)"),
    "dust_sigma_h": P(0.5, "m", "Assumed", SB + " DUST"),
    "dust_sigma_v": P(0.3, "m", "Assumed", SB + " DUST"),
    "dust_z": P(0.3, "m", "Assumed", SB + " DUST"),
    "dust_life": P(4.0, "s", "Assumed", "plume decays linearly to nothing over this time after each dig or dump"),
}
LEVELS = ("off", "nominal", "heavy")


def val(name, level):
    x = PARAMS[name].value
    return x[level] if isinstance(x, dict) else x


def table():
    """Markdown table of PARAMS (for the README and the dashboard)."""
    rows = ["| Parameter | Value | Unit | Status | Source |", "| --- | --- | --- | --- | --- |"]
    for k, p in PARAMS.items():
        rows.append(f"| {k} | {p.value} | {p.unit} | {p.status} | {p.source} |")
    return "\n".join(rows)


def _upsample(a, h, w):
    """Bilinear resize of a small 2D array to (h, w)."""
    gh, gw = a.shape
    yi = np.linspace(0, gh - 1, h)
    xi = np.linspace(0, gw - 1, w)
    y0, x0 = np.floor(yi).astype(int), np.floor(xi).astype(int)
    y1, x1 = np.minimum(y0 + 1, gh - 1), np.minimum(x0 + 1, gw - 1)
    fy, fx = (yi - y0)[:, None], (xi - x0)[None, :]
    top = a[y0][:, x0] * (1 - fx) + a[y0][:, x1] * fx
    bot = a[y1][:, x0] * (1 - fx) + a[y1][:, x1] * fx
    return top * (1 - fy) + bot * fy


def correlated_noise(h, w, corr_px, rng):
    """Unit-variance Gaussian noise correlated over about corr_px pixels."""
    gh, gw = max(2, int(math.ceil(h / corr_px)) + 1), max(2, int(math.ceil(w / corr_px)) + 1)
    n = _upsample(rng.standard_normal((gh, gw)), h, w)
    return n / max(float(n.std()), 1e-6)


class DustPlumes:
    """Dust clouds from digs and dumps: Gaussian in space, fading linearly over dust_life seconds."""

    def __init__(self, level):
        self.level, self.plumes = level, []

    def add(self, x, y, t):
        if self.level != "off":
            self.plumes.append((float(x), float(y), float(t)))

    def active(self, t):
        life = val("dust_life", self.level)
        self.plumes = [p for p in self.plumes if t - p[2] < life]
        return self.plumes

    def optical_depth(self, origin, dirs, dist, t):
        """Optical depth from origin along unit dirs (N, 3) to distance dist (N,), summed over active plumes.

        The extinction beta * exp(-dh^2 / 2 sh^2 - dz^2 / 2 sv^2) is integrated along each ray in closed form (the
        Gaussian is anisotropic, so the ray is rescaled by (sh, sh, sv) first)."""
        tau = np.zeros(len(dirs))
        life = val("dust_life", self.level)
        sh, sv = val("dust_sigma_h", self.level), val("dust_sigma_v", self.level)
        S = np.array([sh, sh, sv])
        for px, py, t0 in self.active(t):
            beta = val("dust_beta", self.level) * max(0.0, 1 - (t - t0) / life)
            c = np.array([px, py, val("dust_z", self.level)])
            o = (np.asarray(origin, float) - c) / S                   # scaled space: isotropic unit Gaussian
            d = dirs / S
            a = np.einsum("ij,ij->i", d, d)                           # |d|^2
            b = d @ o                                                 # d . o
            cc = o @ o
            # integral_0^L exp(-(a s^2 + 2 b s + cc) / 2) ds
            k = np.exp(-(cc - b ** 2 / a) / 2) * np.sqrt(np.pi / (2 * a))
            erf = np.vectorize(math.erf)
            lo = erf(b / np.sqrt(2 * a))
            hi = erf((a * dist + b) / np.sqrt(2 * a))
            tau += beta * k * (hi - lo)
        return tau


class D455Realism:
    def __init__(self, level, rng):
        self.level, self.rng = level, rng
        f = (val("driver_width", level) / 2) / math.tan(math.radians(val("hfov", level)) / 2)
        self.k = val("subpixel", level) / (f * val("baseline", level)) if level != "off" else 0.0

    def depth(self, z, tau=None):
        """z: depth image (h, w) in metres, 0 or nan = no data. tau: optical depth to each pixel's surface (h, w),
        or None. Returns the noisy depth with 0 where the camera reports nothing."""
        z = np.nan_to_num(np.asarray(z, np.float32), nan=0.0, posinf=0.0)
        if self.level == "off":
            return z
        lv, rng = self.level, self.rng
        h, w = z.shape
        valid = (z >= val("min_depth", lv)) & (z <= val("max_depth", lv))
        trans = np.ones_like(z) if tau is None else np.exp(-tau)
        p_inv = val("invalid_frac", lv) + (1 - trans)
        # edges: neighbours that disagree a lot (flying pixels on boulder rims and crater lips)
        dz = np.zeros_like(z)
        dz[:, 1:] = np.maximum(dz[:, 1:], np.abs(np.diff(z, axis=1)))
        dz[1:, :] = np.maximum(dz[1:, :], np.abs(np.diff(z, axis=0)))
        edge = dz > val("edge_rel", lv) * np.maximum(z, 1e-3)
        p_inv = np.where(edge, 1 - (1 - p_inv) * (1 - val("edge_drop", lv)), p_inv)
        valid &= rng.random(z.shape) >= np.clip(p_inv, 0, 1)
        sig = self.k * z ** 2 / np.maximum(trans, 0.05)
        n = 0.8 * correlated_noise(h, w, val("noise_corr_px", lv), rng) + 0.6 * rng.standard_normal(z.shape)
        out = z + (sig * n).astype(np.float32)
        return np.where(valid, out, 0.0).astype(np.float32)

    def ir(self, mono):
        """mono: uint8 image. Adds shot and read noise."""
        if self.level == "off":
            return mono
        lv = self.level
        x = mono.astype(np.float32)
        sd = np.sqrt(val("ir_read_dn", lv) ** 2 + val("ir_shot_gain", lv) * x)
        return np.clip(x + sd * self.rng.standard_normal(x.shape), 0, 255).astype(np.uint8)


class ImuRealism:
    def __init__(self, level, rng):
        self.level, self.rng = level, rng
        if level == "off":
            self.bg, self.ba = np.zeros(3), np.zeros(3)
            return
        self.bg = np.radians(val("gyro_bias", level)) * rng.standard_normal(3)
        self.ba = val("accel_bias", level) * rng.standard_normal(3)
        hz = val("imu_hz", level)
        self.sg = np.radians(val("gyro_nd", level)) * math.sqrt(hz)      # per-sample sd from the noise density
        self.sa = val("accel_nd", level) * math.sqrt(hz)

    def apply(self, gyro, accel):
        if self.level == "off":
            return np.asarray(gyro, float), np.asarray(accel, float)
        return (np.asarray(gyro, float) + self.bg + self.sg * self.rng.standard_normal(3),
                np.asarray(accel, float) + self.ba + self.sa * self.rng.standard_normal(3))


class Slip:
    """Longitudinal slip ratio s, redrawn every slip_tau seconds: the body moves (1 - s) times the wheel surface
    speed. The sim drives the physics wheels at (1 - s) of the commanded speed and reports encoder speeds divided by
    (1 - s), so wheel odometry over-reads the way it does on loose regolith."""

    def __init__(self, level, rng):
        self.level, self.rng = level, rng
        self.s, self.t_next = 0.0, -1.0

    def ratio(self, t):
        if self.level == "off":
            return 0.0
        if t >= self.t_next:
            m, sd = val("slip", self.level)
            self.s = float(np.clip(self.rng.normal(m, sd), 0.0, 0.6))
            self.t_next = t + val("slip_tau", self.level)
        return self.s


def rut_depth(mass_kg, n_wheels, wheel_width, wheel_diam, g=9.81):
    """Bekker / Wong (2012) Eq. (6) static sinkage of a rigid wheel, lunar-soil kc, kphi, n (m)."""
    n = val("n", "nominal")
    kc = val("kc", "nominal") * 100 ** (n + 1)
    kphi = val("kphi", "nominal") * 100 ** (n + 2)
    load = mass_kg * g / n_wheels
    return (3 * load / ((3 - n) * (kc + wheel_width * kphi) * math.sqrt(wheel_diam))) ** (2 / (2 * n + 1))


class Ruts:
    """Stamps wheel tracks into the height map H (rows = x, cols = y) while the rover drives. A cell under a wheel is
    lowered to H0 - depth (never dug deeper by later passes; digging by the dig tool still wins), and a low berm
    holding berm_frac of the trench section is raised on each side (sim/bp1_sandbox/ruts.py shape)."""

    def __init__(self, level, xs, ys, H0, depth, wheel_width, wheel_xy):
        self.on = level != "off"
        self.xs, self.ys, self.H0 = xs, ys, H0
        self.depth, self.half = depth, wheel_width / 2
        self.wheel_xy = np.asarray(wheel_xy, float)          # wheel contact points in the base frame (x, y)
        self.last = None

    def stamp(self, H, x, y, yaw):
        """Lower H under each wheel since the last call. Returns the touched index box (i0, i1, j0, j1) or None."""
        if not self.on:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        P = np.array([[x + c * wx - s * wy, y + s * wx + c * wy] for wx, wy in self.wheel_xy])
        if self.last is None:
            self.last = P
            return None
        if np.max(np.hypot(*(P - self.last).T)) < val("rut_min_move", "nominal"):
            return None
        res = self.xs[1] - self.xs[0]
        box = [10 ** 9, -1, 10 ** 9, -1]
        for a, b in zip(self.last, P):
            reach = self.half + val("berm_width", "nominal") + res
            lo = np.minimum(a, b) - reach
            hi = np.maximum(a, b) + reach
            i0, i1 = np.searchsorted(self.xs, lo[0]), np.searchsorted(self.xs, hi[0])
            j0, j1 = np.searchsorted(self.ys, lo[1]), np.searchsorted(self.ys, hi[1])
            if i1 <= i0 or j1 <= j0:
                continue
            X, Y = np.meshgrid(self.xs[i0:i1], self.ys[j0:j1], indexing="ij")
            ab = b - a
            L2 = float(ab @ ab)
            tt = np.clip(((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / L2, 0, 1) if L2 > 0 else np.zeros_like(X)
            d = np.hypot(X - a[0] - tt * ab[0], Y - a[1] - tt * ab[1])
            under = d <= self.half
            sub = H[i0:i1, j0:j1]
            h0 = self.H0[i0:i1, j0:j1]
            floor = h0 - self.depth
            # only on the open bed: not on rocks (well above the original surface) and not in dug holes
            bed = (sub < h0 + 0.03) & (sub > h0 - self.depth - 0.005)
            ok = under & bed & (sub > floor)
            # side berms: a raised-cosine ridge just outside the wheel, holding berm_frac of the trench section
            bw = val("berm_width", "nominal")
            ring = (d > self.half) & (d <= self.half + bw) & (sub > h0 - 0.002) & (sub < h0 + 0.03) & ~under
            if ok.any() or ring.any():
                hb = 2 * val("berm_frac", "nominal") * self.depth * (2 * self.half) / (2 * bw)   # peak of 1-cos profile
                prof = hb * 0.5 * (1 + np.cos(np.pi * (2 * (d - self.half) / bw - 1)))
                sub[ring] = np.maximum(sub[ring], (h0 + prof)[ring])
                sub[ok] = floor[ok]
                box = [min(box[0], i0), max(box[1], i1), min(box[2], j0), max(box[3], j1)]
        self.last = P
        return None if box[1] < 0 else tuple(box)


if __name__ == "__main__":
    print(table())
