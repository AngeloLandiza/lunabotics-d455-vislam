"""Generic LiDAR and D455 depth models over the arena height field."""
import numpy as np
import params as pr
from params import v
from raycast import march, flat_ground_range


def rot(yaw_deg, pitch_down_deg):
    """World-from-sensor rotation: pitch down about y, then yaw about z (x forward, y left, z up)."""
    p, y = np.radians(pitch_down_deg), np.radians(yaw_deg)
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry


def sensor_pose(robot_xy_yaw, mount_xyz, pitch_down):
    x, y, yaw = robot_xy_yaw
    Rz = rot(yaw, 0.0)
    origin = np.array([x, y, 0.0]) + Rz @ np.asarray(mount_xyz, float)
    return origin, rot(yaw, pitch_down)


def sensor_pose_R(robot_xy_yaw, mount_xyz, R_mount):
    """Like sensor_pose, for any robot-from-sensor rotation R_mount (for example an upside-down LiDAR)."""
    x, y, yaw = robot_xy_yaw
    Rz = rot(yaw, 0.0)
    return np.array([x, y, 0.0]) + Rz @ np.asarray(mount_xyz, float), Rz @ R_mount


def axis_rot(axis, deg):
    """Rotation by deg about a unit axis (Rodrigues)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    a = np.radians(deg)
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def l1_dirs(prof, rng):
    """Unitree L1-like scan for one frame, in the sensor frame (z = the scanner's axis, hemisphere z >= 0).

    The L1 user manual (v1.1) gives 11 Hz horizontal and 180 Hz vertical scanning, 43200 samples/s of which 21600 are
    effective, a 360 x 90 deg hemispherical field of view, and a mirror that measures "within a 180 deg range". Our
    reading (Inferred): a core spins about z at 11 Hz while a mirror sweeps the beam through a vertical half-circle
    from one horizon over the zenith to the other at 180 Hz; samples taken while the mirror faces the housing are lost.
    Phases are random per frame.
    """
    fs, T = prof["samples_per_s"], prof["frame_s"]
    t = np.arange(int(round(fs * T))) / fs
    phi = rng.uniform(0, 2 * np.pi) + 2 * np.pi * prof["f_h"] * t
    m = np.mod(rng.uniform(0, 2 * np.pi) + 2 * np.pi * prof["f_v"] * t, 2 * np.pi)
    keep = m <= np.pi
    phi, m = phi[keep], m[keep]
    return np.stack([np.cos(m) * np.cos(phi), np.cos(m) * np.sin(phi), np.sin(m)], -1)


def spinning_dirs(prof):
    el = np.radians(np.linspace(prof["vmin"], prof["vmax"], prof["channels"]))
    az = np.radians(np.arange(-180, 180, prof["hres"]))
    E, A = np.meshgrid(el, az, indexing="ij")
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    return d


def rosette_dirs(prof, rng):
    """Approximate non-repetitive rosette: off-axis angle r = (fov/2)|sin(k t)|, roll angle t, random phase per frame."""
    n = prof["points"]
    t = np.linspace(0, 2 * np.pi, n, endpoint=False) * 9.0 + rng.uniform(0, 2 * np.pi)
    r = np.radians(prof["fov"] / 2) * np.abs(np.sin(prof["petals"] * t / 9.0 * np.sqrt(2)))
    d = np.stack([np.cos(r), np.sin(r) * np.cos(t), np.sin(r) * np.sin(t)], -1)
    return d


class Lidar:
    def __init__(self, profile_name, mount_xyz=None, pitch_down=None, mode="first", mount_R=None, body=None,
                 mount_err_deg=0.0):
        """mount_R (robot-from-sensor rotation) overrides pitch_down. body is a robot.BodyCaster: the robot's own body
        then blocks and returns rays. mount_err_deg tilts the real sensor about its own y axis while the robot keeps
        placing points with the nominal mount (an extrinsic calibration error)."""
        self.name = profile_name
        self.prof = pr.LIDAR_PROFILES[profile_name]
        self.mount = v(pr.MOUNTS, "lidar_xyz") if mount_xyz is None else mount_xyz
        if pitch_down is None:
            pitch_down = 40.0 if self.prof["kind"] == "rosette" else v(pr.MOUNTS, "lidar_pitch_down")
        self.pitch = pitch_down
        self.R_mount = rot(0.0, pitch_down) if mount_R is None else np.asarray(mount_R, float)
        self.R_true = self.R_mount @ axis_rot([0, 1, 0], mount_err_deg)
        self.mode = mode
        self.body = body
        self.sigma = self.prof.get("range_sigma", v(pr.LIDAR_GENERIC, "range_sigma"))
        r_ref = self.prof.get("max_range_ref", v(pr.LIDAR_GENERIC, "max_range_ref"))
        self.p_th = v(pr.LIDAR_GENERIC, "rho_ref") / r_ref ** 2
        self.rmax = self.prof.get("rmax", 8.0)

    def dirs_s(self, rng):
        k = self.prof["kind"]
        if k == "spinning":
            return spinning_dirs(self.prof)
        if k == "l1":
            return l1_dirs(self.prof, rng)
        return rosette_dirs(self.prof, rng)

    def sigma_z(self, r):
        return np.full_like(r, self.sigma)

    def scan(self, arena, robot, rng, dust=None):
        origin, R = sensor_pose_R(robot, self.mount, self.R_true)
        d_s = self.dirs_s(rng)
        dirs = d_s @ R.T
        m = march(arena, origin, dirs, self.rmax, step=0.02, dust=dust)
        t = m["t_hit"]
        P = origin + np.nan_to_num(t)[:, None] * dirs
        normals = arena.normal(P[:, 0], P[:, 1])
        cos_override, is_body = None, None
        if self.body is not None:
            t, cos_override, is_body = self._occlude(d_s, t, m)
        out = self.apply(origin, dirs, t, normals, m, rng, dust_on=dust is not None, cos_override=cos_override,
                         is_body=is_body)
        out["dirs_s"] = d_s
        out["R_mount"] = self.R_mount
        if not np.allclose(self.R_true, self.R_mount):
            out = believed_mount(out, sensor_pose_R(robot, self.mount, self.R_mount)[1] @ R.T)
        return out

    def _occlude(self, d_s, t, m):
        """The robot's body blocks every ray that reaches it before the terrain; those rays end on the body."""
        if self.prof["kind"] == "spinning":
            tb, cb = self.body.table(self.name, d_s, self.mount, self.R_true)
        else:
            tb, cb = self.body.cast(self.mount, d_s @ self.R_true.T)
        blocked = np.isfinite(tb) & ~(np.nan_to_num(t, nan=np.inf) < tb)
        t = np.where(blocked, tb, t)
        s = m["samples"]
        keep = ~(blocked[s["ray"]] & (s["t"] >= tb[s["ray"]]))
        if not keep.all():
            m["samples"] = {k: a[keep] for k, a in s.items()}
            if "tau" in s:   # optical depth up to the body: that of the last free-space sample before it
                tau_b = np.zeros(len(t))
                np.maximum.at(tau_b, m["samples"]["ray"], m["samples"]["tau"])
                m["tau_hit"] = np.where(blocked, tau_b, m["tau_hit"])
        return t, np.where(blocked, cb, np.nan), blocked

    def apply(self, origin, dirs, t, normals, m, rng, dust_on, cos_override=None, is_body=None):
        """BP-1 layer on ideal hits: return power vs threshold, blind zone, dust echoes, range noise.

        t are ideal ranges (nan for no hit), normals the surface normals at the hits, m the march dict holding
        tau_hit, free-space samples and step. Used both on our own ray marcher and on Isaac Sim's ideal output.
        """
        hit = np.isfinite(t)
        rho_bp1 = v(pr.BP1, "lidar_reflectivity")
        rho = rho_bp1
        cosi = np.clip(-(normals * dirs).sum(1), 0, 1)
        is_body = np.zeros(len(dirs), bool) if is_body is None else is_body
        if is_body.any():
            cosi = np.where(is_body, np.nan_to_num(cos_override, nan=1.0), cosi)
            rho = np.where(is_body, v(pr.ROBOT, "body_reflectivity"), rho_bp1)
        power = rho * cosi / np.maximum(np.nan_to_num(t, nan=1.0), 1e-3) ** 2 * np.exp(-2 * m["tau_hit"])
        target_ok = hit & (np.nan_to_num(t) >= self.prof["min_range"]) & (power >= self.p_th)
        r_meas = np.where(target_ok, t, np.nan)
        is_dust = np.zeros(len(dirs), bool)
        if dust_on:
            s = m["samples"]
            kappa = v(pr.DUST, "echo_kappa")
            p_evt = 1 - np.exp(-kappa * s["alpha"] * m["step"])
            evt = rng.random(len(p_evt)) < p_evt
            # Particle clusters treated as small Lambertian patches facing the sensor (Assumed reflectivity = rho)
            pc = rho_bp1 / np.maximum(s["t"], 1e-3) ** 2 * np.exp(-2 * s["tau"])
            evt &= (pc >= self.p_th) & (s["t"] >= self.prof["min_range"])
            if evt.any():
                ray, tt, pw = s["ray"][evt], s["t"][evt], pc[evt]
                order = np.lexsort((tt, ray))
                ray, tt, pw = ray[order], tt[order], pw[order]
                first = np.r_[True, ray[1:] != ray[:-1]]
                ray, tt, pw = ray[first], tt[first], pw[first]
                if self.mode == "first":
                    take = np.isnan(r_meas[ray]) | (tt < r_meas[ray])
                else:  # strongest
                    take = np.isnan(r_meas[ray]) | (pw > power[ray])
                r_meas[ray[take]] = tt[take]
                is_dust[ray[take]] = True
        valid = np.isfinite(r_meas)
        r_noisy = r_meas + rng.normal(0, self.sigma, len(r_meas))
        pts = origin + np.nan_to_num(r_noisy)[:, None] * dirs
        return {
            "sensor": self.name, "origin": origin, "dirs": dirs, "r": np.where(valid, r_noisy, np.nan),
            "valid": valid, "pts": pts, "is_dust": is_dust, "r_true_surface": t,
            "r_flat": flat_ground_range(origin, dirs), "samples": m["samples"],
            "sigma": np.full(len(dirs), self.sigma), "sigma_r": np.full(len(dirs), self.sigma),
            "is_body": is_body & valid & ~is_dust,
        }


class D455Depth:
    def __init__(self, mount_xyz=None, pitch_down=None, subpixel=None):
        self.name = "d455"
        self.mount = v(pr.MOUNTS, "d455_xyz") if mount_xyz is None else mount_xyz
        self.pitch = v(pr.MOUNTS, "d455_pitch_down") if pitch_down is None else pitch_down
        self.w, self.h = v(pr.D455, "sim_size")
        hf, vf = np.radians(v(pr.D455, "hfov")), np.radians(v(pr.D455, "vfov"))
        self.f_native = (v(pr.D455, "native_width") / 2) / np.tan(hf / 2)   # 674 px at 1280 wide
        self.B = v(pr.D455, "baseline")
        self.sd = v(pr.D455, "subpixel_field") if subpixel is None else subpixel
        u = (np.arange(self.w) + 0.5) / self.w * 2 - 1
        vv = (np.arange(self.h) + 0.5) / self.h * 2 - 1
        U, V = np.meshgrid(u * np.tan(hf / 2), vv * np.tan(vf / 2))
        d = np.stack([np.ones_like(U), -U, -V], -1).reshape(-1, 3)
        self.d_s = d / np.linalg.norm(d, axis=1, keepdims=True)
        self.zmin, self.zmax = v(pr.D455, "min_depth"), v(pr.D455, "max_depth")

    def sigma_z(self, z):
        return z ** 2 * self.sd / (self.f_native * self.B)

    def scan(self, arena, robot, rng, dust=None):
        origin, R = sensor_pose(robot, self.mount, self.pitch)
        dirs = self.d_s @ R.T
        m = march(arena, origin, dirs, self.zmax / self.d_s[:, 0].min(), step=0.02, dust=dust)
        z = m["t_hit"] * self.d_s[:, 0]              # depth along the optical axis
        return self.apply(origin, dirs, z, m, rng)

    def apply(self, origin, dirs, z, m, rng):
        """BP-1 layer on an ideal depth image (z along the optical axis, nan for no hit, row-major h x w):
        range limits, dropouts from dust, and stereo noise sigma_z = z^2 sigma_d / (f B) inflated by 1/T in dust."""
        valid = np.isfinite(z) & (np.nan_to_num(z) >= self.zmin) & (np.nan_to_num(z) <= self.zmax)
        trans = np.exp(-m["tau_hit"])                # one-way transmittance to the surface
        p_inv = v(pr.D455, "invalid_frac_clear") + (1 - trans ** v(pr.DUST, "camera_invalid_gain"))
        valid &= rng.random(len(z)) >= np.clip(p_inv, 0, 1)
        sig = self.sigma_z(np.nan_to_num(z)) / np.maximum(trans, 0.05)
        zn = z + rng.normal(0, 1, len(z)) * sig
        r = zn / self.d_s[:, 0]
        pts = origin + np.nan_to_num(r)[:, None] * dirs
        return {
            "sensor": self.name, "origin": origin, "dirs": dirs, "r": np.where(valid, r, np.nan), "valid": valid,
            "pts": pts, "is_dust": np.zeros(len(z), bool), "r_true_surface": z / self.d_s[:, 0],
            "r_flat": flat_ground_range(origin, dirs), "samples": m["samples"],
            "sigma": np.where(valid, sig, np.nan), "sigma_r": np.where(valid, sig / self.d_s[:, 0], np.nan),
            "depth_img": np.where(valid, zn, np.nan).reshape(self.h, self.w),
        }


def believed_mount(scan, Q):
    """Re-express a scan as the robot sees it when its sensor extrinsics are off: rays, points and free-space samples
    are rotated about the sensor origin by Q (believed-from-true, world frame)."""
    o = scan["origin"]
    out = dict(scan)
    out["dirs"] = scan["dirs"] @ Q.T
    out["pts"] = o + (scan["pts"] - o) @ Q.T
    out["r_flat"] = flat_ground_range(o, out["dirs"])
    sm = dict(scan["samples"])
    P = o + (np.stack([sm["x"], sm["y"], sm["z"]], -1) - o) @ Q.T
    sm["x"], sm["y"], sm["z"] = P[:, 0], P[:, 1], P[:, 2]
    out["samples"] = sm
    return out
