"""The team's robot as an occluder: geometry from the URDF, sensor mounts, and exact ray casting against the body.

Geometry source: sim/robot/uic_bot_2027.xacro, derived from the 2026 robot in educationmoment/EDT-Lunabotics-2026
(src/description_pkg/urdf/uic_bot.xacro, commit a720e46). The collision meshes it names (body.stl and the camera
meshes) come from that repository's src/description_pkg/meshes, so a checkout of it is needed:
  set EDT_ROBOT_DIR to the checkout, or clone it to ~/lunabotics/edt-lunabotics-2026.
Only numpy is needed, so this also runs under Isaac Sim's Python.

Frames: the URDF's base_link (x forward, y left, z up). The sandbox robot frame has the same axes with its origin on
the ground under base_link; base_link sits at the wheel axle height plus the wheel radius above that ground
(0.1 + 0.1651 = 0.2651 m for this URDF). Sensor mounts below are in the sandbox robot frame.

Ray casting is exact (Moller-Trumbore ray-triangle tests in float64). To keep it fast in numpy, rays are binned by
direction and each triangle is tested only against rays inside a cone that contains it (its bounding sphere seen
from the sensor), which can only add candidate pairs, never drop a hit.
"""
import hashlib
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO = os.path.normpath(os.path.join(HERE, "..", "robot", "uic_bot_2027.xacro"))
CACHE = os.path.join(HERE, "robot_cache")
EDT_CANDIDATES = [os.path.expanduser("~/lunabotics/edt-lunabotics-2026"),
                  "/home/claude/educationmoment/edt-lunabotics-2026",
                  os.path.normpath(os.path.join(HERE, "..", "..", "..", "edt-lunabotics-2026"))]


def find_edt_dir():
    cands = ([os.environ["EDT_ROBOT_DIR"]] if os.environ.get("EDT_ROBOT_DIR") else []) + EDT_CANDIDATES
    for c in cands:
        if os.path.isdir(os.path.join(c, "src", "description_pkg")):
            return c
    raise FileNotFoundError("EDT-Lunabotics-2026 checkout not found; set EDT_ROBOT_DIR (looked in: "
                            + ", ".join(cands) + ")")


def load_stl(path):
    """Triangles (n, 3, 3) of a binary or ASCII STL file, in the file's units (metres for these meshes)."""
    with open(path, "rb") as f:
        b = f.read()
    n = struct.unpack("<I", b[80:84])[0] if len(b) >= 84 else 0
    if len(b) == 84 + 50 * n:
        a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        return a["v"].astype(np.float64)
    v = [list(map(float, ln.split()[1:4])) for ln in b.decode(errors="ignore").splitlines()
         if ln.strip().startswith("vertex")]
    return np.array(v, np.float64).reshape(-1, 3, 3)


def rpy_matrix(r, p, y):
    """URDF fixed-axis roll, pitch, yaw: R = Rz(y) Ry(p) Rx(r). Positive pitch turns +x downwards."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def cylinder_tris(radius, length, n=48):
    """Closed cylinder along z, centred on the origin."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([radius * np.cos(a), radius * np.sin(a)], 1)
    top = np.c_[ring, np.full(n, length / 2)]
    bot = np.c_[ring, np.full(n, -length / 2)]
    k = np.arange(n)
    k1 = (k + 1) % n
    side = np.concatenate([np.stack([bot[k], bot[k1], top[k1]], 1), np.stack([bot[k], top[k1], top[k]], 1)])
    ct, cb = np.array([0, 0, length / 2]), np.array([0, 0, -length / 2])
    caps = np.concatenate([np.stack([np.tile(ct, (n, 1)), top[k], top[k1]], 1),
                           np.stack([np.tile(cb, (n, 1)), bot[k1], bot[k]], 1)])
    return np.concatenate([side, caps])


def box_tris(size):
    sx, sy, sz = np.asarray(size, float) / 2
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    f = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return np.array([[c[a], c[b], c[d]] for a, b, cc, d in f] + [[c[b], c[cc], c[d]] for a, b, cc, d in f])


def _origin(el):
    o = el.find("origin") if el is not None else None
    xyz = [float(t) for t in o.get("xyz", "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
    rpy = [float(t) for t in o.get("rpy", "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
    return np.array(xyz), np.array(rpy)


class Robot:
    """Links, joints and collision geometry of the URDF, placed in the sandbox robot frame."""

    def __init__(self, xacro=XACRO, edt_dir=None):
        self.xacro = xacro
        self.edt_dir = edt_dir or find_edt_dir()
        root = ET.parse(xacro).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            xyz, rpy = _origin(j)
            self.joints[j.get("name")] = {"parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                                          "type": j.get("type"), "xyz": xyz, "rpy": rpy}
        self.links = {}
        for ln in root.findall("link"):
            geoms = []
            for col in ln.findall("collision"):
                xyz, rpy = _origin(col)
                g = col.find("geometry")
                mesh, cyl, box = g.find("mesh"), g.find("cylinder"), g.find("box")
                if mesh is not None:
                    fn = mesh.get("filename").replace("file:///", "").replace("file://", "")
                    fn = fn.replace("$(find description_pkg)", os.path.join(self.edt_dir, "src", "description_pkg"))
                    geoms.append({"kind": "mesh", "file": fn, "xyz": xyz, "rpy": rpy})
                elif cyl is not None:
                    geoms.append({"kind": "cylinder", "radius": float(cyl.get("radius")),
                                  "length": float(cyl.get("length")), "xyz": xyz, "rpy": rpy})
                elif box is not None:
                    geoms.append({"kind": "box", "size": [float(t) for t in box.get("size").split()],
                                  "xyz": xyz, "rpy": rpy})
            self.links[ln.get("name")] = geoms
        # link poses in base_link
        self.T = {"base_link": (np.eye(3), np.zeros(3))}
        pending = dict(self.joints)
        while pending:
            done = [n for n, j in pending.items() if j["parent"] in self.T]
            assert done, f"joints with unknown parents: {sorted(pending)}"
            for n in done:
                j = pending.pop(n)
                Rp, tp = self.T[j["parent"]]
                Rj = rpy_matrix(*j["rpy"])
                self.T[j["child"]] = (Rp @ Rj, tp + Rp @ j["xyz"])
        # base_link height above the ground: lowest wheel point
        wheel_bottoms = []
        for name, geoms in self.links.items():
            if "wheel" in name:
                for g in geoms:
                    if g["kind"] == "cylinder":
                        wheel_bottoms.append(self.T[name][1][2] - g["radius"])
        assert wheel_bottoms, "no wheel cylinders found"
        self.base_height = -min(wheel_bottoms)
        self.wheels = sorted(((self.T[n][1] + [0, 0, self.base_height]).tolist(),
                              next(g for g in gs if g["kind"] == "cylinder")["radius"],
                              next(g for g in gs if g["kind"] == "cylinder")["length"])
                             for n, gs in self.links.items() if "wheel" in n)

    def to_robot(self, p):
        return np.asarray(p, float) + np.array([0.0, 0.0, self.base_height])

    def triangles(self, exclude=("lidar3d_link",)):
        """All collision triangles in the robot frame, plus the link name of each triangle."""
        out, names = [], []
        for name, geoms in self.links.items():
            if name in exclude or name not in self.T:
                continue
            Rl, tl = self.T[name]
            for g in geoms:
                if g["kind"] == "mesh":
                    tri = load_stl(g["file"])
                elif g["kind"] == "cylinder":
                    tri = cylinder_tris(g["radius"], g["length"])
                else:
                    tri = box_tris(g["size"])
                Rg = rpy_matrix(*g["rpy"])
                tri = tri @ Rg.T + g["xyz"]
                tri = tri @ Rl.T + tl
                out.append(self.to_robot(tri.reshape(-1, 3)).reshape(-1, 3, 3))
                names += [name] * len(tri)
        return np.concatenate(out), np.array(names)

    def mount(self, link):
        """(xyz in the robot frame, pitch down in degrees) of a sensor link; the sandbox models yaw = roll = 0."""
        j = next(j for j in self.joints.values() if j["child"] == link)
        r, p, y = j["rpy"]
        assert abs(r) < 1e-6 and abs(y) < 1e-6, f"{link}: sandbox sensors are forward-facing with no roll"
        return tuple(self.to_robot(self.T[link][1]).tolist()), float(np.degrees(p))

    def extents(self, exclude=("lidar3d_link",)):
        tri, _ = self.triangles(exclude)
        P = tri.reshape(-1, 3)
        return P.min(0), P.max(0)

    def digest(self):
        h = hashlib.sha1(open(self.xacro, "rb").read())
        for geoms in self.links.values():
            for g in geoms:
                if g["kind"] == "mesh":
                    h.update(str(os.path.getsize(g["file"])).encode())
        return h.hexdigest()[:12]


# ---------------------------------------------------------------- exact ray casting

def cast(origin, dirs, tris, R=None, bin_deg=2.0, chunk=4_000_000):
    """First hit of each ray from one origin against triangles.

    origin (3,), dirs (n, 3) unit vectors, tris (m, 3, 3), all in one frame. R (3, 3), optional, rotates sensor-frame
    vectors into that frame; binning then happens in the sensor frame, away from the poles. Returns the range t (nan
    for no hit) and the index of the triangle hit (-1 for none).
    """
    origin = np.asarray(origin, float)
    D = np.asarray(dirs, float)
    V = tris - origin
    if R is not None:
        D = D @ R
        V = V @ R
    n = len(D)
    t_best = np.full(n, np.inf)
    i_best = np.full(n, -1)
    b = np.radians(bin_deg)
    n_az, n_el = int(np.ceil(2 * np.pi / b)), int(np.ceil(np.pi / b))
    # rays into direction bins
    el_r = np.arcsin(np.clip(D[:, 2], -1, 1))
    az_r = np.arctan2(D[:, 1], D[:, 0])
    rb = np.clip(((el_r + np.pi / 2) / b).astype(int), 0, n_el - 1) * n_az + ((az_r + np.pi) / b).astype(int) % n_az
    # triangles into every bin their bounding cone touches
    c = V.mean(1)
    rho = np.linalg.norm(V - c[:, None], axis=2).max(1)
    dist = np.linalg.norm(c, axis=1)
    alpha = np.where(dist > rho, np.arcsin(np.clip(rho / np.maximum(dist, 1e-12), 0, 1)), np.pi) + 1e-9
    el_c = np.arcsin(np.clip(c[:, 2] / np.maximum(dist, 1e-12), -1, 1))
    az_c = np.arctan2(c[:, 1], c[:, 0])
    lo, hi = el_c - alpha, el_c + alpha
    e0 = np.clip(((lo + np.pi / 2) / b).astype(int), 0, n_el - 1)
    e1 = np.clip(((hi + np.pi / 2) / b).astype(int), 0, n_el - 1)
    polar = (lo <= -np.pi / 2 + b) | (hi >= np.pi / 2 - b)
    # azimuth half-width of the cone: asin(sin(alpha) / cos(el)) at the cone's largest |elevation| bounds it
    cosmax = np.cos(np.minimum(np.maximum(np.abs(lo), np.abs(hi)), np.pi / 2 - 1e-6))
    ratio = np.sin(np.minimum(alpha, np.pi / 2)) / cosmax
    half = np.where(polar | (alpha >= np.pi / 2) | (ratio >= 1), np.pi, np.arcsin(np.clip(ratio, 0, 1)))
    a0 = np.floor((az_c - half + np.pi) / b).astype(int)
    a1 = np.floor((az_c + half + np.pi) / b).astype(int)
    full = (a1 - a0 + 1) >= n_az
    a0 = np.where(full, 0, a0)
    a1 = np.where(full, n_az - 1, a1)
    occupied = np.zeros(n_az * n_el, bool)
    occupied[rb] = True
    ne, na = e1 - e0 + 1, a1 - a0 + 1
    cnt = ne * na
    tri_id = np.repeat(np.arange(len(V)), cnt)
    k = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
    eb = np.repeat(e0, cnt) + k // np.repeat(na, cnt)
    ab = (np.repeat(a0, cnt) + k % np.repeat(na, cnt)) % n_az
    pb = eb * n_az + ab
    keep = occupied[pb]
    pb, tri_id = pb[keep], tri_id[keep]
    order = np.argsort(pb, kind="stable")
    pb, tri_id = pb[order], tri_id[order]
    ray_order = np.argsort(rb, kind="stable")
    rbs = rb[ray_order]
    bins = np.unique(rbs)
    r_start, r_end = np.searchsorted(rbs, bins, "left"), np.searchsorted(rbs, bins, "right")
    t_start, t_end = np.searchsorted(pb, bins, "left"), np.searchsorted(pb, bins, "right")
    v0, e1v, e2v = V[:, 0], V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]
    for k_, _ in enumerate(bins):
        if t_end[k_] == t_start[k_]:
            continue
        rays = ray_order[r_start[k_]:r_end[k_]]
        tids = tri_id[t_start[k_]:t_end[k_]]
        step = max(1, chunk // max(len(rays), 1))
        for s in range(0, len(tids), step):
            tt = tids[s:s + step]
            d = D[rays][:, None, :]
            p = np.cross(d, e2v[tt][None])
            det = (e1v[tt][None] * p).sum(-1)
            ok = np.abs(det) > 1e-12
            inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
            sv = -v0[tt][None]
            u = (sv * p).sum(-1) * inv
            q = np.cross(sv, e1v[tt][None])
            vv = (d * q).sum(-1) * inv
            t = (e2v[tt][None] * q).sum(-1) * inv
            hit = ok & (u >= 0) & (vv >= 0) & (u + vv <= 1) & (t > 1e-9)
            t = np.where(hit, t, np.inf)
            j = t.argmin(1)
            tm = t[np.arange(len(rays)), j]
            better = tm < t_best[rays]
            t_best[rays[better]] = tm[better]
            i_best[rays[better]] = tt[j[better]]
    return np.where(np.isfinite(t_best), t_best, np.nan), i_best


def tri_normals(tris):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def perturbed(d_s, deg):
    """Sensor-frame directions turned by +-deg in azimuth and elevation (8 neighbours), shape (8, n, 3)."""
    out = []
    a = np.radians(deg)
    for da, de in ((a, 0), (-a, 0), (0, a), (0, -a), (a, a), (a, -a), (-a, a), (-a, -a)):
        el = np.arcsin(np.clip(d_s[:, 2], -1, 1)) + de
        az = np.arctan2(d_s[:, 1], d_s[:, 0]) + da
        out.append(np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1))
    return np.stack(out)


def body_table(robot, name, d_s, mount_xyz, pitch_down, inflate_deg=0.0):
    """Per-ray range to the body for a sensor with fixed sensor-frame directions d_s, at a mount in the robot frame.

    Returns {"t": range or nan, "cos": |cos| of incidence, "tri": triangle index, "link": link names}. With
    inflate_deg > 0, "t" is the nearest body range among the ray and its 8 neighbours +-inflate_deg away: the table
    a self-mask uses to cover calibration error. Cached in robot_cache/ by geometry, mount and directions.
    """
    from sensors import rot   # local import: sensors imports params, which robot.py itself does not need
    key = hashlib.sha1(np.ascontiguousarray(d_s, np.float64).tobytes()
                       + repr((robot.digest(), tuple(np.round(mount_xyz, 6)), round(pitch_down, 6),
                               round(inflate_deg, 6))).encode()).hexdigest()[:16]
    path = os.path.join(CACHE, f"{name}_{key}.npz")
    if os.path.exists(path):
        z = np.load(path, allow_pickle=False)
        return {k: z[k] for k in z.files}
    tris, links = robot.triangles()
    R = rot(0.0, pitch_down)
    origin = np.asarray(mount_xyz, float)
    t, tri = cast(origin, d_s @ R.T, tris, R=R)
    if inflate_deg > 0:
        for dp in perturbed(d_s, inflate_deg):
            tp, _ = cast(origin, dp @ R.T, tris, R=R)
            t = np.fmin(t, tp)
    hit = tri >= 0
    cosi = np.full(len(d_s), np.nan)
    cosi[hit] = np.abs((tri_normals(tris[tri[hit]]) * (d_s[hit] @ R.T)).sum(1))
    out = {"t": t, "cos": cosi, "tri": tri, "link": np.where(hit, links[np.maximum(tri, 0)], "")}
    os.makedirs(CACHE, exist_ok=True)
    np.savez_compressed(path, **out)
    return out


if __name__ == "__main__":
    rb = Robot()
    lo, hi = rb.extents()
    print(f"robot from {rb.xacro}\n  meshes from {rb.edt_dir}\n  base_link {rb.base_height:.4f} m above ground")
    print(f"  body extents (robot frame): x {lo[0]:+.3f}..{hi[0]:+.3f}  y {lo[1]:+.3f}..{hi[1]:+.3f}  "
          f"z {lo[2]:+.3f}..{hi[2]:+.3f}")
    for link in ("lidar3d_link", "d455_link"):
        print(f"  {link}: mount {np.round(rb.mount(link)[0], 4)}  pitch down {rb.mount(link)[1]:.2f} deg")
    print("  wheels:", [(np.round(c, 3).tolist(), r, w) for c, r, w in rb.wheels])
