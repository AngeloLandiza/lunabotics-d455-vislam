"""Self-mask: keep the robot's own body out of the map.

A LiDAR on top of the robot sees parts of the robot. Those returns sit 0.1 to 0.6 m above the ground inside the
robot's footprint, so without a mask they become boulders under and behind the robot. Two filters, both run on the
scan as the robot sees it (nominal extrinsics):

- table: a body range per direction in the sensor frame, computed once from the URDF (robot.BodyCaster) on a 0.25 deg
  grid and widened by inflate_deg (the farthest body range within that angle) to cover calibration error and the
  grid step. A return is dropped when the direction has a body range and the return is no farther than it plus margin
  (3 sigma of the range noise); directions with no body within inflate_deg are never dropped. This is the per-beam body depth design from
  research/step1/01-lidar-occlusion.md (Inferred there); robot_body_filter (ROS 1) does the same per point with padded
  collision shapes.
- box: drop every return inside the robot's bounding box grown by box_margin, the crop box many teams use.

A dropped ray gives neither a point nor free space, so the ground the body hides stays unknown instead of clear.
"""
import numpy as np
from scipy import ndimage

import params as pr
from params import v

BIN_DEG = 0.25


class MaskTable:
    def __init__(self, body, mount_xyz, R_mount, inflate_deg=None, bin_deg=BIN_DEG):
        self.b = np.radians(bin_deg)
        self.n_az, self.n_el = int(round(360 / bin_deg)), int(round(180 / bin_deg))
        az = -np.pi + (np.arange(self.n_az) + 0.5) * self.b
        el = -np.pi / 2 + (np.arange(self.n_el) + 0.5) * self.b
        E, A = np.meshgrid(el, az, indexing="ij")
        d_s = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
        t, _ = body.table("masktable", d_s, mount_xyz, R_mount)
        T = np.where(np.isfinite(t), t, -np.inf).reshape(self.n_el, self.n_az)     # -inf: no body
        self.inflate = v(pr.ROBOT, "mask_inflate") if inflate_deg is None else inflate_deg
        if self.inflate > 0:
            d = np.radians(self.inflate)
            out = np.empty_like(T)
            for i, e in enumerate(el):            # azimuth reach of a cap of radius d at this elevation
                c = np.cos(min(abs(e) + d, np.pi / 2 - 1e-6))
                half = np.pi if np.sin(d) >= c else np.arcsin(np.sin(d) / c)
                w = min(int(np.ceil(half / self.b)), self.n_az // 2)
                out[i] = ndimage.maximum_filter1d(T[i], 2 * w + 1, mode="wrap")
            T = ndimage.maximum_filter1d(out, 2 * int(np.ceil(d / self.b)) + 1, axis=0, mode="nearest")
        self.T = np.where(np.isfinite(T), T, np.nan)                              # nan: no body, never dropped

    def lookup(self, d_s):
        el = np.arcsin(np.clip(d_s[:, 2], -1, 1))
        az = np.arctan2(d_s[:, 1], d_s[:, 0])
        i = np.clip(((el + np.pi / 2) / self.b).astype(int), 0, self.n_el - 1)
        j = np.clip(((az + np.pi) / self.b).astype(int), 0, self.n_az - 1)
        return self.T[i, j]


def box_of(robot, margin=None):
    lo, hi = robot.extents()
    m = v(pr.ROBOT, "box_margin") if margin is None else margin
    return lo - m, hi + m


def apply(scan, robot_pose, table=None, box=None, margin=None):
    """Drop body returns from a scan. Returns a masked copy of the scan and the dropped-ray flags."""
    margin = v(pr.ROBOT, "mask_margin") if margin is None else margin
    r = scan["r"]
    drop = np.zeros(len(r), bool)
    if table is not None:
        tb = table.lookup(scan["dirs_s"])
        drop |= np.isfinite(r) & np.isfinite(tb) & (r <= np.nan_to_num(tb) + margin)
    if box is not None:
        x, y, yaw = robot_pose
        a = np.radians(yaw)
        P = scan["pts"] - [x, y, 0.0]
        px, py = np.cos(a) * P[:, 0] + np.sin(a) * P[:, 1], -np.sin(a) * P[:, 0] + np.cos(a) * P[:, 1]
        lo, hi = box
        inside = (px > lo[0]) & (px < hi[0]) & (py > lo[1]) & (py < hi[1]) & (P[:, 2] > lo[2]) & (P[:, 2] < hi[2])
        drop |= np.isfinite(r) & inside
    if not drop.any():
        return scan, drop
    out = dict(scan)
    out["r"] = np.where(drop, np.nan, r)
    out["valid"] = scan["valid"] & ~drop
    out["is_body"] = scan["is_body"] & ~drop
    return out, drop
