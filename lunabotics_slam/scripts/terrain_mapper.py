#!/usr/bin/env python3
"""Semi-dynamic 2.5D terrain (elevation) map for Lunabotics digging.

Each cell holds a height estimate, its variance and a baseline height. Per point
cloud (in the map frame, from the SLAM pose):
  * Height-drift compensation: the scan's vertical offset against the map (median
    over overlapping cells, real changes excluded) is removed first, so SLAM height
    drift isn't mistaken for digging (elevation_mapping_cupy's drift compensation).
  * Kalman fusion of each cell's mean measured height with the stored height,
    measurement noise growing with range^2 (Fankhauser et al. 2018, elevation_mapping).
  * Change detection: a measurement more than `change_threshold` sigmas from the
    stored height means the terrain changed (dug or dumped) -> the cell is reset to
    the new measurement instead of being averaged, once two consecutive scans agree.
  * Visibility cleanup: cells the sensor now sees *through* (stored height above a
    ray to a farther point, e.g. the old surface over a fresh hole) are cleared
    (elevation_mapping_cupy).
  * Unobserved cells slowly gain variance, so the map keeps adapting.
Change = height - baseline gives dug (cut) and dumped (fill) volume, e.g. berm volume.
The baseline follows the fused height until a cell's first detected change, then
freezes, so sensor noise alone never shows up as dug or dumped volume.

Topics: /terrain/cloud (PointCloud2: elevation, change, variance per cell),
        /terrain/elevation (Image 32FC1, NaN = unknown), /terrain/volume ([cut, fill] m^3)
Services: /terrain/reset_baseline, /terrain/clear (std_srvs/Trigger)
"""
import math
import os
import time

import numpy as np


class TerrainGrid:
    def __init__(self, resolution, size_x, size_y, origin_x, origin_y,
                 change_threshold=3.0, time_variance=1e-5, max_variance=0.05,
                 cleanup_margin=0.05, drift_window=0.15, change_confirmations=2, max_drift_step=0.05,
                 baseline_sigma=0.012, max_tilt_deg=4.0):
        self.res = resolution
        self.nx, self.ny = int(round(size_x / resolution)), int(round(size_y / resolution))
        self.x0, self.y0 = origin_x - size_x / 2, origin_y - size_y / 2  # lower corner
        self.change_threshold = change_threshold
        self.time_variance = time_variance
        self.max_variance = max_variance
        self.cleanup_margin = cleanup_margin
        self.drift_window = drift_window
        self.change_confirmations = change_confirmations
        self.max_drift_step = max_drift_step
        self.drift = np.zeros(3)             # scan-vs-map correction: offset, x slope, y slope
        self.baseline_var = baseline_sigma ** 2
        self.max_tilt = math.tan(math.radians(max_tilt_deg))
        self.clear()
        self.baseline = np.full((self.nx, self.ny), np.nan, np.float32)
        self.frozen = np.zeros((self.nx, self.ny), bool)  # baseline fixed after a change

    def clear(self):
        self.height = np.full((self.nx, self.ny), np.nan, np.float32)
        self.variance = np.full((self.nx, self.ny), np.nan, np.float32)
        self.disagree = np.zeros((self.nx, self.ny), np.int8)

    def cells(self, x, y):
        ix = np.floor((x - self.x0) / self.res).astype(np.int64)
        iy = np.floor((y - self.y0) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def age(self, dt):
        known = ~np.isnan(self.variance)
        self.variance[known] = np.minimum(self.variance[known] + self.time_variance * dt,
                                          self.max_variance)

    def update(self, pts, sigma2, origin, n_rays=1500):
        """pts: (N,3) map-frame points; sigma2: (N,) height variance; origin: sensor xyz."""
        ix, iy, ok = self.cells(pts[:, 0], pts[:, 1])
        pts, sigma2, ix, iy = pts[ok], sigma2[ok], ix[ok], iy[ok]
        if len(pts) == 0:
            return 0
        origin_cell = ((origin[0] - self.x0) / self.res - 0.5, (origin[1] - self.y0) / self.res - 0.5)
        self._visibility_cleanup(pts, origin, n_rays)

        # Per-cell mean height and mean measurement variance this frame.
        flat = ix * self.ny + iy
        n = np.bincount(flat, minlength=self.nx * self.ny)
        hit = n > 0
        z = (np.bincount(flat, pts[:, 2], self.nx * self.ny)[hit] / n[hit]).astype(np.float32)
        s2 = (np.bincount(flat, sigma2, self.nx * self.ny)[hit] / n[hit]).astype(np.float32)
        idx = np.flatnonzero(hit)
        h, v = self.height.ravel(), self.variance.ravel()

        new = np.isnan(h[idx])
        cx = (idx // self.ny - origin_cell[0]) * self.res     # cell offsets from the sensor (m)
        cy = (idx % self.ny - origin_cell[1]) * self.res
        if (~new).sum() >= 30:  # remove this scan's height + tilt error against the map
            d = z[~new] - h[idx[~new]]
            keep = np.abs(d - np.median(d)) < self.drift_window      # robust: digs cover few cells
            A = np.c_[np.ones(len(d)), cx[~new], cy[~new]]
            fit = None
            for _ in range(2):                                         # fit, trim, refit
                if keep.sum() < 30:
                    break
                fit = np.linalg.lstsq(A[keep], d[keep], rcond=None)[0]
                keep &= np.abs(d - A @ fit) < 0.5 * self.drift_window
            # SLAM drift (height, and ~1 deg of tilt) changes slowly; a sudden jump (bad
            # pose / glitch) must not be "corrected" toward the stored map, or one bad burst
            # would lock in forever. Change detection re-maps those cells instead.
            if fit is not None and abs(fit[0] - self.drift[0]) <= self.max_drift_step \
                    and np.all(np.abs(fit[1:]) <= self.max_tilt):
                self.drift = fit
                z = z - (fit[0] + fit[1] * cx + fit[2] * cy)
        d2 = (z - h[idx]) ** 2 / (v[idx] + s2)
        outlier = ~new & (d2 > self.change_threshold ** 2)
        count = self.disagree.ravel()
        count[idx[outlier]] += 1
        count[idx[~outlier]] = 0
        changed = outlier & (count[idx] >= self.change_confirmations)
        count[idx[changed]] = 0
        fuse = ~new & ~outlier            # unconfirmed outliers are simply not fused
        reset = new | changed

        h[idx[reset]], v[idx[reset]] = z[reset], s2[reset]
        f, vf, sf = idx[fuse], v[idx[fuse]], s2[fuse]
        h[f] = (sf * h[f] + vf * z[fuse]) / (vf + sf)
        v[f] = vf * sf / (vf + sf)

        # Baseline tracks the estimate until the cell is well observed (then it freezes,
        # so later gradual digging still counts as volume) or first changes.
        b, frozen = self.baseline.ravel(), self.frozen.ravel()
        frozen[idx[changed & ~np.isnan(b[idx])]] = True
        track = idx[~frozen[idx]]
        b[track] = h[track]
        frozen[track[v[track] < self.baseline_var]] = True
        return int(hit.sum())

    def _visibility_cleanup(self, pts, origin, n_rays):
        if len(pts) > n_rays:
            pts = pts[np.random.default_rng().choice(len(pts), n_rays, replace=False)]
        vec = pts - origin
        length = np.linalg.norm(vec[:, :2], axis=1)
        steps = np.maximum((length / self.res).astype(np.int64), 1)
        # Sample each ray every cell, stopping two cells short of its end point.
        t = np.arange(steps.max(), dtype=np.float32)
        ray, k = np.nonzero(t[None, :] < (steps - 2)[:, None])
        if len(ray) == 0:
            return
        frac = (k / steps[ray]).astype(np.float32)[:, None]
        sample = origin + frac * vec[ray]
        ix, iy, ok = self.cells(sample[:, 0], sample[:, 1])
        ix, iy, sample = ix[ok], iy[ok], sample[ok]
        stored = self.height[ix, iy]
        # Ray passed clearly below the stored surface (margin grows with the cell's uncertainty).
        margin = np.maximum(self.cleanup_margin, 3 * np.sqrt(np.nan_to_num(self.variance[ix, iy])))
        through = stored > sample[:, 2] + margin
        self.height[ix[through], iy[through]] = np.nan
        self.variance[ix[through], iy[through]] = np.nan
        # Seeing through a surface means it changed: keep its baseline so the
        # re-measured (e.g. dug) height counts as volume instead of a new baseline.
        had_baseline = ~np.isnan(self.baseline[ix[through], iy[through]])
        self.frozen[ix[through][had_baseline], iy[through][had_baseline]] = True

    def volumes(self, min_change):
        known = ~np.isnan(self.height) & ~np.isnan(self.baseline)
        change = np.where(known, self.height - self.baseline, 0.0)
        noise = np.where(known, 2 * np.sqrt(np.nan_to_num(self.variance)), 0.0)
        significant = np.abs(change) > np.maximum(min_change, noise)
        area = self.res ** 2
        cut = float(-change[significant & (change < 0)].sum() * area)
        fill = float(change[significant & (change > 0)].sum() * area)
        return cut, fill

    def save(self, path):
        np.savez_compressed(path, height=self.height, variance=self.variance,
                            baseline=self.baseline, frozen=self.frozen,
                            res=self.res, x0=self.x0, y0=self.y0)

    def load(self, path):
        d = np.load(path)
        if d['height'].shape != self.height.shape or float(d['res']) != self.res:
            raise ValueError('saved terrain has a different grid size/resolution')
        self.height, self.variance, self.baseline = d['height'], d['variance'], d['baseline']
        self.frozen = d['frozen']


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    import rclpy
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class TerrainMapper(Node):
        def __init__(self):
            super().__init__('terrain_mapper')
            p = lambda name, default: self.declare_parameter(name, default).value
            self.frame = p('map_frame', 'map')
            self.min_range, self.max_range = p('min_range', 0.3), p('max_range', 3.5)
            self.sigma_a, self.sigma_b = p('sigma_a', 0.005), p('sigma_b', 0.005)
            self.min_change = p('min_change', 0.03)
            self.save_path = os.path.expanduser(p('save_path', '~/.ros/lunabotics_terrain.npz'))
            self.grid = TerrainGrid(p('resolution', 0.05), p('size_x', 10.0), p('size_y', 10.0),
                                    p('origin_x', 0.0), p('origin_y', 0.0),
                                    p('change_threshold', 3.0), p('time_variance', 1e-5),
                                    p('max_variance', 0.05), p('cleanup_margin', 0.05),
                                    p('drift_window', 0.15), p('change_confirmations', 2),
                                    p('max_drift_step', 0.05), p('baseline_sigma', 0.012),
                                    p('max_tilt_deg', 4.0))
            if p('load', False) and os.path.exists(self.save_path):
                self.grid.load(self.save_path)
                self.get_logger().info(f'loaded terrain baseline from {self.save_path}')

            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.latest = None
            self.last_update = None
            self.create_subscription(PointCloud2, p('cloud_topic', '/lidar/points'),
                                     lambda m: setattr(self, 'latest', m), 2)
            self.cloud_pub = self.create_publisher(PointCloud2, '/terrain/cloud', 1)
            self.image_pub = self.create_publisher(Image, '/terrain/elevation', 1)
            self.volume_pub = self.create_publisher(Float64MultiArray, '/terrain/volume', 1)
            self.create_service(Trigger, '/terrain/reset_baseline', self.reset_baseline)
            self.create_service(Trigger, '/terrain/clear', self.clear)
            self.create_timer(1.0 / p('update_rate', 5.0), self.tick)
            self.create_timer(5.0, self.log_volume)
            self.PointField, self.PointCloud2, self.Image = PointField, PointCloud2, Image
            self.Float64MultiArray, self.Time, self.Duration = Float64MultiArray, Time, Duration

        def reset_baseline(self, _req, res):
            self.grid.baseline = self.grid.height.copy()
            self.grid.frozen[:] = False
            res.success, res.message = True, 'baseline = current terrain'
            return res

        def clear(self, _req, res):
            self.grid.clear()
            res.success, res.message = True, 'terrain cleared (baseline kept)'
            return res

        def tick(self):
            msg, self.latest = self.latest, None
            if msg is None:
                return
            try:
                tf = self.tf.lookup_transform(self.frame, msg.header.frame_id,
                                              self.Time.from_msg(msg.header.stamp),
                                              timeout=self.Duration(seconds=0.1))
            except Exception as e:  # no SLAM pose yet
                self.get_logger().warn(f'no {self.frame} pose for cloud: {e}',
                                       throttle_duration_sec=5.0)
                return
            off = {f.name: f.offset // 4 for f in msg.fields}
            raw = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
            pts = raw[:, [off['x'], off['y'], off['z']]]
            pts = pts[np.isfinite(pts).all(axis=1)]
            r = np.linalg.norm(pts, axis=1)
            keep = (r > self.min_range) & (r < self.max_range)
            pts, r = pts[keep], r[keep]
            R = quat_to_matrix(tf.transform.rotation)
            t = tf.transform.translation
            origin = np.array([t.x, t.y, t.z], np.float32)
            world = (pts @ R.T + origin).astype(np.float32)
            sigma2 = (self.sigma_a + self.sigma_b * r ** 2) ** 2

            now = time.monotonic()
            if self.last_update is not None:
                self.grid.age(now - self.last_update)
            self.last_update = now
            self.grid.update(world, sigma2, origin)
            self.publish(msg.header.stamp)

        def publish(self, stamp):
            g = self.grid
            known = ~np.isnan(g.height)
            ix, iy = np.nonzero(known)
            h = g.height[known]
            change = h - np.nan_to_num(g.baseline[known], nan=0.0)
            cloud = np.stack([g.x0 + (ix + 0.5) * g.res, g.y0 + (iy + 0.5) * g.res, h,
                              h, change, g.variance[known]], axis=1).astype(np.float32)
            msg = self.PointCloud2()
            msg.header.frame_id, msg.header.stamp = self.frame, stamp
            msg.height, msg.width = 1, len(cloud)
            msg.fields = [self.PointField(name=n, offset=4 * i, datatype=self.PointField.FLOAT32,
                                          count=1)
                          for i, n in enumerate(['x', 'y', 'z', 'elevation', 'change', 'variance'])]
            msg.point_step, msg.row_step = 24, 24 * len(cloud)
            msg.is_dense, msg.data = True, cloud.tobytes()
            self.cloud_pub.publish(msg)

            img = self.Image()
            img.header = msg.header
            img.height, img.width = g.nx, g.ny  # row = x index, column = y index, from x0/y0
            img.encoding, img.step = '32FC1', 4 * g.ny
            img.data = g.height.astype(np.float32).tobytes()
            self.image_pub.publish(img)

        def log_volume(self):
            cut, fill = self.grid.volumes(self.min_change)
            self.volume_pub.publish(self.Float64MultiArray(data=[cut, fill]))
            known = int((~np.isnan(self.grid.height)).sum())
            self.get_logger().info(f'terrain: {known} cells known, dug {cut:.3f} m^3, '
                                   f'dumped {fill:.3f} m^3')

    rclpy.init()
    node = TerrainMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C / ros2 launch stop
        pass
    finally:
        node.grid.save(node.save_path)
        node.get_logger().info(f'terrain saved to {node.save_path}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
