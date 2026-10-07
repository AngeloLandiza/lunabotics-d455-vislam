#!/usr/bin/env python3
"""Boulder / crater / wall / pillar segmentation for drivers and planners.

Height relative to the *local* ground (a wide median of the terrain map, so slopes
and the bed's unevenness cancel) is thresholded into bumps (> boulder_height) and
pits (< -crater_depth). Those only seed a detection: each one is grown out to its edge
through connected cells past a shallow threshold (boulder_edge / crater_edge, hysteresis
like Canny edges) and unseen cells inside are filled, so a bowl-shaped crater is sized
rim to rim and not just by its deep middle. Connected blobs are sized and labelled:

  boulder    bump 12-90 cm across          crater     natural pit 15-110 cm across
  dug pit    pit the robot dug (terrain mapper's change < 0 since the baseline)
  pile       bump the robot dumped (change > 0), e.g. the berm
  structure  long or large bump: ducts along the walls, the column box (not drawn; see below)

Walls and the pillar (the central column box) are vertical, which the 2.5D terrain map
averages into mush. They come from RTAB-Map's 3D map instead: the highest point per cell
above the same local ground, then connected blobs taller than wall_height (OpenCV
components + minAreaRect):

  wall       blob longer than wall_length: bin walls and the ducts at their base
  pillar     compact blob (pillar_size) higher than pillar_height (boulders are < 0.3 m)

Input:  /terrain/cloud (terrain_mapper.py: x, y, elevation, change per 5 cm cell)
        /cloud_map     (RTAB-Map's 3D point map, same map frame)
Output: /terrain/obstacles      visualization_msgs/MarkerArray: each obstacle's segmented
                                cells, outline and label (pillar: also its fitted box)
        /terrain/obstacle_grid  nav_msgs/OccupancyGrid (100 = boulder/crater/structure/
                                wall/pillar, 0 = known free, -1 = unknown), for path planning
"""
import math

import cv2
import numpy as np
from scipy import ndimage

KINDS = {  # rgba
    'boulder': (1.0, 0.45, 0.1, 0.9), 'crater': (0.2, 0.5, 1.0, 0.9), 'dug pit': (0.1, 0.9, 0.9, 0.8),
    'pile': (1.0, 0.9, 0.2, 0.8), 'structure': (0.6, 0.6, 0.6, 0.6),
    'wall': (0.85, 0.85, 0.9, 0.7), 'pillar': (0.9, 0.2, 0.9, 0.85),
}


def local_ground(H, res, window=1.0, outlier=0.06):
    """Ground height under every cell of H (NaN = unknown), ignoring bumps, pits and walls.

    Pass 1: a low percentile follows the regolith even where ducts or the column fill
    most of the window. Pass 2: cells further than outlier from it are replaced by the
    nearest regolith and the median is taken, so rims and walls don't bias the ground.
    Returns (ground, H with unknown cells filled from the nearest known one).
    """
    known = ~np.isnan(H)
    idx = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
    filled = H[tuple(idx)]
    k = max(int(window / res) | 1, 3)
    ground = ndimage.percentile_filter(filled, 30, size=k, mode='nearest')
    out = np.abs(filled - ground) > outlier
    if out.any() and (~out).any():
        idx = ndimage.distance_transform_edt(out, return_distances=False, return_indices=True)
        ground = ndimage.median_filter(filled[tuple(idx)], size=k, mode='nearest')
    return ground, filled


def segment(H, change, res, ground_window=1.0, boulder_height=0.08, crater_depth=0.06,
            boulder_edge=0.03, crater_edge=0.015, max_grow=0.15, changed=0.04, boulder_size=(0.12, 0.9),
            crater_size=(0.15, 1.1), boulder_max_height=0.6):
    """H, change: (nx, ny) grids (NaN = unknown). Returns (blob dicts, obstacle mask, ground).

    Blob: kind, i, j (centroid cell), diameter (m), height (m, + bump / - pit), cells.
    """
    known = ~np.isnan(H)
    if known.sum() < 20:
        return [], np.zeros(H.shape, bool), None
    ground, filled = local_ground(H, res, ground_window, min(boulder_height, crater_depth))
    near = ndimage.binary_dilation(known, iterations=1)            # bridge single unknown cells
    rel = np.where(near, filled - ground, 0.0)
    chg = np.nan_to_num(change, nan=0.0)
    kernel = np.ones((3, 3), np.uint8)
    blobs, mask = [], np.zeros(H.shape, bool)
    for sign, thresh, edge in ((1, boulder_height, boulder_edge), (-1, crater_depth, crater_edge)):
        seed = cv2.morphologyEx((sign * rel > thresh).astype(np.uint8), cv2.MORPH_OPEN, kernel)  # drop noise
        # Grow each seed to its edge: keep the shallow-threshold regions that contain a seed,
        # at most max_grow beyond it (noise can't spread an outline along a trench).
        _, lab = cv2.connectedComponents(((sign * rel > edge) | (seed > 0)).astype(np.uint8), connectivity=8)
        reach = cv2.dilate(seed, kernel, iterations=max(int(round(max_grow / res)), 1))
        m = np.isin(lab, np.unique(lab[seed > 0])) & (lab > 0) & (reach > 0)
        m = ndimage.binary_fill_holes(m).astype(np.uint8)       # unseen floor behind a crater's lip
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m, connectivity=8)
        for b in range(1, n):
            cells = lab == b
            area = stats[b, cv2.CC_STAT_AREA] * res * res
            diam = 2 * math.sqrt(area / math.pi)
            extent = max(stats[b, cv2.CC_STAT_WIDTH], stats[b, cv2.CC_STAT_HEIGHT]) * res
            peak = float((sign * rel[cells]).max()) * sign
            dc = float(np.median(chg[cells]))
            if sign > 0:
                if dc > changed:
                    kind = 'pile'
                elif extent > boulder_size[1] or extent > 2.5 * diam or peak > boulder_max_height:
                    kind = 'structure'
                elif diam >= boulder_size[0]:
                    kind = 'boulder'
                else:
                    continue
            else:
                if dc < -changed:
                    kind = 'dug pit'
                elif crater_size[0] <= diam <= crater_size[1] and extent < 2.5 * diam:
                    kind = 'crater'
                else:
                    continue
            # cv2 centroid is (col, row) = (j, i)
            blobs.append({'kind': kind, 'i': float(cent[b][1]), 'j': float(cent[b][0]), 'diameter': diam,
                          'height': peak, 'cells': cells})
            if kind in ('boulder', 'crater', 'structure'):
                mask |= cells
    return blobs, mask, ground


def structures(points, x0, y0, shape, res, ground_window=1.0, wall_height=0.12, wall_length=1.0,
               pillar_height=0.35, pillar_size=(0.2, 0.9), min_points=2):
    """Walls and the pillar from 3D map points (N x 3, map frame) on the grid (x0, y0, shape).

    Ground comes from the same points (lowest point per cell, local_ground), so drift
    between this map and the terrain map doesn't matter. A cell is tall if min_points
    points are > wall_height above the ground. Returns blob dicts like segment(); a
    pillar also has 'rect', its cv2.minAreaRect in cells.
    """
    i = np.floor((points[:, 0] - x0) / res).astype(int)
    j = np.floor((points[:, 1] - y0) / res).astype(int)
    ok = (i >= 0) & (j >= 0) & (i < shape[0]) & (j < shape[1])
    i, j, z = i[ok], j[ok], points[ok, 2]
    if len(z) < 20:
        return [], None
    low = np.full(shape, np.nan, np.float32)
    np.fmin.at(low, (i, j), z)
    ground, _ = local_ground(low, res, ground_window, wall_height / 2)
    h = z - ground[i, j]
    tall = np.zeros(shape, np.int32)
    np.add.at(tall, (i[h > wall_height], j[h > wall_height]), 1)
    rel = np.full(shape, 0.0, np.float32)
    np.maximum.at(rel, (i, j), h)
    m = cv2.morphologyEx((tall >= min_points).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab = cv2.connectedComponents(m, connectivity=8)

    def blob(cells):
        pts = np.argwhere(cells).astype(np.float32)            # (i, j)
        rect = cv2.minAreaRect(pts)
        return {'i': float(pts[:, 0].mean()), 'j': float(pts[:, 1].mean()), 'diameter': (max(rect[1]) + 1) * res,
                'height': float(np.percentile(rel[cells], 95)), 'cells': cells, 'rect': rect}

    blobs = []
    for b in range(1, n):
        o = blob(lab == b)
        if o['diameter'] > wall_length:
            o['kind'] = 'wall'
        elif o['height'] > pillar_height and pillar_size[0] <= o['diameter'] <= pillar_size[1]:
            # The pillar's faces can show up as separate pieces: merge pieces < one pillar apart.
            near = [q for q in blobs if q['kind'] == 'pillar'
                    and math.hypot(q['i'] - o['i'], q['j'] - o['j']) * res < pillar_size[1] * 0.7]
            if near:
                blobs.remove(near[0])
                o = blob(near[0]['cells'] | o['cells'])
            o['kind'] = 'pillar'
        else:
            continue                                             # boulders etc.: segment() has them
        blobs.append(o)
    return blobs, ground


def main():
    import rclpy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray

    def xyz(msg, *names):
        off = {f.name: f.offset // 4 for f in msg.fields}
        a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
        return [a[:, off[n]] for n in names]

    class Segmenter(Node):
        def __init__(self):
            super().__init__('obstacle_segmenter')
            p = lambda n, d: self.declare_parameter(n, d).value
            self.res = p('resolution', 0.05)
            self.kw = dict(ground_window=p('ground_window', 1.0), boulder_height=p('boulder_height', 0.08),
                           crater_depth=p('crater_depth', 0.06), boulder_edge=p('boulder_edge', 0.03),
                           crater_edge=p('crater_edge', 0.015), max_grow=p('max_grow', 0.15),
                           changed=p('changed', 0.04))
            self.skw = dict(ground_window=self.kw['ground_window'], wall_height=p('wall_height', 0.12),
                            wall_length=p('wall_length', 1.0), pillar_height=p('pillar_height', 0.35))
            self.latest = self.map3d = None
            self.walls_cache = (None, None)                            # (key, result)
            self.create_subscription(PointCloud2, '/terrain/cloud', lambda m: setattr(self, 'latest', m), 1)
            self.create_subscription(PointCloud2, p('map_cloud_topic', '/cloud_map'),
                                     lambda m: setattr(self, 'map3d', np.c_[tuple(xyz(m, 'x', 'y', 'z'))]), 1)
            self.marker_pub = self.create_publisher(MarkerArray, '/terrain/obstacles', 1)
            self.grid_pub = self.create_publisher(OccupancyGrid, '/terrain/obstacle_grid', 1)
            self.create_timer(1.0 / p('rate', 2.0), self.tick)

        def tick(self):
            msg, self.latest = self.latest, None
            if msg is None or msg.width == 0:
                return
            x, y, elev, chg = xyz(msg, 'x', 'y', 'elevation', 'change')
            r = self.res
            x0, y0 = x.min() - r / 2 - 0.5, y.min() - r / 2 - 0.5        # 0.5 m margin for the median
            i = np.round((x - x0) / r - 0.5).astype(int)
            j = np.round((y - y0) / r - 0.5).astype(int)
            H = np.full((i.max() + 11, j.max() + 11), np.nan, np.float32)
            C = H.copy()
            H[i, j], C[i, j] = elev, chg
            blobs, mask, ground = segment(H, C, r, **self.kw)
            if ground is None:
                return
            surface = np.where(np.isnan(H), ground, H)                  # drape terrain blobs on this
            blobs = [b for b in blobs if b['kind'] != 'structure']      # walls / pillar come from 3D
            if self.map3d is not None:
                key = (id(self.map3d), x0, y0, H.shape)                 # only redo on a new 3D map
                if self.walls_cache[0] != key:
                    self.walls_cache = (key, structures(self.map3d, x0, y0, H.shape, r, **self.skw))
                walls, g3 = self.walls_cache[1]
                if walls:
                    tall = np.any([b['cells'] for b in walls], axis=0)
                    mask |= tall
                    # Terrain bumps on a wall or the pillar (duct pieces, the column's foot) are part of it.
                    tall = ndimage.binary_dilation(tall, iterations=2)
                    blobs = [b for b in blobs if b['height'] < 0 or not (b['cells'] & tall).any()]
                blobs += walls

            arr = MarkerArray()
            clear = Marker()
            clear.header = msg.header
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            to_xy = lambda ii, jj: (x0 + (ii + 0.5) * r, y0 + (jj + 0.5) * r)

            def marker(kind, mtype, alpha=None):
                m = Marker()
                m.header, m.ns, m.id, m.type = msg.header, kind, len(arr.markers), mtype
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = KINDS[kind]
                if alpha is not None:
                    m.color.a = alpha
                arr.markers.append(m)
                return m

            for b in blobs:
                tall = b['kind'] in ('wall', 'pillar')
                ci, cj = np.nonzero(b['cells'])
                # Segmented cells: thin tiles draped on the terrain, or for walls and the pillar
                # columns from the ground up to their seen height.
                base = g3 if tall else surface
                cells = marker(b['kind'], Marker.CUBE_LIST, 0.45)
                cells.scale.x = cells.scale.y = r
                cells.scale.z = max(b['height'], 0.05) if tall else 0.015
                cx, cy = to_xy(ci, cj)
                cz = base[ci, cj] + (cells.scale.z / 2 if tall else 0.01)
                cells.points = [Point(x=float(a), y=float(b_), z=float(c)) for a, b_, c in zip(cx, cy, cz)]
                # Outline (cv2 contour points are (col, row) = (j, i))
                contours, _ = cv2.findContours(b['cells'].astype(np.uint8), cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    oj, oi = c[:, 0, 0], c[:, 0, 1]
                    ox, oy = to_xy(oi, oj)
                    oz = base[oi, oj] + (b['height'] if tall else 0.03)
                    line = marker(b['kind'], Marker.LINE_STRIP, 1.0)
                    line.scale.x = 0.015
                    line.points = [Point(x=float(a), y=float(b_), z=float(c_)) for a, b_, c_ in zip(ox, oy, oz)]
                    if len(line.points) > 1:
                        line.points.append(line.points[0])
                bx, by = to_xy(b['i'], b['j'])
                label = marker(b['kind'], Marker.TEXT_VIEW_FACING, 1.0)
                label.text = b['kind'] if b['kind'] == 'wall' else f"{b['kind']} {b['diameter'] * 100:.0f} cm"
                label.scale.z = 0.08
                ii, jj = min(int(round(b['i'])), H.shape[0] - 1), min(int(round(b['j'])), H.shape[1] - 1)
                label.pose.position.x, label.pose.position.y = bx, by
                label.pose.position.z = float(base[ii, jj]) + max(b['height'], 0.0) + 0.12
            self.marker_pub.publish(arr)

            g = OccupancyGrid()
            g.header = msg.header
            g.info.resolution = float(r)
            g.info.width, g.info.height = H.shape[0], H.shape[1]       # grid x = i, y = j
            g.info.origin.position.x, g.info.origin.position.y = float(x0), float(y0)
            g.info.origin.orientation.w = 1.0
            occ = np.where(~np.isnan(H), 0, -1).astype(np.int8)
            occ[mask] = 100
            g.data = occ.T.ravel().tolist()                             # row-major: y rows, x columns
            self.grid_pub.publish(g)
            counts = {}
            for b in blobs:
                counts[b['kind']] = counts.get(b['kind'], 0) + 1
            self.get_logger().info(f'obstacles: {counts}', throttle_duration_sec=10.0)

    rclpy.init()
    node = Segmenter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
