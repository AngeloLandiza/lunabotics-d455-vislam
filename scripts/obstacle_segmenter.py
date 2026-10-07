#!/usr/bin/env python3
"""Boulder / crater segmentation on the terrain map, for drivers and planners.

Height relative to the *local* ground (a wide median of the terrain map, so slopes
and the bed's unevenness cancel) is thresholded into bumps (> boulder_height) and
pits (< -crater_depth). Connected blobs are sized and labelled:

  boulder    bump 12-90 cm across          crater     natural pit 15-110 cm across
  dug pit    pit the robot dug (terrain mapper's change < 0 since the baseline)
  pile       bump the robot dumped (change > 0), e.g. the berm
  structure  long or large bump: ducts along the walls, the column box

Input:  /terrain/cloud (terrain_mapper.py: x, y, elevation, change per 5 cm cell)
Output: /terrain/obstacles      visualization_msgs/MarkerArray (cylinder + label each)
        /terrain/obstacle_grid  nav_msgs/OccupancyGrid (100 = boulder/crater/structure,
                                0 = known free, -1 = unknown), for path planning
"""
import math

import cv2
import numpy as np
from scipy import ndimage

KINDS = {  # rgba
    'boulder': (1.0, 0.45, 0.1, 0.9), 'crater': (0.2, 0.5, 1.0, 0.9), 'dug pit': (0.1, 0.9, 0.9, 0.8),
    'pile': (1.0, 0.9, 0.2, 0.8), 'structure': (0.6, 0.6, 0.6, 0.6),
}


def segment(H, change, res, ground_window=1.0, boulder_height=0.08, crater_depth=0.06,
            changed=0.04, boulder_size=(0.12, 0.9), crater_size=(0.15, 1.1), boulder_max_height=0.6):
    """H, change: (nx, ny) grids (NaN = unknown). Returns (list of blob dicts, obstacle mask).

    Blob: kind, i, j (centroid cell), diameter (m), height (m, + bump / - pit), cells.
    """
    known = ~np.isnan(H)
    if known.sum() < 20:
        return [], np.zeros(H.shape, bool)
    # Fill unknown with the nearest known height so the median sees no holes.
    idx = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
    filled = H[tuple(idx)]
    k = max(int(ground_window / res) | 1, 3)
    # Pass 1: a low percentile follows the regolith even where ducts or the column fill
    # most of the window. Pass 2: take bumps and pits out (filled from the nearest
    # regolith) and use the median, so rims and walls don't bias the ground.
    ground = ndimage.percentile_filter(filled, 30, size=k, mode='nearest')
    out = np.abs(filled - ground) > min(boulder_height, crater_depth)
    if out.any() and (~out).any():
        idx = ndimage.distance_transform_edt(out, return_distances=False, return_indices=True)
        ground = ndimage.median_filter(filled[tuple(idx)], size=k, mode='nearest')
    near = ndimage.binary_dilation(known, iterations=1)            # bridge single unknown cells
    rel = np.where(near, filled - ground, 0.0)
    chg = np.nan_to_num(change, nan=0.0)
    kernel = np.ones((3, 3), np.uint8)
    blobs, mask = [], np.zeros(H.shape, bool)
    for sign, thresh in ((1, boulder_height), (-1, crater_depth)):
        m = (sign * rel > thresh).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)          # drop single-cell noise
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
    return blobs, mask


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray

    class Segmenter(Node):
        def __init__(self):
            super().__init__('obstacle_segmenter')
            p = lambda n, d: self.declare_parameter(n, d).value
            self.res = p('resolution', 0.05)
            self.kw = dict(ground_window=p('ground_window', 1.0), boulder_height=p('boulder_height', 0.08),
                           crater_depth=p('crater_depth', 0.06), changed=p('changed', 0.04))
            self.latest = None
            self.create_subscription(PointCloud2, '/terrain/cloud', lambda m: setattr(self, 'latest', m), 1)
            self.marker_pub = self.create_publisher(MarkerArray, '/terrain/obstacles', 1)
            self.grid_pub = self.create_publisher(OccupancyGrid, '/terrain/obstacle_grid', 1)
            self.create_timer(1.0 / p('rate', 2.0), self.tick)

        def tick(self):
            msg, self.latest = self.latest, None
            if msg is None or msg.width == 0:
                return
            off = {f.name: f.offset // 4 for f in msg.fields}
            a = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
            x, y = a[:, off['x']], a[:, off['y']]
            r = self.res
            x0, y0 = x.min() - r / 2 - 0.5, y.min() - r / 2 - 0.5        # 0.5 m margin for the median
            i = np.round((x - x0) / r - 0.5).astype(int)
            j = np.round((y - y0) / r - 0.5).astype(int)
            H = np.full((i.max() + 11, j.max() + 11), np.nan, np.float32)
            C = H.copy()
            H[i, j], C[i, j] = a[:, off['elevation']], a[:, off['change']]
            blobs, mask = segment(H, C, r, **self.kw)

            arr = MarkerArray()
            clear = Marker()
            clear.header = msg.header
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            known = ~np.isnan(H)
            for k, b in enumerate(blobs):
                bx, by = x0 + (b['i'] + 0.5) * r, y0 + (b['j'] + 0.5) * r
                hz = float(np.nanmedian(H[b['cells']])) if known[b['cells']].any() else 0.0
                for kind_id, mtype in ((0, Marker.CYLINDER), (1, Marker.TEXT_VIEW_FACING)):
                    m = Marker()
                    m.header = msg.header
                    m.ns, m.id, m.type = b['kind'], 2 * k + kind_id, mtype
                    m.pose.orientation.w = 1.0
                    m.pose.position.x, m.pose.position.y = bx, by
                    m.color.r, m.color.g, m.color.b, m.color.a = KINDS[b['kind']]
                    if mtype == Marker.CYLINDER:
                        m.scale.x = m.scale.y = b['diameter']
                        m.scale.z = max(abs(b['height']), 0.02)
                        m.pose.position.z = hz - b['height'] / 2 if b['height'] < 0 else hz
                    else:
                        m.text = f"{b['kind']} {b['diameter'] * 100:.0f} cm"
                        m.scale.z = 0.08
                        m.pose.position.z = hz + abs(b['height']) + 0.12
                        m.color.a = 1.0
                    arr.markers.append(m)
            self.marker_pub.publish(arr)

            g = OccupancyGrid()
            g.header = msg.header
            g.info.resolution = float(r)
            g.info.width, g.info.height = H.shape[0], H.shape[1]       # grid x = i, y = j
            g.info.origin.position.x, g.info.origin.position.y = float(x0), float(y0)
            g.info.origin.orientation.w = 1.0
            occ = np.where(known, 0, -1).astype(np.int8)
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
