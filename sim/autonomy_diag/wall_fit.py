"""(a) Wall-fit quality with the TRUE pose, from an arena.sh --autonomy bag.

Every wall_every s: that interval's L1 wall-band points (base_link, body box removed, band from autonomy_core
wall_band), placed in the arena with the TRUE pose at each cloud's time, expressed in the true robot frame at the fix
time, and matched by autonomy_core.wall_icp (the fix's ICP) seeded at the truth, with the same wall-offset shift WallFilter.wall_fix uses.
Reports (fit - truth) in x / y / yaw, lam and n_used, and for fits off by more than 5 cm (or 0.5 deg) where the
in-gate points (|r| < 0.5 m) that are NOT within 5 cm of a wall face lie, by arena structure.

Usage: python3 sim/autonomy_diag/wall_fit.py ~/bags/run10 [out.json]
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'lunabotics_slam', 'scripts'))
sys.path.insert(0, HERE)
import autonomy_core as ac          # noqa: E402
import bag_io                       # noqa: E402

localize = ac.localize
A = ac.A27 if hasattr(ac, 'A27') else __import__('arena')


def se2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def label(p, layout):
    """Arena structure nearest to a world point (x, y)."""
    x, y = p
    off = ac.val('wall_offset')
    for px, py in A.POLES:
        if math.hypot(x - px, y - py) < 0.15:
            return 'range pole'
    cx, cy, cs = A.COLUMN
    if abs(x - cx) < cs / 2 + 0.1 and abs(y - cy) < cs / 2 + 0.1:
        return 'column'
    for b in layout[0]:
        if math.hypot(x - b.x, y - b.y) < b.diameter / 2 + 0.1:
            return 'boulder'
    bx0, by0, bx1, by1 = A.ZONES['berm']
    if bx0 - 0.1 <= x <= bx1 + 0.1 and by0 - 0.1 <= y <= by1 + 0.1:
        return 'berm box (dump pile)'
    d_duct = min(x, y, A.LENGTH - x, A.WIDTH - y)
    if -off - 0.1 < d_duct < 0.1:
        return 'duct / wall base'
    if d_duct < -off - 0.1:
        return 'beyond the wall (glass / frame)'
    return 'open floor'


def main():
    bag = os.path.expanduser(sys.argv[1])
    d = bag_io.load(bag, ['/ground_truth/base_pose', '/l1/points', '/tf_static'])
    T_bl = d['T_base'][d['cloud_frame']]
    gt = d['gt']
    off = ac.val('wall_offset')
    L, W = ac.ARENA_L, ac.ARENA_W
    Lw, Ww = L + 2 * off, W + 2 * off
    o = np.array([off, off, 0.0])
    zlo, zhi = ac.val('wall_band')
    hb = ac.val('base_height')
    every = ac.drift.v(ac.drift.PARAMS, 'wall_every')
    layout = A.layout(0)
    rows, clutter = [], {}
    buf, t_fix = [], None
    for t, P in d['clouds']:
        if t < gt[0, 0] or t > gt[-1, 0]:
            continue
        Pb = P @ T_bl[:3, :3].T + T_bl[:3, 3]
        Pb = Pb[~ac.body_mask(Pb)]
        q = Pb[(Pb[:, 2] + hb > zlo) & (Pb[:, 2] + hb < zhi), :2]
        x, y, a = bag_io.interp_pose(gt, t)[0]
        c, s = math.cos(a), math.sin(a)
        buf.append(np.stack([x + c * q[:, 0] - s * q[:, 1], y + s * q[:, 0] + c * q[:, 1]], -1))
        t_fix = t_fix if t_fix is not None else t
        if t - t_fix < every:
            continue
        t_fix = t
        pts = np.concatenate(buf)
        buf = []
        truth = bag_io.interp_pose(gt, t)[0]
        Xi = np.linalg.inv(se2(*truth))
        Pl = pts @ Xi[:2, :2].T + Xi[:2, 2]
        if len(Pl) < ac.drift.v(ac.drift.PARAMS, 'wall_min_pts'):
            rows.append({'t': t, 'status': 'few', 'n': len(Pl)})
            continue
        pose, lam, n_used = ac.wall_icp(Pl, truth + o, Lw, Ww)
        e = pose - o - truth
        e[2] = (e[2] + math.pi) % (2 * math.pi) - math.pi
        row = {'t': round(t, 2), 'n': len(Pl), 'n_used': int(n_used), 'lam': None if lam is None else float(lam),
               'ex': float(e[0]), 'ey': float(e[1]), 'eyaw_deg': float(math.degrees(e[2])),
               'truth': [round(float(v), 3) for v in truth]}
        rows.append(row)
        if math.hypot(e[0], e[1]) > 0.05 or abs(math.degrees(e[2])) > 0.5:
            # in-gate points, placed with the TRUE pose, that are not on a wall face
            r, _ = localize.wall_residuals(Pl, truth + o, Lw, Ww)
            gate = np.abs(r) < 0.5
            on_wall = np.abs(r) < 0.05
            P_world = pts[gate & ~on_wall]
            labs = [label(p, layout) for p in P_world]
            u, cnt = np.unique(labs, return_counts=True)
            row['clutter'] = dict(zip(u.tolist(), cnt.tolist()))
            for k, v in row['clutter'].items():
                clutter[k] = clutter.get(k, 0) + v
    ok = [r for r in rows if 'ex' in r]
    exy = np.array([math.hypot(r['ex'], r['ey']) for r in ok])
    ey = np.array([abs(r['eyaw_deg']) for r in ok])
    lam = np.array([r['lam'] if r['lam'] is not None else np.nan for r in ok])
    nu = np.array([r['n_used'] for r in ok])
    summ = {'fits': len(ok), 'few': len(rows) - len(ok),
            'xy_err_cm': {'median': 100 * float(np.median(exy)), 'p95': 100 * float(np.percentile(exy, 95)),
                          'max': 100 * float(exy.max())},
            'x_err_cm_median_abs': 100 * float(np.median(np.abs([r['ex'] for r in ok]))),
            'y_err_cm_median_abs': 100 * float(np.median(np.abs([r['ey'] for r in ok]))),
            'yaw_err_deg': {'median': float(np.median(ey)), 'p95': float(np.percentile(ey, 95)), 'max': float(ey.max())},
            'lam': {'min': float(np.nanmin(lam)), 'median': float(np.nanmedian(lam)),
                    'below_lam_min': int(np.sum(~(lam > ac.drift.v(ac.drift.PARAMS, 'wall_lam_min'))))},
            'n_used': {'min': int(nu.min()), 'median': float(np.median(nu))},
            'fits_off_5cm_or_0.5deg': int(np.sum((exy > 0.05) | (ey > 0.5))),
            'clutter_points_in_bad_fits': clutter}
    print(json.dumps(summ, indent=1))
    bad = sorted([r for r in ok if 'clutter' in r], key=lambda r: -math.hypot(r['ex'], r['ey']))[:10]
    for r in bad:
        print('  t %.1f truth %s: err %.1f/%.1f cm %.2f deg, lam %s, n_used %d, clutter %s'
              % (r['t'], r['truth'], 100 * r['ex'], 100 * r['ey'], r['eyaw_deg'],
                 None if r['lam'] is None else round(r['lam'], 2), r['n_used'], r['clutter']))
    if len(sys.argv) > 2:
        json.dump({'summary': summ, 'fits': rows}, open(sys.argv[2], 'w'), indent=1)


if __name__ == '__main__':
    main()
