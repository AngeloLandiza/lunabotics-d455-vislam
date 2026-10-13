"""(b) Replay arena_localizer.py's WallFilter offline from an arena.sh --autonomy bag, in variants.

Same event logic as lunabotics_slam/scripts/arena_localizer.py: gyro bias from calib_s of standstill, first tag fix
initialises, later tag fixes while standing in the start zone update, prediction at the IMU rate from the encoder
speed (x wheel_scale) and the gyro yaw rate in base_link, L1 wall-band points buffered per cloud, a wall fix every
wall_every s. Variants change autonomy_core PARAMS before each replay:
  now          as committed (wheel_scale 0.84)
  lam_use_015  wall_lam_use = 0.15 (only well-conditioned wall fits)
  wheel095     wheel_scale = 0.95 (before 71ca022)
Reports xy error vs /ground_truth/base_pose (sampled at 1 Hz) and the fix / gated / relock / weak counts.

Usage: python3 sim/autonomy_diag/replay_localizer.py ~/bags/run10 [out.json] [gyro bias deg/s from the live log]
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

VARIANTS = {'now': {'cap': 'rise'}, 'slip_guard': {'cap': 'rise', 'slip': True},
            'slip_guard_nocap': {'cap': False, 'slip': True}}


def replay(d, overrides, wheel_scale=0.84, calib_s=2.0, bias=None, cap=True, slip=False):
    import dataclasses
    overrides = dict(overrides)
    wheel_scale = overrides.pop('wheel_scale', wheel_scale)
    cap = overrides.pop('cap', cap)      # the node's parameter, not an autonomy_core one
    slip = overrides.pop('slip', slip)   # arena_localizer slip_guard
    saved = {k: ac.PARAMS[k] for k in overrides}
    for k, v in overrides.items():
        ac.PARAMS[k] = dataclasses.replace(ac.PARAMS[k], value=v)
    try:
        f = ac.WallFilter(scale=wheel_scale)
        sg = ac.SlipGuard(scale=wheel_scale) if slip else None
        R_imu = d['T_base'][d['imu_frame']][:3, :3]
        T_cl = d['T_base'][d['cloud_frame']]
        ev = ([(t, 0, i) for i, t in enumerate(d['wheel'][:, 0])] + [(t, 1, i) for i, t in enumerate(d['imu'][:, 0])]
              + [(t, 2, i) for i, t in enumerate(d['tags'][:, 0])] + [(c[0], 3, i) for i, c in enumerate(d['clouds'])])
        ev.sort()
        # bias: the live node's standstill estimate (its log line 'gyro bias ... deg/s') when the bag starts after it;
        # otherwise estimated here, which is wrong if the robot already pivots (wheel speed 0) inside the window
        v_wheel, t_wheel, calib, t_c0, t_last, counts, track = 0.0, None, [], None, None, {}, []
        events = []                     # each wall fix with the true error before and after (bag ground truth)
        bias = None if bias is None else math.radians(bias)
        x0, x1, y0, y1 = ac.v(ac.pr.ARENA, 'start_zone')
        every = ac.drift.v(ac.drift.PARAMS, 'wall_every')
        for t, kind, i in ev:
            if kind == 0:
                v = d['wheel'][i, 1]
                if cap and t_wheel is not None and t > t_wheel:            # arena_localizer 6030536: the body cannot
                    dv = 1.5 * ac.v(ac.a2.PARAMS, 'a_wheel_max') * (t - t_wheel)   # speed up faster than the drive ramp
                    if cap == 'rise':                                       # limit only growth in |v|; a drop
                        v = float(np.sign(v) * min(abs(v), abs(v_wheel) + dv)) # (spin ending) passes straight through
                    else:
                        v = float(np.clip(v, v_wheel - dv, v_wheel + dv))
                v_wheel, t_wheel = v, t
            elif kind == 1:
                wz = float((R_imu @ d['imu'][i, 1:4])[2])
                if bias is None:
                    if t_wheel is None or abs(v_wheel) > 0.005:
                        calib, t_c0 = [], None
                        continue
                    if abs(math.degrees(wz)) >= 2.0:                     # arena_localizer 71ca022: still gyro too
                        calib, t_c0 = [], None
                        continue
                    t_c0 = t_c0 or t
                    calib.append(wz)
                    if t - t_c0 >= calib_s:
                        bias = float(np.mean(calib))
                    continue
                v_use = v_wheel
                if sg is not None:
                    wb = R_imu @ d['imu'][i, 1:4]
                    wb[2] -= bias
                    v_use = sg.step(t, wb, R_imu @ d['imu'][i, 4:7], v_wheel)
                if not f.ready:
                    continue
                f.predict(v_use, wz - bias, t)
                track.append((t, *f.x))
                t_last = t_last if t_last is not None else t
                if t - t_last >= every:
                    t_last = t
                    x_before = f.x.copy()
                    r = f.wall_fix()
                    if r:
                        counts[r] = counts.get(r, 0) + 1
                        g = bag_io.interp_pose(d['gt'], np.array([t]))[0]
                        events.append({'t': round(t, 2), 'kind': r, 'err_before_cm': round(100 * float(np.hypot(
                            *(x_before[:2] - g[:2]))), 1), 'err_after_cm': round(100 * float(np.hypot(*(f.x[:2] - g[:2]))), 1),
                            'est': np.round(f.x, 3).tolist(), 'gt': np.round(g[:3], 3).tolist(),
                            'sd_cm': round(100 * float(np.sqrt(f.Pc[0, 0] + f.Pc[1, 1])), 1)})
            elif kind == 2:
                if bias is None:
                    continue
                z = d['tags'][i, 1:4]
                if not f.ready:
                    f.init(z, t)
                    continue
                if x0 <= f.x[0] <= x1 and y0 <= f.x[1] <= y1 and abs(v_wheel) < 0.005:
                    ok = f.pose_update(z, trust=True)
                    k = 'tag ' + ('used' if ok else 'gated')
                    counts[k] = counts.get(k, 0) + 1
            else:
                if not f.ready:
                    continue
                P = d['clouds'][i][1]
                Pb = P @ T_cl[:3, :3].T + T_cl[:3, 3]
                f.add_wall_points(Pb[~ac.body_mask(Pb)], t)
        track = np.array(track)
        if sg is not None:
            counts['slip episodes'], counts['slip s'] = sg.episodes, round(sg.slip_s, 2)
        counts['events'] = events
        return track, counts
    finally:
        for k, v in saved.items():
            ac.PARAMS[k] = v


def score(track, gt, est_live=None):
    ts = np.arange(max(track[0, 0], gt[0, 0]), min(track[-1, 0], gt[-1, 0]), 1.0)
    k = np.clip(np.searchsorted(track[:, 0], ts), 0, len(track) - 1)
    E = track[k, 1:3]
    G = bag_io.interp_pose(gt, ts)[:, :2]
    e = np.hypot(*(E - G).T)
    return {'samples': len(ts), 'median_cm': 100 * float(np.median(e)), 'p95_cm': 100 * float(np.percentile(e, 95)),
            'max_cm': 100 * float(e.max())}


def main():
    bag = os.path.expanduser(sys.argv[1])
    bias = float(sys.argv[3]) if len(sys.argv) > 3 else None     # deg/s, from the live run's log
    d = bag_io.load(bag)
    out = {}
    if len(d['est']):
        out['live'] = score(d['est'], d['gt'])
        print('live /autonomy/pose:', out['live'])
    for name, ov in VARIANTS.items():
        track, counts = replay(d, ov, bias=bias)
        events = counts.pop('events')
        bad = [e for e in events if e['err_after_cm'] > e['err_before_cm'] + 5]
        out[name] = {**score(track, d['gt']), 'counts': counts, 'events': events, 'worsening_fixes': bad, 'overrides': {k: list(v) if isinstance(v, tuple) else v
                                                                               for k, v in ov.items()}}
        print(name, {k: v for k, v in out[name].items() if k != 'events'})
    if len(sys.argv) > 2:
        json.dump(out, open(sys.argv[2], 'w'), indent=1)


if __name__ == '__main__':
    main()
