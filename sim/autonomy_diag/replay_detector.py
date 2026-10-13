"""(d) Replay hazard_detector.py's HazardMap offline from an arena.sh --autonomy bag, with different poses.

Same frame logic as lunabotics_slam/scripts/hazard_detector.py (ground refit every 1 s, as its publish timer does) for the L1 (the D455 depth stream is not in the bag):
body points masked, the cloud placed with the pose at its stamp, frames skipped while |yaw rate| over the last 0.2 s
exceeds turn_max, the footprint marked as driven over every 3 cm. Pose variants:
  gt        /ground_truth/base_pose: what the map would be with a perfect localizer
  est       /autonomy/pose (the live localizer): should reproduce the live map
  *_tilt    the same with roll / pitch from the IMU's gravity (arena_localizer publishes them in /autonomy/pose and
            hazard_detector uses them): yaw-only placement of 3+ m returns put a 0.1 m false bump on the berm route
Every variant starts where the live detector did, at the first /autonomy/pose (14.6 s in run 14): the L1 frames before
it (rover at the start facing the west wall for the tag turn) place the bed ~0.3 m high and spoil the first ground fit,
which left the gt map nearly empty (run 14: 2 hazards, 10 of 13 obstacles missed).
Hazards are scored against the seed's true layout: matched (centre within r_true + 0.25), false, missed.

Usage: python3 sim/autonomy_diag/replay_detector.py ~/bags/run14 <seed> [out.json]
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


def rate_and_speed(P, t, half=0.1):
    a, b = bag_io.interp_pose(P, np.array([t - 2 * half, t]))
    dt = 2 * half
    return ac.wrap(b[2] - a[2]) / dt, math.hypot(*(b[:2] - a[:2])) / dt


def imu_tilt(d):
    """(t, roll, pitch) at the IMU rate from the accelerometer + gyro, as arena_localizer publishes them."""
    R = d['T_base'][d['imu_frame']][:3, :3]
    g, out, tp = ac.SlipGuard(), [], None
    for row in d['imu']:
        t = row[0]
        g.gravity(R @ row[1:4], R @ row[4:7], 0.0 if tp is None else min(max(t - tp, 0.0), 0.5))
        tp = t
        out.append((t, *g.tilt()))
    return np.array(out)


def replay(d, P, turn_max=0.3, still_skip=False, t0=None, tilt=None):
    m = ac.HazardMap()
    T_bs = d['T_base'][d['cloud_frame']]
    last_swept = None
    n_used = n_turn = n_still = 0
    t_refit = None                      # the node refits the ground every publish_s (1 s) before publishing
    t_lo, t_hi = max(P[0, 0], t0 if t0 is not None else -1e9) + 0.25, P[-1, 0]
    for t, cloud in d['clouds']:
        if t < t_lo or t > t_hi or len(cloud) == 0:
            continue
        if t_refit is None:
            t_refit = t
        elif t - t_refit >= 1.0:
            m.refit()
            t_refit = t
        pose = bag_io.interp_pose(P, np.array([t]))[0]
        if last_swept is None or math.hypot(*(pose[:2] - last_swept[:2])) > 0.03 or \
                abs(ac.wrap(pose[2] - last_swept[2])) > 0.05:
            m.drove(pose)
            last_swept = pose
        w, sp = rate_and_speed(P, t)
        if abs(w) > turn_max:
            n_turn += 1
            continue
        if still_skip and sp < 0.03 and abs(w) > 0.1:
            n_still += 1
            continue
        Pb = ac.apply_T(T_bs, cloud)
        keep = ~ac.body_mask(Pb)
        if tilt is not None:
            k = min(int(np.searchsorted(tilt[:, 0], t)), len(tilt) - 1)
            T_ab = ac.base_T(*pose, roll=tilt[k, 1], pitch=tilt[k, 2])
        else:
            T_ab = ac.base_T(*pose)
        Pa = ac.apply_T(T_ab, Pb[keep])
        origin = ac.apply_T(T_ab, T_bs[:3, 3][None])[0]
        m.add(ac.make_scan(origin, Pa, ac.val('l1_sigma'), 'l1'))
        n_used += 1
    m.refit()
    return m.objects(), {'frames': n_used, 'skipped_turning': n_turn, 'skipped_in_place': n_still,
                         'duct_drops': sorted({x[1:3] for x in m.dropped if x[0] == 'duct'})}


def score(objs, truth):
    hz, found = [], set()
    for o in objs:
        dd = min((math.hypot(o['x'] - t[1], o['y'] - t[2]) - t[3], t) for t in truth)
        ok = dd[0] < 0.25
        if ok:
            found.add(dd[1])
        hz.append({'kind': o['kind'], 'x': round(o['x'], 2), 'y': round(o['y'], 2), 'r': round(o['r'], 2),
                   'ev': round(float(o['evidence']), 1), 'match': ok,
                   'nearest': [dd[1][0], round(dd[1][1], 2), round(dd[1][2], 2), round(float(dd[0]), 2)]})
    missed = [(k, round(x, 2), round(y, 2)) for k, x, y, r in truth if (k, x, y, r) not in found]
    return {'n': len(hz), 'matched': sum(h['match'] for h in hz), 'false': sum(not h['match'] for h in hz),
            'missed': missed, 'hazards': hz}


def main():
    bag, seed = os.path.expanduser(sys.argv[1]), int(sys.argv[2])
    d = bag_io.load(bag, ['/ground_truth/base_pose', '/autonomy/pose', '/l1/points', '/tf_static', '/camera/imu'])
    tl = imu_tilt(d)
    B, C = ac.A27.layout(seed)[0], ac.A27.layout(seed)[1]
    truth = [('boulder', o.x, o.y, o.diameter / 2) for o in B] + [('crater', o.x, o.y, o.diameter / 2) for o in C]
    out = {}
    for name, P, tilt in (('gt', d['gt'], None), ('est', d['est'], None), ('gt_tilt', d['gt'], tl),
                          ('est_tilt', d['est'], tl)):
        objs, info = replay(d, P, tilt=tilt, t0=d['est'][0, 0])
        out[name] = {**info, **score(objs, truth)}
        print(name, {k: v for k, v in out[name].items() if k != 'hazards'})
    if len(sys.argv) > 3:
        json.dump(out, open(sys.argv[3], 'w'), indent=1)


if __name__ == '__main__':
    main()
