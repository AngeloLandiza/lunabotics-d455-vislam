"""Report on an arena.sh --autonomy bag: localizer error, wheel-held-back episodes, dig/dump requests, hazards vs truth.

Usage: python3 sim/autonomy_diag/run_report.py ~/bags/run11 [seed] [out.json]
Held-back episode: a 0.5 s window where wheel_scale x encoder distance exceeds the true distance by > 0.1 m (the
wheels turn but the body is held back: soft bed, ruts, or pushing on something); reported with the commanded speed.
"""
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'lunabotics_slam', 'scripts'))
sys.path.insert(0, HERE)
import autonomy_core as ac          # noqa: E402
import bag_io                       # noqa: E402


def extra(bag):
    """cmd_vel (t_received, v, w), /arena/dig requests, last /autonomy/hazards."""
    comp = any(f.endswith('.zstd') for f in os.listdir(bag))
    r = rosbag2_py.SequentialCompressionReader() if comp else rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/cmd_vel', '/arena/dig', '/autonomy/hazards', '/clock') if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    cmd, digs, haz, clock = [], [], None, 0.0
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == '/clock':
            clock = m.clock.sec + m.clock.nanosec * 1e-9
        elif topic == '/cmd_vel':
            cmd.append((clock, m.linear.x, m.angular.z))
        elif topic == '/arena/dig':
            digs.append([round(clock, 2)] + [round(float(v), 3) for v in m.data])
        else:
            haz = np.array(m.data).reshape(-1, 5)
    return np.array(cmd) if cmd else np.zeros((0, 3)), digs, haz


def main():
    bag = os.path.expanduser(sys.argv[1])
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    d = bag_io.load(bag, ['/ground_truth/base_pose', '/wheel/odom', '/autonomy/pose'])
    cmd, digs, haz = extra(bag)
    gt, est, w = d['gt'], d['est'], d['wheel']
    out = {}
    ts = np.arange(est[0, 0], min(est[-1, 0], gt[-1, 0]), 1.0)
    e = np.hypot(*(bag_io.interp_pose(est, ts)[:, :2] - bag_io.interp_pose(gt, ts)[:, :2]).T)
    out['localizer'] = {'samples': len(ts), 'median_cm': 100 * float(np.median(e)),
                        'p95_cm': 100 * float(np.percentile(e, 95)), 'max_cm': 100 * float(e.max())}
    P = gt[:, 1:3]
    out['true_path_m'] = float(np.hypot(*np.diff(P, axis=0).T).sum())
    scale = 0.84
    eps = []
    for t in np.arange(gt[0, 0], gt[-1, 0] - 0.5, 0.5):
        a, b = bag_io.interp_pose(gt, [t, t + 0.5])
        true = float(np.hypot(*(b[:2] - a[:2])))
        m = (w[:, 0] >= t) & (w[:, 0] < t + 0.5)
        if m.sum() < 2:
            continue
        enc = scale * float(abs(np.trapz(w[m, 1], w[m, 0])))
        if enc - true > 0.1:
            mc = (cmd[:, 0] >= t) & (cmd[:, 0] < t + 0.5) if len(cmd) else np.zeros(0, bool)
            eps.append({'t': round(float(t), 1), 'true_m': round(true, 3), 'enc_m': round(enc, 3),
                        'enc_speed': round(float(np.mean(np.abs(w[m, 1]))), 3),
                        'cmd_v': round(float(np.mean(cmd[mc, 1])), 3) if mc.any() else None,
                        'where': [round(float(v), 2) for v in a[:2]]})
    out['held_back'] = eps
    out['digs'] = digs
    A = ac.A27
    B, C = A.layout(seed)[0], A.layout(seed)[1]
    truth = [('boulder', o.x, o.y, o.diameter / 2) for o in B] + [('crater', o.x, o.y, o.diameter / 2) for o in C]
    hz = []
    if haz is not None:
        for k in haz:
            dd = min((math.hypot(k[1] - t[1], k[2] - t[2]) - t[3], t[0], t[1], t[2]) for t in truth)
            hz.append({'kind': ('boulder', 'crater')[int(k[0])], 'x': round(float(k[1]), 2), 'y': round(float(k[2]), 2),
                       'r': round(float(k[3]), 2), 'ev': round(float(k[4]), 1), 'nearest': dd[1],
                       'nearest_xy': [round(dd[2], 2), round(dd[3], 2)], 'edge_m': round(float(dd[0]), 2)})
        found = set()
        for kind, x, y, r in truth:
            if any(math.hypot(h['x'] - x, h['y'] - y) < r + 0.25 for h in hz):
                found.add((kind, x, y))
        out['missed'] = [(k, round(x, 2), round(y, 2)) for k, x, y, r in truth if (k, x, y) not in found]
    out['hazards'] = hz
    print(json.dumps(out, indent=1))
    if len(sys.argv) > 3:
        json.dump(out, open(sys.argv[3], 'w'), indent=1)


if __name__ == '__main__':
    main()
