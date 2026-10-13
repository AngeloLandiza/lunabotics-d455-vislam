"""(c) Where does the rover turn about during pivots, and can the IMU tell?

Run 11 replay: pivots with encoder speed about 0 moved base_link about 0.2 m per radian next to the fresh hole, far more
than the icr_offset model (0.0475 m). For a rigid body turning at rate w about a point c (base_link frame), base_link
moves with v = w z x (0 - c), so from ground truth c = (-v_y / w, v_x / w). The accelerometer at the IMU position p
sees, after gravity, a = -w^2 (p - c) + dw/dt z x (p - c): solve for p - c, and c = p - (p - c).

For every 0.5 s window with |w| > w_min and |encoder speed| < 0.03 m/s (a pivot): the true c, the IMU c, and the
base_link displacement over the window from truth, from the icr_offset model, and from the IMU c. Then per pivot
(consecutive windows): total turn, true displacement, model and IMU displacement error.

Usage: python3 sim/autonomy_diag/pivot_icr.py ~/bags/run11 [out.json]
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

WIN, W_MIN = 0.5, 0.3


def main():
    d = bag_io.load(os.path.expanduser(sys.argv[1]),
                    ['/ground_truth/base_pose', '/wheel/odom', '/camera/imu', '/tf_static'])
    T = d['T_base'][d['imu_frame']]
    R, p = T[:3, :3], T[:3, 3]
    imu = d['imu']
    t_i = imu[:, 0]
    w_b = imu[:, 1:4] @ R.T
    a_b = imu[:, 4:7] @ R.T
    # gravity in base_link with SlipGuard's tracker
    sg = ac.SlipGuard()
    a_lin = np.zeros((len(t_i), 3))
    for k in range(len(t_i)):
        sg.gravity(w_b[k], a_b[k], 0.0 if k == 0 else t_i[k] - t_i[k - 1])
        a_lin[k] = a_b[k] - sg.g
    gt = d['gt']
    enc = d['wheel']
    rows = []
    for t0 in np.arange(max(gt[0, 0], t_i[0]) + 1.0, min(gt[-1, 0], t_i[-1]) - 1.0, WIN):
        t1 = t0 + WIN
        m = (t_i >= t0) & (t_i < t1)
        if m.sum() < 10:
            continue
        w = float(w_b[m, 2].mean())
        ve = np.abs(enc[(enc[:, 0] >= t0) & (enc[:, 0] < t1), 1])
        if abs(w) < W_MIN or not len(ve) or ve.mean() > 0.03:
            continue
        P0, P1 = bag_io.interp_pose(gt, np.array([t0, t1]))
        dyaw = ac.wrap(P1[2] - P0[2])
        wt = dyaw / WIN
        if abs(wt) < 1e-3:
            continue
        # true base_link velocity in the body frame at the window's middle yaw
        ym = P0[2] + dyaw / 2
        c_, s_ = math.cos(ym), math.sin(ym)
        dv = (P1[:2] - P0[:2]) / WIN
        vb = np.array([c_ * dv[0] + s_ * dv[1], -s_ * dv[0] + c_ * dv[1]])
        c_true = np.array([-vb[1] / wt, vb[0] / wt])
        # IMU: mean linear acceleration and angular acceleration over the window
        a = a_lin[m, :2].mean(0)
        tt = t_i[m]
        alpha = float(np.polyfit(tt - tt.mean(), w_b[m, 2], 1)[0]) if m.sum() > 3 else 0.0
        M = np.array([[-w * w, -alpha], [alpha, -w * w]])
        dvec = np.linalg.solve(M, a)
        c_imu = p[:2] - dvec

        def disp(c):            # base_link displacement over the window for a turn about c (body frame at t0)
            cw = np.array([math.cos(P0[2]) * c[0] - math.sin(P0[2]) * c[1],
                           math.sin(P0[2]) * c[0] + math.cos(P0[2]) * c[1]])
            r = -cw
            rot = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
            return rot @ r - r

        true_d = P1[:2] - P0[:2]
        rows.append({'t': round(float(t0), 2), 'xy': np.round(P0[:2], 3).tolist(), 'w': round(w, 3),
                     'dyaw': round(float(dyaw), 3), 'c_true': np.round(c_true, 3).tolist(),
                     'c_imu': np.round(c_imu, 3).tolist(),
                     'd_true': np.round(true_d, 3).tolist(),
                     'err_model_cm': round(100 * float(np.hypot(*(disp(np.array([-ac.val('icr_offset'), 0.0]))
                                                                    - true_d))), 1),
                     'err_imu_cm': round(100 * float(np.hypot(*(disp(c_imu) - true_d))), 1),
                     'err_still_cm': round(100 * float(np.hypot(*true_d)), 1)})
    # group consecutive windows into pivots
    piv, cur = [], []
    for r in rows:
        if cur and r['t'] - cur[-1]['t'] > WIN + 1e-6:
            piv.append(cur)
            cur = []
        cur.append(r)
    if cur:
        piv.append(cur)
    summ = []
    for g in piv:
        summ.append({'t0': g[0]['t'], 't1': g[-1]['t'] + WIN, 'xy': g[0]['xy'],
                     'turn_rad': round(sum(abs(r['dyaw']) for r in g), 2),
                     'true_path_cm': round(sum(r['err_still_cm'] for r in g), 1),
                     'model_err_cm': round(sum(r['err_model_cm'] for r in g), 1),
                     'imu_err_cm': round(sum(r['err_imu_cm'] for r in g), 1),
                     'c_true_median': np.round(np.median([r['c_true'] for r in g], 0), 3).tolist(),
                     'c_imu_median': np.round(np.median([r['c_imu'] for r in g], 0), 3).tolist()})
    e_m = [r['err_model_cm'] for r in rows]
    e_i = [r['err_imu_cm'] for r in rows]
    out = {'imu_pos_base': np.round(p, 3).tolist(), 'windows': len(rows),
           'per_window_err_cm': {'model_median': float(np.median(e_m)) if e_m else None,
                                 'imu_median': float(np.median(e_i)) if e_i else None,
                                 'model_p95': float(np.percentile(e_m, 95)) if e_m else None,
                                 'imu_p95': float(np.percentile(e_i, 95)) if e_i else None},
           'pivots': summ, 'rows': rows}
    print(json.dumps({k: v for k, v in out.items() if k != 'rows'}, indent=1))
    if len(sys.argv) > 2:
        json.dump(out, open(sys.argv[2], 'w'), indent=1)


if __name__ == '__main__':
    main()
