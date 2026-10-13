"""L1 heights around a spot with the true pose placed two ways: full 6-DoF (/ground_truth/base_pose position and
quaternion) and planar (x, y, yaw at the bed height, as bag_io / the HazardMap did before b039076), binned by range.

Found the run 13/14 "corridor object" at (3.0-3.1, 0.64-0.95): flat in 6-DoF at every range, 0.1-0.17 m high in planar
placement at 3+ m range when the rover was rolled/pitched 1-3 deg.
Usage: python3 sim/autonomy_diag/corridor6dof.py BAG [x y radius]   (default 3.05 0.8 0.4)
"""
import math
import os
import sys

import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'lunabotics_slam', 'scripts'))
sys.path.insert(0, HERE)
import autonomy_core as ac, bag_io, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot
CX, CY, CR = (float(a) for a in sys.argv[2:5]) if len(sys.argv) > 4 else (3.05, 0.8, 0.4)
for bag in [os.path.expanduser(sys.argv[1])]:
    run = os.path.basename(bag.rstrip('/'))
    d = bag_io.load(bag, ['/ground_truth/base_pose', '/autonomy/pose', '/l1/points', '/tf_static'])
    r = rosbag2_py.SequentialCompressionReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/ground_truth/base_pose'])); cls = get_message('geometry_msgs/msg/PoseStamped')
    F = []
    while r.has_next():
        _, b, _ = r.read_next(); m = deserialize_message(b, cls); p, q = m.pose.position, m.pose.orientation
        F.append((bag_io.st(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    F = np.array(F)
    eul = Rot.from_quat(F[:, 4:8]).as_euler('xyz', degrees=True)
    print(f'{run}: base z p5/50/95 {np.percentile(F[:,3],[5,50,95]).round(3)}  roll p5/95 {np.percentile(eul[:,0],[5,95]).round(2)}  pitch p5/95 {np.percentile(eul[:,1],[5,95]).round(2)}')
    T_bs = d['T_base'][d['cloud_frame']]; E = d['est']
    rows = []
    for t, cl in d['clouds']:
        if t < E[0, 0] or len(cl) == 0: continue
        i = min(np.searchsorted(F[:, 0], t), len(F) - 1)
        T = np.eye(4); T[:3, :3] = Rot.from_quat(F[i, 4:8]).as_matrix(); T[:3, 3] = F[i, 1:4]
        Pb = ac.apply_T(T_bs, cl); Pb = Pb[~ac.body_mask(Pb)]
        Q = ac.apply_T(T, Pb); o = ac.apply_T(T, T_bs[:3, 3][None])[0]
        Q2 = ac.apply_T(ac.base_T(*bag_io.interp_pose(d['gt'], np.array([t]))[0]), Pb)   # planar placement
        k = np.hypot(Q[:, 0] - CX, Q[:, 1] - CY) < CR
        for q, q2 in zip(Q[k], Q2[k]): rows.append((t, *q, np.linalg.norm(q - o), q2[2], eul[i, 0], eul[i, 1]))
    R = np.array(rows)
    R = R[R[:, 3] < 0.6]                      # drop the rare tall returns (separate question)
    print(f'  n {len(R)}  6-DoF z p50/p95/max {np.percentile(R[:,3],[50,95]).round(3)} {R[:,3].max():.3f}   planar z p50/p95 {np.percentile(R[:,5],[50,95]).round(3)}')
    for a, b in [(0, 2), (2, 2.5), (2.5, 3), (3, 3.5), (3.5, 5)]:
        s = R[(R[:, 4] >= a) & (R[:, 4] < b)]
        if len(s): print(f'   range {a}-{b} m: n {len(s):5d}  6-DoF z p50 {np.median(s[:,3]):.3f} p95 {np.percentile(s[:,3],95):.3f}  planar z p50 {np.median(s[:,5]):.3f} p95 {np.percentile(s[:,5],95):.3f}  rover roll {np.median(s[:,6]):.2f} pitch {np.median(s[:,7]):.2f}')
