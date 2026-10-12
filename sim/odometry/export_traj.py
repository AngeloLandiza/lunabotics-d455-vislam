"""Save the true drives of drift.py (same seeds, same slip draws) for rendering elsewhere, e.g. D455 stereo in Isaac Sim.

Usage: python3 export_traj.py out_dir [--seeds 5] [--seed0 3000] [--slip nominal] [--hz 30]
Each file traj_<seed>_<slip>.npz holds t, x, y, yaw (rad; robot frame origin on the ground under base_link),
vl, vr, wz_true at --hz, and the leg ends (log index at --hz, kind).
"""
import argparse
import os

import numpy as np

import drift


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--slip", default="nominal")
    ap.add_argument("--hz", type=float, default=30.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for lv in a.slip.split(","):
        for s in range(a.seed0, a.seed0 + a.seeds):
            log, marks = drift.drive(s, lv)
            step = max(1, int(round(1.0 / (a.hz * 0.01))))
            keep = np.arange(0, len(log), step)
            ends = np.array([[int(np.searchsorted(keep, i)), k == "dump"] for i, k, _ in marks], int)
            np.savez(os.path.join(a.out, f"traj_{s}_{lv}.npz"), t=log[keep, 0], x=log[keep, 1], y=log[keep, 2],
                     yaw=log[keep, 3], vl=log[keep, 4], vr=log[keep, 5], wz=log[keep, 6], leg_end_idx=ends[:, 0],
                     leg_end_is_dump=ends[:, 1])
            print(s, lv, len(keep), "poses,", f"{log[-1, 0]:.1f} s")


if __name__ == "__main__":
    main()
