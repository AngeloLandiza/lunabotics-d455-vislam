"""Monte Carlo test of boulder and crater detection: 3 sensors x clear/dust x 3 detectors.

Single frames give detection probability vs range. Four-frame approach sequences (robot drives 0.75 m)
show what accumulating views buys. Every input number is in params.py with its status.
Usage: python3 evaluate.py [n_trials] [out_dir] [first_seed]
"""
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

from arena import Arena
from detect import Grid, accumulate, detect, score
from dust import Plume
from sensors import Lidar, D455Depth

SENSORS = ["spin16", "rosette70", "d455"]
DETECTORS = ["baseline", "carve", "logodds"]


def make_sensor(name):
    return D455Depth() if name == "d455" else Lidar(name)


def trial(seed):
    rng = np.random.default_rng(seed)
    arena = Arena(rng)
    x, y, yaw = rng.uniform(0.3, 1.5), rng.uniform(0.8, 4.2), rng.normal(0, 15)
    plume_xy = (x + 0.8 * np.cos(np.radians(yaw)), y + 0.8 * np.sin(np.radians(yaw)))
    grid = Grid(arena.L, arena.W, 0.05)
    single, seq = [], []
    for sname in SENSORS:
        sensor = make_sensor(sname)
        for dust_on in (False, True):
            dust = Plume(*plume_xy) if dust_on else None
            scans = []
            for k in range(4):
                adv = 0.25 * k
                robot = (x + adv * np.cos(np.radians(yaw)), y + adv * np.sin(np.radians(yaw)), yaw)
                scans.append(sensor.scan(arena, robot, rng, dust=dust))
            L1 = accumulate(scans[:1], grid)
            L4 = accumulate(scans, grid)
            for det in DETECTORS:
                d1, _ = detect(L1, grid, det, scans[:1])
                rec, fa, wall = score(d1, arena, L1, grid, scans[0]["origin"])
                for r in rec:
                    single.append(dict(r, sensor=sname, dust=dust_on, detector=det, seed=seed))
                single.append({"fa": fa, "wall": wall, "sensor": sname, "dust": dust_on, "detector": det, "seed": seed})
                d4, _ = detect(L4, grid, det, scans)
                rec4, fa4, wall4 = score(d4, arena, L4, grid, scans[-1]["origin"])
                for r in rec4:
                    seq.append(dict(r, sensor=sname, dust=dust_on, detector=det, seed=seed))
                seq.append({"fa": fa4, "wall": wall4, "sensor": sname, "dust": dust_on, "detector": det, "seed": seed})
    return single, seq


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    out = sys.argv[2] if len(sys.argv) > 2 else "results"
    seed0 = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
    os.makedirs(out, exist_ok=True)
    t0 = time.time()
    with Pool(4) as p:
        res = p.map(trial, range(seed0, seed0 + n))
    single = [r for a, _ in res for r in a]
    seq = [r for _, b in res for r in b]
    with open(os.path.join(out, "records.json"), "w") as f:
        json.dump({"n_trials": n, "seed0": seed0, "seconds": time.time() - t0, "single": single, "sequence": seq}, f)
    print(f"{n} trials in {time.time() - t0:.0f} s -> {out}/records.json")


if __name__ == "__main__":
    main()
