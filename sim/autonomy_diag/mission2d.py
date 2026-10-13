"""The live mission (autonomy_core.MissionRunner, the goals mission_runner.py draws) on Angelo's arena in 2D, over many
seeds: how often the mission meets tight dig spots, unreachable dumps and the berm pole gates, before any sensor or
localizer error. A planning check, not a physics one.

Per seed: the arena.sh layout (arena.layout(seed)), its start pose (arena.start_pose(seed)), the goals mission_runner.py
plans for that seed. The robot is arena2d.SkidSteer at the given slip level (planner bench model: wheel slip and ICR
expansion, no pivot slide), driven by MissionRunner on the TRUE pose; hazards (true discs) are revealed once within
`reveal` m of base_link, as the L1 detector's range gate does. Counted per seed:
  tight     dig goals the mission moved before trying (turn clearance < dig_open)
  dumps     dump goals reached / skipped; no_route   plans that found no route
  moved     goals moved after failed plans or the pivot budget; pinned  'cannot turn (pinned)' backs
  pole_cm   closest approach of the footprint to a berm range pole (sim/arena.py POLES), cm; gate: < 5 cm
  contact   s with the footprint inside a hazard, the column, the berm box or a wall (arena2d.clearance < 0)
  end       done / STUCK reason / timeout

Usage: python3 sim/autonomy_diag/mission2d.py [n_seeds=50] [first_seed=0] [level=nominal] [out.json]
"""
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'lunabotics_slam', 'scripts'))
sys.path.insert(0, os.path.join(REPO, 'sim'))
import autonomy_core as ac          # noqa: E402
A = ac.A27                          # sim/arena.py (the name "arena" is the BP-1 sandbox one)

a2 = ac.a2
REVEAL = 4.0
T_MAX = 900.0                       # s of mission time per seed
FP = ac.v(a2.PARAMS, 'footprint')   # x0, x1, y0, y1 about base_link


def pole_gap(pose):
    """Closest distance (m) from the footprint rectangle to any berm pole's surface."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    best = 9.0
    for px, py in A.POLES:
        lx, ly = c * (px - pose[0]) + s * (py - pose[1]), -s * (px - pose[0]) + c * (py - pose[1])
        dx, dy = max(FP[0] - lx, 0.0, lx - FP[1]), max(FP[2] - ly, 0.0, ly - FP[3])
        best = min(best, math.hypot(dx, dy) - A.POLE_R)
    return best


def run_seed(args):
    seed, level = args
    boulders, craters = A.layout(seed)
    truth = [{'kind': 'boulder', 'x': o.x, 'y': o.y, 'r': o.diameter / 2} for o in boulders] + \
            [{'kind': 'crater', 'x': o.x, 'y': o.y, 'r': o.diameter / 2} for o in craters]
    goals = ac.mission_goals(np.random.default_rng(seed))
    start = A.start_pose(seed)
    robot = a2.SkidSteer(start, level, np.random.default_rng([seed, 7]))
    run = ac.MissionRunner(goals, ac.RobotModel(mult=a2.SLIP_LEVELS[level][2]))
    w_true = ac.make_world(truth)
    known, events, contact, t, k = [], [], 0.0, 0.0, 0
    cv = cw = 0.0
    gap = 9.0
    while run.state not in ('done', 'stuck') and t < T_MAX:
        if k % 5 == 0:                                       # 20 Hz control on the true pose
            for h in truth:
                if h not in known and math.hypot(h['x'] - robot.pose[0], h['y'] - robot.pose[1]) < REVEAL:
                    known.append(h)
            cv, cw, ev = run.step(robot.pose.copy(), t, known, 0.05)
            if ev:
                events.append((ev[0], round(math.hypot(*(robot.pose[:2] - np.asarray(ev[1][:2]))), 3)))
        robot.step(cv, cw, a2.SIM_DT)
        t += a2.SIM_DT
        k += 1
        if k % 10 == 0:
            gap = min(gap, pole_gap(robot.pose))
            if a2.clearance(w_true, robot.pose) < 0:
                contact += 10 * a2.SIM_DT
    log = [m for _, m in run.log]
    kinds = [kd for kd, _ in goals]
    return {'seed': seed, 'start': [round(x, 2) for x in start],
            'goals': [[kd, *[round(float(x), 2) for x in g]] for kd, g in goals],
            'digs': sum(e[0] == 'dig' for e in events), 'dumps': sum(e[0] == 'dump' for e in events),
            'n_dig': kinds.count('dig'), 'n_dump': kinds.count('dump'),
            'tight': sum('tight spot' in m for m in log),
            'dump_skipped': sum(' dump: skipped' in m for m in log),
            'dig_skipped': sum(' dig: skipped' in m for m in log),
            'no_route': sum('no route' in m for m in log),
            'moved': sum(': moved to' in m or 'moving the goal' in m for m in log),
            'pinned': sum('pinned' in m for m in log),
            'pole_cm': round(100 * gap, 1), 'contact_s': round(contact, 2), 't': round(t, 1),
            'end': run.why_stuck if run.state == 'stuck' else ('done' if run.state == 'done' else 'timeout'),
            'log': log}


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    level = sys.argv[3] if len(sys.argv) > 3 else 'nominal'
    out = sys.argv[4] if len(sys.argv) > 4 else None
    with Pool(int(os.environ.get('JOBS', 4))) as p:
        rows = p.map(run_seed, [(s, level) for s in range(first, first + n)], chunksize=1)
    print(f'{"seed":>4} {"digs":>5} {"dumps":>5} {"tight":>5} {"dmpskip":>7} {"noroute":>7} {"moved":>5} {"pinned":>6} '
          f'{"pole_cm":>7} {"contact":>7} {"t":>6}  end')
    for r in rows:
        print(f'{r["seed"]:4d} {r["digs"]:2d}/{r["n_dig"]:<2d} {r["dumps"]:2d}/{r["n_dump"]:<2d} {r["tight"]:5d} '
              f'{r["dump_skipped"]:7d} {r["no_route"]:7d} {r["moved"]:5d} {r["pinned"]:6d} {r["pole_cm"]:7.1f} '
              f'{r["contact_s"]:7.2f} {r["t"]:6.0f}  {r["end"]}')
    N = len(rows)
    goals = sum(r['n_dig'] + r['n_dump'] for r in rows)
    summ = {'seeds': N, 'level': level, 'reveal_m': REVEAL,
            'all_goals_done': sum(r['digs'] == r['n_dig'] and r['dumps'] == r['n_dump'] for r in rows),
            'digs_reached': f"{sum(r['digs'] for r in rows)}/{sum(r['n_dig'] for r in rows)}",
            'dumps_reached': f"{sum(r['dumps'] for r in rows)}/{sum(r['n_dump'] for r in rows)}",
            'seeds_with_tight_dig': sum(r['tight'] > 0 for r in rows),
            'tight_dig_goals': sum(r['tight'] for r in rows),
            'seeds_with_dump_skipped': sum(r['dump_skipped'] > 0 for r in rows),
            'seeds_pole_gate_lt5cm': sum(r['pole_cm'] < 5 for r in rows),
            'seeds_pinned': sum(r['pinned'] > 0 for r in rows),
            'seeds_contact': sum(r['contact_s'] > 0 for r in rows),
            'ends': {e: sum(r['end'] == e for r in rows) for e in sorted({r['end'] for r in rows})},
            'goals_total': goals}
    print(json.dumps(summ, indent=1))
    if out:
        json.dump({'summary': summ, 'rows': rows}, open(out, 'w'), indent=1)


if __name__ == '__main__':
    main()
