#!/usr/bin/env python3
"""Speed + decision-quality benchmark for the Cosmos 3 Edge rover agent.

Builds 20 arena situations with the agent's own status report (dig spot reached, full bucket
at the stand point, blocked by a boulder, refused dump, ...), each paired with a camera frame
from a logged run, and checks the model's action against what a sensible controller would
accept. Run it against a running sim/cosmos_server.py (any --quant):

    python3 sim/cosmos_bench.py [--budget 0] [--trouble-budget 128] [--repeats 2] [--url ...]

--budget is the reasoning budget for routine steps and --trouble-budget the one after a
refused / stopped / timed-out action, as in the agent (cosmos.sh --think-tokens).

Prints the pass rate, parse failures and seconds per decision. Needs ROS (the agent builds
the status text) but no sim; it uses its own ROS domain so a running sim isn't disturbed.
"""
import argparse
import base64
import glob
import json
import math
import os
import re
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '77'
import cv2  # noqa: E402
import rclpy  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cosmos_agent as C  # noqa: E402


def in_exc(a):
    return a[0] == 'go_to' and a[1][0] <= 2.5


def to_berm(a):     # heading for the berm box / its stand points
    return a[0] == 'go_to' and 3.6 <= a[1][0] <= 6.5 and 0.3 <= a[1][1] <= 1.5


def is_(*names):
    return lambda a: a[0] in names


def turn(lo, hi):
    return lambda a: a[0] == 'turn' and lo <= a[1][0] <= hi


def back(a):
    return a[0] == 'drive' and a[1][0] < 0


def facing_box(a):   # from the stand point facing +y: turn to +x, or move along +x / end facing +x
    return to_berm(a) and (a[1][0] >= 4.2 or (len(a[1]) > 2 and abs(a[1][2]) <= 30))


def any_of(*tests):
    return lambda a: any(t(a) for t in tests)


B = ('boulder', 'boulder 34 cm')
# name, pose (x, y, heading deg), bucket full, digs, dumps, time left s, last result, obstacles, hazard, accept
SCENARIOS = [
    ('start facing the wall', (0.52, 1.00, 180), False, 0, 0, 900, 'none yet (run just started)', [], 'wall',
     any_of(in_exc, is_('turn'))),   # a dig here would reach the duct and is refused
    ('at the dig spot', (1.20, 1.00, 0), False, 0, 0, 840, 'arrived at (1.20, 1.00), heading 0 deg', [], None,
     is_('dig')),
    ('at the 4th dig spot', (1.70, 1.50, 90), False, 3, 3, 400, 'arrived at (1.70, 1.50), heading 90 deg', [], None,
     is_('dig')),
    ('empty in the obstacle zone', (3.00, 2.00, 0), False, 1, 1, 600, 'done, heading now 0 deg', [], None,
     in_exc),
    ('full at the stand point', (4.05, 0.90, 0), True, 1, 0, 700, 'arrived at (4.05, 0.90), heading 0 deg', [], None,
     is_('dump')),
    ('full at the 2nd stand point', (4.55, 0.90, 2), True, 2, 1, 600, 'arrived at (4.55, 0.90), heading 2 deg', [],
     None, is_('dump')),
    ('just dug', (1.20, 1.00, 0), True, 1, 0, 800, 'dug 10,000 cm3 at (1.20, 1.00); bucket FULL', [], 'dip',
     to_berm),
    ('full mid-way', (3.00, 1.00, 0), True, 1, 0, 750, 'arrived at (3.00, 1.00), heading 0 deg', [], None,
     to_berm),
    ('full short of the box', (2.40, 0.95, -4), True, 1, 0, 760, 'arrived at (2.40, 0.95), heading -4 deg', [], None,
     to_berm),
    ('empty after a dump', (4.10, 0.90, 0), False, 1, 1, 650, 'dumped 10,000 cm3 at (4.75, 0.90) by SLAM', [],
     'raised', in_exc),
    ('dig refused outside the zone', (3.20, 1.00, 180), False, 1, 1, 600,
     'REFUSED: digging is only allowed in the excavation/starting zone (x < 2.5); you are in obstacle', [], None,
     in_exc),
    ('dump refused short of the box', (3.00, 0.90, 0), True, 2, 1, 500,
     'REFUSED: the pile would land at (3.65, 0.90), outside the construction zone; go_to (4.55, 0.90) and face +x '
     'first', [], None, to_berm),
    ('boulder ahead on the way', (3.60, 1.00, 0), True, 1, 0, 700, 'STOPPED after 1.10 m at (3.60, 1.00): raised '
     'obstacle 0.55 m ahead; 0.46 m short of the goal', [(B[0], B[1], 4.10, 1.00)], 'raised',
     any_of(to_berm, is_('turn'), back)),
    ('empty far up the excavation zone', (1.00, 3.50, 90), False, 2, 2, 450, 'arrived at (1.00, 3.50), heading 90 deg',
     [], None, any_of(is_('dig'), in_exc)),
    ('full at the stand point facing +y', (4.05, 0.90, 90), True, 1, 0, 700, 'arrived at (4.05, 0.90), heading 90 deg',
     [], None, any_of(turn(-120, -60), facing_box)),
    ('full and stuck', (3.20, 1.80, 45), True, 1, 0, 650, 'TIMED OUT at (3.20, 1.80), heading 45 deg (stuck? try '
     'another direction)', [('pillar', 'pillar', 3.44, 2.50)], 'raised',
     any_of(to_berm, is_('turn'), back)),
    ('full, 40 s left', (3.50, 1.00, 0), True, 3, 2, 40, 'arrived at (3.50, 1.00), heading 0 deg', [], None,
     to_berm),
    ('empty beside the dig spot', (2.00, 0.90, 180), False, 1, 1, 550, 'arrived at (2.00, 0.90), heading 180 deg', [],
     None, any_of(is_('dig'), in_exc)),
    ('full at the 3rd stand point', (5.05, 0.90, 1), True, 3, 2, 420, 'arrived at (5.05, 0.90), heading 1 deg',
     [], None, any_of(is_('dump'), to_berm)),
    ('empty at the far end', (6.00, 0.90, 0), False, 3, 3, 380, 'dumped 10,000 cm3 at (6.65, 0.90) by SLAM', [], 'wall',
     in_exc),
]
HAZARDS = {None: [], 'wall': [(0.6, 0.0, 'high'), (0.6, 0.3, 'high'), (0.6, -0.3, 'high')],
           'raised': [(0.55, 0.05, 'high')], 'dip': [(0.5, 0.0, 'low')]}


def frames():
    """Logged camera frames with their true pose, to pair each scenario with a similar view."""
    out = []
    for log in glob.glob(os.path.expanduser('~/.ros/cosmos_runs/2*/log.jsonl')):
        for line in open(log):
            d = json.loads(line)
            m = re.search(r'x=([-\d.]+) y=([-\d.]+) heading=([-\d.]+)', d['status'])
            img = os.path.join(os.path.dirname(log), f'step_{d["step"]:03d}.jpg')
            if m and os.path.exists(img):
                out.append((float(m[1]), float(m[2]), float(m[3]), img))
    return out


def build(node, sc):
    name, (x, y, hd), full, digs, dumps, left, last, obstacles, haz, _ = sc
    node.pose = (x, y, math.radians(hd))
    node.load = C.BUCKET if full else 0.0
    node.n_digs, node.n_dumps, node.cycles = digs, dumps, dumps
    node.slam_berm = dumps * 0.0085
    node.t0 = node.now() - (node.run_seconds - left)
    node.last_result = last
    node.history = [('...', 'earlier steps')] if digs else []
    node.obstacles = obstacles
    node.T_arena_map = __import__('numpy').eye(4)
    return node.status(HAZARDS[haz])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='http://127.0.0.1:8000/v1/chat/completions')
    ap.add_argument('--budget', type=int, default=0)
    ap.add_argument('--trouble-budget', type=int, default=128)
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'out_dir:=/tmp/cosmos_bench_agent',   # keep ~/.ros/cosmos_runs clean
                     '-p', f'think_tokens:={a.budget}', '-p', f'trouble_think_tokens:={a.trouble_budget}'])
    node = C.CosmosAgent()
    pics = frames()
    rows, t_all = [], []
    for sc in SCENARIOS:
        status = build(node, sc)
        x, y, hd = sc[1]
        img = min(pics, key=lambda p: math.hypot(p[0] - x, p[1] - y) + abs(C.wrap(math.radians(p[2] - hd))))[3]
        jpg = base64.b64encode(cv2.imencode('.jpg', cv2.imread(img, cv2.IMREAD_GRAYSCALE))[1]).decode()
        node.url = a.url
        for _ in range(a.repeats):
            t = time.time()
            answer, reasoning = node.ask_model(status, jpg, node.budget_for(node.last_result))   # as the agent does
            dt = time.time() - t
            t_all.append(dt)
            act = C.parse_action(answer) or C.parse_action(reasoning)
            ok = act is not None and sc[-1](act)
            rows.append((sc[0], act, ok, dt))
            print(f'{"PASS" if ok else "FAIL"}  {dt:5.1f} s  {sc[0]:34s} {act}', flush=True)
    n = len(rows)
    passed = sum(r[2] for r in rows)
    fails = sum(r[1] is None for r in rows)
    print(f'\n== budget {a.budget} (trouble {a.trouble_budget}): {passed}/{n} pass ({100 * passed / n:.0f}%), {fails} unparsed, '
          f'{sum(t_all) / n:.2f} s per decision (median {sorted(t_all)[n // 2]:.2f})')
    if a.out:
        with open(a.out, 'w') as f:
            json.dump({'budget': a.budget, 'pass': passed, 'n': n, 'seconds': sum(t_all) / n,
                       'rows': [[r[0], r[1], r[2], r[3]] for r in rows]}, f)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
