"""Regression gate: every saved arena.sh --autonomy bag through the localizer and hazard detector replays, one pass/fail
table. No live run until it is green (Angelo, 2026-10-13).

Per bag (RUNS below: bag, gyro bias from the live log, arena seed):
  localizer  replay_localizer.replay for each named variant; pass: p95 < P95_MAX and max < MAX_MAX (cm vs truth)
  detector   replay_detector est_tilt (the bag's live /autonomy/pose, L1 placed with the IMU roll / pitch, starting at the
             first pose); pass: false hazards <= FALSE_MAX and no false hazard in the berm corridor (CORRIDOR)
Variant names are looked up in replay_localizer.VARIANTS and in its env families (SWEEP, SLIPSWEEP, ODO), or given
inline as name='{"slip": false}' (JSON overrides on BASE).

Usage: python3 sim/autonomy_diag/regress.py [variant ...] [--runs 10,11,...] [--no-detector] [--out file.json]
       (default variant: now). Exit status 1 when anything fails.
Runs go in parallel (JOBS, default 4), one process per bag: never run two readers on the same bag at once (rosbag2
decompresses each .zstd to a .db3 beside it, and two readers corrupt it: 'database disk image is malformed').
"""
import importlib
import json
import math
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# run -> (bag, gyro bias deg/s as the live arena_localizer logged it, arena seed). 12-14: the bias passed to their
# replays at the time (their live logs are gone; the bags' own standstill windows give 0.106-0.107).
RUNS = {10: ('~/bags/run10', 0.108, 0), 11: ('~/bags/run11', 0.108, 0), 12: ('~/bags/run12', 0.107, 0),
        13: ('~/bags/run13', 0.107, 0), 14: ('~/bags/run14', 0.107, 0), 15: ('~/bags/run15', 0.118, 0),
        16: ('~/bags/run16', 0.111, 0)}
P95_MAX, MAX_MAX = 20.0, 40.0          # cm
FALSE_MAX = 3
CORRIDOR = (2.5, 3.7, 0.6, 1.6)        # x0, x1, y0, y1: the drive from the start zone to the berm's west end


def all_variants():
    """replay_localizer's variants from its default set and every env family."""
    out = {}
    for flag in (None, 'SWEEP', 'SLIPSWEEP', 'ODO'):
        saved = {f: os.environ.pop(f, None) for f in ('SWEEP', 'SLIPSWEEP', 'ODO')}
        if flag:
            os.environ[flag] = '1'
        try:
            import replay_localizer
            rl = importlib.reload(replay_localizer)
            out.update(rl.VARIANTS)
        finally:
            os.environ.pop(flag, None) if flag else None
            os.environ.update({f: v for f, v in saved.items() if v is not None})
    return out


def resolve(names):
    import replay_localizer as rl
    known = all_variants()
    res = {}
    for n in names:
        if '=' in n:
            k, js = n.split('=', 1)
            res[k] = {**rl.BASE, **json.loads(js)}
        elif n in known:
            res[n] = known[n]
        else:
            sys.exit(f'unknown variant {n!r}; known: {", ".join(sorted(known))}')
    return res


def check_run(args):
    run, variants, detector = args
    import bag_io
    import replay_localizer as rl
    import replay_detector as rd
    import autonomy_core as ac
    bag, bias, seed = RUNS[run]
    bag = os.path.expanduser(bag)
    if not os.path.isdir(bag):
        return {'run': run, 'missing': True}
    d = bag_io.load(bag)
    out = {'run': run, 'bias': bias, 'seed': seed, 'loc': {}}
    if len(d['est']):
        out['live'] = rl.score(d['est'], d['gt'])
    for name, ov in variants.items():
        track, counts = rl.replay(d, dict(ov), bias=bias)
        s = rl.score(track, d['gt'])
        s['pass'] = s['p95_cm'] < P95_MAX and s['max_cm'] < MAX_MAX
        s['slip'] = (counts.get('slip episodes'), counts.get('slip s'))
        out['loc'][name] = s
    if detector and len(d['est']):
        B, C = ac.A27.layout(seed)[0], ac.A27.layout(seed)[1]
        truth = [('boulder', o.x, o.y, o.diameter / 2) for o in B] + [('crater', o.x, o.y, o.diameter / 2) for o in C]
        objs, info = rd.replay(d, d['est'], tilt=rd.imu_tilt(d), t0=d['est'][0, 0])
        s = rd.score(objs, truth)
        x0, x1, y0, y1 = CORRIDOR
        corr = [(h['x'], h['y'], h['r']) for h in s['hazards'] if not h['match'] and x0 <= h['x'] <= x1 and y0 <= h['y'] <= y1]
        out['det'] = {'n': s['n'], 'matched': s['matched'], 'false': s['false'], 'missed': len(s['missed']),
                      'corridor': corr, 'pass': s['false'] <= FALSE_MAX and not corr,
                      'false_list': [(h['x'], h['y'], h['r']) for h in s['hazards'] if not h['match']]}
    return out


def main():
    argv = sys.argv[1:]
    runs, out_path, detector, names = list(RUNS), None, True, []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--runs':
            runs = [int(x) for x in argv[i + 1].split(',')]
            i += 2
        elif a == '--out':
            out_path = argv[i + 1]
            i += 2
        elif a == '--no-detector':
            detector = False
            i += 1
        else:
            names.append(a)
            i += 1
    variants = resolve(names or ['now'])
    with Pool(int(os.environ.get('JOBS', min(len(runs), 4)))) as p:
        rows = p.map(check_run, [(r, variants, detector) for r in runs], chunksize=1)
    ok_all = True
    vw = max(len(n) for n in variants) + 1
    print(f'gate: localizer p95 < {P95_MAX:.0f} cm and max < {MAX_MAX:.0f} cm; detector false <= {FALSE_MAX}, '
          f'corridor x {CORRIDOR[0]}-{CORRIDOR[1]} y {CORRIDOR[2]}-{CORRIDOR[3]} clear')
    print(f'{"run":>4} {"variant":<{vw}} {"median":>7} {"p95":>7} {"max":>7} {"slip":>10}  result')
    for r in rows:
        if r.get('missing'):
            print(f'{r["run"]:4d} bag missing: {RUNS[r["run"]][0]}  FAIL')
            ok_all = False
            continue
        if 'live' in r:
            lv = r['live']
            print(f'{r["run"]:4d} {"(live)":<{vw}} {lv["median_cm"]:7.1f} {lv["p95_cm"]:7.1f} {lv["max_cm"]:7.1f} {"":>10}')
        for name, s in r['loc'].items():
            ok_all &= s['pass']
            sl = '-' if s['slip'][0] is None else f'{s["slip"][0]}/{s["slip"][1]}s'
            print(f'{r["run"]:4d} {name:<{vw}} {s["median_cm"]:7.1f} {s["p95_cm"]:7.1f} {s["max_cm"]:7.1f} {sl:>10}  '
                  f'{"pass" if s["pass"] else "FAIL"}')
        if 'det' in r:
            dd = r['det']
            ok_all &= dd['pass']
            print(f'{r["run"]:4d} {"detector est_tilt":<{vw}} hazards {dd["n"]}, matched {dd["matched"]}, false {dd["false"]}, '
                  f'missed {dd["missed"]}, corridor {dd["corridor"] or "clear"}  {"pass" if dd["pass"] else "FAIL"}')
    print('GATE', 'GREEN' if ok_all else 'RED')
    if out_path:
        json.dump({'gate': 'green' if ok_all else 'red', 'variants': variants, 'rows': rows}, open(out_path, 'w'),
                  indent=1, default=str)
    sys.exit(0 if ok_all else 1)


if __name__ == '__main__':
    main()
