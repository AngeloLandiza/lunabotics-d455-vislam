"""Threshold sweeps and a speed gate, computed from approach.py records (no new scans).

Decision rule (Wald's sequential test, Annals of Mathematical Statistics 16(2):117-186, 1945): keep adding evidence
until it crosses an upper bound (accept "obstacle") or a lower bound (accept "clear"); Wald's bounds on the
likelihood ratio are A = (1 - beta) / alpha and B = beta / (1 - alpha). Our per-ray log-odds steps (+0.85, -0.4) are
not calibrated likelihood ratios, so Wald's error guarantees do not carry over (Inferred). This script therefore
measures the error rates directly for a sweep of confirmation thresholds.

Policies, scored on every hazard (boulder, crater, column) whose disk crosses the robot's corridor:
- constant speed v: a hazard is missed when it is first confirmed (pooled evidence >= threshold) closer than the
  stopping distance d_stop(v), or never confirmed before the robot reaches it;
- speed gate: drive v_fast while nothing is unresolved within d_stop(v_fast) ahead (no candidate with evidence
  >= one observation, no unobserved corridor cell), else v_slow. Missed hazards are scored at the speed the robot
  had when the hazard was confirmed.
A phantom stop is a frame in which a candidate that matches no real obstacle (and is not a wall) reaches the
threshold inside the stopping distance ahead.

Usage: python3 speed_gate.py [results_approach]
"""
import json
import os
import sys

import numpy as np

import params as pr
from params import v

T_LAT = 0.3        # s, sensing + processing + command latency (Assumed)
DECEL = 0.5        # m/s^2, braking on BP-1 (Assumed; not measured)
MARGIN = 0.10      # m kept between the stopped robot and the hazard edge (Assumed)
SPEEDS = [0.25, 0.5, 0.75, 1.0, 1.25]
V_SLOW = 0.25
GATE_FAST = [0.75, 1.0, 1.25]
UNSURE = v(pr.DETECT, "l_occ")          # one net supporting observation makes a candidate "unresolved"
SWEEP = {"spin16": [1.7, 2.55, 3.4, 5.1, 6.8, 8.5, 10.2, 12.75, 17.0],
         "d455": [0.85, 1.7, 2.55, 3.4, 5.1, 6.8, 8.5, 10.2]}
LABEL = {"spin16": "16-beam LiDAR", "d455": "D455", "fused": "LiDAR or D455"}


def d_stop(vel):
    return vel * T_LAT + vel ** 2 / (2 * DECEL) + MARGIN


def wilson(k, n, z=1.96):
    if n == 0:
        return np.nan, np.nan, np.nan
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(c - h, 0.0), min(c + h, 1.0)


class Run:
    """One approach by one sensor, with per-frame arrays ready for any threshold."""

    def __init__(self, trial, run, meta):
        self.meta = meta
        self.front, self.half_w, self.ds = meta["FRONT"], meta["HALF_W"], meta["DS"]
        self.s = np.array([f["s"] for f in run["frames"]])
        self.unseen = np.array([np.inf if f["first_unseen"] is None else f["first_unseen"] for f in run["frames"]])
        self.obst = trial["obstacles"]
        self.frames = run["frames"]
        self.length = trial["length"]

    def confirm_frame(self, k, thr):
        """First frame index at which obstacle k reaches thr, or None."""
        for i, f in enumerate(self.frames):
            for d in f["dets"]:
                if d[7] == "obstacle" and k in d[8] and d[3] >= thr:
                    return i
        return None

    def ahead(self, d, i, look):
        gap = d[5] - (self.s[i] + self.front)
        return d[6] < self.half_w + 0.25 and 0 < gap <= look

    def phantom(self, i, thr, look):
        return any(d[7] == "fa" and d[3] >= thr and self.ahead(d, i, look) for d in self.frames[i]["dets"])

    def unresolved(self, i, look):
        return (self.unseen[i] <= look
                or any(d[7] != "wall" and d[3] >= UNSURE and self.ahead(d, i, look) for d in self.frames[i]["dets"]))


def clearance(run, o, i):
    return o["along_edge"] - (run.s[i] + run.front)


def score_constant(runs, thr, vel):
    """Per hazard: missed or not; plus phantom stops per 10 m. runs = one list of Run per approach (one sensor, or both for "fused")."""
    out, phantom, metres = [], 0, 0.0
    for group in runs:
        r0 = group[0]
        look = d_stop(vel) + r0.ds
        for o in r0.obst:
            best = None
            for r, t in zip(group, thr):
                i = r.confirm_frame(o["k"], t)
                if i is not None:
                    c = clearance(r, o, i)
                    best = c if best is None else max(best, c)
            out.append({"kind": o["kind"], "missed": best is None or best < d_stop(vel), "clear_at_confirm": best})
        n = len(r0.s)
        phantom += sum(any(r.phantom(i, t, look) for r, t in zip(group, thr)) for i in range(n))
        metres += n * r0.ds
    return out, 10 * phantom / metres


def score_gate(runs, thr, v_fast, v_slow=V_SLOW):
    out, phantom, metres, time_s = [], 0, 0.0, 0.0
    for group in runs:
        r0 = group[0]
        look = d_stop(v_fast) + r0.ds
        n = len(r0.s)
        vel = np.array([v_slow if any(r.unresolved(i, look) for r in group) else v_fast for i in range(n)])
        prev = np.r_[v_slow, vel[:-1]]                 # speed while driving into frame i
        for o in r0.obst:
            ok = False
            first = None
            for r, t in zip(group, thr):
                i = r.confirm_frame(o["k"], t)
                if i is not None:
                    first = i if first is None else min(first, i)
            if first is not None:
                ok = clearance(r0, o, first) >= d_stop(prev[first])
            out.append({"kind": o["kind"], "missed": not ok})
        phantom += sum(any(r.phantom(i, t, d_stop(vel[i]) + r0.ds) for r, t in zip(group, thr)) for i in range(n))
        metres += n * r0.ds
        time_s += float(np.sum(r0.ds / vel))
    return out, 10 * phantom / metres, metres / time_s


def summarize(recs):
    res = {}
    for name, kinds in (("craters", ("crater",)), ("boulders", ("boulder", "column")), ("all", ("crater", "boulder", "column"))):
        sel = [r["missed"] for r in recs if r["kind"] in kinds]
        p, lo, hi = wilson(sum(sel), len(sel))
        res[name] = {"missed": int(sum(sel)), "n": len(sel), "rate": p, "lo": lo, "hi": hi}
    return res


def load(path):
    data = json.load(open(path))
    meta, trials = data["meta"], data["trials"]
    runs = {}
    for t in trials:
        for r in t["runs"]:
            runs.setdefault((r["sensor"], r["dust"]), []).append((t["seed"], Run(t, r, meta)))
    return meta, runs


def groups(runs, sensor, dust, seeds):
    if sensor == "fused":
        a = dict(runs[("spin16", dust)])
        b = dict(runs[("d455", dust)])
        return [[a[s], b[s]] for s in seeds if s in a and s in b]
    d = dict(runs[(sensor, dust)])
    return [[d[s]] for s in seeds if s in d]


def thr_for(sensor, chosen):
    return [chosen["spin16"], chosen["d455"]] if sensor == "fused" else [chosen[sensor]]


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "results_approach"
    meta, runs = load(os.path.join(d, "approaches.json"))
    seeds = sorted({s for v_ in runs.values() for s, _ in v_})
    tune, test = seeds[: len(seeds) // 2], seeds[len(seeds) // 2:]
    report = {"meta": meta, "tune_seeds": [tune[0], tune[-1]], "test_seeds": [test[0], test[-1]],
              "assumed": {"T_LAT": T_LAT, "DECEL": DECEL, "MARGIN": MARGIN, "V_SLOW": V_SLOW, "UNSURE": UNSURE},
              "sweep": [], "constant": [], "gate": [], "confirm_clearance": []}

    # 1. Threshold sweep on the tuning half: missed hazards vs phantom stops at 0.75 m/s, per sensor and air.
    for sensor in ("spin16", "d455"):
        for dust in (False, True):
            g = groups(runs, sensor, dust, tune)
            for thr in SWEEP[sensor]:
                for vel in (0.5, 0.75, 1.0):
                    rec, ph = score_constant(g, [thr], vel)
                    s = summarize(rec)
                    report["sweep"].append({"sensor": sensor, "dust": dust, "thr": thr, "speed": vel, "phantom_per_10m": ph,
                                            **{k: s[k]["rate"] for k in s}})
    # Pick per sensor: the lowest threshold whose phantom stops stay <= 0.5 per 10 m at 0.75 m/s in dust (tuning half).
    chosen = {}
    for sensor in ("spin16", "d455"):
        ok = [r["thr"] for r in report["sweep"] if r["sensor"] == sensor and r["dust"] and r["speed"] == 0.75
              and r["phantom_per_10m"] <= 0.5]
        chosen[sensor] = min(ok) if ok else max(SWEEP[sensor])
    report["chosen"] = chosen

    # 2. Test half, chosen thresholds: constant speeds, the gate, and how far ahead hazards are confirmed.
    for sensor in ("spin16", "d455", "fused"):
        thr = thr_for(sensor, chosen)
        for dust in (False, True):
            g = groups(runs, sensor, dust, test)
            for vel in SPEEDS:
                rec, ph = score_constant(g, thr, vel)
                report["constant"].append({"sensor": sensor, "dust": dust, "speed": vel, "d_stop": d_stop(vel),
                                           "phantom_per_10m": ph, **summarize(rec)})
            for vf in GATE_FAST:
                rec, ph, mean_v = score_gate(g, thr, vf)
                report["gate"].append({"sensor": sensor, "dust": dust, "v_fast": vf, "v_slow": V_SLOW, "mean_speed": mean_v,
                                       "phantom_per_10m": ph, **summarize(rec)})
            rec, _ = score_constant(g, thr, 0.0)
            for kind in ("crater", "boulder"):
                c = [r["clear_at_confirm"] for r in rec if r["kind"] == kind]
                got = [x for x in c if x is not None and x > 0]
                report["confirm_clearance"].append({"sensor": sensor, "dust": dust, "kind": kind, "n": len(c),
                                                    "never_confirmed": len(c) - len(got),
                                                    "median_m": float(np.median(got)) if got else None,
                                                    "p10_m": float(np.percentile(got, 10)) if got else None})
    with open(os.path.join(d, "speed_gate.json"), "w") as f:
        json.dump(report, f, indent=1, default=float)
    write_tables(report, os.path.join(d, "speed_gate.md"))
    print(open(os.path.join(d, "speed_gate.md")).read())


def pct(x):
    return "n/a" if x is None or not np.isfinite(x) else f"{100 * x:.0f}%"


def write_tables(rep, path):
    m = rep["meta"]
    L = [f"# Speed gate and threshold sweep ({m['n_arenas']} approaches, seeds {m['seed0']}-{m['seed0'] + m['n_arenas'] - 1})",
         "",
         f"Thresholds chosen on seeds {rep['tune_seeds'][0]}-{rep['tune_seeds'][1]}: 16-beam LiDAR {rep['chosen']['spin16']}, "
         f"D455 {rep['chosen']['d455']} (lowest with at most 0.5 phantom stops per 10 m at 0.75 m/s in dust). "
         f"Everything below is on the other half, seeds {rep['test_seeds'][0]}-{rep['test_seeds'][1]}.",
         f"Stopping distance d_stop(v) = v x {T_LAT} s + v^2 / (2 x {DECEL} m/s^2) + {MARGIN} m (all Assumed). "
         f"Frames every {m['DS']} m; corridor half-width {m['HALF_W']} m; dust plume {m['PLUME_AHEAD']} m ahead of the start.",
         "", "## How far ahead of the front each hazard is first confirmed", "",
         "| Sensor | Air | Kind | Hazards | Never confirmed | Median clearance | 10th percentile |", "|" + " --- |" * 7]
    metres = lambda x: "n/a" if x is None else f"{x:.2f} m"  # noqa: E731
    for r in rep["confirm_clearance"]:
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['kind']} | {r['n']} | {r['never_confirmed']} | "
                 f"{metres(r['median_m'])} | {metres(r['p10_m'])} |")
    L += ["", "## Constant speed: missed hazards and phantom stops", "",
          "| Sensor | Air | Speed | Stopping distance | Missed craters | Missed boulders | Phantom stops per 10 m |", "|" + " --- |" * 7]
    for r in rep["constant"]:
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['speed']} m/s | {r['d_stop']:.2f} m | "
                 f"{r['craters']['missed']}/{r['craters']['n']} ({pct(r['craters']['rate'])}) | "
                 f"{r['boulders']['missed']}/{r['boulders']['n']} ({pct(r['boulders']['rate'])}) | {r['phantom_per_10m']:.2f} |")
    L += ["", f"## Speed gate: fast when nothing is unresolved within the fast stopping distance, else {V_SLOW} m/s", "",
          "| Sensor | Air | Fast speed | Mean speed | Missed hazards (95% interval) | Phantom stops per 10 m |", "|" + " --- |" * 6]
    for r in rep["gate"]:
        a = r["all"]
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['v_fast']} m/s | {r['mean_speed']:.2f} m/s | "
                 f"{a['missed']}/{a['n']} ({pct(a['rate'])}, {pct(a['lo'])}-{pct(a['hi'])}) | {r['phantom_per_10m']:.2f} |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
