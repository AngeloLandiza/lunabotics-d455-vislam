"""Threshold sweeps and a speed gate, computed from approach.py records (no new scans).

Decision rule (Wald's sequential test, Annals of Mathematical Statistics 16(2):117-186, 1945): keep adding evidence
until it crosses an upper bound (accept "obstacle") or a lower bound (accept "clear"); Wald's bounds on the
likelihood ratio are A = (1 - beta) / alpha and B = beta / (1 - alpha). Our per-ray log-odds steps (+0.85, -0.4) are
not calibrated likelihood ratios, so Wald's error guarantees do not carry over (Inferred). This script therefore
measures the error rates directly for a sweep of confirmation thresholds.

The robot starts from rest and changes speed at ACCEL / DECEL; its speed at each frame follows from the speeds
commanded before it. Policies, scored on every hazard (boulder, crater, column) whose disk crosses the robot's corridor:
- constant speed v: v commanded from the start;
- speed gate: v_fast commanded while nothing is unresolved within d_stop(v_fast) ahead, else v_slow. Unresolved means
  unobserved corridor ground, or a candidate with at least one net supporting observation (UNSURE) that has not yet
  reached the threshold. A confirmed hazard does not slow the gate: it goes to the planner, which steers around it.
A hazard is missed when it is first confirmed closer than the stopping distance at the robot's speed in that frame,
or never confirmed before the robot reaches it. A phantom stop is a frame in which a candidate that is no real
obstacle (nor a wall, nor within NEAR of a real obstacle's edge) reaches the threshold inside the stopping distance.

Usage: python3 speed_gate.py results_dir [more_results_dirs ...]
One directory: thresholds are tuned on the lower half of its seeds and tested on the upper half. Several: tuned on
the first directory, tested on the others. Output goes to the first directory.
"""
import json
import os
import sys

import numpy as np

import params as pr
from params import v

T_LAT = 0.3        # s, sensing + processing + command latency (Assumed)
DECEL = 0.5        # m/s^2, braking on BP-1 (Assumed; not measured)
ACCEL = 0.5        # m/s^2, speeding up (Assumed, same as braking)
MARGIN = 0.10      # m kept between the stopped robot and the hazard edge (Assumed)
SPEEDS = [0.25, 0.5, 0.75, 1.0, 1.25]
V_SLOW = 0.25
GATE_FAST = [0.75, 1.0, 1.25]
TUNE_SPEED = 1.0   # thresholds are picked for this driving speed
PHANTOM_LIMIT = 0.5                      # phantom stops per 10 m allowed when picking a threshold (Assumed)
UNSURE = v(pr.DETECT, "l_occ")           # one net supporting observation makes a candidate "unresolved"
NEAR = 2 * v(pr.DETECT, "match_margin")  # a candidate this close to a real obstacle's edge is a displaced view of it
SWEEP = {"spin16": [1.7, 2.55, 3.4, 5.1, 6.8, 8.5, 10.2, 12.75, 17.0],
         "l1": [0.85, 1.7, 2.55, 3.4, 5.1, 6.8, 8.5, 10.2, 12.75, 17.0],
         "d455": [0.85, 1.7, 2.55, 3.4, 5.1, 6.8, 8.5, 10.2]}
LABEL = {"spin16": "16-beam LiDAR", "l1": "Unitree L1", "d455": "D455", "fused": "LiDAR or D455"}
LIDAR, SENSORS = "spin16", ("spin16", "d455")     # set by load() from the records


def set_sensors(runs):
    """The LiDAR in the records (spin16 or l1) and the sensors present."""
    global LIDAR, SENSORS
    names = {s for s, _ in runs}
    lidars = sorted(names - {"d455"})
    assert len(lidars) <= 1, f"records mix LiDARs {lidars}"
    LIDAR = lidars[0] if lidars else None
    SENSORS = tuple(s for s in (LIDAR, "d455") if s in names)
    return SENSORS + (("fused",) if len(SENSORS) == 2 else ())
KINDS = (("craters", ("crater",)), ("boulders", ("boulder", "column")), ("all", ("crater", "boulder", "column")))


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


def speed_profile(targets, ds):
    """Speed at each frame, from rest; targets[i] is the speed commanded after frame i."""
    vel = np.zeros(len(targets))
    for i in range(len(targets) - 1):
        t = targets[i]
        if t >= vel[i]:
            vel[i + 1] = min(t, np.sqrt(vel[i] ** 2 + 2 * ACCEL * ds))
        else:
            vel[i + 1] = max(t, np.sqrt(max(vel[i] ** 2 - 2 * DECEL * ds, 0.0)))
    return vel


class Run:
    """One approach by one sensor, with per-frame records ready for any threshold."""

    def __init__(self, trial, run, meta, every):
        self.front, self.half_w, self.ds = meta["FRONT"], meta["HALF_W"], meta["DS"]
        self.s = np.array([f["s"] for f in run["frames"]])
        self.unseen = np.array([np.inf if f["first_unseen"] is None else f["first_unseen"] for f in run["frames"]])
        self.obst = trial["obstacles"]
        self.frames = []
        for f in run["frames"]:
            dets = []
            for d in f["dets"]:
                if d[7] == "fa" and any(np.hypot(d[1] - x, d[2] - y) < r + NEAR for _, x, y, r in every):
                    d = d[:7] + ["near", d[8]]
                dets.append(d)
            self.frames.append(dets)

    def confirm_frame(self, k, thr):
        """First frame index at which obstacle k reaches thr, or None."""
        for i, dets in enumerate(self.frames):
            for d in dets:
                if d[7] == "obstacle" and k in d[8] and d[3] >= thr:
                    return i
        return None

    def ahead(self, d, i, look):
        gap = d[5] - (self.s[i] + self.front)
        return d[6] < self.half_w + 0.25 and 0 < gap <= look

    def phantom(self, i, thr, look):
        return any(d[7] == "fa" and d[3] >= thr and self.ahead(d, i, look) for d in self.frames[i])

    def unresolved(self, i, look, thr):
        """Reasons to slow down in frame i (an empty set: nothing unresolved)."""
        why = {d[7] for d in self.frames[i] if d[7] != "wall" and UNSURE <= d[3] < thr and self.ahead(d, i, look)}
        if self.unseen[i] <= look:
            why.add("unseen")
        return why


def clearance(run, o, i):
    return o["along_edge"] - (run.s[i] + run.front)


def drive(groups, thr, v_fast, gate):
    """Drive every approach once. groups = one list of Run per approach (one sensor, or both for "fused").
    Returns per-hazard records, phantom stops per 10 m, mean speed, and the share of distance driven slow by reason."""
    recs, phantom, metres, time_s = [], 0, 0.0, 0.0
    slow = {"unseen": 0.0, "candidate": 0.0, "any": 0.0}
    for group in groups:
        r0 = group[0]
        n, ds = len(r0.s), r0.ds
        if n < 2:
            continue
        if gate:
            look = d_stop(v_fast) + ds
            why = [set().union(*(r.unresolved(i, look, t) for r, t in zip(group, thr))) for i in range(n)]
            targets = np.array([V_SLOW if w else v_fast for w in why])
            for w in why[:-1]:
                slow["any"] += ds * bool(w)
                slow["unseen"] += ds * ("unseen" in w)
                slow["candidate"] += ds * bool(w - {"unseen"})
        else:
            targets = np.full(n, v_fast)
        vel = speed_profile(targets, ds)
        for o in r0.obst:
            idx = [i for i in (r.confirm_frame(o["k"], t) for r, t in zip(group, thr)) if i is not None]
            first = min(idx) if idx else None
            ok = first is not None and clearance(r0, o, first) >= d_stop(vel[first])
            recs.append({"kind": o["kind"], "missed": not ok, "first": first,
                         "clear_at_confirm": None if first is None else clearance(r0, o, first)})
        phantom += sum(any(r.phantom(i, t, d_stop(vel[i]) + ds) for r, t in zip(group, thr)) for i in range(n))
        metres += (n - 1) * ds
        time_s += float(np.sum(ds / (0.5 * (vel[:-1] + vel[1:]))))
    return recs, 10 * phantom / metres, metres / time_s, {k: x / metres for k, x in slow.items()}


def summarize(recs):
    res = {}
    for name, kinds in KINDS:
        sel = [r["missed"] for r in recs if r["kind"] in kinds]
        p, lo, hi = wilson(sum(sel), len(sel))
        res[name] = {"missed": int(sum(sel)), "n": len(sel), "rate": p, "lo": lo, "hi": hi}
    return res


def arena_obstacles(trial):
    """Every obstacle in the arena (kind, x, y, radius). Older records lack them, so the arena is rebuilt from its seed."""
    if "arena_obstacles" in trial:
        return trial["arena_obstacles"]
    from arena import Arena
    a = Arena(np.random.default_rng(trial["seed"]))
    for o in trial["obstacles"]:
        assert abs(a.obstacles[o["k"]].radius - o["radius"]) < 1e-9, f"seed {trial['seed']}: rebuilt arena differs"
    return [[o.kind, o.x, o.y, o.radius] for o in a.obstacles]


def load(paths):
    meta, runs, by_dir = None, {}, []
    for path in paths:
        data = json.load(open(os.path.join(path, "approaches.json")))
        meta = meta or data["meta"]
        seeds = []
        for t in data["trials"]:
            every = arena_obstacles(t)
            seeds.append(t["seed"])
            for r in t["runs"]:
                runs.setdefault((r["sensor"], r["dust"]), []).append((t["seed"], Run(t, r, data["meta"], every)))
        by_dir.append(sorted(seeds))
    set_sensors(runs)
    return meta, runs, by_dir


def groups(runs, sensor, dust, seeds):
    if sensor == "fused":
        a = dict(runs[(LIDAR, dust)])
        b = dict(runs[("d455", dust)])
        return [[a[s], b[s]] for s in seeds if s in a and s in b]
    d = dict(runs[(sensor, dust)])
    return [[d[s]] for s in seeds if s in d]


def thr_for(sensor, chosen):
    return [chosen[LIDAR], chosen["d455"]] if sensor == "fused" else [chosen[sensor]]


def main():
    dirs = sys.argv[1:] or ["results_approach"]
    meta, runs, by_dir = load(dirs)
    if len(by_dir) == 1:
        seeds = by_dir[0]
        tune, test = seeds[: len(seeds) // 2], seeds[len(seeds) // 2:]
    else:
        tune, test = by_dir[0], sorted(s for b in by_dir[1:] for s in b)
    report = {"meta": meta, "dirs": dirs, "tune_seeds": [tune[0], tune[-1], len(tune)],
              "test_seeds": [test[0], test[-1], len(test)],
              "assumed": {"T_LAT": T_LAT, "DECEL": DECEL, "ACCEL": ACCEL, "MARGIN": MARGIN, "V_SLOW": V_SLOW,
                          "UNSURE": UNSURE, "NEAR": NEAR, "TUNE_SPEED": TUNE_SPEED, "PHANTOM_LIMIT": PHANTOM_LIMIT},
              "sweep": [], "constant": [], "gate": [], "confirm_clearance": []}

    # 1. Threshold sweep on the tuning seeds: missed hazards and phantom stops at constant speeds, per sensor and air.
    for sensor in SENSORS:
        for dust in (False, True):
            if (sensor, dust) not in runs:
                continue
            g = groups(runs, sensor, dust, tune)
            for thr in SWEEP[sensor]:
                for vel in (0.75, 1.0, 1.25):
                    rec, ph, _, _ = drive(g, [thr], vel, gate=False)
                    s = summarize(rec)
                    warn = [r["clear_at_confirm"] for r in rec if r["first"] is not None and r["clear_at_confirm"] > 0]
                    report["sweep"].append({"sensor": sensor, "dust": dust, "thr": thr, "speed": vel, "phantom_per_10m": ph,
                                            "warn_p10": float(np.percentile(warn, 10)) if warn else None,
                                            "never": sum(r["first"] is None or r["clear_at_confirm"] <= 0 for r in rec),
                                            **{k: s[k]["rate"] for k in s}})
    # Pick per sensor: the lowest threshold whose phantom stops stay within the limit at TUNE_SPEED in dust.
    chosen = {}
    tune_dust = any(k[1] for k in runs)          # clear-only records are tuned in clear air
    for sensor in SENSORS:
        ok = [r["thr"] for r in report["sweep"] if r["sensor"] == sensor and r["dust"] == tune_dust
              and r["speed"] == TUNE_SPEED and r["phantom_per_10m"] <= PHANTOM_LIMIT]
        chosen[sensor] = min(ok) if ok else max(SWEEP[sensor])
    report["chosen"] = chosen

    # 2. Test seeds, chosen thresholds: constant speeds, the gate, and how far ahead hazards are confirmed.
    for sensor in SENSORS + (("fused",) if len(SENSORS) == 2 else ()):
        thr = thr_for(sensor, chosen)
        for dust in (False, True):
            if (LIDAR if sensor == "fused" else sensor, dust) not in runs:
                continue
            g = groups(runs, sensor, dust, test)
            for vel in SPEEDS:
                rec, ph, mean_v, _ = drive(g, thr, vel, gate=False)
                report["constant"].append({"sensor": sensor, "dust": dust, "speed": vel, "d_stop": d_stop(vel),
                                           "mean_speed": mean_v, "phantom_per_10m": ph, **summarize(rec)})
            for vf in GATE_FAST:
                rec, ph, mean_v, slow = drive(g, thr, vf, gate=True)
                report["gate"].append({"sensor": sensor, "dust": dust, "v_fast": vf, "v_slow": V_SLOW, "mean_speed": mean_v,
                                       "phantom_per_10m": ph, "slow_share": slow, **summarize(rec)})
            rec, _, _, _ = drive(g, thr, 0.25, gate=False)
            for name, kinds in KINDS[:2]:
                sel = [r for r in rec if r["kind"] in kinds]
                never = [r for r in sel if r["first"] is None or r["clear_at_confirm"] <= 0]
                moving = [r["clear_at_confirm"] for r in sel if r not in never and r["first"] > 0]
                report["confirm_clearance"].append({
                    "sensor": sensor, "dust": dust, "kind": name, "n": len(sel),
                    "at_start": sum(r not in never and r["first"] == 0 for r in sel),
                    "while_driving": len(moving), "never": len(never),
                    "median_m": float(np.median(moving)) if moving else None,
                    "p10_m": float(np.percentile(moving, 10)) if moving else None})
    out = dirs[0]
    with open(os.path.join(out, "speed_gate.json"), "w") as f:
        json.dump(report, f, indent=1, default=float)
    write_tables(report, os.path.join(out, "speed_gate.md"))
    plot(report, out)
    print(open(os.path.join(out, "speed_gate.md")).read())


def pct(x):
    return "n/a" if x is None or not np.isfinite(x) else f"{100 * x:.0f}%"


def write_tables(rep, path):
    m, a = rep["meta"], rep["assumed"]
    tu, te = rep["tune_seeds"], rep["test_seeds"]
    L = [f"# Speed gate and threshold sweep ({tu[2] + te[2]} approaches)",
         "",
         f"Thresholds tuned on seeds {tu[0]}-{tu[1]} ({tu[2]} arenas): "
         + ", ".join(f"{LABEL[k]} {t}" for k, t in rep["chosen"].items())
         + f" (the lowest with at most {a['PHANTOM_LIMIT']} phantom stops per 10 m at "
         f"{a['TUNE_SPEED']} m/s in dust, or in clear air when the records have no dust). Everything below is on seeds {te[0]}-{te[1]} ({te[2]} arenas), not used for tuning.",
         f"The robot starts from rest and speeds up or brakes at {ACCEL} / {DECEL} m/s^2. Stopping distance "
         f"d_stop(v) = v x {T_LAT} s + v^2 / (2 x {DECEL} m/s^2) + {MARGIN} m. Candidates within {NEAR:.2f} m of a real "
         f"obstacle's edge count as views of it, not phantoms. All of these numbers are Assumed. Frames every {m['DS']} m; "
         f"corridor half-width {m['HALF_W']} m; dust plume {m['PLUME_AHEAD']} m ahead of the start.",
         "", "## How far ahead of the front each hazard is first confirmed", "",
         "Hazards already confirmed before the robot moves are counted apart: their distance says where the arena put "
         "them, not how far the sensor reaches.", "",
         "| Sensor | Air | Kind | Hazards | Confirmed before moving | Confirmed while driving | Median clearance | "
         "10th percentile | Never |", "|" + " --- |" * 9]
    metres = lambda x: "n/a" if x is None else f"{x:.2f} m"  # noqa: E731
    for r in rep["confirm_clearance"]:
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['kind']} | {r['n']} | {r['at_start']} | "
                 f"{r['while_driving']} | {metres(r['median_m'])} | {metres(r['p10_m'])} | {r['never']} |")
    L += ["", "## Constant speed: missed hazards and phantom stops", "",
          "| Sensor | Air | Speed | Stopping distance | Mean speed | Missed craters | Missed boulders | "
          "Phantom stops per 10 m |", "|" + " --- |" * 8]
    for r in rep["constant"]:
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['speed']} m/s | {r['d_stop']:.2f} m | "
                 f"{r['mean_speed']:.2f} m/s | {r['craters']['missed']}/{r['craters']['n']} ({pct(r['craters']['rate'])}) | "
                 f"{r['boulders']['missed']}/{r['boulders']['n']} ({pct(r['boulders']['rate'])}) | {r['phantom_per_10m']:.2f} |")
    L += ["", f"## Speed gate: fast while nothing is unresolved within the fast stopping distance, else {V_SLOW} m/s", "",
          "| Sensor | Air | Fast speed | Mean speed | Distance driven slow (unobserved ground / unconfirmed candidate) | "
          "Missed hazards (95% interval) | Phantom stops per 10 m |", "|" + " --- |" * 7]
    for r in rep["gate"]:
        h, s = r["all"], r["slow_share"]
        L.append(f"| {LABEL[r['sensor']]} | {'dust' if r['dust'] else 'clear'} | {r['v_fast']} m/s | {r['mean_speed']:.2f} m/s | "
                 f"{pct(s['any'])} ({pct(s['unseen'])} / {pct(s['candidate'])}) | "
                 f"{h['missed']}/{h['n']} ({pct(h['rate'])}, {pct(h['lo'])}-{pct(h['hi'])}) | {r['phantom_per_10m']:.2f} |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


COLOR = {"spin16": "#2a78d6", "l1": "#2a78d6", "d455": "#1baf7a", "fused": "#6250d6"}     # validated: CVD and normal-vision separation pass


def plot(rep, d):
    """sweep_tradeoff.png (tuning seeds) and speed_misses.png (test seeds)."""
    from figures import plt, INK, INK2, MUTED, SURF
    a = rep["assumed"]
    air = ((False, "--", "clear air"), (True, "-", "dust plume ahead"))
    # 1. Threshold sweep: phantom stops (top) and missed hazards (bottom) at the tuning speed, per sensor.
    fig, axes = plt.subplots(2, len(SENSORS), figsize=(5 * len(SENSORS), 6.4), sharex="col", squeeze=False)
    for j, sensor in enumerate(SENSORS):
        for dust, ls, lab in air:
            rows = sorted((r for r in rep["sweep"] if r["sensor"] == sensor and r["dust"] == dust
                           and r["speed"] == a["TUNE_SPEED"]), key=lambda r: r["thr"])
            x = [r["thr"] for r in rows]
            axes[0, j].plot(x, [r["phantom_per_10m"] for r in rows], ls, color=COLOR[sensor], lw=2, marker="o", ms=4,
                            mec=SURF, mew=1, label=lab)
            axes[1, j].plot(x, [100 * r["all"] for r in rows], ls, color=COLOR[sensor], lw=2, marker="o", ms=4,
                            mec=SURF, mew=1, label=lab)
        axes[0, j].axhline(a["PHANTOM_LIMIT"], color=MUTED, lw=0.8, ls=":")
        axes[0, j].text(max(SWEEP[sensor]), a["PHANTOM_LIMIT"], f"limit {a['PHANTOM_LIMIT']:g}", ha="right", va="bottom",
                        fontsize=8, color=INK2)
        for ax in axes[:, j]:
            ax.axvline(rep["chosen"][sensor], color=INK, lw=1.0, alpha=0.6)
        axes[0, j].text(rep["chosen"][sensor], axes[0, j].get_ylim()[1], f" chosen {rep['chosen'][sensor]:g}", ha="left",
                        va="top", fontsize=8, color=INK)
        axes[0, j].set_title(LABEL[sensor], loc="left", color=INK)
        axes[1, j].set_xlabel("Confirmation threshold (summed log-odds evidence)")
        axes[1, j].set_ylim(bottom=-0.5)
        axes[0, j].legend(frameon=False, fontsize=8.5, loc="center right")
    axes[0, 0].set_ylabel(f"Phantom stops per 10 m\nat {a['TUNE_SPEED']:g} m/s")
    axes[1, 0].set_ylabel(f"Hazards missed (%)\nat {a['TUNE_SPEED']:g} m/s")
    fig.suptitle(f"Picking the confirmation threshold, tuning seeds {rep['tune_seeds'][0]}-{rep['tune_seeds'][1]}",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "sweep_tradeoff.png"), dpi=150)
    plt.close(fig)

    # 2. Missed hazards against mean speed, per sensor: constant speeds (lines) and the speed gate (diamonds).
    shown = SENSORS + (("fused",) if len(SENSORS) == 2 else ())
    fig, axes = plt.subplots(1, len(shown), figsize=(3.7 * len(shown), 4.2), sharey=True, squeeze=False)
    axes = axes[0]
    for ax, sensor in zip(axes, shown):
        for dust, ls, lab in air:
            rows = [r for r in rep["constant"] if r["sensor"] == sensor and r["dust"] == dust]
            if not rows:
                continue
            x = [r["mean_speed"] for r in rows]
            y = np.array([[100 * r["all"][k] for k in ("rate", "lo", "hi")] for r in rows])
            ax.plot(x, y[:, 0], ls, color=COLOR[sensor], lw=2, marker="o", ms=4, mec=SURF, mew=1,
                    label=f"constant speed, {lab}")
            if dust:
                ax.fill_between(x, y[:, 1], y[:, 2], color=COLOR[sensor], alpha=0.12, lw=0)
                for r, xi, yi in zip(rows, x, y[:, 0]):
                    ax.annotate(f"{r['speed']:g}", (xi, yi), xytext=(0, -13), textcoords="offset points", ha="center",
                                fontsize=7.5, color=INK2)
            g = [r for r in rep["gate"] if r["sensor"] == sensor and r["dust"] == dust]
            ax.plot([r["mean_speed"] for r in g], [100 * r["all"]["rate"] for r in g], "D", ms=6, ls="",
                    mfc=COLOR[sensor] if dust else SURF, mec=COLOR[sensor], mew=1.4, label=f"speed gate, {lab}")
        ax.set_title(LABEL[sensor], loc="left", color=INK)
        ax.set_xlabel("Mean speed from rest (m/s)")
    axes[0].set_ylabel("Hazards missed (%)")
    h, lab = axes[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=4, frameon=False, fontsize=8.5)
    fig.suptitle(f"Missed hazards against speed, held-out seeds {rep['test_seeds'][0]}-{rep['test_seeds'][1]} "
                 f"(labels: commanded speed; band: 95% interval in dust)", x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(os.path.join(d, "speed_misses.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
