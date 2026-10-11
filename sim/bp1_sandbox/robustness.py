"""Floor roughness, floor waves and attitude error: how much they cost the detector at the speeds that passed.

Each argument is a results folder written by approach.py for one case on the same seeds; its dust runs, if it has
any, are scored as a separate case. Every case is scored with the thresholds in params.py and the same rules as speed_gate.py: missed hazards at constant speed from
rest, phantom stops ahead (false candidates confirmed inside the stopping distance), and the false alarms left in the
map (false candidates above the threshold anywhere in the arena, per frame).

Usage: python3 robustness.py out_dir case_dir [case_dir ...]
"""
import json
import os
import sys

import numpy as np

import params as pr
from params import v
from speed_gate import LABEL, KINDS, groups, load, drive, summarize, thr_for

SPEEDS = (0.75, 1.0)
SENSORS = ("spin16", "d455", "fused")
CHOSEN = {"spin16": v(pr.DETECT, "l_decide_lidar"), "d455": v(pr.DETECT, "l_decide_d455")}


def case_label(meta):
    parts = []
    if abs(meta.get("surface_rms", 0.01) - 0.01) > 1e-9:
        parts.append(f"roughness {100 * meta['surface_rms']:g} cm")
    if meta.get("undulation", 0) > 0:
        parts.append(f"floor waves {100 * meta['undulation']:g} cm")
    if meta.get("pitch_err", 0) > 0:
        parts.append(f"pitch error {meta['pitch_err']:g} deg")
    label = ", ".join(parts) or "baseline (1 cm roughness, flat, exact attitude)"
    if meta.get("ground_fit") == "local":
        label += f", local ground fit ({100 * meta.get('ground_sigma', 0.3):g} cm window)"
    return label


def map_false_alarms(group, thr):
    """False candidates (class fa) at or above the threshold anywhere in the arena: mean per frame, by kind."""
    n, pos, neg = 0, 0, 0
    for runs in group:
        for r, t in zip(runs, thr):
            for dets in r.frames:
                n += 1
                pos += sum(d[7] == "fa" and d[0] == "pos" and d[3] >= t for d in dets)
                neg += sum(d[7] == "fa" and d[0] == "neg" and d[3] >= t for d in dets)
    return pos / max(n, 1), neg / max(n, 1)


def main():
    out, dirs = sys.argv[1], sys.argv[2:]
    os.makedirs(out, exist_ok=True)
    rows = []
    for d in dirs:
        meta, runs, by_dir = load([d])
        seeds = by_dir[0]
        for dust in (False, True):
            if ("spin16", dust) in runs:
                rows += score_case(case_label(meta) + (", dust plume" if dust else ""), d, runs, dust, seeds)
    with open(os.path.join(out, "robustness.json"), "w") as f:
        json.dump({"thresholds": CHOSEN, "speeds": SPEEDS, "rows": rows}, f, indent=1, default=float)
    write_md(rows, os.path.join(out, "robustness.md"))
    try:
        plot(rows, out)
    except ImportError as e:   # matplotlib missing: the tables still stand, and plot() can rerun from robustness.json
        print(f"no figure ({e})")


def score_case(label, d, runs, dust, seeds):
    rows = []
    for sensor in SENSORS:
        g = groups(runs, sensor, dust, seeds)
        thr = thr_for(sensor, CHOSEN)
        fa_pos, fa_neg = map_false_alarms(g, thr)
        row = {"case": label, "dir": d, "dust": dust, "sensor": sensor, "n_arenas": len(seeds),
               "map_fa_pos": fa_pos, "map_fa_neg": fa_neg}
        for vel in SPEEDS:
            rec, ph, _, _ = drive(g, thr, vel, gate=False)
            s = summarize(rec)
            row[f"v{vel}"] = {"phantom_per_10m": ph, **{k: s[k] for k, _ in KINDS}}
        rows.append(row)
        print(f"{label:48s} {LABEL[sensor]:14s} map FA/frame boulder {fa_pos:.2f} crater {fa_neg:.2f} | "
              + " | ".join(f"{vel} m/s: craters {row[f'v{vel}']['craters']['missed']}/{row[f'v{vel}']['craters']['n']}, "
                           f"boulders {row[f'v{vel}']['boulders']['missed']}/{row[f'v{vel}']['boulders']['n']}, "
                           f"phantom {row[f'v{vel}']['phantom_per_10m']:.2f}" for vel in SPEEDS), flush=True)
    return rows


def write_md(rows, path):
    L = ["# Floor roughness, floor waves and attitude error", "",
         f"Clear air, thresholds from params.py (LiDAR {CHOSEN['spin16']}, D455 {CHOSEN['d455']}), constant speed from rest; "
         "same arenas, starts and sensor noise in every case.", "",
         "| Case | Sensor | False boulders in map per frame | False craters in map per frame | "
         + " | ".join(f"Missed at {vel} m/s (craters, boulders) | Phantom stops per 10 m at {vel} m/s" for vel in SPEEDS) + " |",
         "|" + " --- |" * (4 + 2 * len(SPEEDS))]
    for r in rows:
        cells = [r["case"], LABEL[r["sensor"]], f"{r['map_fa_pos']:.2f}", f"{r['map_fa_neg']:.2f}"]
        for vel in SPEEDS:
            x = r[f"v{vel}"]
            cells += [f"{x['craters']['missed']}/{x['craters']['n']}, {x['boulders']['missed']}/{x['boulders']['n']}",
                      f"{x['phantom_per_10m']:.2f}"]
        L.append("| " + " | ".join(cells) + " |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


def plot(rows, out):
    from figures import plt, INK, SURF
    from speed_gate import COLOR
    cases = list(dict.fromkeys(r["case"] for r in rows))
    fig, axes = plt.subplots(1, 3, figsize=(12, 0.5 * len(cases) + 2.2), sharey=True)
    ypos = np.arange(len(cases))[::-1]
    panels = (("False craters in the map per frame", lambda r: r["map_fa_neg"]),
              ("Phantom stops per 10 m at 0.75 m/s", lambda r: r["v0.75"]["phantom_per_10m"]),
              ("Craters missed at 1.0 m/s (%)", lambda r: 100 * r["v1.0"]["craters"]["rate"]))
    for ax, (title, val) in zip(axes, panels):
        for k, sensor in enumerate(("spin16", "d455")):
            xs = [val(next(r for r in rows if r["case"] == c and r["sensor"] == sensor)) for c in cases]
            ax.barh(ypos + (0.2 if k == 0 else -0.2), xs, height=0.38, color=COLOR[sensor], edgecolor=SURF, lw=2,
                    label=LABEL[sensor])
        ax.set_title(title, loc="left", color=INK, fontsize=10)
    axes[0].set_yticks(ypos, cases, fontsize=8.5)
    axes[0].legend(frameon=False, fontsize=8.5, loc="lower right")
    fig.suptitle("Floor roughness, floor waves and attitude error (clear air, same arenas in every case)",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "robustness.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
