"""Score self-mask, calibration-error and rut cases from approach.py records with fixed thresholds.

Each case folder is one approach.py run on the same seeds. Thresholds come from speed_gate.json in a reference
folder (speed_gate.py tunes them on the lower half of its seeds), and every case is scored on the seeds at or above
--test-from, so no case is scored on the seeds its thresholds were tuned on. Per case, sensor and air:
- body points: body returns that reach the map per frame (the mask's leaks);
- lost: non-body returns the mask removed per frame (ground it threw away);
- blind ahead: in each frame, the gap from the front edge to the nearest ground return in the corridor (this scan only);
- footprint phantoms: share of frames with a false candidate at or above the threshold within 0.1 m of the robot's
  footprint (the planner would see the robot standing in an obstacle);
- left on path: false candidates at or above the threshold on the driven path at the last frame (map clutter a later
  drive meets);
- missed hazards and phantom stops ahead at constant speed, and the speed gate's mean speed, as in speed_gate.py.

Usage: python3 selfmask_eval.py out_dir ref_dir case_dir [case_dir ...] [--test-from SEED]
"""
import argparse
import json
import os

import numpy as np

import params as pr
import speed_gate as sg
from params import v

PAD = 0.10       # m around the footprint for footprint phantoms (Assumed)


def case_name(meta):
    if meta.get("label"):
        return meta["label"]
    parts = [meta.get("lidar", "spin16"), f"body {meta.get('body', 'none')}"]
    if meta.get("mask_err"):
        parts.append(f"tilt {meta['mask_err']:g} deg")
    if meta.get("ruts"):
        parts.append(f"ruts {100 * meta['ruts']:g} cm")
    return ", ".join(parts)


def raw_metrics(path, test):
    """Per (sensor, dust): body points, lost rays and blind-ahead gaps from the frame records."""
    data = json.load(open(os.path.join(path, "approaches.json")))
    out = {}
    for t in data["trials"]:
        if t["seed"] not in test:
            continue
        for r in t["runs"]:
            m = out.setdefault((r["sensor"], r["dust"]), {"body": [], "lost": [], "blind": [], "no_ground": 0})
            for f in r["frames"]:
                m["body"].append(f.get("body_pts", 0))
                m["lost"].append(f.get("lost", 0))
                if f.get("blind_ahead") is None:
                    m["no_ground"] += 1
                else:
                    m["blind"].append(f["blind_ahead"])
    return data["meta"], out


def phantom_metrics(group, thr, meta):
    """Footprint phantoms (share of frames) and false candidates left on the driven path (per approach)."""
    back, front, hw = meta.get("BACK", 0.45), meta["FRONT"], meta["HALF_W"]
    frames, in_fp, left = 0, 0, []
    for runs in group:
        r = runs[0]
        for i, dets in enumerate(r.frames):
            frames += 1
            s = r.s[i]
            in_fp += any(d[7] == "fa" and d[3] >= thr and s - back - PAD <= d[5] <= s + front + PAD and d[6] < hw + PAD
                         for d in dets)
        s_end = r.s[-1]
        left.append(sum(d[7] == "fa" and d[3] >= thr and -back <= d[5] <= s_end + front and d[6] < hw
                        for d in r.frames[-1]))
    return in_fp / max(frames, 1), float(np.mean(left)) if left else np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("ref")
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--test-from", type=int, default=3024)
    a = ap.parse_args()
    ref = json.load(open(os.path.join(a.ref, "speed_gate.json")))
    chosen = dict(ref["chosen"])
    chosen.setdefault("d455", v(pr.DETECT, "l_decide_d455"))
    rows = []
    for d in a.cases:
        meta, runs, by_dir = sg.load([d])
        test = [s for s in by_dir[0] if s >= a.test_from]
        _, raw = raw_metrics(d, set(test))
        for sensor in sg.SENSORS:
            if sensor not in chosen:
                print(f"{d}: no threshold for {sensor}, skipped")
                continue
            for dust in (False, True):
                if (sensor, dust) not in runs:
                    continue
                g = sg.groups(runs, sensor, dust, test)
                thr = chosen[sensor]
                row = {"case": case_name(meta), "dir": d, "sensor": sensor, "dust": dust, "n": len(g), "thr": thr}
                m = raw[(sensor, dust)]
                row.update(body_per_frame=float(np.mean(m["body"])), lost_per_frame=float(np.mean(m["lost"])),
                           blind_median=float(np.median(m["blind"])) if m["blind"] else None,
                           blind_p90=float(np.percentile(m["blind"], 90)) if m["blind"] else None,
                           frames_no_ground=m["no_ground"] / max(len(m["body"]), 1))
                row["footprint_phantom_share"], row["left_on_path"] = phantom_metrics(g, thr, meta)
                for vel in (0.75, 1.0):
                    rec, ph, _, _ = sg.drive(g, [thr], vel, gate=False)
                    s = sg.summarize(rec)
                    row[f"v{vel:g}"] = {"missed": s["all"]["missed"], "n": s["all"]["n"], "rate": s["all"]["rate"],
                                        "lo": s["all"]["lo"], "hi": s["all"]["hi"], "phantom_per_10m": ph}
                rec, ph, mean_v, slow = sg.drive(g, [thr], 1.0, gate=True)
                s = sg.summarize(rec)
                row["gate1"] = {"mean_speed": mean_v, "missed": s["all"]["missed"], "n": s["all"]["n"],
                                "phantom_per_10m": ph, "slow_share": slow["any"]}
                rows.append(row)
    os.makedirs(a.out, exist_ok=True)
    rep = {"ref": a.ref, "chosen": chosen, "test_from": a.test_from, "pad": PAD, "rows": rows}
    with open(os.path.join(a.out, "selfmask_eval.json"), "w") as f:
        json.dump(rep, f, indent=1, default=float)
    write_table(rep, os.path.join(a.out, "selfmask_eval.md"))
    print(open(os.path.join(a.out, "selfmask_eval.md")).read())


def write_table(rep, path):
    m = lambda x: "n/a" if x is None else f"{x:.2f} m"  # noqa: E731
    L = ["# Self-mask, calibration error and ruts", "",
         f"Thresholds from {rep['ref']}/speed_gate.json: " + ", ".join(f"{sg.LABEL.get(k, k)} {t}" for k, t in
                                                                       rep["chosen"].items())
         + f". Scored on seeds from {rep['test_from']} up, which the thresholds were not tuned on. Footprint phantoms "
         f"count false candidates at or above the threshold within {rep['pad']} m of the robot's footprint.", "",
         "| Case | Sensor | Air | Approaches | Body points per frame | Ground removed per frame | Blind ahead (median / 90th pct) "
         "| Frames with a phantom on the robot | Phantoms left on the path | Missed at 1.0 m/s | Phantom stops per 10 m at 1.0 m/s "
         "| Gate mean speed (missed) |", "|" + " --- |" * 12]
    for r in rep["rows"]:
        c, g = r["v1"], r["gate1"]
        L.append(f"| {r['case']} | {sg.LABEL.get(r['sensor'], r['sensor'])} | {'dust' if r['dust'] else 'clear'} | {r['n']} "
                 f"| {r['body_per_frame']:.1f} | {r['lost_per_frame']:.1f} | {m(r['blind_median'])} / {m(r['blind_p90'])} "
                 f"| {100 * r['footprint_phantom_share']:.0f}% | {r['left_on_path']:.2f} "
                 f"| {c['missed']}/{c['n']} ({sg.pct(c['rate'])}) | {c['phantom_per_10m']:.2f} "
                 f"| {g['mean_speed']:.2f} m/s ({g['missed']}/{g['n']}) |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
