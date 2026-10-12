"""Approach run on Isaac Sim hits next to the same run on the sandbox ray marcher, scored with speed_gate.py's rules.

Usage: python3 isaac_approach.py [isaac_out/approach_seed1.npz] [seed] [n_noise]
The capture comes from isaac_capture.py with the approach poses of approach.trial(seed) (run_isaac_approach.sh makes
both). approach.trial runs unchanged twice: once with the sandbox sensors, once with sensors whose scan() returns the
Isaac hits for that pose passed through the BP-1 layer (isaac_validate.isaac_lidar_scan / isaac_d455_scan). Both get
the same random stream, so noise draws line up wherever the ideal hits agree. Output: isaac_out/approach_seed<seed>/
{sandbox,isaac}/approaches.json (speed_gate.py format) and compare.md / compare.json.
With n_noise > 0 it also repeats both runs with n_noise independent sensor-noise streams (each scan draws from
default_rng((noise, sensor, dust, frame)), identical for both sources) and writes noise.md / noise.json, because one
noise draw decides where dust echoes cluster into false alarms.
"""
import json
import os
import sys

import numpy as np

import approach
import params as pr
import speed_gate as sg
from params import v
from isaac_validate import isaac_d455_scan, isaac_lidar_scan
from sensors import D455Depth, Lidar
from telemetry import Telemetry


def frame_of(poses, robot):
    k = int(np.argmin(np.abs(poses - np.asarray(robot)).sum(1)))
    assert np.allclose(poses[k], robot, atol=1e-4), f"no Isaac capture at pose {robot}"
    return k


def noise_rng(noise, sensor, dust, k, rng):
    """The trial's own stream when noise is None, else a stream fixed by (noise, sensor, dust, frame)."""
    if noise is None:
        return rng
    return np.random.default_rng([noise, 1 if sensor == "d455" else 0, int(dust is not None), k])


def isaac_sensors(cap, noise=None):
    poses = cap["poses"]

    class IsaacLidar(Lidar):
        def scan(self, arena, robot, rng, dust=None):
            k = frame_of(poses, robot)
            return isaac_lidar_scan(cap, k, arena, self, noise_rng(noise, "spin16", dust, k, rng), dust)

    class IsaacD455(D455Depth):
        def scan(self, arena, robot, rng, dust=None):
            k = frame_of(poses, robot)
            return isaac_d455_scan(cap, k, self, noise_rng(noise, "d455", dust, k, rng), dust)

    return IsaacLidar, IsaacD455


def sandbox_sensors(cap, noise=None):
    poses = cap["poses"]

    class SbLidar(Lidar):
        def scan(self, arena, robot, rng, dust=None):
            k = frame_of(poses, robot)
            return Lidar.scan(self, arena, robot, noise_rng(noise, "spin16", dust, k, rng), dust)

    class SbD455(D455Depth):
        def scan(self, arena, robot, rng, dust=None):
            k = frame_of(poses, robot)
            return D455Depth.scan(self, arena, robot, noise_rng(noise, "d455", dust, k, rng), dust)

    return SbLidar, SbD455


def approach_poses(seed):
    """Robot poses of approach.trial(seed): the same draws for the arena and the start, then DS steps to the end."""
    from arena import Arena
    rng = np.random.default_rng(seed)
    arena = Arena(rng)
    x0, y0 = rng.uniform(0.3, 0.8), rng.uniform(0.8, 4.2)
    yaw = float(np.clip(rng.normal(0, 10), -25, 25))
    h, _ = approach.path_frame(x0, y0, yaw)
    lim = [(approach.X_END - x0) / h[0]]
    if h[1] > 1e-6:
        lim.append((arena.W - 0.5 - y0) / h[1])
    elif h[1] < -1e-6:
        lim.append((0.5 - y0) / h[1])
    n = int(max(0.0, min(lim)) // approach.DS) + 1
    return [(x0 + i * approach.DS * h[0], y0 + i * approach.DS * h[1], yaw) for i in range(n)]


def run_trial(seed, lidar_cls, d455_cls):
    approach.Lidar, approach.D455Depth = lidar_cls, d455_cls
    try:
        return approach.trial(seed)
    finally:
        approach.Lidar, approach.D455Depth = Lidar, D455Depth


def save(trial, path):
    meta = {"n_arenas": 1, "seed0": trial["seed"], "DS": approach.DS, "FRONT": approach.FRONT,
            "HALF_W": approach.HALF_W, "X_END": approach.X_END, "PLUME_AHEAD": approach.PLUME_AHEAD,
            "LOOK": approach.LOOK, "ground_fit": v(pr.DETECT, "ground_fit"),
            "det_fields": ["kind", "x", "y", "evidence", "cells", "along", "lateral", "class", "match"]}
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, "approaches.json"), "w") as f:
        json.dump({"meta": meta, "trials": [trial]}, f)


def score(path, seed):
    """Per sensor and air: when each path hazard is first confirmed, misses and phantom stops at constant speeds and
    with the gate, and the per-frame gate decisions (speed_gate.py rules and thresholds from params.py)."""
    meta, runs, _ = sg.load([path])
    out = {}
    for sensor in ("spin16", "d455"):
        thr = [sg.thr_for(sensor, {"spin16": v(pr.DETECT, "l_decide_lidar"), "d455": v(pr.DETECT, "l_decide_d455")})[0]]
        for dust in (False, True):
            g = sg.groups(runs, sensor, dust, [seed])
            r = g[0][0]
            hz = [{"k": o["k"], "kind": o["kind"], "first": r.confirm_frame(o["k"], thr[0]),
                   "clear_at_confirm": None if r.confirm_frame(o["k"], thr[0]) is None
                   else round(sg.clearance(r, o, r.confirm_frame(o["k"], thr[0])), 3)} for o in r.obst]
            pol = {}
            for vel in (0.5, 0.75, 1.0):
                rec, ph, _, _ = sg.drive(g, thr, vel, gate=False)
                pol[f"const_{vel}"] = {"missed": sum(x["missed"] for x in rec), "phantom_per_10m": round(ph, 2)}
            rec, ph, mean_v, slow = sg.drive(g, thr, 1.0, gate=True)
            pol["gate_1.0"] = {"missed": sum(x["missed"] for x in rec), "phantom_per_10m": round(ph, 2),
                               "mean_speed": round(mean_v, 3), "slow_share": round(slow["any"], 3)}
            look = sg.d_stop(1.0) + r.ds
            gate = [sorted(r.unresolved(i, look, thr[0])) for i in range(len(r.s))]
            fa = [sum(d[7] == "fa" and d[3] >= thr[0] for d in dets) for dets in r.frames]
            out[f"{sensor}_{'dust' if dust else 'clear'}"] = {"thr": thr[0], "hazards": hz, "policies": pol,
                                                            "gate_reasons": gate, "fa_confirmed_per_frame": fa}
    return out


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "poses":    # isaac_approach.py poses <seed>  ->  isaac_capture.py --poses
        print(";".join(f"{x:.6f},{y:.6f},{yaw:.6f}" for x, y, yaw in approach_poses(int(sys.argv[2]))))
        return
    cap_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join("isaac_out", "approach_seed1.npz")
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    cap = np.load(cap_path)
    base = os.path.join(os.path.dirname(cap_path), f"approach_seed{seed}")
    n_noise = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    tel = Telemetry(kind="isaac", total=2 + 2 * n_noise, args={"capture": cap_path, "seed": seed, "noise": n_noise},
                    label=f"Isaac vs sandbox approach, seed {seed}")
    t_sb = run_trial(seed, Lidar, D455Depth)
    tel.progress(1, stage="sandbox run")
    t_is = run_trial(seed, *isaac_sensors(cap))
    tel.progress(2, stage="isaac run")
    assert np.allclose(t_sb["start"], t_is["start"])
    save(t_sb, os.path.join(base, "sandbox"))
    save(t_is, os.path.join(base, "isaac"))
    res = {"sandbox": score(os.path.join(base, "sandbox"), seed), "isaac": score(os.path.join(base, "isaac"), seed)}
    with open(os.path.join(base, "compare.json"), "w") as f:
        json.dump({"seed": seed, "capture": cap_path, "start": t_sb["start"], "obstacles": t_sb["obstacles"], **res}, f,
                  indent=1, default=float)

    ds = approach.DS
    L = [f"# Approach on Isaac Sim hits vs sandbox (seed {seed})", "",
         f"Start ({t_sb['start'][0]:.2f}, {t_sb['start'][1]:.2f}), heading {t_sb['start'][2]:.1f} deg, "
         f"{len(t_sb['runs'][0]['frames'])} frames {ds} m apart; ground fit {v(pr.DETECT, 'ground_fit')}; thresholds "
         f"LiDAR {v(pr.DETECT, 'l_decide_lidar')}, D455 {v(pr.DETECT, 'l_decide_d455')}. Rules and Assumed numbers from "
         "speed_gate.py.", "",
         "Hazards in the path, first confirmed at frame (clearance from the robot's front edge, m):", "",
         "| hazard | " + " | ".join(f"{c} sandbox | {c} Isaac" for c in ("LiDAR clear", "LiDAR dust", "D455 clear", "D455 dust")) + " |",
         "| --- |" + " --- |" * 8]
    keys = ["spin16_clear", "spin16_dust", "d455_clear", "d455_dust"]
    for j, o in enumerate(t_sb["obstacles"]):
        cells = []
        for k in keys:
            for src in ("sandbox", "isaac"):
                h = res[src][k]["hazards"][j]
                cells.append("never" if h["first"] is None else f"{h['first']} ({h['clear_at_confirm']:.2f})")
        L.append(f"| {o['kind']} {2 * o['radius'] * 100:.0f} cm, {o['along_edge']:.2f} m ahead | " + " | ".join(cells) + " |")
    L += ["", "Policies (missed hazards / phantom stops per 10 m):", "",
          "| case | " + " | ".join(f"{p} sandbox | {p} Isaac" for p in ("0.5 m/s", "0.75 m/s", "1.0 m/s", "gate 1.0")) + " |",
          "| --- |" + " --- |" * 8]
    for k in keys:
        cells = []
        for p in ("const_0.5", "const_0.75", "const_1.0", "gate_1.0"):
            for src in ("sandbox", "isaac"):
                x = res[src][k]["policies"][p]
                cells.append(f"{x['missed']} / {x['phantom_per_10m']:.2f}")
        L.append(f"| {k.replace('_', ' ')} | " + " | ".join(cells) + " |")
    L += ["", "Gate decisions per frame (fast/slow at v_fast 1.0, same reasons rule as speed_gate.py):", "",
          "| case | frames agreeing | sandbox slow frames | Isaac slow frames | confirmed false alarms, sandbox / Isaac (frame-sum) |",
          "| --- | --- | --- | --- | --- |"]
    for k in keys:
        a, b = res["sandbox"][k], res["isaac"][k]
        same = sum(bool(x) == bool(y) for x, y in zip(a["gate_reasons"], b["gate_reasons"]))
        L.append(f"| {k.replace('_', ' ')} | {same}/{len(a['gate_reasons'])} | {sum(map(bool, a['gate_reasons']))} | "
                 f"{sum(map(bool, b['gate_reasons']))} | {sum(a['fa_confirmed_per_frame'])} / {sum(b['fa_confirmed_per_frame'])} |")
    md = "\n".join(L) + "\n"
    open(os.path.join(base, "compare.md"), "w").write(md)
    print(md)

    if n_noise:
        noise_study(cap, seed, base, n_noise, keys, tel)
    tel.end(lidar_dust_fa_sandbox=sum(res["sandbox"]["spin16_dust"]["fa_confirmed_per_frame"]),
            lidar_dust_fa_isaac=sum(res["isaac"]["spin16_dust"]["fa_confirmed_per_frame"]))


def noise_study(cap, seed, base, n_noise, keys, tel=None):
    """Both sources over n_noise noise streams: per case, how often each path hazard is confirmed in time at 1.0 m/s,
    phantom stops per 10 m at 1.0 m/s, and confirmed false alarms summed over frames."""
    rows = {src: {k: {"missed_1.0": [], "phantom_1.0": [], "fa_frames": [], "first": []} for k in keys}
            for src in ("sandbox", "isaac")}
    for noise in range(n_noise):
        for src, maker in (("sandbox", sandbox_sensors), ("isaac", isaac_sensors)):
            t = run_trial(seed, *maker(cap, noise))
            path = os.path.join(base, f"noise{noise}", src)
            save(t, path)
            res = score(path, seed)
            for k in keys:
                r = rows[src][k]
                r["missed_1.0"].append(res[k]["policies"]["const_1.0"]["missed"])
                r["phantom_1.0"].append(res[k]["policies"]["const_1.0"]["phantom_per_10m"])
                r["fa_frames"].append(sum(res[k]["fa_confirmed_per_frame"]))
                r["first"].append([h["first"] for h in res[k]["hazards"]])
        print(f"noise stream {noise + 1}/{n_noise} done", flush=True)
        if tel is not None:
            tel.progress(2 + 2 * (noise + 1), stage=f"noise stream {noise + 1}/{n_noise}")
    with open(os.path.join(base, "noise.json"), "w") as f:
        json.dump({"seed": seed, "n_noise": n_noise, **rows}, f, indent=1)
    L = [f"# Isaac vs sandbox over {n_noise} sensor-noise streams (seed {seed})", "",
         "Same arena, path and thresholds as compare.md; each stream gives both sources the same noise draws per frame.",
         "", "| case | missed at 1.0 m/s (sum) sandbox / Isaac | phantom stops per 10 m at 1.0 m/s, mean sandbox / Isaac "
         "| streams with any phantom stop sandbox / Isaac | confirmed false-alarm frames, mean sandbox / Isaac |",
         "| --- | --- | --- | --- | --- |"]
    for k in keys:
        a, b = rows["sandbox"][k], rows["isaac"][k]
        L.append(f"| {k.replace('_', ' ')} | {sum(a['missed_1.0'])} / {sum(b['missed_1.0'])} | "
                 f"{np.mean(a['phantom_1.0']):.2f} / {np.mean(b['phantom_1.0']):.2f} | "
                 f"{sum(x > 0 for x in a['phantom_1.0'])} / {sum(x > 0 for x in b['phantom_1.0'])} | "
                 f"{np.mean(a['fa_frames']):.1f} / {np.mean(b['fa_frames']):.1f} |")
    L += ["", "First confirmation frame per path hazard, per stream (sandbox | Isaac):", ""]
    for k in keys:
        L.append(f"- {k.replace('_', ' ')}: " + "; ".join(
            f"{a} | {b}" for a, b in zip(rows["sandbox"][k]["first"], rows["isaac"][k]["first"])))
    md = "\n".join(L) + "\n"
    open(os.path.join(base, "noise.md"), "w").write(md)
    print(md)


if __name__ == "__main__":
    main()
