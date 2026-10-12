"""Run missions for each planner on the same seeds and slip levels, and score them.

Usage: python3 compare.py out_dir [--planners tangent,gcs] [--seeds 100] [--seed0 3000] [--slip none,nominal,heavy]
                          [--workers 4]
       python3 compare.py out_dir --merge a/compare.json,nav2/nav2_default.json,...   (summarize saved runs together,
                          e.g. the Nav2 runs from nav2/run_nav2.sh; times are paired over the seeds every planner finished)

A mission is a start in the start zone and PARAMS["cycles"] dig -> dump cycles (arena2d.mission). Each planner plans
every leg from wherever the robot stopped. A leg that cannot be planned, times out or ends outside the goal tolerance
fails the mission there. Leg time counts from the leg start to the robot standing still at the goal, planning
excluded (the solve time is reported apart, from this machine, not the Orin NX).
"""
import argparse
import json
import logging
import os
import time
from multiprocessing import Pool

import numpy as np

import arena2d as a2

logging.getLogger("drake").setLevel(logging.WARNING)


def make_controller(name, w, robot, goal, cache):
    if name == "tangent":
        from tangent_route import TangentController
        return TangentController(w, robot, goal)
    if name == "gcs":
        from custom_gcs import GcsController
        return GcsController(w, robot, goal, cache)
    raise ValueError(name)


def mission_run(args):
    name, seed, level = args
    logging.getLogger("drake").setLevel(logging.WARNING)
    w = a2.world(seed)
    start, goals = a2.mission(w, np.random.default_rng(seed))
    robot = a2.SkidSteer(start, level, np.random.default_rng([seed, 7]))
    cache, legs = {}, []
    tol = a2.v(a2.PARAMS, "goal_tol")
    ok = True
    for k, (kind, g) in enumerate(goals):
        c = make_controller(name, w, robot, g, cache)
        info = {key: val for key, val in c.info.items() if key in ("inflate", "solve_s", "plan_s", "fail", "length", "pieces")}
        if info.get("fail"):
            legs.append({"k": k, "kind": kind, "planned": False, **info})
            ok = False
            break
        m = a2.run_leg(w, robot, g, c)
        m.pop("worst_pose", None)
        m["ok"] = bool(m["reached"] and m["e_xy"] <= tol[0] and m["e_yaw"] <= tol[1])
        legs.append({"k": k, "kind": kind, "planned": True, **info, **m})
        if not m["ok"]:
            ok = False
            break
    return {"planner": name, "seed": seed, "slip": level, "complete": ok, "legs": legs,
            "time": float(sum(l.get("time", 0.0) for l in legs))}


def summarize(runs, planners, levels):
    btol = a2.v(a2.PARAMS, "berm_tol")
    out = []
    for lv in levels:
        done_by_all = None
        for p in planners:
            seeds = {r["seed"] for r in runs if r["planner"] == p and r["slip"] == lv and r["complete"]}
            done_by_all = seeds if done_by_all is None else done_by_all & seeds
        for p in planners:
            rs = [r for r in runs if r["planner"] == p and r["slip"] == lv]
            legs = [l for r in rs for l in r["legs"] if l.get("planned")]
            dumps = [l for l in legs if l["kind"] == "dump" and l.get("ok")]
            paired = [r["time"] for r in rs if r["seed"] in (done_by_all or set())]
            plan = [l["plan_s"] for r in rs for l in r["legs"]]
            out.append({
                "slip": lv, "planner": p, "missions": len(rs), "complete": sum(r["complete"] for r in rs),
                "unplanned_legs": sum(not l.get("planned") for r in rs for l in r["legs"]),
                "legs_with_contact": sum(l["contact_s"] > 0 for l in legs),
                "min_clearance_p5": float(np.percentile([l["min_clearance"] for l in legs], 5)) if legs else None,
                "paired_missions": len(paired),
                "mission_time_median": float(np.median(paired)) if paired else None,
                "mission_time_p95": float(np.percentile(paired, 95)) if paired else None,
                "dump_on_target": float(np.mean([l["e_xy"] <= btol[0] and l["e_yaw"] <= btol[1] for l in dumps])) if dumps else None,
                "plan_s_median": float(np.median(plan)) if plan else None,
                "plan_s_p95": float(np.percentile(plan, 95)) if plan else None,
            })
    return out


def table(summ):
    L = ["| Slip | Planner | Missions complete | Legs it could not plan | Legs with contact | Mission travel time, "
         "median / 95th pct (paired) | Dumps within 5 cm, 5 deg | Plan time median / 95th pct |", "|" + " --- |" * 8]
    for s in summ:
        mt = (f"{s['mission_time_median']:.1f} / {s['mission_time_p95']:.1f} s ({s['paired_missions']})"
              if s["mission_time_median"] is not None else "-")
        dt = f"{100 * s['dump_on_target']:.0f}%" if s["dump_on_target"] is not None else "-"
        pt = f"{s['plan_s_median']:.2f} / {s['plan_s_p95']:.2f} s" if s["plan_s_median"] is not None else "-"
        L.append(f"| {s['slip']} | {s['planner']} | {s['complete']} of {s['missions']} | {s['unplanned_legs']} | "
                 f"{s['legs_with_contact']} | {mt} | {dt} | {pt} |")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--planners", default="tangent,gcs")
    ap.add_argument("--seeds", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=3000)
    ap.add_argument("--slip", default="none,nominal,heavy")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--merge", default="", help="comma-separated run files to summarize instead of running")
    a = ap.parse_args()
    if a.merge:
        runs = [r for f in a.merge.split(",") for r in json.load(open(f))["runs"]]
        planners = list(dict.fromkeys(r["planner"] for r in runs))
        levels = [lv for lv in a2.SLIP_LEVELS if any(r["slip"] == lv for r in runs)]
        summ = summarize(runs, planners, levels)
        os.makedirs(a.out, exist_ok=True)
        json.dump({"meta": {"merged": a.merge.split(","), "planners": planners}, "summary": summ},
                  open(os.path.join(a.out, "summary.json"), "w"), indent=1)
        md = table(summ)
        open(os.path.join(a.out, "summary.md"), "w").write(md + "\n")
        print(md)
        return
    planners, levels = a.planners.split(","), a.slip.split(",")
    for s in range(a.seed0, a.seed0 + a.seeds):      # build the world cache once, before the workers fork
        a2.world(s)
    jobs = [(p, s, lv) for lv in levels for s in range(a.seed0, a.seed0 + a.seeds) for p in planners]
    t0 = time.time()
    runs = []
    with Pool(a.workers) as pool:
        for i, r in enumerate(pool.imap_unordered(mission_run, jobs, chunksize=2)):
            runs.append(r)
            if (i + 1) % 20 == 0:
                print(f"{i + 1}/{len(jobs)} missions, {time.time() - t0:.0f} s", flush=True)
    summ = summarize(runs, planners, levels)
    os.makedirs(a.out, exist_ok=True)
    meta = {"planners": planners, "slip_levels": {k: a2.SLIP_LEVELS[k] for k in levels}, "seeds": [a.seed0, a.seed0 + a.seeds - 1],
            "params": {k: [p.value, p.unit, p.status, p.source] for k, p in a2.PARAMS.items()}, "seconds": time.time() - t0}
    json.dump({"meta": meta, "summary": summ, "runs": runs}, open(os.path.join(a.out, "compare.json"), "w"))
    md = table(summ)
    open(os.path.join(a.out, "compare.md"), "w").write(md + "\n")
    print(md)
    print(f"{time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
