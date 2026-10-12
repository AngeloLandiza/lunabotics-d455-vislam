"""Table of the tag sweep: one row per camera/tag setup and layout. Usage: python3 summarize.py results/sweep_v1"""
import glob
import json
import os
import sys

root = sys.argv[1]
rows = ["| FOV (rad) | Pitch (deg up) | Tag height (m) | Layout | Seen at start heading | Never seen after turning | Turn needed, median (deg) | Both walls in view | xy error median [95% CI] (cm) | xy error p95 (cm) | Yaw > 5 deg |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
key = lambda d: [float(x) for x in os.path.basename(d).replace("hfov", "").replace("pitch", "").replace("h", "").split("_")]
for d in sorted(glob.glob(os.path.join(root, "*")), key=key):
    s = json.load(open(os.path.join(d, "summary.json")))
    p = s["params"]
    for name, L in s["layouts"].items():
        f = lambda x, k=1, m=1: "-" if x is None else f"{x * m:.{k}f}"
        ci = L["e_xy_median_ci"]
        rows.append(f"| {p['hfov']['value']} | {p['cam_mount']['value'][3]:g} | {p['tag_height']['value']} | {name} | "
                    f"{f(L['seen_frac'], 0, 100)}% | {f(L['never_frac'], 0, 100)}% | {f(L['turn_median_deg'], 0)} | "
                    f"{f(L['two_planes_frac'], 0, 100)}% | {f(L['e_xy_median'], 1, 100)} [{f(ci[0], 1, 100)}, {f(ci[1], 1, 100)}] | "
                    f"{f(L['e_xy_p95'], 1, 100)} | {f(L['bad_yaw_frac'], 1, 100)}% |")
open(os.path.join(root, "table.md"), "w").write("\n".join(rows) + "\n")
print("\n".join(rows))
