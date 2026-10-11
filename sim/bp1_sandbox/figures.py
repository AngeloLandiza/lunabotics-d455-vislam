"""Figures and summary table from results/records.json, plus one example arena.
Usage: python3 figures.py [results_dir]
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
import numpy as np

from arena import Arena
from dust import Plume
from sensors import Lidar, D455Depth

SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SENSOR_COLOR = {"spin16": "#2a78d6", "rosette70": "#eb6834", "d455": "#1baf7a"}
SENSOR_LABEL = {"spin16": "16-beam spinning LiDAR", "rosette70": "Rosette LiDAR (70° FOV)", "d455": "D455 depth"}
DIVERGING = LinearSegmentedColormap.from_list("div", ["#1c5cab", "#86b6ef", "#f0efec", "#ef9a99", "#b8312f"])

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
})

BINS = np.arange(0.5, 5.01, 0.5)
DET_LABEL = {"baseline": "fixed thresholds", "carve": "carving + solidity", "logodds": "log-odds evidence"}
DETS = ("baseline", "carve", "logodds")


def wilson(k, n, z=1.96):
    if n == 0:
        return np.nan, np.nan, np.nan
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(c - h, 0.0), min(c + h, 1.0)


def pd_curve(recs, kind, sensor, dust, det):
    r = [x for x in recs if x.get("kind") == kind and x["sensor"] == sensor and x["dust"] == dust
         and x["detector"] == det and x["in_view"]]
    rng = np.array([x["range"] for x in r])
    hit = np.array([x["detected"] for x in r])
    out = []
    for a, b in zip(BINS[:-1], BINS[1:]):
        m = (rng >= a) & (rng < b)
        out.append((0.5 * (a + b), int(hit[m].sum()), int(m.sum())))
    return out


def fig_pd(recs, path, title):
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), sharex=True, sharey=True)
    for i, kind in enumerate(["boulder", "crater"]):
        for j, dust in enumerate([False, True]):
            ax = axes[i, j]
            for s in SENSOR_COLOR:
                for det, ls in (("logodds", "-"), ("carve", "--")):
                    c = pd_curve(recs, kind, s, dust, det)
                    xs = [x for x, k, n in c if n >= 5]
                    ys = [k / n for x, k, n in c if n >= 5]
                    ax.plot(xs, ys, ls, color=SENSOR_COLOR[s], lw=2, marker="o", ms=4,
                            label=f"{SENSOR_LABEL[s]}, {DET_LABEL[det]}")
            ax.set_title(f"{kind.capitalize()}s, {'dust plume ahead' if dust else 'clear air'}", color=INK, loc="left")
            ax.set_ylim(-0.03, 1.03)
            if i == 1:
                ax.set_xlabel("Range from sensor to obstacle center (m)")
            if j == 0:
                ax.set_ylabel("Share of in-view obstacles detected")
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=3, frameon=False, fontsize=9)
    fig.suptitle(title, x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout(rect=(0, 0.09, 1, 0.96))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_fa(recs, path, title):
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9), sharey=True)
    style = {"baseline": dict(alpha=0.35, hatch="//"), "carve": dict(alpha=0.65, hatch=".."), "logodds": dict(alpha=1.0)}
    for j, dust in enumerate([False, True]):
        ax = axes[j]
        for k, s in enumerate(SENSOR_COLOR):
            for m, det in enumerate(DETS):
                fa = [x["fa"] for x in recs if "fa" in x and x["sensor"] == s and x["dust"] == dust and x["detector"] == det]
                xpos = k * 3.6 + m * 1.02
                ax.bar(xpos, np.mean(fa), width=1.0, color=SENSOR_COLOR[s], edgecolor=SURF, linewidth=0, **style[det])
                ax.text(xpos, np.mean(fa) + 0.05, f"{np.mean(fa):.2f}", ha="center", va="bottom", fontsize=7.5, color=INK2)
        ax.set_xticks([k * 3.6 + 1.02 for k in range(3)], [SENSOR_LABEL[s] for s in SENSOR_COLOR], fontsize=8.5)
        ax.set_title(f"{'Dust plume ahead' if dust else 'Clear air'}", loc="left", color=INK)
        ax.grid(axis="x", visible=False)
    axes[0].set_ylabel("False alarms per frame (mean)")
    fig.suptitle(title + "  (bars per sensor: fixed thresholds, carving + solidity, log-odds evidence)",
                 x=0.01, ha="left", color=INK, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def table(recs, label):
    rows = []
    bands = [(0.5, 2.0), (2.0, 3.0), (3.0, 4.5)]
    for s in SENSOR_COLOR:
        for dust in (False, True):
            for det in DETS:
                cells = []
                for kind in ("boulder", "crater"):
                    r = [x for x in recs if x.get("kind") == kind and x["sensor"] == s and x["dust"] == dust
                         and x["detector"] == det and x["in_view"]]
                    for a, b in bands:
                        sel = [x["detected"] for x in r if a <= x["range"] < b]
                        p, lo, hi = wilson(sum(sel), len(sel))
                        cells.append("n/a" if not sel else f"{p:.2f} [{lo:.2f}-{hi:.2f}] n={len(sel)}")
                fa = [x["fa"] for x in recs if "fa" in x and x["sensor"] == s and x["dust"] == dust and x["detector"] == det]
                wl = [x["wall"] for x in recs if "wall" in x and x["sensor"] == s and x["dust"] == dust and x["detector"] == det]
                rows.append(f"| {SENSOR_LABEL[s]} | {'dust' if dust else 'clear'} | {DET_LABEL[det]} | " + " | ".join(cells)
                            + f" | {np.mean(fa):.2f} | {np.mean(wl):.2f} |")
    head = ("| Sensor | Air | Detector | Boulder 0.5-2 m | Boulder 2-3 m | Boulder 3-4.5 m | Crater 0.5-2 m | "
            "Crater 2-3 m | Crater 3-4.5 m | False alarms per frame | Wall detections per frame |\n|" + " --- |" * 11)
    return (f"### {label}\n\nDetection rate of in-view obstacles, with 95% Wilson interval and count. False alarms exclude "
            f"detections within the wall band of the arena edge, which are real wall returns and are counted separately.\n\n"
            f"{head}\n" + "\n".join(rows))


def fig_example(path):
    rng = np.random.default_rng(1)
    A = Arena(rng)
    robot = (1.0, 2.5, 0.0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for ax, dust in zip(axes, (None, Plume(1.8, 2.5))):
        im = ax.imshow(A.h.T, origin="lower", cmap=DIVERGING, norm=TwoSlopeNorm(0, -0.3, 0.3),
                       extent=(A.x0, A.x0 + A.h.shape[0] * A.res, A.y0, A.y0 + A.h.shape[1] * A.res))
        s = Lidar("spin16").scan(A, robot, np.random.default_rng(2), dust=dust)
        P, ok, du = s["pts"], s["valid"], s["is_dust"]
        ax.scatter(P[ok & ~du, 0], P[ok & ~du, 1], s=0.4, color=INK, alpha=0.5, linewidths=0, label="Surface return")
        if du.any():
            ax.scatter(P[du, 0], P[du, 1], s=3, color="#eb6834", linewidths=0, label="Dust echo")
        ax.plot(*s["origin"][:2], marker="^", ms=9, color="#2a78d6", label="LiDAR")
        ax.set_xlim(0, 6.8)
        ax.set_ylim(0, 5.0)
        ax.set_aspect("equal")
        ax.grid(False)
        ax.set_title("Clear air" if dust is None else "Dust plume 0.8 m ahead (assumed density)", loc="left", color=INK)
        from matplotlib.lines import Line2D
        hs = [Line2D([], [], marker="o", ls="", color=INK, ms=3, label="Surface return"),
              Line2D([], [], marker="^", ls="", color="#2a78d6", ms=8, label="LiDAR")]
        if du.any():
            hs.insert(1, Line2D([], [], marker="o", ls="", color="#eb6834", ms=4, label="Dust echo"))
        ax.legend(handles=hs, loc="upper right", fontsize=8, frameon=True, framealpha=0.9)
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    cb = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.02)
    cb.set_label("Terrain height (m)", color=INK2)
    fig.suptitle("One synthetic arena with a 16-beam LiDAR scan (top view)", x=0.01, ha="left", color=INK, fontsize=12)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    cam = D455Depth()
    for ax, dust in zip(axes, (None, Plume(1.8, 2.5))):
        s = cam.scan(A, robot, np.random.default_rng(3), dust=dust)
        im = ax.imshow(s["depth_img"], cmap=LinearSegmentedColormap.from_list("seq", ["#cde2fb", "#3987e5", "#0d366b"]),
                       vmin=0.5, vmax=4.5)
        ax.set_title("D455 depth, clear air" if dust is None else "D455 depth, dust plume (black = no depth)",
                     loc="left", color=INK)
        ax.set_facecolor("#0b0b0b")
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
    cb = fig.colorbar(im, ax=axes, shrink=0.85, pad=0.02)
    cb.set_label("Depth (m)", color=INK2)
    fig.savefig(path.replace("example_lidar", "example_d455"), dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "results"
    data = json.load(open(os.path.join(d, "records.json")))
    n = data["n_trials"]
    fig_pd(data["single"], os.path.join(d, "pd_vs_range_single_frame.png"),
           f"Detection vs range, one frame ({n} random arenas)")
    fig_pd(data["sequence"], os.path.join(d, "pd_vs_range_four_frames.png"),
           f"Detection vs range, four frames over 0.75 m of driving ({n} random arenas)")
    fig_fa(data["single"], os.path.join(d, "false_alarms_single_frame.png"), "False alarms, one frame")
    fig_fa(data["sequence"], os.path.join(d, "false_alarms_four_frames.png"), "False alarms, four frames")
    fig_example(os.path.join(d, "example_lidar.png"))
    with open(os.path.join(d, "tables.md"), "w") as f:
        s0 = data.get("seed0", 1000)
        f.write(f"# Sandbox results ({n} random arenas, seeds {s0}-{s0 + n - 1})\n\n")
        f.write(table(data["single"], "One frame") + "\n\n")
        f.write(table(data["sequence"], "Four frames over 0.75 m") + "\n")
    print("figures written to", d)


if __name__ == "__main__":
    main()
