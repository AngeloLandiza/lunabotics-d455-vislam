"""Laser-line brightness on BP-1 against the target patches in guidebook Figure 7 (figure7_bp1_reflectivity.jpg).

For every image row of each photo: find the line (largest R - G), convert sRGB to linear light, sum the red excess
over the line (+-5 px) above the median of two flanks (9-16 px either side), and divide by the line's slope so a
slanted line counts per unit length. Rows are labelled by their flank colour: target black, mid grey, light grey, or
BP-1 (brownish, below the mound top). Rows where the flank changes (patch edges) are left out.

Usage: python3 measure_fig7.py [figure7_bp1_reflectivity.jpg] [out.json]
"""
import json
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PANELS = {"10 deg": (100, 270), "16 deg": (330, 470), "21 deg": (580, 730)}   # x ranges of the three photos
ROWS = (140, 462)
HALF, FL0, FL1 = 5, 9, 16
CLASSES = ("black", "mid grey", "light grey", "BP-1")


def linear(c):
    c = c / 255
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def panel_rows(im, L, c0, c1):
    R, G = im[..., 0], im[..., 1]
    rows = []
    for y in range(*ROWS):
        seg = R[y, c0:c1] - G[y, c0:c1]
        if seg.max() < 50:
            continue
        x = c0 + int(np.argmax(seg))
        fl = np.r_[x - FL1:x - FL0, x + FL0 + 1:x + FL1 + 1]
        bg = np.median(L[y, fl], axis=0)
        rows.append((y, x, *bg, (L[y, x - HALF:x + HALF + 1, 0] - bg[0]).sum()))
    return np.array(rows)


def classify(r):
    y, x, bgR, bgG, bgB, ex = r.T
    slope = np.gradient(np.convolve(x, np.ones(7) / 7, mode="same"))
    ex = ex * np.cos(np.arctan(slope))
    brown = (bgR / np.maximum(bgB, 1e-4) > 1.8) & (bgG > 0.01)
    d = np.abs(np.diff(bgG)) < 0.004
    stable = np.r_[False, d] & np.r_[d, False]
    top = y[brown].min()
    cls = np.full(len(y), "", object)
    cls[(bgG < 0.012) & ~brown & (y < top - 3)] = "black"
    cls[(bgG > 0.015) & (bgG < 0.04) & ~brown & (y < top - 3)] = "mid grey"
    cls[(bgG > 0.09) & ~brown & (y < top - 3)] = "light grey"
    cls[brown] = "BP-1"
    cls[~stable] = ""
    return cls, ex, bgG


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "figure7_bp1_reflectivity.jpg")
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "fig7_measure.json")
    im = np.asarray(Image.open(src).convert("RGB")).astype(float)
    L = linear(im)
    res = {}
    for name, (c0, c1) in PANELS.items():
        cls, ex, bgG = classify(panel_rows(im, L, c0, c1))
        p = {c: {"rows": int((cls == c).sum()), "patch_linear": float(np.median(bgG[cls == c])),
                 "line": float(np.median(ex[cls == c])), "line_p25": float(np.percentile(ex[cls == c], 25)),
                 "line_p75": float(np.percentile(ex[cls == c], 75))} for c in CLASSES}
        # BP-1 placed on the target's line-vs-patch curve (log-linear between the two nearest patches)
        pts = sorted((p[c]["line"], np.log10(p[c]["patch_linear"])) for c in CLASSES[:3])
        lines, logs = zip(*pts)
        p["bp1_equiv_patch"] = float(10 ** np.interp(p["BP-1"]["line"], lines, logs))
        p["bp1_equiv_over_light"] = p["bp1_equiv_patch"] / p["light grey"]["patch_linear"]
        p["light_over_black_patch"] = p["light grey"]["patch_linear"] / p["black"]["patch_linear"]
        p["light_over_black_line"] = p["light grey"]["line"] / p["black"]["line"]
        res[name] = p
        print(f"{name}: line brightness  black {p['black']['line']:.2f}  mid grey {p['mid grey']['line']:.2f}  "
              f"light grey {p['light grey']['line']:.2f}  BP-1 {p['BP-1']['line']:.2f};  patches differ "
              f"{p['light_over_black_patch']:.0f}x, their lines {p['light_over_black_line']:.2f}x;  BP-1 equals a patch of "
              f"{p['bp1_equiv_over_light']:.2f} x light grey")
    with open(out, "w") as f:
        json.dump({"source": os.path.basename(src), "panels": res}, f, indent=1)


if __name__ == "__main__":
    main()
