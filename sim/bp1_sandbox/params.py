"""Every number the sandbox uses, with where it came from.

status:
  Verified - read in a source (URL or guidebook section given)
  Inferred - derived from verified facts (derivation given)
  Assumed  - placeholder chosen by us; replace it with a measured or sourced value
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class P:
    value: object
    unit: str
    status: str
    source: str


GUIDEBOOK = "2027 Lunabotics Guidebook (research/step2/00-guidebook-arena-bp1.md)"
D455_PAGE = "https://www.intelrealsense.com/depth-camera-d455/"
INTEL_TUNING = "https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance"

ARENA = {
    "length_x": P(6.8, "m", "Verified", GUIDEBOOK + " 11.2.1"),
    "width_y": P(5.0, "m", "Verified", GUIDEBOOK + " 11.2.1"),
    "grid_res": P(0.01, "m", "Assumed", "sandbox resolution choice"),
    "berm_center": P((5.38, 0.9), "m", "Verified", GUIDEBOOK + " 11.2.3"),
    "berm_size": P((2.2, 0.9), "m", "Verified", GUIDEBOOK + " 11.2.2 (orientation Assumed: 2.2 m along x)"),
    "start_zone": P((0.0, 2.0, 0.0, 2.0), "m (x0,x1,y0,y1)", "Assumed", "zone sizes are not in the guidebook text"),
    "obstacle_zone_x": P((2.0, 4.4), "m", "Assumed", "Figure 9 labels unreadable; band chosen between start and berm"),
    "column_xy_r": P((3.4, 2.5, 0.15), "m", "Assumed", "'central support column' (11.4.2); size and position not in text"),
    "surface_rms": P(0.01, "m", "Assumed", "raked BP-1 roughness not measured"),
    "surface_corr_len": P(0.25, "m", "Assumed", "roughness correlation length"),
    "gravel_per_m2": P(4.0, "1/m^2", "Assumed", "gravel ~2 cm diameter exists (11.1); density not given"),
    "gravel_diam": P(0.02, "m", "Verified", GUIDEBOOK + " 11.1 item 2"),
}

OBSTACLES = {
    "n_boulders": P(3, "", "Verified", GUIDEBOOK + " 11.4.1 (minimum)"),
    "n_craters": P(3, "", "Verified", GUIDEBOOK + " 11.4.2 (minimum)"),
    "boulder_diam": P((0.30, 0.40), "m", "Verified", GUIDEBOOK + " 11.4.1"),
    "boulder_height": P((0.10, 0.30), "m", "Assumed", "11.4.1 says only 'varying heights'"),
    "crater_diam": P((0.30, 0.50), "m", "Inferred", "11.4.2: depth and width not exceeding ~40-50 cm"),
    "crater_depth": P((0.10, 0.40), "m", "Assumed", "upper bound from 11.4.2; lower bound chosen"),
    "crater_rim_frac": P(0.10, "of depth", "Assumed", "rim height; replace with a published crater profile"),
    "min_separation": P(0.8, "m", "Assumed", "spacing between obstacle centers"),
}

BP1 = {
    "depth": P(0.45, "m", "Verified", GUIDEBOOK + " 11.1"),
    "particle_size_most_under": P(100e-6, "m", "Verified", GUIDEBOOK + " 11.1 g"),
    "density_compacted": P((1.5, 1.8), "g/cm^3", "Verified", GUIDEBOOK + " 11.1 f"),
    "density_top_raked": P(0.75, "g/cm^3", "Verified", GUIDEBOOK + " 11.1 h"),
    "lidar_reflectivity": P(0.10, "", "Assumed", "NOT KNOWN. Guidebook Figure 7 lets teams measure it; needs the PDF figure"),
}

# Sensor mounts in the robot frame (x forward, y left, z up; origin on the ground under the robot center)
MOUNTS = {
    "d455_xyz": P((0.35, 0.0, 0.60), "m", "Assumed", "Angelo: front, 'somewhat high'"),
    "d455_pitch_down": P(40.0, "deg", "Verified", "Angelo's sensor layout"),
    "lidar_xyz": P((0.0, 0.0, 0.80), "m", "Assumed", "Angelo: on top of the robot"),
    "lidar_pitch_down": P(30.0, "deg", "Assumed", "Angelo: 'angled downwards'; angle not given"),
}

D455 = {
    "hfov": P(87.0, "deg", "Verified", D455_PAGE),
    "vfov": P(58.0, "deg", "Verified", D455_PAGE),
    "baseline": P(0.095, "m", "Verified", D455_PAGE),
    "native_width": P(1280, "px", "Assumed", "focal length is computed at this width"),
    "sim_size": P((212, 120), "px", "Assumed", "downsampled for speed"),
    "subpixel_best": P(0.08, "px", "Verified", INTEL_TUNING + " (curves use subpixel=0.08)"),
    "subpixel_field": P(0.32, "px", "Inferred", "from '<2% at 4 m' with f=674 px, B=0.095 m (step 1 check)"),
    "min_depth": P(0.40, "m", "Assumed", "minimum depth not checked yet"),
    "max_depth": P(6.0, "m", "Assumed", "useful range cap for the sandbox"),
    "invalid_frac_clear": P(0.02, "", "Assumed", "baseline invalid-pixel fraction on low-texture regolith"),
}

LIDAR_GENERIC = {
    "range_sigma": P(0.02, "m", "Assumed", "replace with the chosen LiDAR's datasheet"),
    "max_range_ref": P(20.0, "m", "Assumed", "range at which a 10% target at normal incidence is just detected"),
    "rho_ref": P(0.10, "", "Assumed", "reference reflectivity for max_range_ref"),
}

LIDAR_PROFILES = {
    # Generic stand-ins until the LiDAR-agnostic research lands. Names describe the pattern type only.
    "spin16": {
        "kind": "spinning", "channels": 16, "vmin": -15.0, "vmax": 15.0, "hres": 0.4,
        "min_range": 0.3, "status": "Assumed generic 16-beam spinning pattern",
    },
    "rosette70": {
        "kind": "rosette", "fov": 70.4, "points": 10000, "petals": 7.0,
        "min_range": 0.21,
        "status": "Approximate non-repetitive pattern; 70.4 deg FOV and 0.21 m blind zone are the Livox MID-70 values "
                  "reported by Xie et al. (https://pmc.ncbi.nlm.nih.gov/articles/PMC11679008); the curve itself is Assumed",
    },
}

DUST = {
    "beta_peak": P(1.5, "1/m", "Assumed", "peak extinction in the dig plume; no BP-1 measurement found yet"),
    "beta_background": P(0.0, "1/m", "Assumed", "clear air away from digging"),
    "plume_sigma_h": P(0.5, "m", "Assumed", "horizontal spread"),
    "plume_sigma_v": P(0.3, "m", "Assumed", "vertical spread"),
    "plume_center_z": P(0.3, "m", "Assumed", "height of densest dust"),
    "echo_kappa": P(0.3, "", "Assumed", "fraction of extinction events strong enough to trigger a LiDAR echo"),
    "camera_invalid_gain": P(1.0, "", "Assumed", "how fast stereo invalid pixels grow as transmittance drops"),
}

DETECT = {
    "cell": P(0.05, "m", "Assumed", "elevation grid"),
    "h_pos": P(0.08, "m", "Assumed", "positive obstacle threshold above fitted ground"),
    "h_neg": P(0.08, "m", "Assumed", "negative obstacle threshold below fitted ground"),
    "min_cells_pos": P(3, "cells", "Assumed", ""),
    "min_cells_neg": P(4, "cells", "Assumed", ""),
    "carve_margin": P(0.08, "m", "Assumed", "a ray sample counts as free space only if it is this far (plus 3 sigma) short of the hit"),
    "wall_band": P(0.15, "m", "Assumed", "detections this close to the arena edge are walls, scored separately from false alarms"),
    "match_margin": P(0.15, "m", "Assumed", "detection counts if its center is within radius + margin"),
    "l_occ": P(0.85, "", "Verified", "log-odds added per supporting observation; value from Pairet et al. Sec. 7.1, https://arxiv.org/abs/2004.12317 (our per-ray use on a height grid is Inferred)"),
    "l_free": P(-0.4, "", "Verified", "log-odds added per contradicting observation; Pairet et al. Sec. 7.1"),
    "l_min": P(-2.0, "", "Verified", "lower clamp; Pairet et al. Sec. 7.1"),
    "l_max": P(3.5, "", "Verified", "upper clamp; Pairet et al. Sec. 7.1"),
    "l_decide_lidar": P(5.1, "", "Assumed", "LiDAR: accept an object when log-odds summed over its cells reach 6 net observations; tuned on seeds 1000-1015"),
    "l_decide_d455": P(0.85, "", "Assumed", "D455: accept an object at 1 net observation (it made no dust false alarms); tuned on seeds 1000-1015"),
    "d_neg": P(0.04, "m", "Assumed", "a ray passing this far below the fitted ground (or 3 vertical sigma) is evidence of a depression"),
    "ground_fit": P("local", "", "Assumed", "'local': robust local planes (Cleveland 1979 LOESS); 'plane': one robust plane for the arena. Local chosen on seeds 3000-3047: same misses on a flat floor, far fewer false stops on floor waves, roughness and pitch error"),
    "ground_sigma": P(0.2, "m", "Assumed", "width of the Gaussian window for the local ground fit; 0.2 m beat 0.3 m on 4 cm floor waves (seeds 3000-3003)"),
    "ground_spread": P(0.10, "m", "Assumed", "local ground fit uses only cells whose returns span less than this in height (0 = all cells); seed 3020: ground cells under 6 cm at p99, dust-plume cells about 90 cm"),
}


def v(table, key):
    return table[key].value


def all_tables():
    return {"ARENA": ARENA, "OBSTACLES": OBSTACLES, "BP1": BP1, "MOUNTS": MOUNTS, "D455": D455,
            "LIDAR_GENERIC": LIDAR_GENERIC, "DUST": DUST, "DETECT": DETECT}
