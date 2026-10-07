"""Procedural lunar terrain for Isaac Sim (numpy only; USD built in isaac_lunar_sim.py).

Same recipe as OmniLRS's procedural layer, cut down: fractal (1/f) height noise,
bowl-shaped craters with raised rims, scattered rocks, and a multi-octave regolith
albedo texture so stereo visual odometry has features to track.
"""
import numpy as np


def fractal_noise(n, rng, beta=2.4):
    """n x n zero-mean noise with a 1/f^beta power spectrum, scaled to unit std."""
    fx = np.fft.fftfreq(n)[:, None]
    fy = np.fft.fftfreq(n)[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2)
    f[0, 0] = 1.0
    spectrum = (rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n))) / f ** (beta / 2)
    spectrum[0, 0] = 0
    out = np.real(np.fft.ifft2(spectrum))
    return out / out.std()


def crater_profile(r, radius):
    """Height offset of a simple crater (depth 0.2 D, rim 0.04 D) at distance r."""
    d = 2 * radius
    depth, rim = 0.2 * d, 0.04 * d
    x = r / radius
    inside = (x ** 2) * (depth + rim) - depth          # parabolic bowl, rim height at x = 1
    outside = rim * np.clip(x, 1, None) ** -3          # ejecta falloff
    return np.where(x < 1, inside, outside)


def heightmap(size=20.0, res=0.05, seed=0, n_craters=25, roughness=0.06, pad_radius=4.0):
    """Square heightmap (m) centered on the origin; no craters within pad_radius (survey area)."""
    rng = np.random.default_rng(seed)
    n = int(round(size / res)) + 1
    c = (np.arange(n) * res) - size / 2
    X, Y = np.meshgrid(c, c, indexing='ij')
    H = roughness * fractal_noise(n, rng)
    for _ in range(n_craters):
        radius = rng.uniform(0.3, 2.5) if rng.random() < 0.85 else rng.uniform(2.5, 4.0)
        while True:
            cx, cy = rng.uniform(-size / 2, size / 2, 2)
            if np.hypot(cx, cy) > pad_radius + radius:
                break
        H += crater_profile(np.hypot(X - cx, Y - cy), radius)
    # Blend to a gentle pad around the start so the rover spawns level.
    w = np.clip((np.hypot(X, Y) - pad_radius * 0.6) / (pad_radius * 0.4), 0, 1)
    H = H * w + (H * 0.2) * (1 - w)
    return X, Y, H.astype(np.float32)


def rocks(size=20.0, seed=0, count=220, pad_radius=4.0, pebble=0.04):
    """(x, y, radius) rocks, power-law sizes; only pebbles (< `pebble` m) inside pad_radius."""
    rng = np.random.default_rng(seed + 1)
    out = []
    while len(out) < count:
        x, y = rng.uniform(-size / 2, size / 2, 2)
        r = min(0.03 * (1 - rng.random()) ** -0.6, 0.35)
        if np.hypot(x, y) < pad_radius and r > pebble:
            continue
        out.append((x, y, r))
    return out


def albedo(px=4096, seed=0, tile_m=5.0):
    """Regolith texture (uint8 grayscale) with features at the scale a rover camera resolves.

    Real regolith simulant is full of clods and grains; smooth noise alone gives
    visual odometry nothing repeatable to track. Layers: broad fractal tone, fine
    grain, and 1-4 cm dark/bright clods (~1 px = 3-9 mm at 1-3 m range).
    """
    import cv2
    rng = np.random.default_rng(seed + 2)
    img = 0.25 * np.tile(fractal_noise(1024, rng, beta=2.0), (px // 1024, px // 1024))
    img += 0.35 * fractal_noise(px, rng, beta=0.8)                 # fine grain
    px_per_m = px / tile_m
    clods = np.zeros((px, px), np.float32)
    n = int(1200 * tile_m ** 2)                                     # ~1200 clods per m^2
    xs, ys = rng.integers(0, px, n), rng.integers(0, px, n)
    radii = (rng.uniform(0.005, 0.02, n) * px_per_m).astype(int) + 1
    shades = rng.choice([-1.0, 1.0], n, p=[0.6, 0.4]) * rng.uniform(0.6, 1.4, n)
    for x, y, r, v in zip(xs, ys, radii, shades):
        cv2.circle(clods, (int(x), int(y)), int(r), float(v), -1, cv2.LINE_AA)
    img += 0.9 * cv2.GaussianBlur(clods, (0, 0), 1.0)
    img = 0.45 + 0.13 * img / img.std()                             # mid-gray regolith, good contrast
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)
