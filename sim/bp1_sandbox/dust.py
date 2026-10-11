"""Dust as a smooth extinction field alpha(x, y, z) in 1/m.

Model form (Inferred hybrid, not a fitted BP-1 model):
- Two-way attenuation of the true return, exp(-2 * integral(alpha)), and a backscatter return from the medium:
  Hahner et al., ICCV 2021, Eq. 6 and 17 (https://arxiv.org/pdf/2108.05249v3).
- Discrete clutter echoes from particles along the beam, kept when they beat the target return:
  Hahner et al., snowfall (https://arxiv.org/pdf/2203.15118), Eq. 11.
Every number in params.DUST is Assumed until BP-1 dust is measured.
"""
import numpy as np
import params as pr
from params import v


class NoDust:
    def alpha(self, x, y, z):
        return np.zeros_like(x)


class Plume:
    """Gaussian dust puff, for example over a dig site."""

    def __init__(self, cx, cy, beta_peak=None, sigma_h=None, sigma_v=None, cz=None, background=None):
        self.cx, self.cy = cx, cy
        self.cz = v(pr.DUST, "plume_center_z") if cz is None else cz
        self.peak = v(pr.DUST, "beta_peak") if beta_peak is None else beta_peak
        self.sh = v(pr.DUST, "plume_sigma_h") if sigma_h is None else sigma_h
        self.sv = v(pr.DUST, "plume_sigma_v") if sigma_v is None else sigma_v
        self.bg = v(pr.DUST, "beta_background") if background is None else background

    def alpha(self, x, y, z):
        r2 = ((x - self.cx) ** 2 + (y - self.cy) ** 2) / self.sh ** 2 + ((z - self.cz) / self.sv) ** 2
        a = self.bg + self.peak * np.exp(-0.5 * r2)
        return np.where(z > -0.5, a, 0.0)
