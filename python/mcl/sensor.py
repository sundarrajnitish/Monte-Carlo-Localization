"""Laser range-finder measurement models (Probabilistic Robotics, ch. 6).

Both models return the **log**-likelihood ``log p(z | x, m)`` for every
particle. Working in log space is essential: the product of 180 densities in
the 1e-3 range underflows double precision long before it becomes useful.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.special import erf

from .data import LASER_OFFSET, MAX_RANGE, beam_angles
from .maps import OccupancyMap


def laser_pose(particles: np.ndarray, offset: float = LASER_OFFSET):
    th = particles[:, 2]
    return particles[:, 0] + offset * np.cos(th), particles[:, 1] + offset * np.sin(th), th


def _phi(x):
    return 0.5 * (1.0 + erf(x / np.sqrt(2.0)))


@dataclass
class BeamModel:
    """Beam (ray-cast) model: mixture of hit / short / max / random (Table 6.1)."""
    z_hit: float = 0.80
    z_short: float = 0.05
    z_max: float = 0.05
    z_rand: float = 0.10
    sigma_hit: float = 10.0        # cm
    lambda_short: float = 0.01     # 1/cm
    max_range: float = MAX_RANGE
    beam_stride: int = 5           # use every k-th beam (beams are not independent)
    temperature: float = 1.0       # divide the log-likelihood (soften over-confidence)

    def components(self, z: np.ndarray, z_star: np.ndarray):
        """The four normalised component densities (useful for plotting)."""
        zm = self.max_range
        s = self.sigma_hit
        inside = (z >= 0) & (z <= zm)
        eta_hit = 1.0 / np.maximum(_phi((zm - z_star) / s) - _phi(-z_star / s), 1e-12)
        p_hit = np.where(inside, eta_hit * np.exp(-0.5 * ((z - z_star) / s) ** 2)
                         / (np.sqrt(2 * np.pi) * s), 0.0)
        lam = self.lambda_short
        eta_short = 1.0 / np.maximum(1.0 - np.exp(-lam * z_star), 1e-12)
        p_short = np.where((z >= 0) & (z <= z_star), eta_short * lam * np.exp(-lam * z), 0.0)
        p_max = (z >= zm - 1.0).astype(float)
        p_rand = np.where((z >= 0) & (z < zm), 1.0 / zm, 0.0)
        return p_hit, p_short, p_max, p_rand

    def density(self, z, z_star):
        p_hit, p_short, p_max, p_rand = self.components(z, z_star)
        return (self.z_hit * p_hit + self.z_short * p_short
                + self.z_max * p_max + self.z_rand * p_rand)

    def log_likelihood(self, particles: np.ndarray, ranges: np.ndarray,
                       omap: OccupancyMap) -> np.ndarray:
        idx = np.arange(0, ranges.size, self.beam_stride)
        lx, ly, th = laser_pose(particles)
        ang = th[:, None] + beam_angles(ranges.size)[idx][None, :]
        z_star = omap.raycast(lx[:, None], ly[:, None], ang, self.max_range)
        self.last_n_beams = idx.size
        p = self.density(ranges[idx][None, :], z_star)
        return np.log(np.maximum(p, 1e-300)).sum(axis=1) / self.temperature


@dataclass
class LikelihoodFieldModel:
    """Likelihood-field ("end-point") model (Table 6.3).

    Each beam end-point is scored by its distance ``d`` to the nearest
    obstacle: ``p = z_hit * N(d; 0, sigma) + z_rand / z_max``. Max-range
    readings carry no end-point and are skipped. No ray casting is required,
    so it is ~10-50x cheaper than the beam model and much smoother in pose.
    """
    z_hit: float = 0.90
    z_rand: float = 0.10
    sigma_hit: float = 10.0        # cm
    max_range: float = MAX_RANGE
    beam_stride: int = 5
    temperature: float = 1.0

    def log_likelihood(self, particles: np.ndarray, ranges: np.ndarray,
                       omap: OccupancyMap) -> np.ndarray:
        idx = np.arange(0, ranges.size, self.beam_stride)
        z = ranges[idx]
        keep = z < self.max_range - 1.0
        idx, z = idx[keep], z[keep]
        self.last_n_beams = idx.size
        lx, ly, th = laser_pose(particles)
        ang = th[:, None] + beam_angles(ranges.size)[idx][None, :]
        ex = lx[:, None] + z[None, :] * np.cos(ang)
        ey = ly[:, None] + z[None, :] * np.sin(ang)
        d = omap.distance_at(ex, ey, outside=np.inf)
        s = self.sigma_hit
        p = (self.z_hit * np.exp(-0.5 * (d / s) ** 2) / (np.sqrt(2 * np.pi) * s)
             + self.z_rand / self.max_range)
        return np.log(p).sum(axis=1) / self.temperature
