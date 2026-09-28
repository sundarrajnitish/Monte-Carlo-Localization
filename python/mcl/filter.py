"""Monte Carlo Localization (particle filter) - Probabilistic Robotics Table 8.2/8.3."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.special import logsumexp

from .maps import OccupancyMap
from .motion import OdometryMotionModel, wrap
from .resample import SCHEMES, effective_sample_size


@dataclass
class MCLConfig:
    n_particles: int = 1000
    resampler: str = "low_variance"
    resample_threshold: float = 0.5     # resample when N_eff < threshold * M
    min_trans: float = 5.0              # cm moved before integrating a new scan
    min_rot: float = np.deg2rad(5.0)    # ... or rad turned
    # Augmented MCL (Table 8.3): random-particle injection for recovery.
    augmented: bool = False
    alpha_slow: float = 0.005
    alpha_fast: float = 0.1


@dataclass
class ParticleFilter:
    omap: OccupancyMap
    motion: OdometryMotionModel
    sensor: object
    config: MCLConfig = field(default_factory=MCLConfig)
    seed: int | None = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.particles = np.zeros((self.config.n_particles, 3))
        self.weights = np.full(self.config.n_particles, 1.0 / self.config.n_particles)
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.last_neff = float(self.config.n_particles)
        self.resampled = False
        self._moved = np.zeros(2)          # accumulated |trans|, |rot| since last scan
        self._first_scan = True

    # ------------------------------------------------------------ initialise
    def init_uniform_free(self):
        """Global localisation: uniform over free space, uniform heading."""
        free = self.omap.free_cells_world()
        n = self.config.n_particles
        pick = free[self.rng.integers(0, len(free), n)]
        jitter = self.rng.uniform(-0.5, 0.5, (n, 2)) * self.omap.resolution
        self.particles = np.column_stack((pick + jitter, self.rng.uniform(-np.pi, np.pi, n)))
        self.weights[:] = 1.0 / n

    def init_region(self, x_range, y_range, theta_range):
        """Uniform over a box (cm, cm, rad), rejecting occupied samples."""
        n = self.config.n_particles
        out = np.empty((0, 3))
        while len(out) < n:
            cand = np.column_stack((self.rng.uniform(*x_range, 4 * n),
                                    self.rng.uniform(*y_range, 4 * n),
                                    self.rng.uniform(*theta_range, 4 * n)))
            out = np.vstack((out, cand[self.omap.is_free(cand[:, 0], cand[:, 1])]))
        self.particles = out[:n].copy()
        self.particles[:, 2] = wrap(self.particles[:, 2])
        self.weights[:] = 1.0 / n

    def init_gaussian(self, mean, std):
        n = self.config.n_particles
        self.particles = self.rng.normal(mean, std, (n, 3))
        self.particles[:, 2] = wrap(self.particles[:, 2])
        self.weights[:] = 1.0 / n

    # --------------------------------------------------------------- steps
    def predict(self, odom_prev, odom_curr):
        self.particles = self.motion.sample(self.particles, odom_prev, odom_curr, self.rng)
        d = np.asarray(odom_curr) - np.asarray(odom_prev)
        self._moved += (np.hypot(d[0], d[1]), abs(wrap(d[2])))

    def should_update(self) -> bool:
        c = self.config
        return self._first_scan or self._moved[0] >= c.min_trans or self._moved[1] >= c.min_rot

    def update(self, ranges: np.ndarray) -> bool:
        """Measurement update + (conditional) resampling. Returns True if applied."""
        self.resampled = False
        if not self.should_update():
            return False
        self._first_scan = False
        self._moved[:] = 0.0

        logw = self.sensor.log_likelihood(self.particles, ranges, self.omap)
        # particles inside walls / outside the map are impossible
        logw = np.where(self.omap.is_free(self.particles[:, 0], self.particles[:, 1]), logw, -np.inf)
        if not np.isfinite(logw).any():                    # total failure -> keep prior
            logw = np.zeros_like(logw)
        logw = logw + np.log(np.maximum(self.weights, 1e-300))
        norm = logsumexp(logw)
        self.weights = np.exp(logw - norm)

        if self.config.augmented:
            # exp(norm) = sum_i w_i p(z|x_i) is the average measurement likelihood.
            # Take its per-beam geometric mean so it stays in a sane numeric range.
            n_used = getattr(self.sensor, "last_n_beams", 0)
            if n_used > 0:
                w_avg = float(np.exp(norm * self.sensor.temperature / n_used))
                c = self.config
                if self.w_slow == 0.0:              # first scan: start both at w_avg
                    self.w_slow = self.w_fast = w_avg
                self.w_slow += c.alpha_slow * (w_avg - self.w_slow)
                self.w_fast += c.alpha_fast * (w_avg - self.w_fast)

        self.last_neff = effective_sample_size(self.weights)
        if self.last_neff < self.config.resample_threshold * self.config.n_particles:
            self.resample()
        return True

    def resample(self):
        n = self.config.n_particles
        idx = SCHEMES[self.config.resampler](self.weights, self.rng)
        new = self.particles[idx].copy()
        if self.config.augmented and self.w_slow > 0:
            p_inject = max(0.0, 1.0 - self.w_fast / self.w_slow)
            k = self.rng.random(n) < p_inject
            if k.any():
                free = self.omap.free_cells_world()
                pick = free[self.rng.integers(0, len(free), k.sum())]
                new[k] = np.column_stack((pick, self.rng.uniform(-np.pi, np.pi, k.sum())))
        self.particles = new
        self.weights = np.full(n, 1.0 / n)
        self.resampled = True

    # ------------------------------------------------------------- output
    def mean(self) -> np.ndarray:
        """Weighted mean pose (circular mean for heading)."""
        w = self.weights
        x = np.sum(w * self.particles[:, 0])
        y = np.sum(w * self.particles[:, 1])
        th = np.arctan2(np.sum(w * np.sin(self.particles[:, 2])),
                        np.sum(w * np.cos(self.particles[:, 2])))
        return np.array([x, y, th])

    def estimate(self, cell: float = 50.0, radius: float = 100.0) -> np.ndarray:
        """Robust pose estimate: weighted mean of the dominant mode.

        The plain weighted mean of a multi-modal belief lands *between* the
        modes (often inside a wall). We histogram the weights on a coarse
        grid, take the heaviest cell and average only the particles within
        ``radius`` of it.
        """
        p, w = self.particles, self.weights
        key = np.floor(p[:, :2] / cell).astype(np.int64)
        k = key[:, 0] * 100003 + key[:, 1]
        uniq, inv = np.unique(k, return_inverse=True)
        best = np.bincount(inv, weights=w).argmax()
        centre = np.average(p[inv == best, :2], axis=0, weights=w[inv == best] + 1e-300)
        near = np.hypot(p[:, 0] - centre[0], p[:, 1] - centre[1]) < radius
        ww = w[near] + 1e-300
        x, y = np.average(p[near, 0], weights=ww), np.average(p[near, 1], weights=ww)
        th = np.arctan2(np.sum(ww * np.sin(p[near, 2])), np.sum(ww * np.cos(p[near, 2])))
        return np.array([x, y, th])

    def spread(self) -> float:
        """Weighted positional standard deviation (cm)."""
        mu = self.mean()
        d2 = (self.particles[:, 0] - mu[0]) ** 2 + (self.particles[:, 1] - mu[1]) ** 2
        return float(np.sqrt(np.sum(self.weights * d2)))
