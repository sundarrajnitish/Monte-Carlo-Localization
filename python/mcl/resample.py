"""Resampling schemes. All take normalised weights and return indices."""
from __future__ import annotations

import numpy as np


def effective_sample_size(w: np.ndarray) -> float:
    """N_eff = 1 / sum(w_i^2) for normalised weights."""
    return float(1.0 / np.sum(w**2))


def multinomial(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """M independent draws - simplest, highest variance."""
    return np.minimum(np.searchsorted(np.cumsum(w), rng.random(w.size), side="right"), w.size - 1)


def stratified(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """One uniform draw inside each of M equal strata."""
    n = w.size
    u = (np.arange(n) + rng.random(n)) / n
    return np.minimum(np.searchsorted(np.cumsum(w), u, side="right"), n - 1)


def low_variance(w: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    """Low-variance (systematic) resampler, Probabilistic Robotics Table 4.4.

    A single random offset r ~ U[0, 1/M) and M equally spaced pointers
    r + m/M. O(M), and if all weights are equal every particle survives
    exactly once - resampling adds no noise.
    """
    n = w.size
    u = rng.random() / n + np.arange(n) / n
    return np.minimum(np.searchsorted(np.cumsum(w), u, side="right"), n - 1)


SCHEMES = {"low_variance": low_variance, "stratified": stratified, "multinomial": multinomial}
