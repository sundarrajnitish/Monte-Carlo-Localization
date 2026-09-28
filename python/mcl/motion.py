"""Odometry motion model (Thrun, Burgard & Fox, *Probabilistic Robotics*, Table 5.6)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def wrap(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def decompose(odom_prev: np.ndarray, odom_curr: np.ndarray, eps: float = 1e-3):
    """Split a relative odometry motion into (rot1, trans, rot2).

    Handles the two classic corner cases that break the textbook formula:

    * pure rotation (``trans ~ 0``): ``atan2(0, 0)`` is meaningless, so
      ``rot1 = 0`` and all of the heading change goes into ``rot2``;
    * reversing: if the heading of travel is more than 90 deg from the robot
      heading we model it as a *negative* translation instead of a
      ~180 deg spin, spin, drive, spin.
    """
    dx, dy = odom_curr[0] - odom_prev[0], odom_curr[1] - odom_prev[1]
    trans = float(np.hypot(dx, dy))
    dtheta = float(wrap(odom_curr[2] - odom_prev[2]))
    if trans < eps:
        return 0.0, 0.0, dtheta
    rot1 = float(wrap(np.arctan2(dy, dx) - odom_prev[2]))
    if abs(rot1) > np.pi / 2:                       # driving backwards
        rot1 = float(wrap(rot1 + np.pi))
        trans = -trans
    rot2 = float(wrap(dtheta - rot1))
    return rot1, trans, rot2


@dataclass
class OdometryMotionModel:
    """Samples ``x_t ~ p(x_t | u_t, x_{t-1})`` for every particle.

    Noise variances (Thrun's alpha_1..alpha_4)::

        var(rot1)  = a1*rot1^2  + a2*trans^2
        var(trans) = a3*trans^2 + a4*(rot1^2 + rot2^2)
        var(rot2)  = a1*rot2^2  + a2*trans^2

    Units: a1 [rad^2/rad^2], a2 [rad^2/cm^2], a3 [cm^2/cm^2], a4 [cm^2/rad^2].
    NB: the alphas scale *variances*; the standard deviation is their square root.
    """
    a1: float = 0.02
    a2: float = 1e-5
    a3: float = 0.01
    a4: float = 1.0

    def sample(self, particles: np.ndarray, odom_prev, odom_curr,
               rng: np.random.Generator) -> np.ndarray:
        rot1, trans, rot2 = decompose(np.asarray(odom_prev), np.asarray(odom_curr))
        if trans == 0.0 and rot2 == 0.0:
            return particles
        n = particles.shape[0]
        sd_r1 = np.sqrt(self.a1 * rot1**2 + self.a2 * trans**2)
        sd_t = np.sqrt(self.a3 * trans**2 + self.a4 * (rot1**2 + rot2**2))
        sd_r2 = np.sqrt(self.a1 * rot2**2 + self.a2 * trans**2)
        r1 = rot1 - rng.normal(0.0, sd_r1, n) if sd_r1 > 0 else np.full(n, rot1)
        t = trans - rng.normal(0.0, sd_t, n) if sd_t > 0 else np.full(n, trans)
        r2 = rot2 - rng.normal(0.0, sd_r2, n) if sd_r2 > 0 else np.full(n, rot2)
        out = particles.copy()
        heading = particles[:, 2] + r1
        out[:, 0] += t * np.cos(heading)
        out[:, 1] += t * np.sin(heading)
        out[:, 2] = wrap(heading + r2)
        return out
