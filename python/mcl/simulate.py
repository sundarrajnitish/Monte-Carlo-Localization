"""Synthetic data with known ground truth, generated inside the real map.

The real log has no ground truth, so to measure *accuracy* (not just
self-consistency) we replay a realistic trajectory, corrupt its odometry with
the motion model and draw laser scans from the beam model.
"""
from __future__ import annotations

import numpy as np

from .data import LASER_OFFSET, MAX_RANGE, N_BEAMS, Record, beam_angles
from .maps import OccupancyMap
from .motion import decompose, wrap


def sample_scan(omap: OccupancyMap, pose, rng, sigma=10.0, p_short=0.05, p_max=0.03,
                p_rand=0.02, lambda_short=0.01, max_range=MAX_RANGE):
    lx = pose[0] + LASER_OFFSET * np.cos(pose[2])
    ly = pose[1] + LASER_OFFSET * np.sin(pose[2])
    z_star = omap.raycast(lx, ly, pose[2] + beam_angles(), max_range)
    z = z_star + rng.normal(0, sigma, N_BEAMS)
    u = rng.random(N_BEAMS)
    short = u < p_short
    z[short] = np.minimum(rng.exponential(1 / lambda_short, short.sum()), z_star[short])
    mx = (u >= p_short) & (u < p_short + p_max)
    z[mx] = max_range
    rnd = (u >= p_short + p_max) & (u < p_short + p_max + p_rand)
    z[rnd] = rng.uniform(0, max_range, rnd.sum())
    z[z_star >= max_range] = max_range
    return np.clip(z, 0, max_range)


def noisy_odometry(truth: np.ndarray, rng, a=(0.05, 2e-5, 0.02, 2.0), bias=(0.0, 4e-4),
                   frozen: int | None = None):
    """Odometry poses (in an arbitrary frame) whose increments are the truth
    increments corrupted with motion-model noise plus a small systematic
    heading bias per cm (~2.3 deg per metre, wheel-radius mismatch style drift). The increment
    into step ``frozen`` is reported as "no motion" (used for kidnapping)."""
    a1, a2, a3, a4 = a
    odom = np.zeros_like(truth)
    odom[0] = (0.0, 0.0, 0.3)            # arbitrary odometry frame origin
    for k in range(1, len(truth)):
        r1, t, r2 = decompose(truth[k - 1], truth[k])
        if (t == 0 and r2 == 0) or k == frozen:
            odom[k] = odom[k - 1]
            continue
        r1n = r1 + rng.normal(0, np.sqrt(a1 * r1**2 + a2 * t**2))
        tn = t + rng.normal(0, np.sqrt(a3 * t**2 + a4 * (r1**2 + r2**2)))
        r2n = r2 + rng.normal(0, np.sqrt(a1 * r2**2 + a2 * t**2)) + bias[1] * abs(t)
        th = odom[k - 1, 2] + r1n
        odom[k] = (odom[k - 1, 0] + tn * np.cos(th), odom[k - 1, 1] + tn * np.sin(th),
                   wrap(th + r2n))
    return odom


def make_dataset(omap: OccupancyMap, truth: np.ndarray, rng, scan_every: int = 2,
                 kidnap_at: int | None = None, kidnap_to: int = 0, kidnap_len: int | None = None):
    """Build a record list. ``truth`` is a (T, 3) pose sequence.

    Kidnapped robot: if ``kidnap_at`` is given, at that step the robot is
    carried to where it was at step ``kidnap_to`` and continues from there
    (for ``kidnap_len`` steps), while the odometry reports no motion for
    the jump.
    """
    truth = np.asarray(truth, float)
    if kidnap_at is not None:
        end = None if kidnap_len is None else kidnap_to + kidnap_len
        truth = np.vstack((truth[:kidnap_at], truth[kidnap_to:end]))
    odom = noisy_odometry(truth, rng, frozen=kidnap_at)
    recs, truth_at_scan = [], []
    for k in range(len(truth)):
        if k % scan_every == 0:
            recs.append(Record("L", odom[k], float(k), sample_scan(omap, truth[k], rng)))
            truth_at_scan.append(truth[k])
        else:
            recs.append(Record("O", odom[k], float(k)))
    return recs, np.asarray(truth_at_scan)


def waypoint_trajectory(waypoints, step: float = 5.0, turn_step: float = np.deg2rad(6)):
    """Drive-turn-drive path through (x, y) waypoints, starting with heading
    towards the first leg. Returns a dense (T, 3) pose array."""
    wp = np.asarray(waypoints, float)
    poses = []
    heading = np.arctan2(wp[1, 1] - wp[0, 1], wp[1, 0] - wp[0, 0])
    pos = wp[0].copy()
    poses.append((*pos, heading))
    for k in range(1, len(wp)):
        target = np.arctan2(wp[k, 1] - pos[1], wp[k, 0] - pos[0])
        dth = wrap(target - heading)
        n_turn = int(np.ceil(abs(dth) / turn_step))
        for i in range(1, n_turn + 1):             # turn on the spot (+ tiny creep)
            poses.append((*pos, wrap(heading + dth * i / n_turn)))
        heading = target
        dist = np.hypot(*(wp[k] - pos))
        n = max(1, int(np.ceil(dist / step)))
        for i in range(1, n + 1):
            p = pos + (wp[k] - pos) * i / n
            poses.append((*p, heading))
        pos = wp[k].copy()
    return np.asarray(poses)


DEFAULT_WAYPOINTS = [(1822, 30), (1822, 180), (1500, 190), (1000, 200), (620, 190),
                     (900, 170), (1400, 180), (2000, 190), (2600, 180)]
