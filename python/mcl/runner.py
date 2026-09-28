"""Replay a robot log through a particle filter and collect a history."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .data import Record
from .filter import ParticleFilter


@dataclass
class History:
    ts: list = field(default_factory=list)
    estimate: list = field(default_factory=list)
    spread: list = field(default_factory=list)
    neff: list = field(default_factory=list)
    updated: list = field(default_factory=list)
    scan_index: list = field(default_factory=list)
    snapshots: list = field(default_factory=list)   # (scan_index, particles copy)

    def arrays(self):
        return {k: np.asarray(v) for k, v in self.__dict__.items() if k != "snapshots"}


def run_log(pf: ParticleFilter, records: list[Record], snapshot_every: int = 0,
            callback=None, odom_source: str = "laser") -> History:
    """Replay a log: odometry changes go to ``predict``, scans to ``update``.

    ``odom_source`` picks which odometry stream drives the motion model:

    * ``"laser"`` (default) - the pose stamped on each ``L`` record, i.e. the
      odometry at the instant the scan was taken;
    * ``"odometry"`` - only the ``O`` records;
    * ``"both"`` - every record in time order. **Don't**: the pose attached
      to a scan lags the ``O`` stream by one reading, so interleaving the two
      makes the robot jitter back and forth (twice the real path length,
      hundreds of fake reversals) and doubles the injected motion noise.
    """
    hist = History()
    prev = None
    scan_i = -1
    for rec in records:
        use = (odom_source == "both" or (odom_source == "laser" and rec.kind == "L")
               or (odom_source == "odometry" and rec.kind == "O"))
        if use:
            if prev is not None and np.any(rec.odom != prev):
                pf.predict(prev, rec.odom)
            prev = rec.odom
        if rec.kind != "L":
            continue
        scan_i += 1
        applied = pf.update(rec.ranges)
        hist.ts.append(rec.ts)
        hist.estimate.append(pf.estimate())
        hist.spread.append(pf.spread())
        hist.neff.append(pf.last_neff)
        hist.updated.append(applied)
        hist.scan_index.append(scan_i)
        if snapshot_every and scan_i % snapshot_every == 0:
            hist.snapshots.append((scan_i, pf.particles.copy()))
        if callback:
            callback(scan_i, rec, pf)
    return hist
