"""Reader for the robot log (CMU-style ``O`` / ``L`` records).

Each line is one of

``O x y theta ts``
    Odometry pose (cm, cm, rad) in the robot's *odometry* frame.
``L x y theta xl yl thetal r_1 ... r_180 ts``
    Odometry pose of the robot, odometry pose of the laser (25 cm ahead of
    the robot centre), 180 range readings in cm (beam ``k`` points at
    ``thetal - 90 deg + k deg``) and a timestamp.

The odometry frame is *not* the map frame: only relative motion between two
odometry poses is meaningful, which is exactly what the odometry motion model
consumes.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

N_BEAMS = 180
LASER_OFFSET = 25.0          # cm, laser is mounted 25 cm ahead of the robot centre
MAX_RANGE = 8183.0           # cm, sensor saturation value found in the log


@dataclass
class Record:
    kind: str                # 'O' or 'L'
    odom: np.ndarray         # (3,) robot pose in odometry frame
    ts: float
    ranges: np.ndarray | None = None   # (180,) for 'L' records


def beam_angles(n: int = N_BEAMS) -> np.ndarray:
    """Beam bearings relative to the laser heading (rad), right to left."""
    return np.deg2rad(np.arange(n) - 90.0)


def load_log(path: str | Path) -> list[Record]:
    records: list[Record] = []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        kind, vals = parts[0], np.array(parts[1:], dtype=float)
        if kind == "O":
            records.append(Record("O", vals[0:3], float(vals[3])))
        elif kind == "L":
            records.append(Record("L", vals[0:3], float(vals[-1]), vals[6:6 + N_BEAMS]))
        else:
            raise ValueError(f"unknown record type {kind!r}")
    records.sort(key=lambda r: r.ts)
    return records
