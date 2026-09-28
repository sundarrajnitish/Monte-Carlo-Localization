"""Occupancy-grid map, distance field and ray casting.

Coordinate conventions (used everywhere in this project, Python, MATLAB and JS):

* World frame: x to the right, y **up**, units are centimetres, origin at the
  bottom-left corner of the map.
* Grid: ``occ[iy, ix]`` where ``iy = 0`` is the *bottom* row. The raw map file
  is stored top-row-first (image order), so it is flipped once on load.
* Cell ``(ix, iy)`` covers ``[ix*res, (ix+1)*res) x [iy*res, (iy+1)*res)``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import ndimage


@dataclass
class OccupancyMap:
    occ: np.ndarray                      # bool (H, W), True = occupied, row 0 = bottom
    resolution: float = 4.0              # cm per cell
    _dist: np.ndarray | None = field(default=None, repr=False)

    # ------------------------------------------------------------------ io
    @classmethod
    def from_file(cls, path: str | Path, resolution: float = 4.0) -> "OccupancyMap":
        """Load a whitespace separated 0/1 grid stored top row first."""
        raw = np.loadtxt(path)
        return cls(occ=np.flipud(raw > 0.5).copy(), resolution=resolution)

    # ------------------------------------------------------------ geometry
    @property
    def shape(self) -> tuple[int, int]:
        return self.occ.shape

    @property
    def width(self) -> float:
        return self.occ.shape[1] * self.resolution

    @property
    def height(self) -> float:
        return self.occ.shape[0] * self.resolution

    def world_to_cell(self, x, y):
        ix = np.floor(np.asarray(x) / self.resolution).astype(int)
        iy = np.floor(np.asarray(y) / self.resolution).astype(int)
        return ix, iy

    def in_bounds(self, x, y):
        x = np.asarray(x)
        y = np.asarray(y)
        return (x >= 0) & (y >= 0) & (x < self.width) & (y < self.height)

    def is_free(self, x, y):
        """True for points inside the map that fall in a free cell."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        ok = self.in_bounds(x, y)
        ix, iy = self.world_to_cell(np.where(ok, x, 0), np.where(ok, y, 0))
        return ok & ~self.occ[iy, ix]

    def free_cells_world(self) -> np.ndarray:
        """Centres (N, 2) of all free cells, in cm."""
        iy, ix = np.nonzero(~self.occ)
        return np.column_stack(((ix + 0.5) * self.resolution, (iy + 0.5) * self.resolution))

    # -------------------------------------------------------- distance field
    @property
    def distance_field(self) -> np.ndarray:
        """Euclidean distance (cm) from each cell centre to the nearest occupied cell."""
        if self._dist is None:
            self._dist = ndimage.distance_transform_edt(~self.occ) * self.resolution
        return self._dist

    def distance_at(self, x, y, outside: float = np.inf):
        """Distance-field lookup; points outside the map get ``outside``."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        ok = self.in_bounds(x, y)
        ix, iy = self.world_to_cell(np.where(ok, x, 0), np.where(ok, y, 0))
        d = self.distance_field[iy, ix]
        return np.where(ok, d, outside)

    # ----------------------------------------------------------- ray casting
    def raycast(self, x, y, angles, max_range: float) -> np.ndarray:
        """Expected range along each ray (vectorised sphere tracing).

        ``x, y, angles`` broadcast together. Each ray advances by the distance
        to the nearest obstacle (minus one cell for safety, at least half a
        cell), which is exact on a grid and ~10x faster than fixed stepping.
        A ray that leaves the map, or travels ``max_range``, returns ``max_range``.
        """
        x, y, angles = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float),
                                           np.asarray(angles, float))
        shape = x.shape
        x0, y0 = x.ravel(), y.ravel()
        c, s = np.cos(angles.ravel()), np.sin(angles.ravel())
        t = np.zeros_like(x0)
        out = np.full_like(x0, max_range)
        active = np.ones(x0.shape, bool)
        half = 0.5 * self.resolution
        df = self.distance_field
        H, W = self.occ.shape
        for _ in range(4096):
            if not active.any():
                break
            idx = np.nonzero(active)[0]
            px = x0[idx] + t[idx] * c[idx]
            py = y0[idx] + t[idx] * s[idx]
            inside = (px >= 0) & (py >= 0) & (px < self.width) & (py < self.height)
            ix = np.clip((px / self.resolution).astype(int), 0, W - 1)
            iy = np.clip((py / self.resolution).astype(int), 0, H - 1)
            d = df[iy, ix]
            hit = inside & (d == 0)
            out[idx[hit]] = np.minimum(t[idx[hit]], max_range)
            done = hit | ~inside | (t[idx] >= max_range)
            active[idx[done]] = False
            step = np.maximum(d - self.resolution, half)
            t[idx[~done]] += step[~done]
        return out.reshape(shape)

    # ---------------------------------------------------------- likelihood
    def likelihood_field(self, sigma: float) -> np.ndarray:
        """Unnormalised Gaussian of the distance field (for visualisation)."""
        return np.exp(-0.5 * (self.distance_field / sigma) ** 2)
