"""Run MCL on the real robot log and animate it.

    python scripts/run_localization.py                     # likelihood field, tracking
    python scripts/run_localization.py --model beam
    python scripts/run_localization.py --global --particles 3000 --temperature 30
    python scripts/run_localization.py --no-show --save out.png
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from mcl import (BeamModel, LikelihoodFieldModel, MCLConfig, OccupancyMap,  # noqa: E402
                 OdometryMotionModel, ParticleFilter, load_log, run_log)
from mcl.sensor import laser_pose  # noqa: E402
from mcl.data import beam_angles  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", choices=["likelihood", "beam"], default="likelihood")
    ap.add_argument("--particles", type=int, default=1000)
    ap.add_argument("--temperature", type=float, default=1.0)
    ap.add_argument("--global", dest="global_", action="store_true", help="start uniformly over the map")
    ap.add_argument("--augmented", action="store_true", help="augmented MCL (random particle injection)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-show", action="store_true", help="do not animate")
    ap.add_argument("--save", type=str, default="", help="save the final figure")
    a = ap.parse_args()

    import matplotlib
    if a.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    omap = OccupancyMap.from_file(ROOT / "data" / "hallway_map.txt")
    recs = load_log(ROOT / "data" / "robotdata.log")
    sensor = (LikelihoodFieldModel(temperature=a.temperature) if a.model == "likelihood"
              else BeamModel(temperature=a.temperature))
    pf = ParticleFilter(omap, OdometryMotionModel(), sensor,
                        MCLConfig(n_particles=a.particles, augmented=a.augmented), seed=a.seed)
    if a.global_:
        pf.init_uniform_free()
    else:
        pf.init_region((1780, 1860), (0, 56), (np.deg2rad(80), np.deg2rad(100)))

    fig, ax = plt.subplots(figsize=(14, 3.6))
    ax.imshow(omap.occ, origin="lower", cmap="gray_r", extent=[0, omap.width, 0, omap.height])
    parts = ax.scatter(pf.particles[:, 0], pf.particles[:, 1], s=1, c="#d9531f")
    (path,) = ax.plot([], [], color="#2a78d6", lw=2)
    ends = ax.scatter([], [], s=4, c="#1baf7a")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    fig.tight_layout()
    est = []

    def cb(i, rec, p):
        e = p.estimate()
        est.append(e)
        if a.no_show:
            return
        parts.set_offsets(p.particles[:, :2])
        path.set_data(*np.asarray(est)[:, :2].T)
        lx, ly, th = laser_pose(e[None, :])
        z = rec.ranges[::3]
        ang = th[0] + beam_angles()[::3]
        ends.set_offsets(np.column_stack((lx[0] + z * np.cos(ang), ly[0] + z * np.sin(ang))))
        ax.set_title(f"scan {i + 1}   N_eff = {p.last_neff:.0f}   spread = {p.spread():.1f} cm")
        plt.pause(0.001)

    t0 = time.perf_counter()
    run_log(pf, recs, callback=cb)
    e = est[-1]
    print(f"{len(est)} scans in {time.perf_counter() - t0:.1f} s; final pose "
          f"x={e[0]:.1f} cm, y={e[1]:.1f} cm, theta={np.rad2deg(e[2]):.1f} deg")
    path.set_data(*np.asarray(est)[:, :2].T)
    parts.set_offsets(pf.particles[:, :2])
    if a.save:
        fig.savefig(a.save, dpi=130)
    if not a.no_show:
        plt.show()


if __name__ == "__main__":
    main()
