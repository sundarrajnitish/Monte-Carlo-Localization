"""Reproduce every number and figure quoted in the README and on the website.

    cd python && python scripts/run_experiments.py            # ~5 min
    cd python && python scripts/run_experiments.py --quick    # smoke test

Writes ``results/*.png``, ``results/results.json`` and the web assets in
``docs/data/``.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from mcl import (BeamModel, LikelihoodFieldModel, MCLConfig, OccupancyMap,  # noqa: E402
                 OdometryMotionModel, ParticleFilter, beam_angles, load_log, run_log, wrap)
from mcl.sensor import laser_pose  # noqa: E402
from mcl.simulate import DEFAULT_WAYPOINTS, make_dataset, waypoint_trajectory  # noqa: E402

RES = ROOT / "results"
WEB = ROOT / "docs" / "data"
START_BOX = dict(x_range=(1780, 1860), y_range=(0, 56),
                 theta_range=(np.deg2rad(80), np.deg2rad(100)))


def log(*a):
    print(*a, flush=True)


def make_pf(omap, sensor, n, seed, **cfg):
    return ParticleFilter(omap, OdometryMotionModel(), sensor, MCLConfig(n_particles=n, **cfg), seed=seed)


def rmse(e, t):
    return float(np.sqrt(np.mean(np.sum((e[:, :2] - t[:, :2]) ** 2, axis=1))))


def dead_reckoning(records, start_pose):
    """Odometry integrated from a known start pose (what you get with no sensor)."""
    o0 = records[0].odom
    out = []
    c0 = np.cos(start_pose[2] - o0[2])
    s0 = np.sin(start_pose[2] - o0[2])
    for r in records:
        if r.kind != "L":
            continue
        dx, dy = r.odom[0] - o0[0], r.odom[1] - o0[1]
        out.append((start_pose[0] + c0 * dx - s0 * dy, start_pose[1] + s0 * dx + c0 * dy,
                    wrap(start_pose[2] + r.odom[2] - o0[2])))
    return np.asarray(out)


def scan_residual(omap, pose, ranges, stride=2):
    """Median distance (cm) from scan end-points to the nearest wall."""
    p = np.asarray(pose)[None, :]
    lx, ly, th = laser_pose(p)
    a = th[0] + beam_angles()[::stride]
    z = ranges[::stride]
    keep = z < 8000
    ex = lx[0] + z[keep] * np.cos(a[keep])
    ey = ly[0] + z[keep] * np.sin(a[keep])
    d = omap.distance_at(ex, ey, outside=np.nan)
    return float(np.nanmedian(d)) if np.isfinite(d).any() else float("nan")


# ----------------------------------------------------------------- E1 real
def exp_real_tracking(omap, recs, quick):
    log("E1 real-data tracking")
    out = {}
    scans = [r for r in recs if r.kind == "L"]
    for name, sensor in [("likelihood_field", LikelihoodFieldModel()), ("beam", BeamModel())]:
        pf = make_pf(omap, sensor, 1000, 0)
        pf.init_region(**START_BOX)
        t0 = time.perf_counter()
        h = run_log(pf, recs, snapshot_every=1)
        dt = time.perf_counter() - t0
        a = h.arrays()
        out[name] = dict(
            estimate=np.round(a["estimate"], 2).tolist(),
            spread=np.round(a["spread"], 2).tolist(),
            neff=np.round(a["neff"], 1).tolist(),
            ms_per_scan=1000 * dt / len(scans),
            residual=[scan_residual(omap, e, s.ranges) for e, s in zip(a["estimate"], scans)],
        )
        if name == "likelihood_field":
            snaps = h.snapshots
        log(f"  {name}: {1000 * dt / len(scans):.1f} ms/scan")
    lf = np.asarray(out["likelihood_field"]["estimate"])
    bm = np.asarray(out["beam"]["estimate"])
    dr = dead_reckoning(recs, lf[0])
    out["dead_reckoning"] = dict(estimate=np.round(dr, 2).tolist(),
                                 residual=[scan_residual(omap, e, s.ranges) for e, s in zip(dr, scans)])
    out["lf_vs_beam_cm"] = dict(mean=float(np.mean(np.hypot(*(lf[:, :2] - bm[:, :2]).T))),
                                max=float(np.max(np.hypot(*(lf[:, :2] - bm[:, :2]).T))))
    out["dr_vs_lf_final_cm"] = float(np.hypot(*(dr[-1, :2] - lf[-1, :2])))
    out["path_length_cm"] = float(np.sum(np.hypot(*np.diff(lf[:, :2], axis=0).T)))
    # the two odometry streams: O records vs the pose stamped on L records
    def stream_stats(rr):
        o = np.array([r.odom for r in rr])
        d = np.diff(o, axis=0)
        mv = np.hypot(d[:, 0], d[:, 1]) > 1e-3
        rel = np.abs(wrap(np.arctan2(d[:, 1], d[:, 0]) - o[:-1, 2]))
        return dict(path_cm=float(np.hypot(d[:, 0], d[:, 1]).sum()), reversals=int((rel[mv] > np.pi / 2).sum()))
    out["odometry_streams"] = {"O only": stream_stats([r for r in recs if r.kind == "O"]),
                               "L only": stream_stats([r for r in recs if r.kind == "L"]),
                               "interleaved": stream_stats(recs)}
    pf = make_pf(omap, LikelihoodFieldModel(), 1000, 0)
    pf.init_region(**START_BOX)
    eb = run_log(pf, recs, odom_source="both").arrays()
    out["interleaved_run"] = dict(
        median_residual=float(np.nanmedian([scan_residual(omap, e, s.ranges) for e, s in zip(eb["estimate"], scans)])),
        mean_spread=float(np.mean(eb["spread"])),
        mean_spread_laser=float(np.mean(out["likelihood_field"]["spread"])))
    out["duration_s"] = float(scans[-1].ts - recs[0].ts)

    fig, ax = plt.subplots(figsize=(14, 3.2))
    ax.imshow(omap.occ, origin="lower", cmap="gray_r", extent=[0, omap.width, 0, omap.height])
    for i, p in snaps[::30]:
        ax.scatter(p[:, 0], p[:, 1], s=0.5, alpha=0.4)
    ax.plot(dr[:, 0], dr[:, 1], "--", color="#999", lw=1.2, label="odometry only")
    ax.plot(bm[:, 0], bm[:, 1], color="#e07b39", lw=1.5, label="MCL, beam model")
    ax.plot(lf[:, 0], lf[:, 1], color="#2f6fdf", lw=1.5, label="MCL, likelihood field")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(RES / "real_tracking.png", dpi=130)
    plt.close(fig)
    return out


# ----------------------------------------------------------------- E2 global
def exp_real_global(omap, recs, ref, quick):
    log("E2 real-data global localisation")
    temps = [1, 3, 10, 30]
    ns = [1000, 3000] if not quick else [1000]
    seeds = range(20 if not quick else 2)
    table = []
    for T in temps:
        for n in ns:
            succ, conv = 0, []
            for seed in seeds:
                pf = make_pf(omap, LikelihoodFieldModel(temperature=T), n, 100 + seed)
                pf.init_uniform_free()
                e = run_log(pf, recs).arrays()["estimate"]
                err = np.hypot(*(e[:, :2] - ref[:, :2]).T)
                ok = bool(np.all(err[-20:] < 30))
                succ += ok
                bad = np.nonzero(err >= 30)[0]
                if ok:
                    conv.append(int(bad[-1] + 1) if len(bad) else 0)
            table.append(dict(temperature=T, n=n, success=succ, runs=len(seeds),
                              median_converge_scan=float(np.median(conv)) if conv else None))
            log(f"  T={T:>3} N={n}: {succ}/{len(seeds)}")
    return table


# ----------------------------------------------------------------- synthetic
def synthetic_set(omap, n_sets, seed0=0, **kw):
    truth = waypoint_trajectory(DEFAULT_WAYPOINTS)
    return [make_dataset(omap, truth, np.random.default_rng(seed0 + k), **kw) for k in range(n_sets)]


def exp_particles(omap, sets, quick):
    log("E3 accuracy vs particle count (synthetic)")
    ns = [25, 50, 100, 250, 500, 1000, 2000] if not quick else [50, 200]
    rows = []
    for name, mk in [("likelihood_field", lambda: LikelihoodFieldModel()), ("beam", lambda: BeamModel())]:
        for n in ns:
            errs, fails, ms = [], 0, []
            for k, (recs, tr) in enumerate(sets):
                pf = make_pf(omap, mk(), n, k)
                pf.init_region(**START_BOX)
                t0 = time.perf_counter()
                e = run_log(pf, recs).arrays()["estimate"]
                ms.append(1000 * (time.perf_counter() - t0) / len(tr))
                r = rmse(e, tr)
                if r > 100:
                    fails += 1
                else:
                    errs.append(r)
            rows.append(dict(model=name, n=n, rmse=float(np.mean(errs)) if errs else None,
                             rmse_sd=float(np.std(errs)) if errs else None,
                             failures=fails, runs=len(sets), ms_per_scan=float(np.mean(ms))))
            log(f"  {name:17s} N={n:5d} rmse={rows[-1]['rmse']} fails={fails} {np.mean(ms):.1f} ms")
    return rows


def exp_resamplers(omap, sets, quick):
    log("E4 resampler comparison (synthetic)")
    rows = []
    for scheme in ["low_variance", "stratified", "multinomial"]:
        for thr, label in [(1.01, "every update"), (0.5, "adaptive (N_eff < M/2)")]:
            errs, uniq, fails = [], [], 0
            for k, (recs, tr) in enumerate(sets):
                pf = make_pf(omap, LikelihoodFieldModel(), 200, 50 + k,
                             resampler=scheme, resample_threshold=thr)
                pf.init_region(**START_BOX)
                u = []

                def cb(i, rec, p, u=u):
                    if p.resampled:
                        u.append(len(np.unique(p.particles[:, 0])))
                e = run_log(pf, recs, callback=cb).arrays()["estimate"]
                r = rmse(e, tr)
                (errs.append(r) if r < 100 else None)
                fails += r >= 100
                uniq.append(np.mean(u) if u else 200)
            rows.append(dict(resampler=scheme, policy=label, rmse=float(np.mean(errs)) if errs else None,
                             unique_after_resample=float(np.mean(uniq)), failures=int(fails), runs=len(sets)))
            log(f"  {scheme:12s} {label:24s} rmse={rows[-1]['rmse']:.2f} unique={np.mean(uniq):.0f}")
    return rows


def recovered_after(err, tol=30.0):
    """Scans until the error drops below ``tol`` for good (None = never)."""
    bad = np.nonzero(err >= tol)[0]
    if len(bad) == 0:
        return 0
    return int(bad[-1] + 1) if bad[-1] + 1 < len(err) - 10 else None


def exp_kidnap(omap, quick):
    log("E5 kidnapped robot (synthetic)")
    truth = waypoint_trajectory(DEFAULT_WAYPOINTS)
    # robot is driving west down the hallway (x ~ 1400) when it is picked up
    # and put back in the start doorway; odometry sees nothing.
    kid = int(np.nonzero((truth[:, 0] < 1400) & (np.cos(truth[:, 2]) < -0.9))[0][0]) & ~1
    out = {"kidnap_scan": kid // 2, "kidnap_from": truth[kid].tolist(), "kidnap_to": truth[0].tolist(),
           "curves": {}}
    seeds = range(5 if not quick else 1)
    for label, aug in [("standard MCL", False), ("augmented MCL", True)]:
        curves = []
        for s in seeds:
            recs, tr = make_dataset(omap, truth, np.random.default_rng(900 + s),
                                    kidnap_at=kid, kidnap_to=0, kidnap_len=320)
            pf = make_pf(omap, LikelihoodFieldModel(), 2000, s, augmented=aug)
            pf.init_region(**START_BOX)
            e = run_log(pf, recs).arrays()["estimate"]
            curves.append(np.hypot(*(e[:, :2] - tr[:, :2]).T))
        c = np.median(np.asarray(curves), axis=0)
        rec = [recovered_after(cv[kid // 2 + 1:]) for cv in curves]
        out["curves"][label] = dict(median_error=np.round(c, 1).tolist(), recovered_after_scans=rec)
        log(f"  {label}: recovered after {rec}")
    return out


def exp_dead_reckoning(omap, sets):
    """Odometry-only vs MCL error over time, and the along/across-corridor split."""
    dr_rows, mcl_rows, ex, ey, spreads = [], [], [], [], []
    for k, (recs, tr) in enumerate(sets):
        dr = dead_reckoning(recs, tr[0])
        dr_rows.append(np.hypot(*(dr[:, :2] - tr[:, :2]).T))
        pf = make_pf(omap, LikelihoodFieldModel(), 1000, 300 + k)
        pf.init_region(**START_BOX)
        h = run_log(pf, recs).arrays()
        e = h["estimate"]
        spreads.append(np.mean(h["spread"]))
        d = e[:, :2] - tr[:, :2]
        mcl_rows.append(np.hypot(d[:, 0], d[:, 1]))
        ex.append(np.sqrt(np.mean(d[:, 0] ** 2)))
        ey.append(np.sqrt(np.mean(d[:, 1] ** 2)))
    c = np.median(np.asarray(dr_rows), axis=0)
    m = np.median(np.asarray(mcl_rows), axis=0)
    return dict(median_error=np.round(c, 1).tolist(), final=float(c[-1]),
                mcl_median_error=np.round(m, 1).tolist(),
                rmse_along_corridor=float(np.mean(ex)), rmse_across_corridor=float(np.mean(ey)),
                mcl_rmse=float(np.mean([np.sqrt(np.mean(r ** 2)) for r in mcl_rows])),
                mcl_mean_spread=float(np.mean(spreads)))


def exp_sensor_curves():
    """Beam-model component curves for the article figure."""
    bm = BeamModel(max_range=1000.0, sigma_hit=20.0, lambda_short=0.005)
    z = np.linspace(0, 1000, 501)
    ph, ps, pm, pr = bm.components(z, np.full_like(z, 600.0))
    return dict(z=z.tolist(), z_star=600.0, hit=ph.tolist(), short=ps.tolist(), rand=pr.tolist())


# ----------------------------------------------------------------- web data
def export_web(omap, recs, real):
    WEB.mkdir(parents=True, exist_ok=True)
    rows = ["".join("1" if v else "0" for v in row) for row in omap.occ]   # row 0 = bottom
    # only the laser records: their odometry stamp drives the motion model
    scans = [[*np.round(r.odom, 4).tolist(), round(r.ts, 3), [int(round(v)) for v in r.ranges]]
             for r in recs if r.kind == "L"]
    json.dump(dict(resolution=omap.resolution, width=omap.occ.shape[1], height=omap.occ.shape[0],
                   rows=rows, start_box=dict(x=START_BOX["x_range"], y=START_BOX["y_range"],
                                             theta=list(START_BOX["theta_range"])),
                   scans=scans,
                   reference=dict(lf=real["likelihood_field"]["estimate"],
                                  beam=real["beam"]["estimate"],
                                  odom=real["dead_reckoning"]["estimate"])),
              open(WEB / "dataset.json", "w"), separators=(",", ":"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    RES.mkdir(exist_ok=True)
    omap = OccupancyMap.from_file(ROOT / "data" / "hallway_map.txt")
    recs = load_log(ROOT / "data" / "robotdata.log")

    results = {}
    results["real_tracking"] = real = exp_real_tracking(omap, recs, args.quick)
    ref = np.asarray(real["likelihood_field"]["estimate"])
    results["real_global"] = exp_real_global(omap, recs, ref, args.quick)
    sets = synthetic_set(omap, 5 if not args.quick else 1)
    results["synthetic_scans"] = len(sets[0][1])
    results["dead_reckoning_synthetic"] = exp_dead_reckoning(omap, sets)
    results["particles"] = exp_particles(omap, sets, args.quick)
    results["resamplers"] = exp_resamplers(omap, sets, args.quick)
    results["kidnap"] = exp_kidnap(omap, args.quick)
    results["sensor_curves"] = exp_sensor_curves()

    small = {k: v for k, v in results.items() if k != "real_tracking"}
    small["real_tracking_summary"] = {k: real[k] for k in ("lf_vs_beam_cm", "dr_vs_lf_final_cm",
                                                           "path_length_cm", "duration_s",
                                                           "odometry_streams", "interleaved_run")}
    small["real_tracking_summary"]["ms_per_scan"] = {m: real[m]["ms_per_scan"] for m in ("likelihood_field", "beam")}
    small["real_tracking_summary"]["median_residual_cm"] = {
        m: float(np.nanmedian(real[m]["residual"])) for m in ("likelihood_field", "beam", "dead_reckoning")}
    small["real_series"] = {m: {k: real[m][k] for k in ("spread", "neff", "residual") if k in real[m]}
                            for m in ("likelihood_field", "beam", "dead_reckoning")}
    json.dump(small, open(RES / "results.json", "w"), indent=1)
    WEB.mkdir(parents=True, exist_ok=True)
    json.dump(small, open(WEB / "results.json", "w"), separators=(",", ":"))
    export_web(omap, recs, real)
    log("done ->", RES)


if __name__ == "__main__":
    main()
