"""Unit tests for the MCL building blocks.  Run:  cd python && python -m pytest -q"""
from pathlib import Path

import numpy as np
import pytest

from mcl import (BeamModel, LikelihoodFieldModel, MCLConfig, OccupancyMap, OdometryMotionModel,
                 ParticleFilter, decompose, effective_sample_size, load_log, low_variance,
                 multinomial, run_log, stratified, wrap)
from mcl.simulate import DEFAULT_WAYPOINTS, make_dataset, waypoint_trajectory

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def omap():
    return OccupancyMap.from_file(ROOT / "data" / "hallway_map.txt")


def box_map():
    occ = np.zeros((50, 100), bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return OccupancyMap(occ, resolution=4.0)


# ------------------------------------------------------------------ map
def test_map_shape_and_orientation(omap):
    assert omap.occ.shape == (85, 700)
    assert omap.width == 2800 and omap.height == 340
    # the start doorway is open at the bottom of the map around x = 1820 cm
    assert omap.is_free(1822, 10) and omap.is_free(1822, 200)
    assert not omap.is_free(1500, 10)            # wall block beside the doorway


def test_raycast_box():
    m = box_map()
    # from the centre (200, 100) the walls' inner faces are at x=4 / x=396, y=4 / y=196
    z = m.raycast(200.0, 100.0, np.array([0, np.pi / 2, np.pi, -np.pi / 2]), 8000.0)
    assert np.allclose(z, [196, 96, 196, 96], atol=m.resolution)


def test_raycast_leaves_map_returns_max():
    occ = np.zeros((20, 20), bool)
    m = OccupancyMap(occ, 4.0)
    assert m.raycast(40.0, 40.0, 0.0, 500.0) == 500.0


def test_distance_field(omap):
    iy, ix = np.nonzero(omap.occ)
    assert np.all(omap.distance_field[iy, ix] == 0)
    assert omap.distance_at(-5, -5) == np.inf


# --------------------------------------------------------------- motion
def test_wrap():
    assert np.isclose(wrap(3 * np.pi), -np.pi)
    assert np.isclose(wrap(-np.pi / 2), -np.pi / 2)


def test_decompose_forward_and_pure_rotation():
    r1, t, r2 = decompose(np.array([0, 0, 0.0]), np.array([10, 0, 0.5]))
    assert np.isclose(r1, 0) and np.isclose(t, 10) and np.isclose(r2, 0.5)
    r1, t, r2 = decompose(np.array([5, 5, 0.2]), np.array([5, 5, -0.3]))
    assert r1 == 0 and t == 0 and np.isclose(r2, -0.5)


def test_decompose_reverse_is_negative_translation():
    r1, t, r2 = decompose(np.array([0, 0, 0.0]), np.array([-10, 0, 0.0]))
    assert np.isclose(t, -10) and np.isclose(r1, 0) and np.isclose(r2, 0)


def test_decompose_across_pi_boundary():
    r1, t, r2 = decompose(np.array([0, 0, 3.1]), np.array([0, 0, -3.1]))
    assert np.isclose(r2, 2 * np.pi - 6.2)


def test_motion_noise_free_is_exact():
    mm = OdometryMotionModel(0, 0, 0, 0)
    p = np.array([[100.0, 50.0, np.pi / 2]])
    out = mm.sample(p, np.array([0, 0, 0.0]), np.array([10, 0, 0.0]), np.random.default_rng(0))
    assert np.allclose(out, [[100, 60, np.pi / 2]])


def test_motion_noise_statistics():
    mm = OdometryMotionModel(a1=0, a2=0, a3=0.01, a4=0)
    p = np.zeros((20000, 3))
    out = mm.sample(p, np.zeros(3), np.array([100.0, 0, 0]), np.random.default_rng(1))
    assert abs(out[:, 0].mean() - 100) < 0.2
    assert abs(out[:, 0].std() - 10) < 0.3          # sqrt(0.01 * 100^2) = 10 cm


# ------------------------------------------------------------ resampling
@pytest.mark.parametrize("fn", [low_variance, stratified, multinomial])
def test_resamplers_follow_weights(fn):
    w = np.array([0.0, 0.5, 0.25, 0.25])
    idx = np.concatenate([fn(np.repeat(w, 1000) / 1000, np.random.default_rng(s)) for s in range(3)])
    counts = np.bincount(idx // 1000, minlength=4) / idx.size
    assert counts[0] == 0
    assert np.allclose(counts, w, atol=0.03)


def test_low_variance_keeps_all_when_uniform():
    w = np.full(100, 0.01)
    assert np.array_equal(np.sort(low_variance(w, np.random.default_rng(3))), np.arange(100))


def test_neff():
    assert np.isclose(effective_sample_size(np.full(10, 0.1)), 10)
    assert np.isclose(effective_sample_size(np.array([1.0, 0, 0])), 1)


# --------------------------------------------------------------- sensors
def test_beam_density_normalised():
    bm = BeamModel(max_range=1000.0)
    z = np.linspace(0, 1000, 200001)
    p = bm.density(z, np.full_like(z, 400.0))
    # continuous part integrates to (1 - z_max); the max-range spike carries z_max
    cont = np.trapezoid(np.where(z < 999, p, 0), z)
    assert abs(cont - (1 - bm.z_max)) < 0.01


@pytest.mark.parametrize("model", [LikelihoodFieldModel(), BeamModel()])
def test_sensor_prefers_true_pose(omap, model):
    from mcl.simulate import sample_scan
    truth = np.array([1300.0, 190.0, np.pi])
    z = sample_scan(omap, truth, np.random.default_rng(0))
    cand = np.array([truth, truth + [0, 40, 0], truth + [0, -40, 0], truth + [0, 0, 0.3]])
    ll = model.log_likelihood(cand, z, omap)
    assert np.argmax(ll) == 0


# ------------------------------------------------------------------ data
def test_load_log():
    recs = load_log(ROOT / "data" / "robotdata.log")
    assert sum(r.kind == "O" for r in recs) == 522
    scans = [r for r in recs if r.kind == "L"]
    assert len(scans) == 240 and scans[0].ranges.shape == (180,)
    assert all(recs[i].ts <= recs[i + 1].ts for i in range(len(recs) - 1))


# ------------------------------------------------------------ end-to-end
def test_tracking_on_synthetic_data(omap):
    truth = waypoint_trajectory(DEFAULT_WAYPOINTS)[:300]
    recs, tr = make_dataset(omap, truth, np.random.default_rng(7))
    pf = ParticleFilter(omap, OdometryMotionModel(), LikelihoodFieldModel(),
                        MCLConfig(n_particles=300), seed=0)
    pf.init_region((1780, 1860), (0, 56), (np.deg2rad(80), np.deg2rad(100)))
    est = run_log(pf, recs).arrays()["estimate"]
    err = np.hypot(*(est[:, :2] - tr[:, :2]).T)
    assert np.sqrt(np.mean(err**2)) < 20


def test_tracking_on_real_log_stays_in_free_space(omap):
    recs = load_log(ROOT / "data" / "robotdata.log")
    pf = ParticleFilter(omap, OdometryMotionModel(), LikelihoodFieldModel(),
                        MCLConfig(n_particles=300), seed=0)
    pf.init_region((1780, 1860), (0, 56), (np.deg2rad(80), np.deg2rad(100)))
    est = run_log(pf, recs).arrays()["estimate"]
    assert omap.is_free(est[:, 0], est[:, 1]).all()
    # robot leaves the doorway and drives ~11 m west down the hallway
    assert est[-1, 0] < 800 and 120 < est[-1, 1] < 260
