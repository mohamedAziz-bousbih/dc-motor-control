"""Parameter identification from (synthetic) voltage-step data."""

import numpy as np
import pytest

from control import identification as ident
from control.motor import MotorParams
from control.pipeline import DATASET


@pytest.fixture(scope="module")
def data() -> ident.StepData:
    return ident.load_step_csv(DATASET)


def _fit_all(data: ident.StepData):
    pt1 = ident.fit_pt1(data)
    pt2 = ident.fit_pt2(data)
    phys = ident.fit_physical(data, ident.physical_initial_guess(data, pt1))
    return pt1, pt2, phys


def _assert_within_ci(fit: ident.FitResult, truth, factor: float = 1.5) -> None:
    """|estimate - truth| <= factor * (95 % half-width), i.e. within ~3 sigma."""
    err = np.abs(fit.values - np.asarray(truth))
    assert np.all(err <= factor * fit.ci95), dict(zip(fit.names, err / fit.ci95))


def test_dataset_is_labelled_synthetic():
    first_line = DATASET.read_text(encoding="utf-8").splitlines()[0]
    assert first_line.startswith("# SYNTHETIC DATA")


def test_csv_round_trip(tmp_path, true_plant):
    d = ident.make_synthetic_step(true_plant, seed=5)
    path = tmp_path / "step.csv"
    ident.save_step_csv(d, path)
    back = ident.load_step_csv(path)
    assert back.header == d.header
    np.testing.assert_allclose(back.omega, d.omega, atol=5e-6)
    np.testing.assert_allclose(back.current, d.current, atol=5e-6)
    assert back.onset == pytest.approx(d.onset)


def test_identified_parameters_match_the_true_plant(data, true_plant):
    p = true_plant
    _, pt2, phys = _fit_all(data)
    _assert_within_ci(pt2, [p.ks, *p.time_constants])
    _assert_within_ci(phys, [p.R, p.L, p.K, p.J, p.b])
    rel = np.abs(pt2.values / [p.ks, *p.time_constants] - 1.0)
    assert rel[0] < 0.005 and rel[1] < 0.01   # Ks, T1: the parameters the design relies on
    rel = np.abs(phys.values / [p.R, p.L, p.K, p.J, p.b] - 1.0)
    assert np.all(rel < [0.01, 0.03, 0.01, 0.01, 0.03])


def test_identification_works_on_a_different_motor():
    """A small, fast motor (T1 = 31 ms, T2 = 0.3 ms) with noisier speed data."""
    p = MotorParams(R=4.0, L=1.2e-3, K=0.05, J=2e-5, b=5e-6)
    d = ident.make_synthetic_step(p, u_step=6.0, t_step=0.01, t_end=0.2, fs=20000.0,
                                  sigma_w=1.0, sigma_i=0.02, seed=7)
    _, pt2, phys = _fit_all(d)
    _assert_within_ci(pt2, [p.ks, *p.time_constants])
    _assert_within_ci(phys, [p.R, p.L, p.K, p.J, p.b])


def test_pt2_confidence_intervals_have_nominal_coverage(true_plant):
    """Monte-Carlo over 40 noise realisations: the 95 % intervals must contain
    the true values in roughly 95 % of the runs (the PT2 structure is exact for
    this plant, so the estimator is unbiased)."""
    p = true_plant
    truth = np.array([p.ks, *p.time_constants])
    runs = 40
    hits = np.zeros(3)
    for seed in range(runs):
        fit = ident.fit_pt2(ident.make_synthetic_step(p, seed=1000 + seed))
        hits += np.abs(fit.values - truth) <= fit.ci95
    assert np.all(hits >= 0.85 * runs), hits / runs


def test_fit_is_reproducible_from_any_starting_point(data, monkeypatch):
    """The Gauss-Newton polish makes the result independent of where the
    solver started, so the gains derived from it are bit-stable."""
    reference = ident.fit_pt2(data).values
    original = ident._initial_pt1
    for factor in (0.6, 1.5):
        monkeypatch.setattr(ident, "_initial_pt1",
                            lambda d, f=factor: tuple(np.array(original(d)) * f))
        np.testing.assert_allclose(ident.fit_pt2(data).values, reference, rtol=1e-12)


def test_analytic_jacobians_match_finite_differences():
    tau = np.linspace(-0.01, 0.2, 400)
    for fun, jac, theta in ((ident.pt1_step, ident.pt1_jacobian, np.log([3.0, 0.05])),
                            (ident.pt2_step, ident.pt2_jacobian, np.log([3.0, 0.05, 0.002]))):
        j = jac(tau, *np.exp(theta))
        for k in range(len(theta)):
            h = np.zeros_like(theta)
            h[k] = 1e-6
            fd = (fun(tau, *np.exp(theta + h)) - fun(tau, *np.exp(theta - h))) / 2e-6
            np.testing.assert_allclose(j[:, k], fd, atol=1e-7 * np.max(np.abs(fd)))


def test_residual_whiteness_rejects_the_pt1_structure(data):
    """PT1 misses the electrical pole: its residuals are correlated. PT2 is the
    exact structure, so its residuals are white like the sensor noise."""
    pt1, pt2, _ = _fit_all(data)
    bound = ident.Z95 / np.sqrt(len(data.t))

    def inside(res):
        return np.mean(np.abs(ident.autocorrelation(res, 50)[1:]) < bound)

    assert inside(pt1.residuals) < 0.2
    assert inside(pt2.residuals) > 0.85
    assert pt2.rmse == pytest.approx(data.pre_step_noise()[0], rel=0.05)
