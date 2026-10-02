"""State feedback by Ackermann's formula and the Luenberger observer."""

from dataclasses import replace

import numpy as np
import pytest
from scipy import signal

from control.discrete import StateController
from control.pipeline import _observer_error_model
from control.statespace import (acker, closed_loop_matrix, ctrb, obsv, observer_gain)


def _sorted(ev):
    ev = np.asarray(ev, dtype=complex)
    return ev[np.lexsort((ev.imag, ev.real))]


@pytest.mark.parametrize("seed", range(5))
def test_ackermann_places_the_poles_like_scipy(seed):
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(3, 3))
    b = rng.normal(size=(3, 1))
    poles = np.array([-1.0, -2.0, -3.5])
    k = acker(a, b[:, 0], poles)
    np.testing.assert_allclose(_sorted(np.linalg.eigvals(a - np.outer(b, k))),
                               _sorted(poles), atol=1e-8)
    k_ref = signal.place_poles(a, b, poles).gain_matrix[0]
    np.testing.assert_allclose(k, k_ref, rtol=1e-7)  # single input: the gain is unique


def test_ackermann_handles_complex_pole_pairs():
    a = np.array([[0.0, 1.0], [0.0, -2.0]])
    b = np.array([0.0, 3.0])
    poles = [-4.0 + 3.0j, -4.0 - 3.0j]
    k = acker(a, b, poles)
    np.testing.assert_allclose(_sorted(np.linalg.eigvals(a - np.outer(b, k))), _sorted(poles))


def test_uncontrollable_system_is_rejected():
    a = np.diag([-1.0, -2.0])
    with pytest.raises(ValueError):
        acker(a, np.array([1.0, 0.0]), [-3.0, -4.0])


def test_design_model_is_controllable_and_observable_from_the_angle(pipeline):
    d = pipeline.designs.ssc
    assert np.linalg.matrix_rank(ctrb(d.phi, d.gamma)) == 2
    assert np.linalg.matrix_rank(obsv(d.phi, d.c)) == 2


def test_observer_and_controller_poles_are_placed(pipeline, summary):
    d = pipeline.designs.ssc
    np.testing.assert_allclose(observer_gain(d.phi, d.c, d.obs_poles_z), d.l, rtol=1e-12)
    np.testing.assert_allclose(_sorted(np.linalg.eigvals(d.phi - np.outer(d.l, d.c))),
                               _sorted(d.obs_poles_z), atol=1e-10)
    assert summary["state_space"]["placed_poles_max_error"] < 1e-9
    # separation principle: plant + integrator + observer under output feedback
    # has exactly the controller poles and the observer poles
    assert summary["state_space"]["separation_principle_max_error"] < 1e-9
    ev = np.linalg.eigvals(closed_loop_matrix(d, d.phi, d.gamma, d.c))
    np.testing.assert_allclose(_sorted(ev), _sorted(np.concatenate([d.ctrl_poles_z,
                                                                      d.obs_poles_z])),
                               atol=1e-9)


def test_observer_error_decays_at_the_rate_of_the_observer_poles(pipeline):
    """With the design model as plant the estimation error obeys
    e[k+1] = (Phi - l c) e[k] exactly, whatever the controller does."""
    d, cfg = pipeline.designs.ssc, pipeline.designs.servo.ssc
    x0 = np.array([0.05, 5.0])
    err = _observer_error_model(d, cfg, x0, 80)
    a_err = d.phi - np.outer(d.l, d.c)
    np.testing.assert_allclose(err[40], np.linalg.matrix_power(a_err, 40) @ x0, atol=1e-12)
    slowest = np.max(np.abs(d.obs_poles_z))
    assert np.linalg.norm(err[60]) < 50.0 * slowest ** 60 * np.linalg.norm(x0)
    assert np.linalg.norm(err[60]) < 1e-6 * np.linalg.norm(x0)


def test_observer_converges_on_the_true_plant(summary):
    """The true plant has the armature inductance the design model ignores;
    the speed estimate still converges within a few observer time constants."""
    conv = summary["state_space"]["observer_speed_error_5pct_s"]
    assert conv["design_model"] < 0.02
    assert conv["true_plant"] < 0.02


def test_observer_tracks_a_moving_motor_better_than_the_difference_quotient(pipeline):
    log = pipeline.series["ssc"]["conv"]["true_plant_quantised"]
    late = log.t > 0.03
    true_speed = log.x[late, 1]
    err_obs = np.sqrt(np.mean((log.extra["xhat"][late, 1] - true_speed) ** 2))
    err_dq = np.sqrt(np.mean((log.extra["omega_meas"][late] - true_speed) ** 2))
    assert err_obs < 0.7 * err_dq


def test_state_controller_reference_model_rejects_bad_configs(pipeline):
    cfg = pipeline.designs.servo.ssc
    for bad in (replace(cfg, ts=0.0), replace(cfg, u_min=1.0, u_max=1.0), replace(cfg, tt=-1.0)):
        with pytest.raises(ValueError):
            StateController(bad)
