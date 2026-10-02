"""First-principles DC-motor model: transfer functions, poles, steady state."""

import numpy as np
import pytest

from control.motor import MotorParams, pt2_speed_model


def _ss_freq(a, b, c_row, b_col, w):
    """C (jwI - A)^-1 B for one input/output pair, evaluated directly."""
    n = a.shape[0]
    return np.array([np.linalg.solve(1j * wk * np.eye(n) - a, b[:, b_col])[c_row] for wk in w])


def test_transfer_functions_match_the_state_space_model(true_plant):
    a, b, _, _ = true_plant.state_space()
    w = np.logspace(-1, 4, 60)
    pairs = ((true_plant.tf_angle(), 0, 0), (true_plant.tf_speed(), 1, 0),
             (true_plant.tf_current(), 2, 0), (true_plant.tf_load_to_speed(), 1, 1))
    for tf, out, inp in pairs:
        np.testing.assert_allclose(tf.freq(w), _ss_freq(a, b, out, inp, w), rtol=1e-9)


def test_steady_state_obeys_the_power_balance(true_plant):
    """For constant u: u i = R i^2 + b w^2 (electrical input = copper + friction loss)."""
    a, b, _, _ = true_plant.state_space()
    u = 6.0
    w, i = np.linalg.solve(a[1:, 1:], -b[1:, 0] * u)
    assert w == pytest.approx(true_plant.ks * u, rel=1e-12)
    assert u * i == pytest.approx(true_plant.R * i ** 2 + true_plant.b * w ** 2, rel=1e-12)


def test_load_torque_reduces_speed_by_the_static_droop(true_plant):
    p = true_plant
    droop = p.tf_load_to_speed().dcgain()
    assert droop == pytest.approx(-p.R / (p.K ** 2 + p.R * p.b), rel=1e-12)


def test_pt2_time_constants_are_the_speed_channel_poles(true_plant):
    t1, t2 = true_plant.time_constants
    np.testing.assert_allclose(np.sort(true_plant.tf_speed().poles().real),
                               [-1.0 / t2, -1.0 / t1], rtol=1e-12)
    w = np.logspace(0, 4, 30)
    np.testing.assert_allclose(pt2_speed_model(true_plant.ks, t1, t2).freq(w),
                               true_plant.tf_speed().freq(w), rtol=1e-9)
    # Electrical and mechanical dynamics are well separated, so the poles are
    # close to the textbook approximations L/R and J R / (K^2 + R b).
    assert t2 < 0.05 * t1
    assert t1 == pytest.approx(true_plant.tau_mech, rel=0.05)
    assert t2 == pytest.approx(true_plant.tau_el, rel=0.05)


def test_complex_poles_are_reported_not_hidden():
    lightly_damped = MotorParams(R=0.5, L=20e-3, K=0.32, J=2.2e-3, b=1e-4)
    with pytest.raises(ValueError):
        _ = lightly_damped.time_constants


def test_angle_is_the_integral_of_speed(true_plant):
    dt = 1e-4
    t = np.arange(0.0, 0.3, dt)
    speed = true_plant.tf_speed().step(t)
    angle = true_plant.tf_angle().step(t)
    trapezoid = np.concatenate([[0.0], np.cumsum(0.5 * (speed[1:] + speed[:-1]) * dt)])
    np.testing.assert_allclose(angle, trapezoid, atol=1e-6)
