"""Discretisation: ZOH plant, Tustin controller, quasi-continuous design."""

import math

import numpy as np
import pytest
from scipy import signal

from control.discrete import LowPass1, PIDConfig, pid_zpoly
from control.lti import c2d_zoh, zblocks_freq


def test_sampled_speed_loop_converges_to_the_continuous_design(summary):
    rows = summary["discretisation"]["speed_loop_convergence"]
    ts = np.array([r["ts_s"] for r in rows])
    dev = np.array([r["max_deviation_rel"] for r in rows])
    order = np.argsort(ts)[::-1]
    ts, dev = ts[order], dev[order]
    assert np.all(np.diff(dev) < 0)                  # smaller Ts, smaller deviation
    # first-order convergence: halving Ts roughly halves the deviation
    halving = [(i, j) for i in range(len(ts)) for j in range(len(ts))
               if math.isclose(ts[i], 2.0 * ts[j])]
    assert halving
    for i, j in halving:
        assert 1.7 < dev[i] / dev[j] < 2.3
    assert dev[ts == 1e-3][0] < 0.03                 # 1 kHz: within 3 % of the design


def test_zoh_acts_like_half_a_sample_of_dead_time(summary):
    d = summary["discretisation"]
    assert d["plant_zoh_phase_lag_at_wc_deg"] == pytest.approx(
        d["half_sample_delay_at_wc_deg"], abs=0.01)


def test_tustin_is_the_most_accurate_controller_substitution(summary):
    m = summary["discretisation"]["position_pd_methods"]
    tustin = abs(m["tustin"]["controller_phase_error_at_wc_deg"])
    assert tustin < 0.1
    assert abs(m["backward_euler"]["controller_phase_error_at_wc_deg"]) > 10.0 * tustin
    assert abs(m["forward_euler"]["controller_phase_error_at_wc_deg"]) > 10.0 * tustin
    # backward Euler adds lag, forward Euler lead: the phase margin moves accordingly
    assert (m["backward_euler"]["pm_true_plant_deg"] < m["tustin"]["pm_true_plant_deg"]
            < m["forward_euler"]["pm_true_plant_deg"])


@pytest.mark.parametrize("gains", [dict(kp=1.3, ki=27.0), dict(kp=36.0, kd=1.65, tf=3e-3),
                                   dict(kp=2.0, ki=40.0, kd=0.05, tf=2e-3)])
def test_implemented_difference_equations_are_the_bilinear_transform(gains):
    """pid_zpoly reproduces mc_pid.c; it must equal scipy's Tustin transform
    of the continuous controller kp + ki/s + kd s / (tf s + 1)."""
    ts = 1e-3
    cfg = PIDConfig(ts=ts, **gains)
    kp, ki, kd, tf = cfg.kp, cfg.ki, cfg.kd, cfg.tf
    num_s, den_s = np.array([kp]), np.array([1.0])
    if ki:
        num_s, den_s = np.polyadd(np.polymul(num_s, [1.0, 0.0]), ki), np.polymul(den_s, [1.0, 0.0])
    if kd:
        num_s = np.polyadd(np.polymul(num_s, [tf, 1.0]), np.polymul(den_s, [kd, 0.0]))
        den_s = np.polymul(den_s, [tf, 1.0])
    num_z, den_z = signal.bilinear(num_s, den_s, fs=1.0 / ts)
    w = np.logspace(0, math.log10(0.99 * math.pi / ts), 50)
    np.testing.assert_allclose(zblocks_freq([pid_zpoly(cfg)[0]], [pid_zpoly(cfg)[1]], w, ts),
                               zblocks_freq([num_z], [den_z], w, ts), rtol=1e-9)


def test_speed_low_pass_is_the_exact_zoh_discretisation():
    tau, ts = 5e-3, 1e-3
    phi, gam = c2d_zoh(np.array([[-1.0 / tau]]), np.array([[1.0 / tau]]), ts)
    f = LowPass1(tau, ts)
    assert 1.0 - f.a == pytest.approx(phi[0, 0], rel=1e-14)
    assert f.a == pytest.approx(gam[0, 0], rel=1e-12)
