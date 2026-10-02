"""The small LTI toolbox: discretisation, simulation and margin computation,
checked against scipy and against loops with known analytic margins."""

import math

import numpy as np
import pytest
from scipy import optimize, signal

from control.lti import (TF, c2d_zoh, dss_freq, margins, spectral_radius, ss2zpoly,
                         zblocks_freq)


def test_c2d_zoh_matches_scipy(true_plant):
    a, b, c, d = true_plant.state_space()
    phi, gam = c2d_zoh(a, b, 1e-3)
    phi_ref, gam_ref, *_ = signal.cont2discrete((a, b, c, d), 1e-3, method="zoh")
    np.testing.assert_allclose(phi, phi_ref, rtol=1e-10, atol=1e-14)
    np.testing.assert_allclose(gam, gam_ref, rtol=1e-10, atol=1e-14)


def test_step_response_matches_scipy():
    g = TF([2.0], [1e-3, 0.05, 1.0])
    t = np.linspace(0.0, 0.3, 3001)
    _, y_ref = signal.step((g.num, g.den), T=t)
    np.testing.assert_allclose(g.step(t), y_ref, atol=1e-6)


def test_feedback_closes_the_loop():
    g = TF([4.0], [1.0, 2.0, 0.0])
    cl = g.feedback()
    np.testing.assert_allclose(cl.den, [1.0, 2.0, 4.0])
    assert cl.dcgain() == pytest.approx(1.0)


def test_margins_of_a_third_order_loop_match_the_analytic_values():
    """L(s) = k / (s (s + 1)(s + 2)): arg L = -180 deg at w = sqrt(2), where
    |L| = k / 6, so the gain margin is 6 / k."""
    k = 2.0
    m = margins(lambda w: k / (1j * w * (1j * w + 1.0) * (1j * w + 2.0)), 1e-2, 1e2)
    wc = optimize.brentq(lambda w: w ** 2 * (w ** 2 + 1) * (w ** 2 + 4) - k ** 2, 0.1, 2.0)
    assert m.w180 == pytest.approx(math.sqrt(2.0), rel=1e-9)
    assert m.gm == pytest.approx(6.0 / k, rel=1e-9)
    assert m.wc == pytest.approx(wc, rel=1e-9)
    pm = 90.0 - math.degrees(math.atan(wc)) - math.degrees(math.atan(wc / 2.0))
    assert m.pm_deg == pytest.approx(pm, abs=1e-7)
    assert m.gm_lower == 0.0  # no lower gain margin: not conditionally stable


def test_margins_of_an_integrator_with_dead_time():
    """L(s) = wc/s exp(-s T): PM = 90 deg - wc T, phase crossover at pi / (2T)."""
    wc, dead = 50.0, 4e-3
    m = margins(lambda w: wc / (1j * w) * np.exp(-1j * w * dead), 1.0, 1e4)
    assert m.wc == pytest.approx(wc, rel=1e-9)
    assert m.pm_deg == pytest.approx(90.0 - math.degrees(wc * dead), abs=1e-7)
    assert m.w180 == pytest.approx(math.pi / (2.0 * dead), rel=1e-9)
    assert m.gm == pytest.approx(math.pi / (2.0 * dead * wc), rel=1e-9)
    assert m.delay_margin == pytest.approx(math.radians(m.pm_deg) / wc)


def test_conditionally_stable_loop_margins_agree_with_closed_loop_roots():
    """L(s) = k (s + 1)^2 / (s^3 (0.01 s + 1)^2) starts at -270 deg and crosses
    -180 deg twice. The simplified Nyquist criterion says the closed loop is
    stable exactly for gain factors between the lower and the upper margin;
    the roots of den + g k num must confirm both boundaries."""
    num = np.polymul([1.0, 1.0], [1.0, 1.0])
    den = np.polymul([1.0, 0.0, 0.0, 0.0], np.polymul([0.01, 1.0], [0.01, 1.0]))
    k = 10.0
    m = margins(lambda w: k * np.polyval(num, 1j * w) / np.polyval(den, 1j * w), 1e-3, 1e4)

    def max_real_pole(gain_factor):
        return np.max(np.real(np.roots(np.polyadd(den, gain_factor * k * num))))

    assert 0.0 < m.gm_lower < 1.0 < m.gm
    assert max_real_pole(1.0) < 0.0
    assert max_real_pole(0.98 * m.gm) < 0.0 < max_real_pole(1.02 * m.gm)
    assert max_real_pole(1.02 * m.gm_lower) < 0.0 < max_real_pole(0.98 * m.gm_lower)


def test_margins_need_a_gain_crossover():
    with pytest.raises(ValueError):
        margins(lambda w: 0.1 / (1j * w + 1.0), 1.0, 1e3)


def test_discrete_frequency_response_and_polynomial_form_agree(true_plant):
    a, b, _, _ = true_plant.state_space()
    ts = 1e-3
    phi, gam = c2d_zoh(a, b[:, [0]], ts)
    c = np.array([0.0, 1.0, 0.0])
    w = np.logspace(0, 3, 40)
    num, den = ss2zpoly(phi, gam[:, 0], c)
    np.testing.assert_allclose(zblocks_freq([num], [den], w, ts),
                               dss_freq(phi, gam[:, 0], c, 0.0, w, ts), rtol=1e-8)


def test_spectral_radius():
    assert spectral_radius(np.poly([0.5, -0.9, 0.3 + 0.4j, 0.3 - 0.4j])) == pytest.approx(0.9)
