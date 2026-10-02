"""Classical designs: speed PI (Bode method) and position PD (pole
compensation + dominant pole pair). Designed on the identified model,
verified on the true plant in the exact sampled-data loop."""

import math

import numpy as np
import pytest

from control.design import (PIDGains, overshoot_from_zeta, pole_pair_params,
                            position_pd_compensation, speed_pi_bode, step_metrics)
from control.lti import TF


def test_speed_pi_hits_the_phase_margin_target(summary):
    sp = summary["speed_pi"]
    assert sp["margins_quasi_continuous"]["pm_deg"] == pytest.approx(
        sp["design"]["pm_target_deg"], abs=0.01)
    assert sp["margins_quasi_continuous"]["wc_rad_s"] == pytest.approx(
        sp["design"]["wc_design_rad_s"], rel=1e-6)


def test_speed_pi_keeps_its_margins_on_the_true_sampled_loop(summary):
    m = summary["speed_pi"]["margins_discrete_true_plant"]
    assert m["pm_deg"] >= 55.0
    assert m["gm_db"] >= 12.0


def test_speed_pi_zero_cancels_the_identified_mechanical_pole(pipeline):
    g = pipeline.designs.speed.gains
    assert g.kp / g.ki == pytest.approx(pipeline.designs.pt2.values[1], rel=1e-12)


def test_speed_pi_bode_design_on_a_textbook_plant():
    """Plant 1/((s+1)(0.1s+1)) with a 10 ms delay, PM target 45 deg: the loop
    built from the returned gains must have exactly that phase margin at the
    returned crossover frequency."""
    plant = TF([1.0], np.polymul([1.0, 1.0], [0.1, 1.0]))
    d = speed_pi_bode(plant, 1.0, TF([1.0], [1.0]), 0.01, 45.0)
    w = np.array([d.wc])
    loop = d.gains.freq(w) * plant.freq(w) * np.exp(-1j * w * 0.01)
    assert abs(loop[0]) == pytest.approx(1.0, rel=1e-9)
    assert 180.0 + math.degrees(np.angle(loop[0])) == pytest.approx(45.0, abs=1e-6)


def test_gain_margins_agree_with_closed_loop_poles(summary):
    """Simplified Nyquist criterion, checked independently: scaling the loop
    gain by 0.98 x GM keeps every closed-loop pole inside the unit circle,
    1.02 x GM moves one outside."""
    for key in ("speed_pi", "position_pd"):
        n = summary[key]["nyquist_check_true_plant"]
        assert n["spectral_radius_nominal"] < 1.0
        assert n["spectral_radius_at_0.98_gm"] < 1.0 < n["spectral_radius_at_1.02_gm"]


def test_position_pd_compensates_the_mechanical_pole(summary):
    pos = summary["position_pd"]
    assert pos["pd_zero_rad_s"] == pytest.approx(pos["true_plant_pole_rad_s"], rel=0.01)
    # The compensated pole stays in the closed loop, right on top of the zero:
    # invisible in the reference response, but it sets the slow tail of the
    # disturbance response.
    assert pos["real_poles_discrete_true_plant_rad_s"][-1] == pytest.approx(
        pos["pd_zero_rad_s"], rel=0.01)


def test_position_pd_dominant_pole_pair(summary):
    pos = summary["position_pd"]
    pair = pos["dominant_pair_discrete_true_plant"]
    assert pair["zeta"] == pytest.approx(pos["design"]["zeta_design"], abs=0.03)
    # The pair predicts the overshoot of the full sampled loop
    assert pos["step_linear"]["c=1"]["overshoot"] == pytest.approx(
        pos["design"]["overshoot_predicted"], abs=0.01)
    assert pos["margins_discrete_true_plant"]["pm_deg"] >= 55.0
    assert pos["margins_discrete_true_plant"]["gm_db"] >= 12.0


def test_position_pd_design_formulas():
    d = position_pd_compensation(ks=3.0, t1=0.05, t2=1e-3, tf=2e-3, ts=1e-3, zeta=0.707)
    g = d.gains
    # zero at -1/T1
    assert g.kp / (g.kd + g.kp * g.tf) == pytest.approx(1.0 / 0.05)
    # remaining loop Kp Ks / (s (T_sigma s + 1)) has the requested damping
    assert 1.0 / (2.0 * math.sqrt(g.kp * 3.0 * d.t_sigma)) == pytest.approx(0.707)
    with pytest.raises(ValueError):
        position_pd_compensation(ks=3.0, t1=0.05, t2=1e-3, tf=0.06, ts=1e-3, zeta=0.7)


def test_pid_gains_transfer_function_matches_its_frequency_response():
    g = PIDGains(kp=2.0, ki=30.0, kd=0.1, tf=0.004)
    w = np.logspace(-1, 4, 50)
    np.testing.assert_allclose(g.as_tf().freq(w), g.freq(w), rtol=1e-12)


def test_second_order_helpers():
    zeta, w0 = 0.5, 10.0
    pole = complex(-zeta * w0, w0 * math.sqrt(1 - zeta ** 2))
    assert pole_pair_params(pole) == pytest.approx((w0, zeta))
    t = np.arange(0.0, 3.0, 1e-4)
    y = TF([w0 ** 2], [1.0, 2 * zeta * w0, w0 ** 2]).step(t)
    m = step_metrics(t, y, 1.0)
    assert m["overshoot"] == pytest.approx(overshoot_from_zeta(zeta), rel=1e-4)
    # 2 % settling time of a second-order system: roughly 4 / (zeta w0)
    assert m["settling_time"] == pytest.approx(4.0 / (zeta * w0), rel=0.2)
    assert overshoot_from_zeta(1.0) == 0.0
