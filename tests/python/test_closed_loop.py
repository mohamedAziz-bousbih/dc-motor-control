"""Closed-loop behaviour of the servo on the true third-order plant:
reference, disturbance and actuating behaviour, anti-windup, bumpless
transfer. The servo here is the Python model that the C code is checked
against sample by sample."""

import math
from dataclasses import replace

import numpy as np
import pytest

from control.lti import c2d_zoh
from control.servo import Loop, Mode, Servo


def test_speed_loop_tracks_and_rejects_a_load_step(summary):
    sp = summary["speed_pi"]
    step = sp["step_20rad_s_quantised_saturated"]
    assert step["overshoot"] < 0.10
    assert step["settling_time"] is not None and step["settling_time"] < 0.1
    # PI = type-1 loop: the load-torque step is rejected completely
    assert sp["load_recovery_2pct_s"] is not None and sp["load_recovery_2pct_s"] < 0.15
    assert 0.0 < sp["load_max_speed_drop_rad_s"] < 0.1 * 20.0


def test_back_calculation_limits_windup(summary):
    for aw in (summary["speed_pi"]["anti_windup_30rad_s"],
               summary["state_space"]["anti_windup_3rad"]):
        assert aw["back_calculation"]["overshoot"] < 0.02
        assert aw["none"]["overshoot"] > 5.0 * aw["back_calculation"]["overshoot"] + 0.1


def test_position_pd_load_and_ramp_errors_match_the_type_1_prediction(summary):
    """The PD loop has one integrator (in the plant): a ramp leaves the error
    v / (Kp Ks), a load torque the error R tau / (K Kp)."""
    pos = summary["position_pd"]
    assert pos["move_2pi_slew"]["ramp_following_error_rad"] == pytest.approx(
        pos["predicted_ramp_error_rad"], rel=0.03)
    assert pos["move_2pi_slew"]["load_steady_state_error_rad"] == pytest.approx(
        pos["predicted_load_error_rad"], rel=0.01)


def test_state_controller_integral_action(summary, drive):
    ss = summary["state_space"]
    count = drive.rad_per_count
    # integral state: no steady-state error under load, down to the encoder resolution
    assert abs(ss["move_2pi_slew_quantised"]["load_steady_state_error_rad"]) < count
    assert abs(ss["move_2pi_slew"]["load_steady_state_error_rad"]) < 1e-6
    # kr = 0: the reference enters through the integrator only, ramp error v k0 / ki
    assert ss["move_2pi_slew_kr0"]["ramp_following_error_rad"] == pytest.approx(
        ss["predicted_ramp_error_kr0_rad"], rel=0.01)
    # kr = 1 removes it
    assert abs(ss["move_2pi_slew"]["ramp_following_error_rad"]) < 1e-3


def test_all_controllers_are_stable_on_the_true_plant(summary):
    assert summary["speed_pi"]["nyquist_check_true_plant"]["spectral_radius_nominal"] < 1.0
    assert summary["position_pd"]["nyquist_check_true_plant"]["spectral_radius_nominal"] < 1.0
    assert summary["state_space"]["spectral_radius_true_plant"] < 1.0


# --- bumpless transfer ---------------------------------------------------------

def _run(pipeline, true_plant, drive, action, k_action, n=300, quantise=True, load=0.1,
         setpoint=10.0):
    """Speed mode towards `setpoint` against a load; `action(servo)` runs
    before the update of sample k_action. Returns the controller outputs."""
    servo = Servo(pipeline.designs.servo, 0)
    servo.set_mode(Mode.SPEED)
    servo.set_setpoint(setpoint)
    a, b, _, _ = true_plant.state_space()
    phi, gam = c2d_zoh(a, b, drive.ts)
    q = drive.rad_per_count
    x = np.zeros(3)
    us = np.empty(n)
    for k in range(n):
        if k == k_action:
            action(servo)
        counts = math.floor(x[0] / q) if quantise else x[0] / q
        us[k] = servo.update(counts)
        x = phi @ x + gam @ np.array([min(max(us[k], -12.0), 12.0), load])
    return us


def test_switch_from_speed_to_state_mode_is_bumpless(pipeline, true_plant, drive):
    u = _run(pipeline, true_plant, drive, lambda s: s.set_mode(Mode.STATE), 200)
    assert abs(u[200] - u[199]) < 1e-9

    def hard_switch(s):  # what happens without the bumpless initialisation
        s.mode = Mode.STATE
        s.target = s.r_eff = s.slew.y = s.theta
        s.ssc.reset(s.theta)

    u_hard = _run(pipeline, true_plant, drive, hard_switch, 200)
    assert abs(u_hard[200] - u_hard[199]) > 1.0


def test_gain_change_keeps_the_output_for_the_same_measurement(pipeline):
    s = Servo(pipeline.designs.servo, 0)
    s.set_mode(Mode.SPEED)
    s.set_setpoint(5.0)
    for counts in range(0, 40, 2):
        s.update(counts)
    pid = s.speed_pid
    before = pid._unsaturated(s.r_eff, s.omega)
    s.set_pid_gains(Loop.SPEED, 3.0 * pid.cfg.kp, 0.5 * pid.cfg.ki, 0.0)
    assert pid._unsaturated(s.r_eff, s.omega) == pytest.approx(before, abs=1e-12)


def test_gain_change_during_a_transient_is_bumpless(pipeline, true_plant, drive):
    """2 rad/s step (no saturation); gains doubled after 10 samples, while the
    speed error is still large."""
    def double_gains(s):
        s.set_pid_gains(Loop.SPEED, 2.0 * s.cfg.speed.kp, 2.0 * s.cfg.speed.ki, 0.0)

    def naive(s):  # new gains, integrator left alone
        s.speed_pid.cfg = replace(s.speed_pid.cfg, kp=2.0 * s.cfg.speed.kp,
                                  ki=2.0 * s.cfg.speed.ki)

    k = 10
    run = dict(n=40, quantise=False, load=0.0, setpoint=2.0)
    u = _run(pipeline, true_plant, drive, double_gains, k, **run)
    u_naive = _run(pipeline, true_plant, drive, naive, k, **run)
    regular = np.max(np.abs(np.diff(u[k - 4:k])))
    # doubled gains make the output move about twice as fast, but it does not jump
    assert abs(u[k] - u[k - 1]) < 3.0 * regular
    assert abs(u_naive[k] - u_naive[k - 1]) > 10.0 * regular


def test_reported_speed_is_the_observer_estimate_only_in_state_mode(pipeline):
    s = Servo(pipeline.designs.servo, 0)
    s.set_mode(Mode.SPEED)
    for counts in (0, 3, 6, 9):
        s.update(counts)
    assert s.omega_hat == s.omega
    s.set_mode(Mode.STATE)
    s.update(12)
    assert s.omega_hat == s.ssc.xhat[1]
    assert s.omega_hat != s.omega


def test_voltage_mode_is_clamped_and_starts_at_zero(pipeline):
    s = Servo(pipeline.designs.servo, 0)
    s.set_mode(Mode.SPEED)
    s.set_setpoint(5.0)
    s.update(0)
    s.set_mode(Mode.VOLTAGE)
    assert s.update(0) == 0.0
    s.set_setpoint(-50.0)
    assert s.update(0) == -pipeline.designs.servo.v_max


def test_nonfinite_output_switches_the_servo_off(pipeline):
    """A NaN/inf controller output must never reach the bridge (mc_servo.c
    mirrors this check)."""
    s = Servo(pipeline.designs.servo, 0)
    s.set_mode(Mode.POSITION)
    s.set_slew(Loop.POSITION, 0.0)
    s.set_setpoint(math.nan)
    assert s.update(0) == 0.0
    assert s.mode == Mode.OFF and s.nonfinite_trips == 1
    s.set_mode(Mode.POSITION)
    assert s.update(0) == pytest.approx(0.0, abs=1e-12)
