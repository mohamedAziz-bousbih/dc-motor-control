"""End-to-end pipeline: identification -> design -> discrete verification.

run() is pure computation. It returns the summary that scripts/run_pipeline.py
writes to docs/results.json, the time series for the plots and the controller
configurations for the firmware header and the golden vectors. Controllers
are always *designed* on the identified model and *verified* on the true
plant (MotorParams defaults), so the error of the identification step is part
of every verification result.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np

from . import identification as ident
from .design import (PIDGains, PositionPDDesign, SpeedPIDesign, overshoot_from_zeta,
                     pole_pair_params, position_pd_compensation, speed_pi_bode, step_metrics)
from .discrete import PIDConfig, SSCConfig, StateController, pid_zpoly, ssc_compensator
from .lti import (TF, c2d_zoh, dss_freq, margins, spectral_radius, ss2zpoly, zblocks_freq,
                  zpoly_closed_loop)
from .motor import DriveParams, MotorParams, pt2_speed_model
from .servo import Mode, Servo, ServoConfig, unlimited
from .simulation import simulate
from .statespace import StateControllerDesign, augment, closed_loop_matrix, design_state_controller

ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / "data" / "step_response_synthetic.csv"


@dataclass(frozen=True)
class Spec:
    """Design specifications. Every number here is a deliberate choice."""

    speed_pm_deg: float = 60.0              # phase margin target of the speed loop
    speed_filter_tau: float = 5e-3          # low-pass on the encoder difference quotient [s]
    speed_tt_factor: float = 1.0            # anti-windup tracking time Tt = factor * Tn
    speed_slew_rate: float = 0.0            # speed setpoint rate limit [rad/s^2], 0 = off
    pos_zeta: float = 1.0 / math.sqrt(2.0)  # damping of the dominant pole pair
    pos_deriv_filter: float = 3e-3          # derivative filter of the position PD [s]
    pos_setpoint_weight_c: float = 1.0      # D on the error: keeps the T1 compensation for r
    pos_slew_rate: float = 20.0             # position setpoint rate limit [rad/s]
    ssc_w0: float = 60.0                    # state controller: dominant pair [rad/s]
    ssc_zeta: float = 0.8
    ssc_integrator_pole: float = 60.0       # real pole of the integral action [rad/s]
    ssc_observer_poles: tuple[float, float] = (300.0, 360.0)  # [rad/s], ~5x faster
    ssc_kr: float = 1.0                     # reference weight, see StateController
    ssc_tt: float = 0.02                    # anti-windup tracking time of the integrator [s]


@dataclass
class Designs:
    pt1: ident.FitResult
    pt2: ident.FitResult
    speed: SpeedPIDesign
    pos: PositionPDDesign
    ssc: StateControllerDesign
    servo: ServoConfig        # nominal configuration: actuator and setpoint limits active


@dataclass
class PipelineOutput:
    summary: dict
    designs: Designs | None = None
    series: dict = field(default_factory=dict)


def _r(x, sig: int = 6):
    """Round floats (recursively) to `sig` significant digits for the JSON file.

    Non-finite values become None (JSON null): NaN marks "did not happen within
    the record" (e.g. no settling), +-inf "does not exist" (e.g. no phase
    crossover, so no gain margin). Strict JSON has no literal for either.
    """
    if isinstance(x, dict):
        return {k: _r(v, sig) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_r(v, sig) for v in x]
    if isinstance(x, (float, np.floating)):
        x = float(x)
        if not math.isfinite(x):
            return None
        if x == 0.0:
            return x
        return float(f"{x:.{sig}g}")
    if isinstance(x, np.integer):
        return int(x)
    return x


def _margins_dict(m) -> dict:
    return {"wc_rad_s": m.wc, "pm_deg": m.pm_deg, "w180_rad_s": m.w180, "gm_db": m.gm_db,
            "w180_lower_rad_s": m.w180_lower, "gm_lower_db": m.gm_lower_db,
            "delay_margin_s": m.delay_margin}


def _angle_zoh(plant_tf: TF, ts: float):
    """ZOH equivalent (Phi, Gamma, c, d) of a continuous SISO transfer function."""
    a, b, c, d = plant_tf.to_ss()
    phi, gam = c2d_zoh(a, b, ts)
    return phi, gam[:, 0], c[0], float(d[0, 0])


def _true_angle_zoh(p: MotorParams, ts: float):
    """ZOH equivalent of the true third-order plant from voltage to angle."""
    a, b, _, _ = p.state_space()
    phi, gam = c2d_zoh(a, b[:, [0]], ts)
    return phi, gam[:, 0], np.array([1.0, 0.0, 0.0]), 0.0


def _nyquist_check(nums, dens, gm: float) -> dict:
    """Simplified Nyquist criterion, checked on the exact discrete loop: the
    closed loop is stable at the nominal gain and loses stability exactly when
    the loop gain is scaled by the gain margin."""
    return {"spectral_radius_nominal": spectral_radius(zpoly_closed_loop(nums, dens, 1.0)),
            "spectral_radius_at_0.98_gm": spectral_radius(zpoly_closed_loop(nums, dens, 0.98 * gm)),
            "spectral_radius_at_1.02_gm": spectral_radius(zpoly_closed_loop(nums, dens, 1.02 * gm))}


def _whiteness(res: np.ndarray, lags: int = 50) -> float:
    """Fraction of autocorrelation lags 1..lags inside the 95 % band of white noise."""
    acf = ident.autocorrelation(res, lags)[1:]
    return float(np.mean(np.abs(acf) < ident.Z95 / math.sqrt(len(res))))


def _servo(cfg: ServoConfig, mode: Mode, counts0: float = 0.0) -> Servo:
    s = Servo(cfg, counts0)
    s.set_mode(mode)
    return s


def _settle_time(t: np.ndarray, y: np.ndarray, target: float, band: float) -> float:
    """Time after t[0] at which |y - target| enters `band` for good (nan if never)."""
    outside = np.flatnonzero(np.abs(y - target) > band)
    if outside.size == 0:
        return 0.0
    if outside[-1] + 1 >= len(t):
        return float("nan")
    return float(t[outside[-1] + 1] - t[0])


# ------------------------------------------------------------------------------
def identify(data: ident.StepData, true: MotorParams, out: PipelineOutput):
    pt1 = ident.fit_pt1(data)
    pt2 = ident.fit_pt2(data)
    guess = ident.physical_initial_guess(data, pt1)
    phys = ident.fit_physical(data, guess)
    t1, t2 = true.time_constants

    def err(v, ref):
        return 100.0 * (v - ref) / ref

    out.summary["identification"] = {
        "dataset": str(DATASET.relative_to(ROOT)).replace("\\", "/"),
        "samples": int(len(data.t)),
        "step_V": data.onset[2],
        "noise_std_pre_step": dict(zip(("omega_rad_s", "current_A"), data.pre_step_noise())),
        "pt1": {**pt1.as_dict(), "rmse": pt1.rmse, "residual_whiteness": _whiteness(pt1.residuals)},
        "pt2": {**pt2.as_dict(), "rmse": pt2.rmse, "residual_whiteness": _whiteness(pt2.residuals)},
        "physical_initial_guess": dict(asdict(guess)),
        "physical": {**phys.as_dict(), "rmse_speed": phys.rmse,
                     "residual_whiteness": _whiteness(phys.residuals)},
        "error_percent": {
            "pt1_Ks": err(pt1.values[0], true.ks),
            "pt1_T_vs_T1_plus_T2": err(pt1.values[1], t1 + t2),
            "pt2_Ks": err(pt2.values[0], true.ks),
            "pt2_T1": err(pt2.values[1], t1),
            "pt2_T2": err(pt2.values[2], t2),
            **{f"physical_{n}": err(v, getattr(true, n)) for n, v in zip(phys.names, phys.values)},
        },
    }
    _, t0, du = data.onset
    tau = data.t - t0
    w_phys, i_phys = ident.physical_step(tau, MotorParams(*phys.values))
    out.series["ident"] = {
        "t": data.t, "omega": data.omega, "current": data.current,
        "pt1": du * ident.pt1_step(tau, *pt1.values),
        "pt2": du * ident.pt2_step(tau, *pt2.values),
        "phys_omega": du * w_phys, "phys_current": du * i_phys,
        "acf_pt1": ident.autocorrelation(pt1.residuals, 50),
        "acf_pt2": ident.autocorrelation(pt2.residuals, 50),
        "acf_bound": ident.Z95 / math.sqrt(len(data.t)),
    }
    return pt1, pt2


# ------------------------------------------------------------------------------
def design(pt1: ident.FitResult, pt2: ident.FitResult, drive: DriveParams, spec: Spec) -> Designs:
    """All controller designs, based on the identified models only."""
    ts, vmax = drive.ts, drive.v_supply
    ks, t1, t2 = pt2.values

    meas = TF([1.0], [spec.speed_filter_tau, 1.0])
    # Quasi-continuous delay model: ZOH (Ts/2) + backward-difference speed estimate (Ts/2)
    d_speed = speed_pi_bode(pt2_speed_model(ks, t1, t2), t1, meas, ts, spec.speed_pm_deg)
    d_pos = position_pd_compensation(ks, t1, t2, spec.pos_deriv_filter, ts, spec.pos_zeta)

    ks1, tm = pt1.values
    w0, z = spec.ssc_w0, spec.ssc_zeta
    s_pair = w0 * complex(-z, math.sqrt(1.0 - z * z))
    ctrl_s = np.array([s_pair, s_pair.conjugate(), -spec.ssc_integrator_pole])
    obs_s = -np.asarray(spec.ssc_observer_poles, dtype=float)
    d_ssc = design_state_controller(ks1, tm, ts, ctrl_s, obs_s)

    g_s, g_p = d_speed.gains, d_pos.gains
    servo = ServoConfig(
        ts=ts, rad_per_count=drive.rad_per_count, v_max=vmax,
        speed=PIDConfig(kp=g_s.kp, ki=g_s.ki, ts=ts, u_min=-vmax, u_max=vmax,
                        tt=spec.speed_tt_factor * d_speed.tn),
        speed_lpf_tau=spec.speed_filter_tau, speed_slew=spec.speed_slew_rate,
        pos=PIDConfig(kp=g_p.kp, kd=g_p.kd, tf=g_p.tf, b=1.0, c=spec.pos_setpoint_weight_c,
                      ts=ts, u_min=-vmax, u_max=vmax),
        pos_slew=spec.pos_slew_rate,
        ssc=SSCConfig(phi=d_ssc.phi, gamma=d_ssc.gamma, l=d_ssc.l, k=d_ssc.k, ki=d_ssc.ki,
                      ts=ts, kr=spec.ssc_kr, u_min=-vmax, u_max=vmax, tt=spec.ssc_tt))
    return Designs(pt1=pt1, pt2=pt2, speed=d_speed, pos=d_pos, ssc=d_ssc, servo=servo)


# ------------------------------------------------------------------------------
def _speed_blocks(cfg: PIDConfig, lpf_tau: float, angle_zoh, ts: float):
    """z-domain blocks of the sampled speed loop: PI, ZOH plant (voltage -> angle)
    followed by the backward difference (z - 1)/(Ts z), and the first-order
    low-pass a z/(z - (1 - a)).

    The angle integrator (pole at z = 1) and the difference quotient (zero at
    z = 1) cancel exactly; the factor is divided out explicitly, otherwise it
    would stay in the characteristic polynomial as an (uncontrollable) root on
    the unit circle: the angle itself is not regulated in speed mode.
    """
    a = 1.0 - math.exp(-ts / lpf_tau)
    num_c, den_c = pid_zpoly(replace(cfg, u_min=-math.inf, u_max=math.inf, tt=0.0))
    num_g, den_g = ss2zpoly(*angle_zoh)
    den_g, rem = np.polydiv(den_g, np.array([1.0, -1.0]))
    if np.max(np.abs(rem)) > 1e-9 * np.max(np.abs(den_g)):
        raise ValueError("angle model has no integrator")
    return ([num_c, num_g, np.array([1.0]), np.array([a, 0.0])],
            [den_c, den_g, np.array([ts, 0.0]), np.array([1.0, -(1.0 - a)])])


def verify_speed(des: Designs, true: MotorParams, drive: DriveParams, spec: Spec,
                 out: PipelineOutput) -> None:
    ts = drive.ts
    ks, t1, t2 = des.pt2.values
    model = pt2_speed_model(ks, t1, t2)
    meas = TF([1.0], [spec.speed_filter_tau, 1.0])
    d, cfg = des.speed, des.servo.speed
    g = d.gains
    w_nyq = 0.999 * math.pi / ts

    def loop_quasi(w):
        return g.freq(w) * model.freq(w) * meas.freq(w) * np.exp(-1j * w * ts)

    model_angle = TF([ks], np.polymul(np.polymul([t1, 1.0], [t2, 1.0]), [1.0, 0.0]))
    blk_model = _speed_blocks(cfg, spec.speed_filter_tau, _angle_zoh(model_angle, ts), ts)
    blk_true = _speed_blocks(cfg, spec.speed_filter_tau, _true_angle_zoh(true, ts), ts)
    m_qc = margins(loop_quasi, 0.1, 1e4)
    m_dm = margins(lambda w: zblocks_freq(*blk_model, w, ts), 0.1, w_nyq)
    m_dt = margins(lambda w: zblocks_freq(*blk_true, w, ts), 0.1, w_nyq)

    # Linear step: continuous design on the true plant vs sampled implementation
    step = 5.0
    lin = unlimited(des.servo)
    t_lin = np.arange(0, 0.3, ts)
    cont = (g.as_tf() * true.tf_speed()).feedback(meas).step(t_lin) * step
    log_lin = simulate(true, drive, _servo(lin, Mode.SPEED), lambda t: step, 0.3,
                       quantise=False, limit_voltage=False)

    # Reference and disturbance behaviour with saturation and encoder quantisation
    w_ref, tau_load, t_load = 20.0, 0.3, 0.25
    log_real = simulate(true, drive, _servo(des.servo, Mode.SPEED),
                        lambda t: w_ref if t >= 0.02 else 0.0, 0.5,
                        load=lambda t: tau_load if t >= t_load else 0.0)
    after = log_real.t >= t_load
    w_after = log_real.x[after, 1]
    before = (log_real.t >= t_load - 0.05) & (log_real.t < t_load)

    # Anti-windup: a step that saturates the bridge, with and without back-calculation
    aw_logs, aw_metrics = {}, {}
    for name, tt in (("back_calculation", cfg.tt), ("none", 0.0)):
        c_aw = replace(des.servo, speed=replace(cfg, tt=tt))
        lg = simulate(true, drive, _servo(c_aw, Mode.SPEED), lambda t: 30.0 if t >= 0.02 else 0.0, 0.5)
        aw_logs[name], aw_metrics[name] = lg, step_metrics(lg.t, lg.x[:, 1], 30.0, 0.02)

    out.summary["speed_pi"] = {
        "design": {"method": "Bode: PI zero on dominant pole, crossover from phase condition",
                   "kp_V_per_rad_s": g.kp, "ki_V_per_rad": g.ki, "tn_s": d.tn,
                   "tt_s": cfg.tt, "wc_design_rad_s": d.wc, "pm_target_deg": spec.speed_pm_deg,
                   "speed_filter_tau_s": spec.speed_filter_tau, "delay_model_s": ts},
        "margins_quasi_continuous": _margins_dict(m_qc),
        "margins_discrete_model": _margins_dict(m_dm),
        "margins_discrete_true_plant": _margins_dict(m_dt),
        "nyquist_check_true_plant": _nyquist_check(*blk_true, m_dt.gm),
        "step_linear_continuous": step_metrics(t_lin, cont, step),
        "step_linear_discrete": step_metrics(log_lin.t, log_lin.x[:, 1], step),
        "step_linear_max_deviation_rel": float(np.max(np.abs(log_lin.x[:, 1] - cont)) / step),
        "step_20rad_s_quantised_saturated": step_metrics(log_real.t[~after], log_real.x[~after, 1],
                                                         w_ref, 0.02),
        "load_step_Nm": tau_load,
        "load_max_speed_drop_rad_s": float(w_ref - np.min(w_after)),
        "load_recovery_2pct_s": _settle_time(log_real.t[after], w_after, w_ref, 0.02 * w_ref),
        "speed_ripple_std_before_load_rad_s": float(np.std(log_real.x[before, 1])),
        "anti_windup_30rad_s": aw_metrics,
    }
    out.series["speed"] = {
        "bode_w": np.logspace(0, math.log10(w_nyq), 600), "loop_quasi": loop_quasi,
        "loop_disc_true": lambda w: zblocks_freq(*blk_true, w, ts),
        "margins": (m_qc, m_dt), "t_lin": t_lin, "cont": cont, "log_lin": log_lin,
        "log_real": log_real, "aw": aw_logs, "t_load": t_load,
    }


# ------------------------------------------------------------------------------
def _pd_blocks(cfg: PIDConfig, angle_zoh):
    num_c, den_c = pid_zpoly(replace(cfg, u_min=-math.inf, u_max=math.inf))
    num_g, den_g = ss2zpoly(*angle_zoh)
    return [num_c, num_g], [den_c, den_g]


def _move_scenario(servo: Servo, true, drive, t_load=0.8, tau=0.2, quantise=True):
    """2 pi move (rate-limited by the servo), then a load-torque step while holding."""
    target = 2.0 * math.pi
    return simulate(true, drive, servo, lambda t: target if t >= 0.05 else 0.0, 1.2,
                    load=lambda t: tau if t >= t_load else 0.0, quantise=quantise)


def _move_errors(log, slew_rate: float, t_load: float = 0.8) -> dict:
    target = 2.0 * math.pi
    t_ramp_end = 0.05 + target / slew_rate
    e = log.r - log.x[:, 0]
    ramp = (log.t > t_ramp_end - 0.1) & (log.t < t_ramp_end)
    hold = log.t > t_load + 0.25
    return {"ramp_following_error_rad": float(np.mean(e[ramp])),
            "load_steady_state_error_rad": float(np.mean(e[hold])),
            "max_abs_error_rad": float(np.max(np.abs(e)))}


def verify_position(des: Designs, true: MotorParams, drive: DriveParams, spec: Spec,
                    out: PipelineOutput) -> None:
    ts = drive.ts
    d, cfg = des.pos, des.servo.pos
    g = d.gains
    blk = _pd_blocks(cfg, _true_angle_zoh(true, ts))
    m_dt = margins(lambda w: zblocks_freq(*blk, w, ts), 0.1, 0.999 * math.pi / ts)
    zp = np.roots(zpoly_closed_loop(*blk))
    sp = np.log(zp.astype(complex)) / ts
    cplx = sp[np.imag(sp) > 1e-6]
    pair = cplx[np.argmax(np.real(cplx))]
    w0, zeta = pole_pair_params(pair)
    real_poles = np.sort(np.real(sp[np.abs(np.imag(sp)) <= 1e-6]))

    # Linear 0.1 rad step: derivative on the error (c = 1) vs on the measurement (c = 0)
    step = 0.1
    lin_logs, lin_metrics = {}, {}
    for cw in (1.0, 0.0):
        c_lin = unlimited(replace(des.servo, pos=replace(cfg, c=cw)))
        lg = simulate(true, drive, _servo(c_lin, Mode.POSITION), lambda t: step if t >= 0.01 else 0.0,
                      0.4, quantise=False, limit_voltage=False)
        lin_logs[cw], lin_metrics[f"c={cw:g}"] = lg, step_metrics(lg.t, lg.x[:, 0], step, 0.01)

    # Pure second-order response the dominant-pole-pair design predicts
    t_pred = np.arange(0, 0.4, ts)
    pred = TF([d.w0 ** 2], [1.0, 2 * d.zeta * d.w0, d.w0 ** 2]).step(t_pred) * step

    move = _move_scenario(_servo(des.servo, Mode.POSITION), true, drive)
    move_lin = _move_scenario(_servo(des.servo, Mode.POSITION), true, drive, quantise=False)

    out.summary["position_pd"] = {
        "design": {"method": "PD zero cancels T1; gain from dominant pole pair damping",
                   "kp_V_per_rad": g.kp, "kd_V_s_per_rad": g.kd, "tf_s": g.tf,
                   "setpoint_weight_c": cfg.c, "t_sigma_s": d.t_sigma, "zeta_design": d.zeta,
                   "w0_design_rad_s": d.w0, "overshoot_predicted": d.overshoot},
        "margins_discrete_true_plant": _margins_dict(m_dt),
        "nyquist_check_true_plant": _nyquist_check(*blk, m_dt.gm),
        "dominant_pair_discrete_true_plant": {"w0_rad_s": w0, "zeta": zeta,
                                              "overshoot_of_pair": overshoot_from_zeta(zeta)},
        "real_poles_discrete_true_plant_rad_s": [float(v) for v in real_poles],
        "pd_zero_rad_s": -g.kp / (g.kd + g.kp * g.tf),
        "true_plant_pole_rad_s": -1.0 / true.time_constants[0],
        "step_linear": lin_metrics,
        "move_2pi_slew": {"slew_rate_rad_s": spec.pos_slew_rate,
                          **_move_errors(move_lin, spec.pos_slew_rate)},
        "move_2pi_slew_quantised": _move_errors(move, spec.pos_slew_rate),
        "predicted_ramp_error_rad": spec.pos_slew_rate / (g.kp * true.ks),
        "predicted_load_error_rad": true.R * 0.2 / (true.K * g.kp),
    }
    out.series["position"] = {"lin": lin_logs, "t_pred": t_pred, "pred": pred, "move": move,
                              "poles_s": sp, "step": step}


# ------------------------------------------------------------------------------
def _observer_error_model(d: StateControllerDesign, cfg: SSCConfig, x0, n: int) -> np.ndarray:
    """Closed loop with the design model itself as the plant: by the separation
    principle the estimation error obeys e[k+1] = (Phi - l c) e[k] exactly."""
    ctrl = StateController(cfg)
    ctrl.reset(0.0)
    x = np.asarray(x0, dtype=float).copy()
    err = np.empty((n, 2))
    for k in range(n):
        err[k] = x - ctrl.xhat
        u = ctrl.update(0.0, x[0])
        x = d.phi @ x + d.gamma * u
    return err


def verify_ssc(des: Designs, true: MotorParams, drive: DriveParams, spec: Spec,
               out: PipelineOutput) -> None:
    ts = drive.ts
    d, cfg = des.ssc, des.servo.ssc
    ks, tm = des.pt1.values

    # Pole placement, separation principle on the design model, robustness on the true plant
    phi_a, gam_a = augment(d.phi, d.gamma, ts)
    placed = np.linalg.eigvals(phi_a - np.outer(gam_a, np.concatenate([d.k, [d.ki]])))
    ev_model = np.linalg.eigvals(closed_loop_matrix(d, d.phi, d.gamma, d.c))
    expected = np.concatenate([d.ctrl_poles_z, d.obs_poles_z])
    phi_t, gam_t, c_t, _ = _true_angle_zoh(true, ts)
    rho_true = float(np.max(np.abs(np.linalg.eigvals(closed_loop_matrix(d, phi_t, gam_t, c_t)))))

    a_c, b_c, c_c = ssc_compensator(cfg)

    def loop_input(w):   # loop broken at the plant input: L = -K(z) G(z)
        return -dss_freq(a_c, b_c, c_c, 0.0, w, ts) * dss_freq(phi_t, gam_t, c_t, 0.0, w, ts)

    m_in = margins(loop_input, 0.1, 0.999 * math.pi / ts)

    # Observer convergence. Design model: exact error dynamics. True plant: the
    # motor turns at 5 rad/s when the state mode is entered with a speed estimate of 0.
    x0_model = np.array([0.05, 5.0])
    err_model = _observer_error_model(d, cfg, x0_model, 100)
    x0 = np.array([0.3, 5.0, true.b * 5.0 / true.K])
    q = drive.rad_per_count
    conv = {}
    for name, quant in (("true_plant", False), ("true_plant_quantised", True)):
        srv = Servo(des.servo, math.floor(x0[0] / q) if quant else x0[0] / q)
        srv.set_mode(Mode.STATE)
        conv[name] = simulate(true, drive, srv, None, 0.1, quantise=quant, x0=x0,
                              record=lambda s: {"xhat": s.ssc.xhat.copy(), "omega_meas": s.omega})
    err_true = conv["true_plant"].x[:, 1] - conv["true_plant"].extra["xhat"][:, 1]

    def t_below(err, frac):
        above = np.flatnonzero(np.abs(err) > frac * abs(err[0]))
        return float((above[-1] + 1) * ts) if above.size else 0.0

    # Linear 0.1 rad step with both reference weights
    step = 0.1
    lin_logs, lin_metrics = {}, {}
    for kr in (0.0, 1.0):
        c_lin = unlimited(replace(des.servo, ssc=replace(cfg, kr=kr)))
        lg = simulate(true, drive, _servo(c_lin, Mode.STATE), lambda t: step if t >= 0.01 else 0.0,
                      0.4, quantise=False, limit_voltage=False)
        lin_logs[kr], lin_metrics[f"kr={kr:g}"] = lg, step_metrics(lg.t, lg.x[:, 0], step, 0.01)

    # Anti-windup: 3 rad step without setpoint rate limit drives the bridge into
    # saturation for ~80 ms; the integrator winds up unless back-calculation is on.
    aw_logs, aw_metrics = {}, {}
    for name, tt in (("back_calculation", cfg.tt), ("none", 0.0)):
        c_aw = replace(des.servo, pos_slew=0.0, ssc=replace(cfg, tt=tt))
        lg = simulate(true, drive, _servo(c_aw, Mode.STATE), lambda t: 3.0 if t >= 0.01 else 0.0, 1.0)
        aw_logs[name], aw_metrics[name] = lg, step_metrics(lg.t, lg.x[:, 0], 3.0, 0.01)

    move = _move_scenario(_servo(des.servo, Mode.STATE), true, drive)
    move_lin = _move_scenario(_servo(des.servo, Mode.STATE), true, drive, quantise=False)
    move_kr0 = _move_scenario(_servo(replace(des.servo, ssc=replace(cfg, kr=0.0)), Mode.STATE),
                              true, drive, quantise=False)

    out.summary["state_space"] = {
        "design_model": {"Ks": ks, "Tm_s": tm, "states": ["theta", "omega"]},
        "controller_poles_s": [[p.real, p.imag] for p in np.log(d.ctrl_poles_z) / ts],
        "observer_poles_s": [float(np.real(p)) for p in np.log(d.obs_poles_z) / ts],
        "k": [float(v) for v in d.k], "ki": d.ki, "l": [float(v) for v in d.l], "kr": cfg.kr,
        "phi": d.phi.tolist(), "gamma": d.gamma.tolist(),
        "placed_poles_max_error": float(max(np.min(np.abs(placed - e)) for e in d.ctrl_poles_z)),
        "separation_principle_max_error": float(max(np.min(np.abs(ev_model - e)) for e in expected)),
        "spectral_radius_design": float(np.max(np.abs(expected))),
        "spectral_radius_true_plant": rho_true,
        "margins_input_true_plant": _margins_dict(m_in),
        "observer_speed_error_5pct_s": {"design_model": t_below(err_model[:, 1], 0.05),
                                        "true_plant": t_below(err_true, 0.05)},
        "step_linear": lin_metrics,
        "move_2pi_slew": _move_errors(move_lin, spec.pos_slew_rate),
        "move_2pi_slew_quantised": _move_errors(move, spec.pos_slew_rate),
        "move_2pi_slew_kr0": _move_errors(move_kr0, spec.pos_slew_rate),
        "predicted_ramp_error_kr0_rad": spec.pos_slew_rate * d.k[0] / d.ki,
        "anti_windup_3rad": aw_metrics,
    }
    out.series["ssc"] = {"conv": conv, "err_model": err_model, "move": move, "lin": lin_logs,
                         "step": step, "aw": aw_logs}


# ------------------------------------------------------------------------------
def _s_of_z(method: str, z: np.ndarray, ts: float) -> np.ndarray:
    if method == "tustin":
        return 2.0 / ts * (z - 1.0) / (z + 1.0)
    if method == "backward_euler":
        return (z - 1.0) / (ts * z)
    if method == "forward_euler":
        return (z - 1.0) / ts
    raise ValueError(method)


def verify_discretisation(des: Designs, true: MotorParams, drive: DriveParams, spec: Spec,
                          out: PipelineOutput) -> None:
    """Quasi-continuous design check.

    1. The sampled speed loop converges to the continuous design as Ts -> 0.
    2. The ZOH equivalent of the plant lags the continuous plant by the Ts/2
       dead time assumed in the quasi-continuous design.
    3. The position PD discretised with three s -> z substitutions.
    """
    cfg = des.servo.speed
    g = PIDGains(kp=cfg.kp, ki=cfg.ki)
    meas = TF([1.0], [spec.speed_filter_tau, 1.0])
    cl = (g.as_tf() * true.tf_speed()).feedback(meas)
    rows, logs = [], {}
    for ts in (2e-3, 1e-3, 5e-4, 2e-4, 1e-4):
        dr = replace(drive, ts=ts)
        c_ts = unlimited(replace(des.servo, ts=ts, speed=replace(cfg, ts=ts),
                                 pos=replace(des.servo.pos, ts=ts), ssc=replace(des.servo.ssc, ts=ts)))
        lg = simulate(true, dr, _servo(c_ts, Mode.SPEED), lambda t: 1.0, 0.3,
                      quantise=False, limit_voltage=False)
        logs[ts] = lg
        rows.append({"ts_s": ts, "max_deviation_rel": float(np.max(np.abs(lg.x[:, 1] - cl.step(lg.t))))})

    ts = drive.ts
    wc = des.speed.wc
    phi, gam, c, dd = _angle_zoh(true.tf_speed(), ts)
    g_zoh = dss_freq(phi, gam, c, dd, np.array([wc]), ts)[0]
    zoh_lag = float(-np.degrees(np.angle(g_zoh / true.tf_speed().freq(np.array([wc]))[0])))

    pos = des.servo.pos
    c_pd = des.pos.gains.as_tf()
    true_zoh = _true_angle_zoh(true, ts)
    wcp = margins(lambda w: c_pd(1j * w) * true.tf_angle().freq(w), 0.1, 1e4).wc
    w_nyq = 0.999 * math.pi / ts
    methods = {}
    for name in ("tustin", "backward_euler", "forward_euler"):
        def c_z(w, name=name):
            return c_pd(_s_of_z(name, np.exp(1j * np.asarray(w) * ts), ts))
        m = margins(lambda w, c_z=c_z: c_z(w) * dss_freq(*true_zoh, w, ts), 0.1, w_nyq)
        ratio = c_z(np.array([wcp]))[0] / c_pd(1j * wcp)
        methods[name] = {"controller_phase_error_at_wc_deg": float(np.degrees(np.angle(ratio))),
                         "controller_gain_error_at_wc_db": float(20 * np.log10(abs(ratio))),
                         "pm_true_plant_deg": m.pm_deg, "gm_true_plant_db": m.gm_db}
    # the Tustin variant is the one implemented: check the substitution against pid_zpoly
    tustin_vs_impl = float(np.max(np.abs(
        c_pd(_s_of_z("tustin", np.exp(1j * np.logspace(0, 3, 50) * ts), ts))
        - zblocks_freq([pid_zpoly(pos)[0]], [pid_zpoly(pos)[1]], np.logspace(0, 3, 50), ts))))

    out.summary["discretisation"] = {
        "speed_loop_convergence": rows,
        "speed_wc_rad_s": wc,
        "plant_zoh_phase_lag_at_wc_deg": zoh_lag,
        "half_sample_delay_at_wc_deg": float(np.degrees(wc * ts / 2.0)),
        "position_pd_wc_continuous_rad_s": wcp,
        "position_pd_methods": methods,
        "tustin_substitution_vs_implementation_max_abs": tustin_vs_impl,
    }
    t_c = np.arange(0.0, 0.3, 1e-4)
    out.series["disc"] = {"rows": rows, "logs": logs, "t_cont": t_c, "cont": cl.step(t_c)}


# ------------------------------------------------------------------------------
def run(dataset: Path = DATASET, spec: Spec = Spec()) -> PipelineOutput:
    true, drive = MotorParams(), DriveParams()
    out = PipelineOutput(summary={
        "note": "Generated by scripts/run_pipeline.py. Plant data are synthetic; all results "
                "are from simulation.",
        "true_plant": {**asdict(true), "Ks": true.ks, "T1": true.time_constants[0],
                       "T2": true.time_constants[1], "tau_el": true.tau_el,
                       "tau_mech": true.tau_mech},
        "drive": {**asdict(drive), "rad_per_count": drive.rad_per_count},
        "spec": {k: (list(v) if isinstance(v, tuple) else v) for k, v in asdict(spec).items()},
    })
    data = ident.load_step_csv(dataset)
    pt1, pt2 = identify(data, true, out)
    des = design(pt1, pt2, drive, spec)
    out.designs = des
    verify_speed(des, true, drive, spec, out)
    verify_position(des, true, drive, spec, out)
    verify_ssc(des, true, drive, spec, out)
    verify_discretisation(des, true, drive, spec, out)
    out.summary = _r(out.summary)
    return out
