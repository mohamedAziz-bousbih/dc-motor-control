"""Figures for docs/img, drawn from the time series of one pipeline run."""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .pipeline import PipelineOutput  # noqa: E402

DPI = 100
GRAY = "0.6"


def _save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=DPI)
    plt.close(fig)


def _ms(t):
    return 1e3 * np.asarray(t)


def identification(out: PipelineOutput, path: Path) -> None:
    s = out.series["ident"]
    fig, ax = plt.subplots(2, 2, figsize=(10, 6.5))
    t = _ms(s["t"])
    a = ax[0, 0]
    a.plot(t, s["omega"], ".", ms=1.5, color=GRAY, label="synthetic measurement")
    a.plot(t, s["pt1"], label="PT1 fit")
    a.plot(t, s["pt2"], label="PT2 fit")
    a.plot(t, s["phys_omega"], "--", label="physical fit")
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]", title="6 V step: speed")
    a.legend(loc="lower right", fontsize=8)

    a = ax[0, 1]
    zoom = (s["t"] >= 0.015) & (s["t"] <= 0.04)
    a.plot(t[zoom], s["omega"][zoom], ".", ms=3, color=GRAY, label="synthetic measurement")
    a.plot(t[zoom], s["pt1"][zoom], label="PT1 fit (starts with a kink)")
    a.plot(t[zoom], s["pt2"][zoom], label="PT2 fit")
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]", title="Zoom on the step onset")
    a.legend(loc="upper left", fontsize=8)

    a = ax[1, 0]
    a.plot(t, s["current"], ".", ms=1.5, color=GRAY, label="synthetic measurement")
    a.plot(t, s["phys_current"], color="C2", label="physical fit")
    a.set(xlabel="t [ms]", ylabel="current [A]", title="6 V step: armature current")
    a.legend(loc="upper right", fontsize=8)

    a = ax[1, 1]
    lags = np.arange(len(s["acf_pt1"]))
    a.stem(lags[1:], s["acf_pt1"][1:], linefmt="C0-", markerfmt="C0.", basefmt=" ", label="PT1")
    a.stem(lags[1:] + 0.3, s["acf_pt2"][1:], linefmt="C1-", markerfmt="C1.", basefmt=" ",
           label="PT2")
    a.axhspan(-s["acf_bound"], s["acf_bound"], color="0.85", label="95 % band (white noise)")
    a.set(xlabel="lag [samples]", ylabel="autocorrelation", title="Residual autocorrelation")
    a.legend(fontsize=8)
    _save(fig, path)


def speed_bode(out: PipelineOutput, path: Path) -> None:
    s = out.series["speed"]
    w = s["bode_w"]
    m_qc, m_dt = s["margins"]
    fig, (am, ap) = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for f, label, m in ((s["loop_quasi"], "quasi-continuous design model", m_qc),
                        (s["loop_disc_true"], "exact discrete loop, true plant", m_dt)):
        lw = f(w)
        ph = np.degrees(np.unwrap(np.angle(lw)))
        ph -= 360.0 * np.ceil((ph[0] - 90.0) / 360.0)
        line, = am.semilogx(w, 20 * np.log10(np.abs(lw)),
                            label=f"{label}: PM {m.pm_deg:.1f} deg, GM {m.gm_db:.1f} dB")
        color = line.get_color()
        ap.semilogx(w, ph, color=color)
        # margins drawn as bars at the crossover frequencies
        ap.plot([m.wc, m.wc], [-180.0, -180.0 + m.pm_deg], color=color, lw=3, alpha=0.6)
        if math.isfinite(m.gm):
            am.plot([m.w180, m.w180], [0.0, -m.gm_db], color=color, lw=3, alpha=0.6)
    am.axhline(0.0, color="k", lw=0.8)
    ap.axhline(-180.0, color="k", lw=0.8)
    am.set(ylabel="|L| [dB]", title="Speed loop: open-loop frequency response with margins")
    ap.set(xlabel="omega [rad/s]", ylabel="arg L [deg]", ylim=(-300, -60))
    am.legend(fontsize=8)
    am.grid(True, which="both", alpha=0.3)
    ap.grid(True, which="both", alpha=0.3)
    _save(fig, path)


def speed_steps(out: PipelineOutput, path: Path) -> None:
    s = out.series["speed"]
    fig, ax = plt.subplots(2, 2, figsize=(10, 6.5))
    a = ax[0, 0]
    a.plot(_ms(s["t_lin"]), s["cont"], label="continuous PI, true plant")
    a.plot(_ms(s["log_lin"].t), s["log_lin"].x[:, 1], "--", label="sampled at 1 kHz (Tustin PI)")
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]", title="5 rad/s step, linear")
    a.legend(fontsize=8)

    lg = s["log_real"]
    a = ax[0, 1]
    a.plot(_ms(lg.t), lg.r, color=GRAY, label="setpoint")
    a.plot(_ms(lg.t), lg.x[:, 1], label="speed")
    a.axvline(1e3 * s["t_load"], color="k", ls=":", lw=0.8)
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]",
          title="20 rad/s step, then load step (dotted line)")
    a.legend(fontsize=8)
    a = ax[1, 1]
    a.plot(_ms(lg.t), lg.u, color="C3")
    a.axvline(1e3 * s["t_load"], color="k", ls=":", lw=0.8)
    a.set(xlabel="t [ms]", ylabel="voltage [V]", title="Controller output (saturates at 12 V)")

    a = ax[1, 0]
    for name, style in (("back_calculation", "-"), ("none", "--")):
        lgw = s["aw"][name]
        a.plot(_ms(lgw.t), lgw.x[:, 1], style, label=f"anti-windup: {name.replace('_', ' ')}")
    a.axhline(30.0, color=GRAY, lw=0.8)
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]", title="30 rad/s step into saturation")
    a.legend(fontsize=8)
    _save(fig, path)


def position(out: PipelineOutput, path: Path) -> None:
    p, c = out.series["position"], out.series["ssc"]
    fig, ax = plt.subplots(2, 2, figsize=(10, 6.5))
    a = ax[0, 0]
    a.plot(_ms(p["t_pred"]) + 10.0, p["pred"], color=GRAY, lw=3, alpha=0.6,
           label="dominant pole pair (design)")
    a.plot(_ms(p["lin"][1.0].t), p["lin"][1.0].x[:, 0], label="PD, D on error (c = 1)")
    a.plot(_ms(p["lin"][0.0].t), p["lin"][0.0].x[:, 0], "--", label="PD, D on measurement (c = 0)")
    a.plot(_ms(c["lin"][1.0].t), c["lin"][1.0].x[:, 0], label="state feedback, kr = 1")
    a.plot(_ms(c["lin"][0.0].t), c["lin"][0.0].x[:, 0], "--", label="state feedback, kr = 0")
    a.set(xlabel="t [ms]", ylabel="angle [rad]", title="0.1 rad step, linear", xlim=(0, 250))
    a.legend(fontsize=7)

    a = ax[0, 1]
    sp = p["poles_s"]
    a.plot(np.real(sp), np.imag(sp), "x", ms=8, label="closed loop (PD, discrete, true plant)")
    pos = out.summary["position_pd"]
    a.plot([pos["pd_zero_rad_s"]], [0.0], "o", mfc="none", ms=8, label="PD zero")
    a.plot([pos["true_plant_pole_rad_s"]], [0.0], "+", ms=10, label="mechanical plant pole")
    zeta = pos["design"]["zeta_design"]
    r = np.linspace(0, 400, 2)
    a.plot(-r * zeta, r * math.sqrt(1 - zeta ** 2), ":", color=GRAY, label=f"zeta = {zeta:.3f}")
    a.plot(-r * zeta, -r * math.sqrt(1 - zeta ** 2), ":", color=GRAY)
    a.annotate("PD zero, plant pole and a closed-loop\n"
               f"pole all near {pos['pd_zero_rad_s']:.1f} 1/s",
               (pos["pd_zero_rad_s"], 0.0), xytext=(-330, -120), fontsize=7,
               arrowprops={"arrowstyle": "->", "lw": 0.8})
    a.set(xlabel="Re s [1/s]", ylabel="Im s [1/s]", xlim=(-400, 20), ylim=(-250, 250),
          title="Pole compensation and dominant pair")
    a.axhline(0, color="k", lw=0.5)
    a.axvline(0, color="k", lw=0.5)
    a.legend(fontsize=7, loc="upper left")

    a = ax[1, 0]
    for lg, label in ((p["move"], "PD"), (c["move"], "state feedback")):
        a.plot(lg.t, lg.r - lg.x[:, 0], label=label)
    a.axvline(0.8, color="k", ls=":", lw=0.8)
    a.set(xlabel="t [s]", ylabel="r - theta [rad]",
          title="2 pi move at 20 rad/s, load step at 0.8 s")
    a.legend(fontsize=8)

    a = ax[1, 1]
    for name, style in (("back_calculation", "-"), ("none", "--")):
        lg = c["aw"][name]
        a.plot(_ms(lg.t), lg.x[:, 0], style, label=f"anti-windup: {name.replace('_', ' ')}")
    a.axhline(3.0, color=GRAY, lw=0.8)
    a.set(xlabel="t [ms]", ylabel="angle [rad]",
          title="State feedback: 3 rad step without rate limit", xlim=(0, 600))
    a.legend(fontsize=8)
    _save(fig, path)


def observer(out: PipelineOutput, path: Path) -> None:
    c = out.series["ssc"]
    lg = c["conv"]["true_plant_quantised"]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    a = ax[0]
    a.plot(_ms(lg.t), lg.x[:, 1], color="k", lw=1, label="true speed")
    a.plot(_ms(lg.t), lg.extra["xhat"][:, 1], label="observer estimate")
    a.plot(_ms(lg.t), lg.extra["omega_meas"], "--", label="filtered difference quotient")
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]",
          title="State mode entered at 5 rad/s, estimate starts at 0")
    a.legend(fontsize=8)
    a = ax[1]
    err_model = np.abs(c["err_model"][:, 1])
    conv = c["conv"]["true_plant"]
    err_true = np.abs(conv.x[:, 1] - conv.extra["xhat"][:, 1])
    k = np.arange(len(err_model))
    a.semilogy(k, err_model, label="design model (exact error dynamics)")
    a.semilogy(np.arange(len(err_true)), err_true, "--", label="true 3rd-order plant")
    a.set(xlabel="sample k (Ts = 1 ms)", ylabel="|speed error| [rad/s]",
          title="Observer error convergence (no quantisation)", ylim=(1e-6, 20))
    a.legend(fontsize=8)
    _save(fig, path)


def discretisation(out: PipelineOutput, path: Path) -> None:
    d = out.series["disc"]
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    a = ax[0]
    a.plot(_ms(d["t_cont"]), d["cont"], color="k", lw=1, label="continuous design")
    for ts in (2e-3, 1e-3, 5e-4):
        lg = d["logs"][ts]
        a.step(_ms(lg.t), lg.x[:, 1], where="post", label=f"Ts = {1e3 * ts:g} ms")
    a.set(xlabel="t [ms]", ylabel="speed [rad/s]", title="Unit speed step: sampled vs continuous",
          xlim=(0, 80))
    a.legend(fontsize=8)
    a = ax[1]
    ts = np.array([r["ts_s"] for r in d["rows"]])
    dev = np.array([r["max_deviation_rel"] for r in d["rows"]])
    a.loglog(1e3 * ts, 100 * dev, "o-")
    a.set(xlabel="Ts [ms]", ylabel="max deviation [% of step]",
          title="Quasi-continuous design: error shrinks with Ts")
    a.grid(True, which="both", alpha=0.3)
    _save(fig, path)


FIGURES = {
    "identification.png": identification,
    "speed_bode.png": speed_bode,
    "speed_steps.png": speed_steps,
    "position.png": position,
    "observer.png": observer,
    "discretisation.png": discretisation,
}


def write_all(out: PipelineOutput, img_dir: Path) -> list[Path]:
    img_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, fn in FIGURES.items():
        fn(out, img_dir / name)
        paths.append(img_dir / name)
    return paths
