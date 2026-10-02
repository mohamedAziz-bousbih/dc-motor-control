"""Parameter identification from an open-loop voltage-step experiment.

Three model structures are fitted by (nonlinear) least squares:

* PT1  (black box):  w/U = Ks / (T s + 1)
* PT2  (black box):  w/U = Ks / ((T1 s + 1)(T2 s + 1))
* physical (grey box): R, L, K, J, b fitted to speed *and* current

The measurement itself is synthetic (see make_synthetic_step); it is produced
from known parameters so the identified values can be checked against truth.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy import optimize

from .lti import lsim_zoh
from .motor import MotorParams

Z95 = 1.959963984540054  # two-sided 95 % quantile of the standard normal


@dataclass
class StepData:
    t: np.ndarray
    u: np.ndarray
    omega: np.ndarray
    current: np.ndarray
    header: list[str] = field(default_factory=list)

    @property
    def onset(self) -> tuple[int, float, float]:
        """Index, time and height of the voltage step."""
        k = int(np.flatnonzero(self.u != self.u[0])[0])
        return k, float(self.t[k]), float(self.u[k] - self.u[0])

    def pre_step_noise(self) -> tuple[float, float]:
        """Noise std of speed and current, estimated from the samples before the step.

        With zero input the plant is at rest, so these samples contain nothing
        but sensor noise - the usual way to weight a multi-output fit.
        """
        k, _, _ = self.onset
        return float(np.std(self.omega[:k], ddof=1)), float(np.std(self.current[:k], ddof=1))


def make_synthetic_step(p: MotorParams, u_step: float = 6.0, t_step: float = 0.02,
                        t_end: float = 0.3, fs: float = 5000.0, sigma_w: float = 0.2,
                        sigma_i: float = 0.02, seed: int = 2026) -> StepData:
    """Simulate a voltage step on the true plant and add white sensor noise."""
    n = int(round(t_end * fs)) + 1
    t = np.arange(n) / fs
    u = np.where(t >= t_step - 0.5 / fs, u_step, 0.0)
    a, b, c, d = p.state_space()
    y = lsim_zoh(a, b, c, d, np.column_stack([u, np.zeros(n)]), 1.0 / fs)
    rng = np.random.default_rng(seed)
    header = [
        "SYNTHETIC DATA - generated from known parameters, not measured on hardware.",
        "Generator: scripts/make_dataset.py (control.identification.make_synthetic_step)",
        f"True parameters: R={p.R} ohm, L={p.L} H, K={p.K} V*s/rad, J={p.J} kg*m^2, b={p.b} N*m*s/rad",
        f"Voltage step {u_step} V at t={t_step} s, fs={fs:g} Hz, white Gaussian sensor noise "
        f"sigma_omega={sigma_w} rad/s, sigma_current={sigma_i} A, numpy seed={seed}",
    ]
    return StepData(t=t, u=u,
                    omega=y[:, 1] + rng.normal(0.0, sigma_w, n),
                    current=y[:, 2] + rng.normal(0.0, sigma_i, n),
                    header=header)


def step_csv_text(data: StepData) -> str:
    lines = [f"# {line}" for line in data.header]
    lines.append("t_s,u_V,omega_radps,current_A")
    lines += [f"{t:.4f},{u:.3f},{w:.5f},{i:.5f}"
              for t, u, w, i in zip(data.t, data.u, data.omega, data.current)]
    return "\n".join(lines) + "\n"


def save_step_csv(data: StepData, path: Path) -> None:
    Path(path).write_text(step_csv_text(data), encoding="utf-8", newline="\n")


def load_step_csv(path: Path) -> StepData:
    header, rows = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                header.append(line[1:].strip())
        f.seek(0)
        reader = csv.reader(ln for ln in f if not ln.startswith("#"))
        next(reader)
        rows = np.array([[float(v) for v in r] for r in reader])
    return StepData(t=rows[:, 0], u=rows[:, 1], omega=rows[:, 2], current=rows[:, 3], header=header)


# --- model step responses ---------------------------------------------------

def pt1_step(tau: np.ndarray, ks: float, t1: float) -> np.ndarray:
    tau = np.maximum(tau, 0.0)
    return ks * (1.0 - np.exp(-tau / t1))


def pt2_step(tau: np.ndarray, ks: float, t1: float, t2: float) -> np.ndarray:
    tau = np.maximum(tau, 0.0)
    if abs(t1 - t2) < 1e-9 * max(t1, t2):
        return ks * (1.0 - (1.0 + tau / t1) * np.exp(-tau / t1))
    return ks * (1.0 - (t1 * np.exp(-tau / t1) - t2 * np.exp(-tau / t2)) / (t1 - t2))


def physical_step(tau: np.ndarray, p: MotorParams) -> tuple[np.ndarray, np.ndarray]:
    """Unit-step response of speed and current, evaluated analytically.

    Uses the (w, i) subsystem, which is invertible:
    x(t) = A^-1 (exp(A t) - I) B, with exp(A t) from the eigendecomposition.
    """
    a = np.array([[-p.b / p.J, p.K / p.J], [-p.K / p.L, -p.R / p.L]])
    bvec = np.array([0.0, 1.0 / p.L])
    lam, v = np.linalg.eig(a)
    vinv_b = np.linalg.solve(v, bvec)
    tau = np.maximum(tau, 0.0)
    # (exp(lam t) - 1) / lam, per eigenvalue
    g = (np.exp(np.outer(tau, lam)) - 1.0) / lam
    x = np.real((g * vinv_b) @ v.T)
    return x[:, 0], x[:, 1]


# --- fitting ------------------------------------------------------------------

@dataclass
class FitResult:
    names: tuple[str, ...]
    values: np.ndarray
    ci95: np.ndarray        # half-width of the approximate 95 % confidence interval
    residuals: np.ndarray   # model - measurement (speed channel for black-box fits)
    rmse: float

    def as_dict(self) -> dict:
        return {n: {"value": float(v), "ci95": float(c)}
                for n, v, c in zip(self.names, self.values, self.ci95)}


def _gauss_newton_polish(fun, jac, x: np.ndarray, max_iter: int = 20) -> np.ndarray:
    """Undamped Gauss-Newton steps from a converged trust-region solution.

    The trust-region solver stops on cost and step tolerances. Near the
    minimum the cost is flat to machine precision while the poorly
    identifiable T2 can still move by ~1e-8 relative, so where it stops
    depends on the starting point and on the BLAS build. Gauss-Newton steps
    driven by the analytic Jacobian converge to the stationary point itself,
    which makes the identified values (and every gain designed from them)
    reproducible to ~1e-15 on any machine.
    """
    cost0 = float(fun(x) @ fun(x))
    for _ in range(max_iter):
        step = np.linalg.lstsq(jac(x), -fun(x), rcond=None)[0]
        x = x + step
        if np.max(np.abs(step)) < 1e-15:
            break
    if not float(fun(x) @ fun(x)) <= cost0 * (1.0 + 1e-12):
        raise RuntimeError("Gauss-Newton polish increased the cost")
    return x


def _log_fit(fun, x0_lin: np.ndarray, names: tuple[str, ...], n_obs: int,
             residual_view, jac=None) -> FitResult:
    """Least squares in log-parameters (keeps every parameter positive).

    `jac` is the analytic Jacobian with respect to the log-parameters; without
    it a finite-difference Jacobian is used and no polish step is possible.
    Confidence intervals come from the Gauss-Newton covariance
    s^2 (J^T J)^-1 in log space, mapped back with the delta method.
    """
    sol = optimize.least_squares(fun, np.log(x0_lin), jac="2-point" if jac is None else jac,
                                 x_scale="jac", xtol=1e-12, ftol=1e-12)
    x = sol.x if jac is None else _gauss_newton_polish(fun, jac, sol.x)
    r = fun(x)
    j = sol.jac if jac is None else jac(x)
    values = np.exp(x)
    dof = max(n_obs - len(values), 1)
    s2 = float(r @ r) / dof
    cov_log = s2 * np.linalg.pinv(j.T @ j)
    ci = Z95 * np.sqrt(np.diag(cov_log)) * values
    res = residual_view(r)
    return FitResult(names=names, values=values, ci95=ci, residuals=res,
                     rmse=float(np.sqrt(np.mean(res ** 2))))


def _initial_pt1(data: StepData) -> tuple[float, float]:
    k0, t0, du = data.onset
    y_end = float(np.mean(data.omega[-len(data.omega) // 10:]))
    ks0 = y_end / du
    k63 = k0 + int(np.argmax(data.omega[k0:] >= 0.632 * y_end))
    return ks0, max(data.t[k63] - t0, data.t[1] - data.t[0])


def pt1_jacobian(tau: np.ndarray, ks: float, t1: float) -> np.ndarray:
    """d pt1_step / d(log Ks, log T), shape (len(tau), 2)."""
    tau = np.maximum(tau, 0.0)
    e = np.exp(-tau / t1)
    return np.column_stack([ks * (1.0 - e), -ks * e * tau / t1])


def pt2_jacobian(tau: np.ndarray, ks: float, t1: float, t2: float) -> np.ndarray:
    """d pt2_step / d(log Ks, log T1, log T2), shape (len(tau), 3); needs T1 != T2."""
    tau = np.maximum(tau, 0.0)
    e1, e2 = np.exp(-tau / t1), np.exp(-tau / t2)
    dt = t1 - t2
    g = (t1 * e1 - t2 * e2) / dt            # pt2_step = ks (1 - g)
    dg_dt1 = (e1 * (1.0 + tau / t1) - g) / dt
    dg_dt2 = (g - e2 * (1.0 + tau / t2)) / dt
    return np.column_stack([ks * (1.0 - g), -ks * t1 * dg_dt1, -ks * t2 * dg_dt2])


def fit_pt1(data: StepData) -> FitResult:
    _, t0, du = data.onset
    tau = data.t - t0
    fun = lambda th: du * pt1_step(tau, *np.exp(th)) - data.omega  # noqa: E731
    jac = lambda th: du * pt1_jacobian(tau, *np.exp(th))  # noqa: E731
    return _log_fit(fun, np.array(_initial_pt1(data)), ("Ks", "T"), len(tau), lambda r: r, jac)


def fit_pt2(data: StepData) -> FitResult:
    _, t0, du = data.onset
    tau = data.t - t0
    ks0, t_sum = _initial_pt1(data)
    fun = lambda th: du * pt2_step(tau, *np.exp(th)) - data.omega  # noqa: E731
    jac = lambda th: du * pt2_jacobian(tau, *np.exp(th))  # noqa: E731
    return _log_fit(fun, np.array([ks0, 0.9 * t_sum, 0.1 * t_sum]), ("Ks", "T1", "T2"),
                    len(tau), lambda r: r, jac)


def physical_initial_guess(data: StepData, pt1: FitResult) -> MotorParams:
    """Rough physical parameters read off the step response by hand-style
    reasoning; only used as the starting point of the grey-box fit.

    * right after the step the back-EMF is still ~0, so the current peak is ~U/R
    * the current reaches 63 % of that peak after about L/R
    * steady state: U = R i + K w  and  K i = b w
    * reduced model: Tm = J R / (K^2 + R b)
    """
    k0, t0, du = data.onset
    i_peak = float(np.max(data.current[k0:]))
    r = du / i_peak
    k63 = k0 + int(np.argmax(data.current[k0:] >= 0.632 * i_peak))
    l = r * max(float(data.t[k63] - t0), float(data.t[1] - data.t[0]))
    tail = len(data.t) // 10
    w_ss = float(np.mean(data.omega[-tail:]))
    i_ss = float(np.mean(data.current[-tail:]))
    k = (du - r * i_ss) / w_ss
    b = max(k * i_ss / w_ss, 1e-9)
    j = float(pt1.values[1]) * (k ** 2 + r * b) / r
    return MotorParams(R=r, L=l, K=k, J=j, b=b)


def fit_physical(data: StepData, guess: MotorParams) -> FitResult:
    """Grey-box output-error fit of R, L, K, J, b to speed and current.

    Both channels are divided by their pre-step noise level so they carry
    comparable weight. Residuals returned are those of the speed channel.
    """
    _, t0, du = data.onset
    tau = data.t - t0
    sw, si = data.pre_step_noise()
    n = len(tau)

    def fun(th):
        r, l, k, j, b = np.exp(th)
        w, i = physical_step(tau, MotorParams(R=r, L=l, K=k, J=j, b=b))
        return np.concatenate([(du * w - data.omega) / sw, (du * i - data.current) / si])

    x0 = np.array([guess.R, guess.L, guess.K, guess.J, guess.b])
    fit = _log_fit(fun, x0, ("R", "L", "K", "J", "b"), 2 * n, lambda r: r[:n] * sw)
    return fit


def autocorrelation(x: np.ndarray, max_lag: int) -> np.ndarray:
    """Normalised sample autocorrelation r(k)/r(0) for k = 0..max_lag."""
    x = np.asarray(x, dtype=float) - np.mean(x)
    r0 = float(x @ x)
    return np.array([float(x[: len(x) - k] @ x[k:]) / r0 for k in range(max_lag + 1)])
