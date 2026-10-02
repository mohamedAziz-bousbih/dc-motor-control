"""Reference (float64) implementations of the discrete controller blocks.

Each class mirrors one module of the C library lib/motorctl line by line:
the same update order, the same discretisation and the same corner cases.
They drive the closed-loop simulations and produce the golden vectors the
C unit tests are compared against, so any change here must be made in the
C code as well (and vice versa) - the golden-vector tests enforce that.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


@dataclass(frozen=True)
class PIDConfig:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    tf: float = 0.0          # derivative filter time constant [s]; required when kd != 0
    b: float = 1.0           # setpoint weight of the P term
    c: float = 0.0           # setpoint weight of the D term (0: derivative on measurement)
    ts: float = 1e-3
    u_min: float = -math.inf
    u_max: float = math.inf
    tt: float = 0.0          # back-calculation tracking time constant [s]; 0 disables


class PID:
    """Two-degree-of-freedom PID, Tustin-discretised (mc_pid.c).

    u = kp (b r - y) + ki * integral(r - y) + kd * D[c r - y],  D(s) = s / (tf s + 1)
    """

    def __init__(self, cfg: PIDConfig):
        if cfg.ts <= 0 or not cfg.u_min < cfg.u_max or cfg.tt < 0 or cfg.tf < 0:
            raise ValueError("invalid PID configuration")
        if cfg.kd != 0.0 and cfg.tf <= 0.0:
            raise ValueError("a derivative term needs a filter time constant tf > 0")
        self.cfg = cfg
        if cfg.tf > 0.0:
            self.ad = (2.0 * cfg.tf - cfg.ts) / (2.0 * cfg.tf + cfg.ts)
            self.bd = 2.0 / (2.0 * cfg.tf + cfg.ts)
        else:
            self.ad = self.bd = 0.0
        self.reset()

    def reset(self) -> None:
        self.integ = 0.0
        self.deriv = 0.0
        self.e_prev = 0.0
        self.xd_prev = 0.0
        self.primed = False

    def _unsaturated(self, r: float, y: float) -> float:
        c = self.cfg
        return c.kp * (c.b * r - y) + self.integ + c.kd * self.deriv

    def update(self, r: float, y: float) -> float:
        c = self.cfg
        e = r - y
        xd = c.c * r - y
        if not self.primed:
            # First sample after a reset: no history, so no derivative kick
            # and a rectangular first integration step.
            self.e_prev, self.xd_prev, self.primed = e, xd, True
        self.deriv = self.ad * self.deriv + self.bd * (xd - self.xd_prev)
        self.integ += c.ki * c.ts * 0.5 * (e + self.e_prev)
        v = self._unsaturated(r, y)
        u = _clamp(v, c.u_min, c.u_max)
        if c.ki != 0.0 and c.tt > 0.0:
            self.integ += (c.ts / c.tt) * (u - v)
        self.e_prev, self.xd_prev = e, xd
        return u

    def set_gains(self, kp: float, ki: float, kd: float, r: float, y: float) -> None:
        """Change gains without a jump in the output (bumpless parameter change).

        The integrator absorbs the difference between the old and the new
        proportional/derivative contribution. Without integral action there is
        no state that could do so, and a leftover offset would act as a
        permanent bias, so the integrator is cleared instead.
        """
        if kd != 0.0 and self.cfg.tf <= 0.0:
            raise ValueError("a derivative term needs a filter time constant tf > 0")
        v_old = self._unsaturated(r, y)
        self.cfg = replace(self.cfg, kp=kp, ki=ki, kd=kd)
        self.integ = self.integ + v_old - self._unsaturated(r, y) if ki != 0.0 else 0.0

    def bumpless_start(self, r: float, y: float, u_now: float) -> None:
        """Initialise the states so the next output continues from u_now.

        Only possible with integral action (see set_gains); a P/PD controller
        starts from its own output kp (b r - y).
        """
        c = self.cfg
        self.deriv = 0.0
        self.e_prev = r - y
        self.xd_prev = c.c * r - y
        self.primed = True
        self.integ = u_now - c.kp * (c.b * r - y) if c.ki != 0.0 else 0.0


class LowPass1:
    """First-order low-pass y += a (x - y), a = 1 - exp(-Ts/tau) (mc_filter.c)."""

    def __init__(self, tau: float, ts: float, y0: float = 0.0):
        if tau < 0 or ts <= 0:
            raise ValueError("invalid filter configuration")
        self.a = 1.0 - math.exp(-ts / tau) if tau > 0 else 1.0
        self.y = y0

    def update(self, x: float) -> float:
        self.y += self.a * (x - self.y)
        return self.y


class SlewLimiter:
    """Rate limiter for setpoints (mc_filter.c). rate <= 0 means unlimited."""

    def __init__(self, rate: float, ts: float, y0: float = 0.0):
        if ts <= 0:
            raise ValueError("invalid sample time")
        self.ts = ts
        self.set_rate(rate)
        self.y = y0

    def set_rate(self, rate: float) -> None:
        self.step_max = rate * self.ts if rate > 0 else 0.0

    def update(self, x: float) -> float:
        if self.step_max > 0.0:
            self.y += _clamp(x - self.y, -self.step_max, self.step_max)
        else:
            self.y = x
        return self.y


@dataclass(frozen=True)
class SSCConfig:
    phi: np.ndarray
    gamma: np.ndarray
    l: np.ndarray
    k: np.ndarray
    ki: float
    ts: float
    kr: float = 1.0          # reference weight on the angle feedback (0 ... 1)
    u_min: float = -math.inf
    u_max: float = math.inf
    tt: float = 0.0          # back-calculation tracking time constant [s]; 0 disables


class StateController:
    """State feedback with integral action and a prediction-form Luenberger
    observer (mc_ssc.c).

    v      = k0 (kr r - x0_hat) - k1 x1_hat - ki x_i,   u = sat(v)
    x_i   += Ts (y - r) + (Ts / tt) (v - u) / ki          (back-calculation)
    x_hat <- Phi x_hat + Gamma u + l (y - x0_hat)

    The observer is fed the saturated u, i.e. the voltage the motor really
    gets, so the estimate stays correct while the output is limited.
    Back-calculation (rather than conditional integration) keeps the update a
    continuous function of v: float32 and float64 cannot take different
    branches at the saturation boundary, which the golden-vector test relies on.

    With kr = 0 the reference enters only through the integrator: the reference
    response has no zero and follows the placed poles, but a ramp is tracked
    with a lag of v k0/ki. kr = 1 adds the zero -ki/k0, which removes that lag
    (the loop becomes type 2 for the reference) at the price of overshoot on
    setpoint steps; with a rate-limited setpoint that overshoot does not occur.
    """

    def __init__(self, cfg: SSCConfig):
        if cfg.ts <= 0 or not cfg.u_min < cfg.u_max or cfg.tt < 0:
            raise ValueError("invalid state-controller configuration")
        self.cfg = cfg
        self.aw = cfg.ts / (cfg.tt * cfg.ki) if cfg.tt > 0.0 and cfg.ki != 0.0 else 0.0
        self.reset(0.0)

    def reset(self, theta0: float) -> None:
        self.xhat = np.array([theta0, 0.0])
        self.xi = 0.0

    def _unsaturated(self, r: float) -> float:
        c = self.cfg
        return c.k[0] * (c.kr * r - self.xhat[0]) - c.k[1] * self.xhat[1] - c.ki * self.xi

    def bumpless_start(self, r: float, theta0: float, omega0: float, u_now: float) -> None:
        """Seed observer and integrator so the next output continues from u_now."""
        c = self.cfg
        self.xhat = np.array([theta0, omega0])
        self.xi = 0.0
        if c.ki != 0.0:
            self.xi = (self._unsaturated(r) - u_now) / c.ki

    def update(self, r: float, y: float) -> float:
        c = self.cfg
        innov = y - self.xhat[0]
        v = self._unsaturated(r)
        u = _clamp(v, c.u_min, c.u_max)
        self.xi += c.ts * (y - r) + self.aw * (v - u)
        x0 = c.phi[0, 0] * self.xhat[0] + c.phi[0, 1] * self.xhat[1] + c.gamma[0] * u + c.l[0] * innov
        x1 = c.phi[1, 0] * self.xhat[0] + c.phi[1, 1] * self.xhat[1] + c.gamma[1] * u + c.l[1] * innov
        self.xhat = np.array([x0, x1])
        return u


def pid_zpoly(cfg: PIDConfig) -> tuple[np.ndarray, np.ndarray]:
    """(num, den) in z of the discretised PID in error form (b = c = 1): exactly
    the difference equations of PID.update, i.e. the Tustin integrator
    ki Ts/2 (z + 1)/(z - 1) and the Tustin derivative filter kd bd (z - 1)/(z - ad)."""
    num, den = np.array([cfg.kp]), np.ones(1)

    def add(n2, d2):
        nonlocal num, den
        num = np.polyadd(np.polymul(num, d2), np.polymul(n2, den))
        den = np.polymul(den, d2)

    if cfg.ki:
        add(cfg.ki * cfg.ts / 2.0 * np.array([1.0, 1.0]), np.array([1.0, -1.0]))
    if cfg.kd:
        ad = (2.0 * cfg.tf - cfg.ts) / (2.0 * cfg.tf + cfg.ts)
        bd = 2.0 / (2.0 * cfg.tf + cfg.ts)
        add(cfg.kd * bd * np.array([1.0, -1.0]), np.array([1.0, -ad]))
    return num, den


def ssc_compensator(cfg: SSCConfig):
    """Observer-based controller as a discrete system from y to u (r = 0, so
    the reference weight kr plays no role).

    Returns (A_c, B_c, C_c) with u = C_c (zI - A_c)^-1 B_c y. Used to break the
    loop at the plant input and read off classical margins of the design.
    """
    k = np.asarray(cfg.k, dtype=float)
    gam = np.asarray(cfg.gamma, dtype=float)
    l = np.asarray(cfg.l, dtype=float)
    a_c = np.zeros((3, 3))
    a_c[:2, :2] = cfg.phi - np.outer(gam, k) - np.outer(l, [1.0, 0.0])
    a_c[:2, 2] = -gam * cfg.ki
    a_c[2, 2] = 1.0
    b_c = np.array([l[0], l[1], cfg.ts])
    c_c = np.array([-k[0], -k[1], -cfg.ki])
    return a_c, b_c, c_c
