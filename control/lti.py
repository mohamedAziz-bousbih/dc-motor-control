"""Minimal LTI toolbox built on numpy/scipy.

Covers what the rest of the package needs and nothing more: rational
continuous-time transfer functions, exact zero-order-hold discretisation,
discrete transfer functions as polynomial pairs in z, simulation of discrete
state-space models and stability margins computed directly from a frequency
response.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy import linalg, optimize, signal


def _as_poly(c) -> np.ndarray:
    c = np.atleast_1d(np.asarray(c, dtype=float))
    nz = np.flatnonzero(c)
    return c[nz[0]:] if nz.size else np.zeros(1)


class TF:
    """Continuous-time SISO transfer function num(s)/den(s).

    Coefficients are given in descending powers of s. The denominator is
    normalised to be monic. Products keep every factor (no pole/zero
    cancellation), which is intentional: a compensated plant pole must stay
    visible when analysing disturbance behaviour.
    """

    def __init__(self, num, den):
        num = _as_poly(num)
        den = _as_poly(den)
        if not np.any(den):
            raise ValueError("denominator must not be zero")
        self.num = num / den[0]
        self.den = den / den[0]

    # evaluation -----------------------------------------------------------
    def __call__(self, s):
        s = np.asarray(s, dtype=complex)
        return np.polyval(self.num, s) / np.polyval(self.den, s)

    def freq(self, w):
        """Frequency response G(jw)."""
        return self(1j * np.asarray(w, dtype=float))

    # algebra --------------------------------------------------------------
    def __mul__(self, other):
        if isinstance(other, TF):
            return TF(np.polymul(self.num, other.num), np.polymul(self.den, other.den))
        return TF(self.num * float(other), self.den)

    __rmul__ = __mul__

    def feedback(self, h: "TF | float" = 1.0) -> "TF":
        """Negative feedback: self / (1 + self*h)."""
        if not isinstance(h, TF):
            h = TF([float(h)], [1.0])
        num = np.polymul(self.num, h.den)
        den = np.polyadd(np.polymul(self.den, h.den), np.polymul(self.num, h.num))
        return TF(num, den)

    # properties -----------------------------------------------------------
    def poles(self) -> np.ndarray:
        return np.roots(self.den)

    def zeros(self) -> np.ndarray:
        return np.roots(self.num)

    def dcgain(self) -> float:
        return float(np.real(self(0.0)))

    def to_ss(self):
        """Controllable canonical state-space realisation (A, B, C, D)."""
        if self.num.size > self.den.size:
            raise ValueError("transfer function is improper")
        return signal.tf2ss(self.num, self.den)

    def step(self, t: np.ndarray) -> np.ndarray:
        """Unit-step response sampled on the uniform grid t (exact at the samples)."""
        a, b, c, d = self.to_ss()
        u = np.ones_like(t, dtype=float)
        return lsim_zoh(a, b, c, d, u[:, None], float(t[1] - t[0]))[:, 0]


def c2d_zoh(a: np.ndarray, b: np.ndarray, ts: float):
    """Exact zero-order-hold discretisation via one matrix exponential.

    exp([[A, B], [0, 0]] * Ts) = [[Phi, Gamma], [0, I]]
    """
    a = np.atleast_2d(np.asarray(a, dtype=float))
    b = np.asarray(b, dtype=float).reshape(a.shape[0], -1)
    n, m = b.shape
    blk = np.zeros((n + m, n + m))
    blk[:n, :n] = a
    blk[:n, n:] = b
    e = linalg.expm(blk * ts)
    return e[:n, :n], e[:n, n:]


def lsim_zoh(a, b, c, d, u: np.ndarray, dt: float, x0=None) -> np.ndarray:
    """Simulate a continuous system for piecewise-constant input samples u[k]."""
    phi, gam = c2d_zoh(a, b, dt)
    c = np.atleast_2d(c)
    d = np.atleast_2d(d)
    x = np.zeros(phi.shape[0]) if x0 is None else np.asarray(x0, dtype=float)
    y = np.empty((u.shape[0], c.shape[0]))
    for k in range(u.shape[0]):
        y[k] = c @ x + d @ u[k]
        x = phi @ x + gam @ u[k]
    return y


def dss_freq(phi, gam, c, d, w, ts) -> np.ndarray:
    """Frequency response of a discrete SISO state-space model at z = exp(j w Ts)."""
    phi = np.atleast_2d(phi)
    gam = np.asarray(gam, dtype=float).reshape(-1)
    c = np.asarray(c, dtype=float).reshape(-1)
    n = phi.shape[0]
    w = np.atleast_1d(np.asarray(w, dtype=float))
    out = np.empty(w.shape, dtype=complex)
    for k, wk in enumerate(w):
        z = np.exp(1j * wk * ts)
        out[k] = c @ np.linalg.solve(z * np.eye(n) - phi, gam) + float(np.squeeze(d))
    return out


def zpoly_closed_loop(nums, dens, gain: float = 1.0) -> np.ndarray:
    """Characteristic polynomial den + gain * num of a loop given as a chain of
    discrete blocks num_i(z)/den_i(z) (coefficients in descending powers of z)."""
    num, den = np.ones(1), np.ones(1)
    for n_i, d_i in zip(nums, dens):
        num = np.polymul(num, n_i)
        den = np.polymul(den, d_i)
    return np.polyadd(den, gain * num)


def zblocks_freq(nums, dens, w, ts: float) -> np.ndarray:
    """Frequency response of a chain of discrete blocks at z = exp(j w Ts)."""
    z = np.exp(1j * np.asarray(w, dtype=float) * ts)
    out = np.ones_like(z)
    for n_i, d_i in zip(nums, dens):
        out = out * np.polyval(n_i, z) / np.polyval(d_i, z)
    return out


def spectral_radius(char_poly) -> float:
    """Largest pole magnitude; < 1 means the discrete closed loop is stable."""
    return float(np.max(np.abs(np.roots(char_poly))))


def ss2zpoly(phi, gam, c, d=0.0):
    """(num, den) of a discrete SISO state-space model, leading zeros trimmed."""
    num, den = signal.ss2tf(np.atleast_2d(phi), np.asarray(gam, dtype=float).reshape(-1, 1),
                            np.asarray(c, dtype=float).reshape(1, -1), np.array([[float(d)]]))
    num = np.trim_zeros(np.where(np.abs(num[0]) < 1e-14 * np.max(np.abs(num[0])), 0.0, num[0]), "f")
    return num, den


@dataclass(frozen=True)
class Margins:
    wc: float            # gain-crossover frequency [rad/s]
    pm_deg: float        # phase margin [deg]
    w180: float          # phase crossover belonging to gm [rad/s] (nan if none)
    gm: float            # upper gain margin: factor the loop gain may grow by (inf if none)
    w180_lower: float    # phase crossover belonging to gm_lower [rad/s] (nan if none)
    gm_lower: float      # lower gain margin: factor < 1 the gain may shrink to (0 if none)
    delay_margin: float  # additional dead time the loop tolerates [s]

    @property
    def gm_db(self) -> float:
        return float(20.0 * np.log10(self.gm)) if np.isfinite(self.gm) else float("inf")

    @property
    def gm_lower_db(self) -> float:
        return float(20.0 * np.log10(self.gm_lower)) if self.gm_lower > 0 else float("-inf")


def margins(loop: Callable[[np.ndarray], np.ndarray], w_min: float, w_max: float,
            n: int = 20000) -> Margins:
    """Gain and phase margins of an open loop L(jw), found numerically.

    The loop is sampled on a logarithmic grid and the phase is unwrapped from
    the low-frequency end. np.angle cannot tell -181 deg from +179 deg, so the
    unwrapped curve is shifted by whole turns until it starts in (-270, 90] deg:
    that range holds the low-frequency phase of every loop of type 0, 1 or 2,
    including a double integrator whose phase dips slightly below -180 deg.
    Every crossing is then refined with Brent's method.

    With several gain crossovers the smallest phase margin is reported. Each
    -180 deg crossing yields a critical gain factor 1/|L|; the smallest factor
    above 1 is the (upper) gain margin, the largest factor below 1 is the lower
    gain margin of a conditionally stable loop (simplified Nyquist criterion:
    the closed loop stays stable for gain changes between the two).
    """
    w = np.logspace(np.log10(w_min), np.log10(w_max), n)
    lw = loop(w)
    ph = np.unwrap(np.angle(lw))
    ph -= 2.0 * np.pi * np.ceil((ph[0] - np.pi / 2.0) / (2.0 * np.pi))
    logmag = np.log(np.abs(lw))

    def phase_near(x: float, i: int) -> float:
        # continuous phase at x, anchored to the unwrapped grid value at w[i]
        return ph[i] + float(np.angle(loop(np.array([x]))[0] / lw[i]))

    pms, wcs = [], []
    for i in np.flatnonzero(np.sign(logmag[:-1]) != np.sign(logmag[1:])):
        wc = optimize.brentq(lambda x: np.log(np.abs(loop(np.array([x]))[0])), w[i], w[i + 1])
        wcs.append(wc)
        pms.append(180.0 + np.degrees(phase_near(wc, i)))
    if not wcs:
        raise ValueError("no gain crossover in the given frequency range")
    k = int(np.argmin(pms))
    wc, pm = wcs[k], pms[k]

    turns = np.floor((ph + np.pi) / (2 * np.pi))
    upper, lower = [], []
    for i in np.flatnonzero(turns[:-1] != turns[1:]):
        # angle(-L) passes through zero exactly where arg L = -180 deg (mod 360)
        x = optimize.brentq(lambda v: np.angle(-loop(np.array([v]))[0]), w[i], w[i + 1])
        g = 1.0 / np.abs(loop(np.array([x]))[0])
        (upper if g > 1.0 else lower).append((g, x))
    gm, w180 = min(upper) if upper else (float("inf"), float("nan"))
    gm_lo, w180_lo = max(lower) if lower else (0.0, float("nan"))

    return Margins(wc=float(wc), pm_deg=float(pm), w180=float(w180), gm=float(gm),
                   w180_lower=float(w180_lo), gm_lower=float(gm_lo),
                   delay_margin=float(np.radians(pm) / wc))
