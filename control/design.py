"""Classical controller design: PI by the frequency-response (Bode) method,
PD by pole compensation plus a dominant-pole-pair gain choice.

Quasi-continuous design: the sampler/ZOH and the backward-difference speed
estimate are treated as a pure delay in the continuous-time open loop, and
the resulting controller is discretised afterwards (Tustin). The exact
discrete-time loop is checked separately in the pipeline and in the tests.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import optimize

from .lti import TF


@dataclass(frozen=True)
class PIDGains:
    """Parallel PID  C(s) = kp + ki/s + kd s/(tf s + 1)  (error-based form)."""

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    tf: float = 0.0

    def as_tf(self) -> TF:
        if self.kd == 0.0 and self.ki == 0.0:
            return TF([self.kp], [1.0])
        if self.kd == 0.0:
            return TF([self.kp, self.ki], [1.0, 0.0])
        if self.ki == 0.0:
            return TF([self.kp * self.tf + self.kd, self.kp], [self.tf, 1.0])
        return TF([self.kp * self.tf + self.kd, self.kp + self.ki * self.tf, self.ki],
                  [self.tf, 1.0, 0.0])

    def freq(self, w):
        s = 1j * np.asarray(w, dtype=float)
        d = self.kd * s / (self.tf * s + 1.0) if self.kd else 0.0
        i = self.ki / s if self.ki else 0.0
        return self.kp + i + d


def solve_phase(loop, target_deg: float, w_min: float, w_max: float, n: int = 20000) -> float:
    """Lowest frequency at which the unwrapped phase of loop(w) equals target_deg."""
    w = np.logspace(np.log10(w_min), np.log10(w_max), n)
    lw = loop(w)
    ph = np.degrees(np.unwrap(np.angle(lw)))
    idx = np.flatnonzero(np.sign(ph[:-1] - target_deg) != np.sign(ph[1:] - target_deg))
    if idx.size == 0:
        raise ValueError(f"phase never reaches {target_deg} deg in [{w_min}, {w_max}] rad/s")
    i = int(idx[0])

    def f(x):
        return ph[i] + np.degrees(np.angle(loop(np.array([x]))[0] / lw[i])) - target_deg

    return float(optimize.brentq(f, w[i], w[i + 1]))


@dataclass(frozen=True)
class SpeedPIDesign:
    gains: PIDGains
    tn: float        # integral time = compensated plant time constant [s]
    wc: float        # designed gain-crossover frequency [rad/s]
    pm_target: float


def speed_pi_bode(plant: TF, t_dominant: float, meas: TF, delay: float,
                  pm_deg: float) -> SpeedPIDesign:
    """PI design in the Bode diagram.

    1. The PI zero is placed on the dominant plant pole (Tn = T1), so the
       closed loop has no slow pole/zero pair in its reference response.
    2. The phase of the open loop no longer depends on Kp: the crossover
       frequency is where arg L(jw) = -180 deg + PM.
    3. Kp makes |L(j wc)| = 1.
    """
    tn = t_dominant

    def loop_unit_gain(w):
        s = 1j * np.asarray(w, dtype=float)
        return (1.0 + s * tn) / (s * tn) * plant.freq(w) * meas.freq(w) * np.exp(-s * delay)

    wc = solve_phase(loop_unit_gain, -180.0 + pm_deg, 1e-2, 1e5)
    kp = 1.0 / float(np.abs(loop_unit_gain(np.array([wc]))[0]))
    return SpeedPIDesign(gains=PIDGains(kp=kp, ki=kp / tn), tn=tn, wc=wc, pm_target=pm_deg)


@dataclass(frozen=True)
class PositionPDDesign:
    gains: PIDGains
    t_sigma: float   # sum of the remaining small time constants [s]
    zeta: float      # designed damping of the dominant pole pair
    w0: float        # designed natural frequency of the dominant pole pair [rad/s]

    @property
    def overshoot(self) -> float:
        return overshoot_from_zeta(self.zeta)


def position_pd_compensation(ks: float, t1: float, t2: float, tf: float, ts: float,
                             zeta: float) -> PositionPDDesign:
    """PD for the IT2 position plant Ks / (s (T1 s + 1)(T2 s + 1)).

    The realisable PD  kp (1 + (kd/kp + tf) s) / (tf s + 1)  has its zero at
    -1/(kd/kp + tf); choosing kd = kp (T1 - tf) cancels the mechanical pole.
    What remains is approximately the IT1 loop Kp Ks / (s (T_sigma s + 1)) with
    T_sigma = T2 + tf + Ts/2 (the last term models the ZOH). Its closed loop is a
    pure second-order system, so the gain follows from the desired damping:
        zeta = 1 / (2 sqrt(Kp Ks T_sigma))  ->  Kp = 1 / (4 zeta^2 Ks T_sigma)
    """
    if not 0.0 < tf < t1:
        raise ValueError("derivative filter must be faster than the compensated pole")
    t_sigma = t2 + tf + ts / 2.0
    kp = 1.0 / (4.0 * zeta ** 2 * ks * t_sigma)
    kd = kp * (t1 - tf)
    w0 = float(np.sqrt(kp * ks / t_sigma))
    return PositionPDDesign(gains=PIDGains(kp=kp, kd=kd, tf=tf), t_sigma=t_sigma,
                            zeta=zeta, w0=w0)


def overshoot_from_zeta(zeta: float) -> float:
    """Step overshoot of a pure second-order system (fraction, not percent)."""
    if zeta >= 1.0:
        return 0.0
    return float(np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta ** 2)))


def pole_pair_params(pole: complex) -> tuple[float, float]:
    """(w0, zeta) of a continuous-time pole s = -zeta w0 +- j w0 sqrt(1 - zeta^2)."""
    w0 = abs(pole)
    return float(w0), float(-pole.real / w0)


def step_metrics(t: np.ndarray, y: np.ndarray, target: float, t_start: float = 0.0,
                 band: float = 0.02) -> dict:
    """Overshoot, 10-90 % rise time and settling time of a step response."""
    m = t >= t_start
    t, y = t[m] - t_start, y[m]
    y0 = y[0]
    span = target - y0
    rel = (y - y0) / span
    overshoot = max(0.0, float(np.max(rel) - 1.0))
    t10 = float(t[np.argmax(rel >= 0.1)])
    t90 = float(t[np.argmax(rel >= 0.9)])
    outside = np.flatnonzero(np.abs(rel - 1.0) > band)
    if outside.size == 0:
        t_settle = 0.0
    elif outside[-1] + 1 < len(t):
        t_settle = float(t[outside[-1] + 1])
    else:
        t_settle = float("nan")  # still outside the band at the end of the record
    return {"overshoot": overshoot, "rise_time": t90 - t10, "settling_time": t_settle}
