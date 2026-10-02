"""First-principles model of a permanent-magnet DC motor.

Armature mesh (Kirchhoff):      L di/dt  = u - R i - K w
Rotor (Newton, rotational):     J dw/dt  = K i - b w - tau_load
Kinematics:                     dtheta/dt = w

For a permanent-magnet machine the torque constant (N m/A) and the back-EMF
constant (V s/rad) are the same number in SI units, so a single K is used.
All quantities are referred to the gearbox output shaft, where the encoder
angle is measured. State vector: x = [theta, w, i], inputs: [u, tau_load].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .lti import TF


@dataclass(frozen=True)
class MotorParams:
    """Electromechanical parameters (SI units, referred to the output shaft).

    The defaults describe a plausible small 12 V brushed gearmotor with an
    added flywheel. They are the "true" plant used to generate the synthetic
    measurement and to verify every controller in simulation; they are not
    taken from the datasheet of a specific product.
    """

    R: float = 2.4       # armature resistance [ohm]
    L: float = 2.6e-3    # armature inductance [H]
    K: float = 0.32      # torque / back-EMF constant [N m/A] = [V s/rad]
    J: float = 2.2e-3    # total inertia [kg m^2]
    b: float = 1.1e-3    # viscous friction [N m s/rad]

    # --- state space --------------------------------------------------------
    def state_space(self):
        """(A, B, C, D) with x = [theta, w, i], u = [voltage, load torque], y = x."""
        a = np.array([
            [0.0, 1.0, 0.0],
            [0.0, -self.b / self.J, self.K / self.J],
            [0.0, -self.K / self.L, -self.R / self.L],
        ])
        b = np.array([
            [0.0, 0.0],
            [0.0, -1.0 / self.J],
            [1.0 / self.L, 0.0],
        ])
        return a, b, np.eye(3), np.zeros((3, 2))

    # --- transfer functions -------------------------------------------------
    def _den(self) -> np.ndarray:
        # (L s + R)(J s + b) + K^2
        return np.array([self.L * self.J, self.R * self.J + self.L * self.b,
                         self.R * self.b + self.K ** 2])

    def tf_speed(self) -> TF:
        """w(s)/U(s) = K / ((Ls + R)(Js + b) + K^2)."""
        return TF([self.K], self._den())

    def tf_angle(self) -> TF:
        """theta(s)/U(s) = w(s)/U(s) / s."""
        return TF([self.K], np.polymul(self._den(), [1.0, 0.0]))

    def tf_current(self) -> TF:
        """i(s)/U(s) = (Js + b) / ((Ls + R)(Js + b) + K^2)."""
        return TF([self.J, self.b], self._den())

    def tf_load_to_speed(self) -> TF:
        """w(s)/tau_load(s) = -(Ls + R) / ((Ls + R)(Js + b) + K^2)."""
        return TF([-self.L, -self.R], self._den())

    # --- derived quantities -------------------------------------------------
    @property
    def ks(self) -> float:
        """Static gain of the speed channel [rad/s per V]."""
        return self.K / (self.R * self.b + self.K ** 2)

    @property
    def time_constants(self) -> tuple[float, float]:
        """(T1, T2) of the PT2 speed channel, T1 >= T2 (both poles are real)."""
        p = np.roots(self._den())
        if np.any(np.abs(np.imag(p)) > 1e-12 * np.abs(p)):
            raise ValueError("speed channel has complex poles; not a PT2 with real time constants")
        t = np.sort(-1.0 / np.real(p))[::-1]
        return float(t[0]), float(t[1])

    @property
    def tau_el(self) -> float:
        """Electrical time constant L/R [s]."""
        return self.L / self.R

    @property
    def tau_mech(self) -> float:
        """Mechanical time constant of the reduced (L = 0) model [s]."""
        return self.J * self.R / (self.K ** 2 + self.R * self.b)


@dataclass(frozen=True)
class DriveParams:
    """Power stage, sensor and sampling assumptions shared by all designs."""

    v_supply: float = 12.0       # H-bridge supply = actuator limit [V]
    counts_per_rev: int = 1320   # quadrature counts per output revolution (x4 decoding)
    ts: float = 1e-3             # controller sample time [s]

    @property
    def rad_per_count(self) -> float:
        return 2.0 * np.pi / self.counts_per_rev


def pt2_speed_model(ks: float, t1: float, t2: float) -> TF:
    """Black-box PT2 speed model Ks / ((T1 s + 1)(T2 s + 1))."""
    return TF([ks], np.polymul([t1, 1.0], [t2, 1.0]))
