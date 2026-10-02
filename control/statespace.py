"""Discrete-time state-space position control: pole placement with
Ackermann's formula, integral action, and a Luenberger observer that
reconstructs the speed from the encoder angle.

Design model (identified PT1 speed channel plus the angle integrator):
    x = [theta, w],  dtheta/dt = w,  dw/dt = (-w + Ks u) / Tm,  y = theta
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .lti import c2d_zoh


def ctrb(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    b = b.reshape(-1, 1)
    cols = [b]
    for _ in range(a.shape[0] - 1):
        cols.append(a @ cols[-1])
    return np.hstack(cols)


def obsv(a: np.ndarray, c: np.ndarray) -> np.ndarray:
    return ctrb(a.T, c.reshape(-1, 1)).T


def acker(a: np.ndarray, b: np.ndarray, poles) -> np.ndarray:
    """Ackermann's formula  k = e_n^T Qc^-1 p(A)  for a single-input system.

    p is the desired characteristic polynomial; the eigenvalues of A - b k are
    then exactly `poles`. Works for continuous and discrete systems alike.
    """
    a = np.atleast_2d(np.asarray(a, dtype=float))
    n = a.shape[0]
    coeffs = np.real(np.poly(poles))
    p_of_a = np.zeros_like(a)
    for c in coeffs:                 # Horner scheme: ((A + a1 I) A + a2 I) ...
        p_of_a = p_of_a @ a + c * np.eye(n)
    qc = ctrb(a, np.asarray(b, dtype=float))
    if np.linalg.matrix_rank(qc) < n:
        raise ValueError("system is not controllable")
    return np.linalg.solve(qc, p_of_a)[-1]


def observer_gain(a: np.ndarray, c: np.ndarray, poles) -> np.ndarray:
    """Luenberger gain l with eig(A - l c) = poles (Ackermann on the dual system)."""
    return acker(np.asarray(a).T, np.asarray(c, dtype=float).reshape(-1), poles)


@dataclass(frozen=True)
class StateControllerDesign:
    ts: float
    phi: np.ndarray          # 2x2 discrete design model
    gamma: np.ndarray        # (2,)
    k: np.ndarray            # state feedback on [theta_hat, w_hat]
    ki: float                # feedback on the integrated position error
    l: np.ndarray            # observer gain (prediction form)
    ctrl_poles_z: np.ndarray
    obs_poles_z: np.ndarray

    c = np.array([1.0, 0.0])  # measured output: the encoder angle


def augment(phi: np.ndarray, gam: np.ndarray, ts: float):
    """Append the integrator x_i[k+1] = x_i[k] + Ts (theta[k] - r[k]) to the model."""
    phi_a = np.zeros((3, 3))
    phi_a[:2, :2] = phi
    phi_a[2, 0] = ts
    phi_a[2, 2] = 1.0
    return phi_a, np.array([gam[0], gam[1], 0.0])


def design_state_controller(ks: float, tm: float, ts: float, ctrl_poles_s, obs_poles_s
                            ) -> StateControllerDesign:
    """Place 3 controller poles (angle, speed, integrator) and 2 observer poles.

    The poles are specified in the s-plane and mapped with z = exp(s Ts), which
    keeps the design intuitive (time constants / damping) while the gains are
    computed for the exact ZOH model.
    """
    a = np.array([[0.0, 1.0], [0.0, -1.0 / tm]])
    b = np.array([0.0, ks / tm])
    phi, gam = c2d_zoh(a, b, ts)
    gam = gam[:, 0]
    zc = np.exp(np.asarray(ctrl_poles_s, dtype=complex) * ts)
    zo = np.exp(np.asarray(obs_poles_s, dtype=complex) * ts)

    phi_a, gam_a = augment(phi, gam, ts)
    ka = acker(phi_a, gam_a, zc)
    l = observer_gain(phi, np.array([1.0, 0.0]), zo)
    return StateControllerDesign(ts=ts, phi=phi, gamma=gam, k=ka[:2], ki=float(ka[2]), l=l,
                                 ctrl_poles_z=zc, obs_poles_z=zo)


def closed_loop_matrix(d: StateControllerDesign, phi_p: np.ndarray, gam_p: np.ndarray,
                       c_p: np.ndarray) -> np.ndarray:
    """System matrix of plant + integrator + observer under the control law
    u = -k x_hat - ki x_i  (reference set to zero).

    State order: [x_plant, x_i, x_hat]. With the design model as plant, its
    eigenvalues are the controller poles united with the observer poles
    (separation principle); with the true plant it tells whether the design
    is robust against the unmodelled armature inductance.
    """
    n = phi_p.shape[0]
    gam_p = np.asarray(gam_p, dtype=float).reshape(-1)
    c_p = np.asarray(c_p, dtype=float).reshape(-1)
    m = np.zeros((n + 3, n + 3))
    # u = -ki x_i - k x_hat
    u_row = np.zeros(n + 3)
    u_row[n] = -d.ki
    u_row[n + 1:] = -d.k
    m[:n, :n] = phi_p
    m[:n, :] += np.outer(gam_p, u_row)
    m[n, :n] = d.ts * c_p
    m[n, n] = 1.0
    m[n + 1:, :n] = np.outer(d.l, c_p)
    m[n + 1:, n + 1:] = d.phi - np.outer(d.l, d.c)
    m[n + 1:, :] += np.outer(d.gamma, u_row)
    return m
