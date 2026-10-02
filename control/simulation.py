"""Sampled-data closed-loop simulation: true motor + servo (discrete controller).

The plant is the full third-order model, discretised exactly (ZOH) at the
controller sample time. Between samples the voltage is constant, which is the
averaged model of a PWM H-bridge whose switching frequency (20 kHz) is far
above the electrical corner frequency. Optional effects: actuator limit
(supply voltage) and encoder quantisation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .lti import c2d_zoh
from .motor import DriveParams, MotorParams
from .servo import Servo


@dataclass
class SimLog:
    t: np.ndarray
    r: np.ndarray            # setpoint seen by the controller (after rate limiting)
    x: np.ndarray            # true plant states [theta, w, i]
    u: np.ndarray            # controller output (before the physical voltage limit)
    counts: np.ndarray       # encoder reading handed to the servo
    extra: dict = field(default_factory=dict)


def simulate(plant: MotorParams, drive: DriveParams, servo: Servo,
             setpoint: Callable[[float], float] | None, t_end: float,
             load: Callable[[float], float] | None = None, quantise: bool = True,
             limit_voltage: bool = True, record: Callable[[Servo], dict] | None = None,
             x0=None) -> SimLog:
    """Run the sampled closed loop for t_end seconds.

    The servo must already be in the desired mode. `setpoint(t)` (if given) is
    written to the servo before every sample. `record(servo)` is called just
    before the servo update, so for the prediction observer it sees the
    estimate x_hat[k|k-1] that the controller is about to use; the returned
    dict entries are collected into SimLog.extra.
    """
    ts = drive.ts
    a, b, _, _ = plant.state_space()
    phi, gam = c2d_zoh(a, b, ts)
    n = int(round(t_end / ts))
    t = np.arange(n) * ts
    x = np.zeros(3) if x0 is None else np.asarray(x0, dtype=float).copy()
    xs, us, rs, cs = np.empty((n, 3)), np.empty(n), np.empty(n), np.empty(n)
    extra: dict[str, list] = {}
    q = drive.rad_per_count
    for k in range(n):
        counts = float(math.floor(x[0] / q)) if quantise else x[0] / q
        if setpoint is not None:
            servo.set_setpoint(setpoint(t[k]))
        if record:
            for key, val in record(servo).items():
                extra.setdefault(key, []).append(val)
        u = servo.update(counts)
        u_app = min(max(u, -drive.v_supply), drive.v_supply) if limit_voltage else u
        xs[k], us[k], rs[k], cs[k] = x, u, servo.r_eff, counts
        tau = load(t[k]) if load else 0.0
        x = phi @ x + gam @ np.array([u_app, tau])
    return SimLog(t=t, r=rs, x=xs, u=us, counts=cs,
                  extra={k: np.asarray(v) for k, v in extra.items()})
