"""Reference model of the firmware's servo module (lib/motorctl/src/mc_servo.c).

One sample of the embedded control loop:

    encoder counts -> angle, filtered difference-quotient speed
                   -> setpoint rate limiter -> active controller -> voltage

The class mirrors mc_servo.c statement by statement (mode handling included),
so the closed-loop simulations exercise exactly the logic that runs on the
ESP32, and the golden-vector test replays one of those simulations against
the C code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import IntEnum

from .discrete import PID, LowPass1, PIDConfig, SlewLimiter, SSCConfig, StateController


class Mode(IntEnum):
    OFF = 0        # output 0 V
    VOLTAGE = 1    # open loop, setpoint in V (identification experiments)
    SPEED = 2      # PI speed control, setpoint in rad/s
    POSITION = 3   # PD position control, setpoint in rad
    STATE = 4      # state feedback + observer, setpoint in rad


class Loop(IntEnum):
    SPEED = 0
    POSITION = 1


@dataclass(frozen=True)
class ServoConfig:
    ts: float                # sample time [s]
    rad_per_count: float     # encoder resolution at the output shaft [rad]
    v_max: float             # actuator limit = H-bridge supply [V]
    speed: PIDConfig
    speed_lpf_tau: float     # low-pass on the difference-quotient speed [s]
    speed_slew: float        # speed setpoint rate limit [rad/s^2], 0 = off
    pos: PIDConfig
    pos_slew: float          # position setpoint rate limit [rad/s], 0 = off (PD and state mode)
    ssc: SSCConfig


class Servo:
    def __init__(self, cfg: ServoConfig, counts0: float = 0.0):
        if cfg.ts <= 0 or cfg.rad_per_count <= 0 or cfg.v_max <= 0:
            raise ValueError("invalid servo configuration")
        self.cfg = cfg
        self.speed_pid = PID(cfg.speed)
        self.pos_pid = PID(cfg.pos)
        self.ssc = StateController(cfg.ssc)
        self.lpf = LowPass1(cfg.speed_lpf_tau, cfg.ts)
        self.slew = SlewLimiter(0.0, cfg.ts)
        self.speed_slew, self.pos_slew = cfg.speed_slew, cfg.pos_slew
        self.mode = Mode.OFF
        self.counts_prev = counts0
        self.theta = counts0 * cfg.rad_per_count
        self.omega = 0.0
        self.target = 0.0
        self.r_eff = 0.0
        self.u = 0.0
        self.nonfinite_trips = 0

    # --- commands -----------------------------------------------------------
    def set_mode(self, mode: Mode) -> None:
        """Switch mode. Closed-loop modes start by holding the current speed or
        angle and continue from the last output where the controller allows."""
        if mode == Mode.SPEED:
            self.target = self.omega
            self.slew = SlewLimiter(self.speed_slew, self.cfg.ts, self.omega)
            self.speed_pid.bumpless_start(self.omega, self.omega, self.u)
        elif mode == Mode.POSITION:
            self.target = self.theta
            self.slew = SlewLimiter(self.pos_slew, self.cfg.ts, self.theta)
            self.pos_pid.bumpless_start(self.theta, self.theta, self.u)
        elif mode == Mode.STATE:
            self.target = self.theta
            self.slew = SlewLimiter(self.pos_slew, self.cfg.ts, self.theta)
            self.ssc.bumpless_start(self.theta, self.theta, self.omega, self.u)
        else:
            # OFF, and VOLTAGE which deliberately starts at 0 V for safety
            self.target = 0.0
        self.r_eff = self.target
        self.mode = Mode(mode)

    def set_setpoint(self, value: float) -> None:
        if self.mode == Mode.VOLTAGE:
            value = min(max(value, -self.cfg.v_max), self.cfg.v_max)
        self.target = value

    def set_pid_gains(self, loop: Loop, kp: float, ki: float, kd: float) -> None:
        """Bumpless gain change of the speed or position PID."""
        if loop == Loop.SPEED:
            self.speed_pid.set_gains(kp, ki, kd, self.r_eff, self.omega)
        else:
            self.pos_pid.set_gains(kp, ki, kd, self.r_eff, self.theta)

    def set_slew(self, loop: Loop, rate: float) -> None:
        if loop == Loop.SPEED:
            self.speed_slew = rate
            if self.mode == Mode.SPEED:
                self.slew.set_rate(rate)
        else:
            self.pos_slew = rate
            if self.mode in (Mode.POSITION, Mode.STATE):
                self.slew.set_rate(rate)

    # --- one control period ---------------------------------------------------
    def update(self, counts: float) -> float:
        c = self.cfg
        dc = counts - self.counts_prev
        self.counts_prev = counts
        self.theta = counts * c.rad_per_count
        self.omega = self.lpf.update(dc * c.rad_per_count / c.ts)
        if self.mode == Mode.VOLTAGE:
            self.r_eff = self.target
            u = self.target
        elif self.mode == Mode.SPEED:
            self.r_eff = self.slew.update(self.target)
            u = self.speed_pid.update(self.r_eff, self.omega)
        elif self.mode == Mode.POSITION:
            self.r_eff = self.slew.update(self.target)
            u = self.pos_pid.update(self.r_eff, self.theta)
        elif self.mode == Mode.STATE:
            self.r_eff = self.slew.update(self.target)
            u = self.ssc.update(self.r_eff, self.theta)
        else:
            u = 0.0
        if not math.isfinite(u):
            # A NaN/inf output must never reach the bridge; switch off.
            self.set_mode(Mode.OFF)
            self.nonfinite_trips += 1
            u = 0.0
        self.u = u
        return u

    @property
    def omega_hat(self) -> float:
        """Observer speed estimate (state mode) or the filtered measurement."""
        return float(self.ssc.xhat[1]) if self.mode == Mode.STATE else self.omega


def unlimited(cfg: ServoConfig) -> ServoConfig:
    """Same controllers without actuator limits or setpoint rate limits, for
    comparing the sampled loop with the linear continuous-time design."""
    inf = math.inf
    return replace(cfg, v_max=inf, speed_slew=0.0, pos_slew=0.0,
                   speed=replace(cfg.speed, u_min=-inf, u_max=inf),
                   pos=replace(cfg.pos, u_min=-inf, u_max=inf),
                   ssc=replace(cfg.ssc, u_min=-inf, u_max=inf))
