/*
 * Discrete two-degree-of-freedom PID controller.
 * Reference model: PID in control/discrete.py (same update order).
 *
 *   u = sat( kp (b r - y) + ki * integral(r - y) + kd * D[c r - y] ),
 *   D(s) = s / (tf s + 1)
 *
 * - Integrator and derivative filter are Tustin-discretised, so the
 *   controller's frequency response matches the continuous design up to the
 *   usual frequency warping (checked in control/pipeline.py).
 * - Setpoint weights: b < 1 softens the proportional kick on setpoint steps;
 *   c = 0 gives derivative-on-measurement (no derivative kick at all),
 *   c = 1 the textbook derivative of the error.
 * - Anti-windup by back-calculation: while the output is saturated the
 *   integrator is driven towards the value that just reaches the limit, with
 *   tracking time constant tt. tt = 0 disables it.
 * - Bumpless operation: gain changes and controller hand-over adjust the
 *   integrator so that the output does not jump.
 */
#ifndef MOTORCTL_MC_PID_H
#define MOTORCTL_MC_PID_H

#include <stdbool.h>

#include "motorctl/mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;    /* proportional gain */
    float ki;    /* integral gain [1/s * kp units] */
    float kd;    /* derivative gain [s * kp units] */
    float tf;    /* derivative filter time constant [s]; required (> 0) when kd != 0 */
    float b;     /* setpoint weight of the P term */
    float c;     /* setpoint weight of the D term (0: derivative on measurement) */
    float ts;    /* sample time [s] */
    float u_min; /* output limits; +-INFINITY for none */
    float u_max;
    float tt;    /* anti-windup tracking time constant [s]; 0 disables */
} mc_pid_config_t;

typedef struct {
    mc_pid_config_t cfg;
    float ad, bd;   /* Tustin derivative filter: d[k] = ad d[k-1] + bd (x[k] - x[k-1]) */
    float aw;       /* Ts / tt, 0 when anti-windup is off */
    float integ;    /* integral contribution to the output (already times ki) */
    float deriv;    /* filtered derivative of (c r - y) */
    float e_prev;
    float xd_prev;
    bool primed;    /* false until the first update after a reset */
} mc_pid_t;

/* Fills a config with neutral values: P controller with kp = 0, b = 1, c = 0,
 * no limits, no anti-windup. */
mc_pid_config_t mc_pid_default_config(float ts);

mc_status_t mc_pid_init(mc_pid_t *pid, const mc_pid_config_t *cfg);
void mc_pid_reset(mc_pid_t *pid);
float mc_pid_update(mc_pid_t *pid, float r, float y);

/* Changes kp, ki, kd without a jump in the output for the given (r, y).
 * Without integral action there is no state to absorb the difference, so
 * the integrator is cleared instead. Fails if kd != 0 but tf == 0. */
mc_status_t mc_pid_set_gains(mc_pid_t *pid, float kp, float ki, float kd, float r, float y);

/* Initialises the states so that the next output continues from u_now
 * (hand-over from another controller). Only possible with integral action. */
void mc_pid_bumpless_start(mc_pid_t *pid, float r, float y, float u_now);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_PID_H */
