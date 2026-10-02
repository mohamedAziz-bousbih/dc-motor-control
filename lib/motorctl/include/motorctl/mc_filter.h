/*
 * Signal conditioning blocks used by the servo module.
 * Reference model: LowPass1 and SlewLimiter in control/discrete.py.
 */
#ifndef MOTORCTL_MC_FILTER_H
#define MOTORCTL_MC_FILTER_H

#include "motorctl/mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* First-order low-pass y[k] = y[k-1] + a (x[k] - y[k-1]), a = 1 - exp(-Ts/tau).
 * This is the exact discretisation of 1/(tau s + 1) for an input that is
 * constant over the sample (step-invariant). tau = 0 gives a pass-through. */
typedef struct {
    float a;
    float y;
} mc_lpf1_t;

mc_status_t mc_lpf1_init(mc_lpf1_t *f, float tau, float ts, float y0);
float mc_lpf1_update(mc_lpf1_t *f, float x);

/* Rate limiter: the output moves towards the input by at most rate * Ts per
 * sample. rate <= 0 disables the limit (output = input). Used to shape
 * setpoint steps into ramps the actuator can follow. */
typedef struct {
    float ts;
    float step_max;
    float y;
} mc_slew_t;

mc_status_t mc_slew_init(mc_slew_t *s, float rate, float ts, float y0);
void mc_slew_set_rate(mc_slew_t *s, float rate);
float mc_slew_update(mc_slew_t *s, float x);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_FILTER_H */
