/*
 * Discrete state-feedback position controller with integral action and a
 * prediction-form Luenberger observer.
 * Reference model: StateController in control/discrete.py.
 *
 * Design model (identified PT1 speed channel + integrator, ZOH at Ts):
 *   x = [theta, omega],  x[k+1] = Phi x[k] + Gamma u[k],  y = theta
 *
 * Per sample:
 *   v      = k0 (kr r - x0_hat) - k1 x1_hat - ki xi,   u = sat(v)
 *   xi    += Ts (y - r) + (Ts / tt) (v - u) / ki          (back-calculation)
 *   x_hat <- Phi x_hat + Gamma u + l (y - x0_hat)
 *
 * The observer estimates the speed from the quantised encoder angle; the
 * gains k, ki come from Ackermann's formula, l from Ackermann on the dual
 * system (control/statespace.py).
 */
#ifndef MOTORCTL_MC_SSC_H
#define MOTORCTL_MC_SSC_H

#include "motorctl/mc_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float phi[2][2];
    float gamma[2];
    float l[2];     /* observer gain */
    float k[2];     /* state feedback on [theta_hat, omega_hat] */
    float ki;       /* feedback on the integrated position error */
    float ts;       /* sample time [s] */
    float kr;       /* reference weight on the angle feedback, 0 ... 1 */
    float u_min;
    float u_max;
    float tt;       /* anti-windup tracking time constant [s]; 0 disables */
} mc_ssc_config_t;

typedef struct {
    mc_ssc_config_t cfg;
    float aw;       /* Ts / (tt ki), 0 when anti-windup is off */
    float xhat[2];  /* predicted state x_hat[k | k-1] */
    float xi;       /* integral of (y - r) */
} mc_ssc_t;

mc_status_t mc_ssc_init(mc_ssc_t *s, const mc_ssc_config_t *cfg);

/* Observer at (theta0, 0), integrator cleared. */
void mc_ssc_reset(mc_ssc_t *s, float theta0);

/* Seeds observer and integrator so that the next output continues from
 * u_now (hand-over from another controller while the motor is moving). */
void mc_ssc_bumpless_start(mc_ssc_t *s, float r, float theta0, float omega0, float u_now);

float mc_ssc_update(mc_ssc_t *s, float r, float y);

#ifdef __cplusplus
}
#endif

#endif /* MOTORCTL_MC_SSC_H */
