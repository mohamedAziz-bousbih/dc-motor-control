#include "motorctl/mc_servo.h"

#include <math.h>
#include <stddef.h>

mc_status_t mc_servo_init(mc_servo_t *s, const mc_servo_config_t *cfg, int32_t counts0)
{
    if (s == NULL || cfg == NULL || !(cfg->ts > 0.0f) || !(cfg->rad_per_count > 0.0f) ||
        !(cfg->v_max > 0.0f)) {
        return MC_EINVAL;
    }
    s->cfg = *cfg;
    if (mc_pid_init(&s->speed_pid, &cfg->speed) != MC_OK ||
        mc_pid_init(&s->pos_pid, &cfg->pos) != MC_OK || mc_ssc_init(&s->ssc, &cfg->ssc) != MC_OK ||
        mc_lpf1_init(&s->lpf, cfg->speed_lpf_tau, cfg->ts, 0.0f) != MC_OK ||
        mc_slew_init(&s->slew, 0.0f, cfg->ts, 0.0f) != MC_OK) {
        return MC_EINVAL;
    }
    s->speed_slew = cfg->speed_slew;
    s->pos_slew = cfg->pos_slew;
    s->mode = MC_MODE_OFF;
    s->counts_prev = counts0;
    s->theta = (float)counts0 * cfg->rad_per_count;
    s->omega = 0.0f;
    s->target = 0.0f;
    s->r_eff = 0.0f;
    s->u = 0.0f;
    s->nonfinite_trips = 0;
    return MC_OK;
}

mc_status_t mc_servo_set_mode(mc_servo_t *s, mc_mode_t mode)
{
    const float ts = s->cfg.ts;
    switch (mode) {
    case MC_MODE_SPEED:
        s->target = s->omega;
        (void)mc_slew_init(&s->slew, s->speed_slew, ts, s->omega);
        mc_pid_bumpless_start(&s->speed_pid, s->omega, s->omega, s->u);
        break;
    case MC_MODE_POSITION:
        s->target = s->theta;
        (void)mc_slew_init(&s->slew, s->pos_slew, ts, s->theta);
        mc_pid_bumpless_start(&s->pos_pid, s->theta, s->theta, s->u);
        break;
    case MC_MODE_STATE:
        s->target = s->theta;
        (void)mc_slew_init(&s->slew, s->pos_slew, ts, s->theta);
        mc_ssc_bumpless_start(&s->ssc, s->theta, s->theta, s->omega, s->u);
        break;
    case MC_MODE_OFF:
    case MC_MODE_VOLTAGE:
        /* VOLTAGE deliberately starts at 0 V rather than at the last output. */
        s->target = 0.0f;
        break;
    default:
        return MC_EINVAL;
    }
    s->r_eff = s->target;
    s->mode = mode;
    return MC_OK;
}

void mc_servo_set_setpoint(mc_servo_t *s, float value)
{
    if (s->mode == MC_MODE_VOLTAGE) {
        value = mc_clampf(value, -s->cfg.v_max, s->cfg.v_max);
    }
    s->target = value;
}

mc_status_t mc_servo_set_pid_gains(mc_servo_t *s, mc_loop_t loop, float kp, float ki, float kd)
{
    if (loop == MC_LOOP_SPEED) {
        return mc_pid_set_gains(&s->speed_pid, kp, ki, kd, s->r_eff, s->omega);
    }
    if (loop == MC_LOOP_POSITION) {
        return mc_pid_set_gains(&s->pos_pid, kp, ki, kd, s->r_eff, s->theta);
    }
    return MC_EINVAL;
}

mc_status_t mc_servo_set_slew(mc_servo_t *s, mc_loop_t loop, float rate)
{
    if (loop == MC_LOOP_SPEED) {
        s->speed_slew = rate;
        if (s->mode == MC_MODE_SPEED) {
            mc_slew_set_rate(&s->slew, rate);
        }
        return MC_OK;
    }
    if (loop == MC_LOOP_POSITION) {
        s->pos_slew = rate;
        if (s->mode == MC_MODE_POSITION || s->mode == MC_MODE_STATE) {
            mc_slew_set_rate(&s->slew, rate);
        }
        return MC_OK;
    }
    return MC_EINVAL;
}

float mc_servo_update(mc_servo_t *s, int32_t counts)
{
    const mc_servo_config_t *c = &s->cfg;
    /* Unsigned subtraction: well defined when the 32-bit count wraps. */
    const int32_t dc = (int32_t)((uint32_t)counts - (uint32_t)s->counts_prev);
    s->counts_prev = counts;
    s->theta = (float)counts * c->rad_per_count;
    s->omega = mc_lpf1_update(&s->lpf, (float)dc * c->rad_per_count / c->ts);
    float u;
    switch (s->mode) {
    case MC_MODE_VOLTAGE:
        s->r_eff = s->target;
        u = s->target;
        break;
    case MC_MODE_SPEED:
        s->r_eff = mc_slew_update(&s->slew, s->target);
        u = mc_pid_update(&s->speed_pid, s->r_eff, s->omega);
        break;
    case MC_MODE_POSITION:
        s->r_eff = mc_slew_update(&s->slew, s->target);
        u = mc_pid_update(&s->pos_pid, s->r_eff, s->theta);
        break;
    case MC_MODE_STATE:
        s->r_eff = mc_slew_update(&s->slew, s->target);
        u = mc_ssc_update(&s->ssc, s->r_eff, s->theta);
        break;
    case MC_MODE_OFF:
    default:
        u = 0.0f;
        break;
    }
    if (!isfinite(u)) {
        /* mc_clampf passes NaN through on purpose; this is the check that
         * catches it. A NaN or infinite output (e.g. float overflow after an
         * extreme setpoint jump) must never reach the bridge, and the
         * controller states that produced it are unusable: switch off. The
         * next closed-loop mode re-initialises every state. */
        (void)mc_servo_set_mode(s, MC_MODE_OFF);
        ++s->nonfinite_trips;
        u = 0.0f;
    }
    s->u = u;
    return u;
}

float mc_servo_omega_hat(const mc_servo_t *s)
{
    return s->mode == MC_MODE_STATE ? s->ssc.xhat[1] : s->omega;
}

const char *mc_mode_name(mc_mode_t mode)
{
    switch (mode) {
    case MC_MODE_OFF:
        return "off";
    case MC_MODE_VOLTAGE:
        return "volt";
    case MC_MODE_SPEED:
        return "speed";
    case MC_MODE_POSITION:
        return "pos";
    case MC_MODE_STATE:
        return "state";
    default:
        return "?";
    }
}
