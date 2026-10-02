#include "motorctl/mc_pid.h"

#include <math.h>
#include <stddef.h>

mc_pid_config_t mc_pid_default_config(float ts)
{
    mc_pid_config_t c;
    c.kp = 0.0f;
    c.ki = 0.0f;
    c.kd = 0.0f;
    c.tf = 0.0f;
    c.b = 1.0f;
    c.c = 0.0f;
    c.ts = ts;
    c.u_min = -INFINITY;
    c.u_max = INFINITY;
    c.tt = 0.0f;
    return c;
}

static bool config_valid(const mc_pid_config_t *c)
{
    /* Written so that NaN anywhere fails the check. */
    if (!(c->ts > 0.0f) || !(c->u_min < c->u_max) || !(c->tt >= 0.0f) || !(c->tf >= 0.0f)) {
        return false;
    }
    if (c->kd != 0.0f && !(c->tf > 0.0f)) {
        return false;
    }
    return isfinite(c->kp) && isfinite(c->ki) && isfinite(c->kd) && isfinite(c->b) &&
           isfinite(c->c);
}

static float unsaturated(const mc_pid_t *pid, float r, float y)
{
    const mc_pid_config_t *c = &pid->cfg;
    return c->kp * (c->b * r - y) + pid->integ + c->kd * pid->deriv;
}

mc_status_t mc_pid_init(mc_pid_t *pid, const mc_pid_config_t *cfg)
{
    if (pid == NULL || cfg == NULL || !config_valid(cfg)) {
        return MC_EINVAL;
    }
    pid->cfg = *cfg;
    if (cfg->tf > 0.0f) {
        /* Tustin: s -> 2/Ts (z-1)/(z+1) in s/(tf s + 1) */
        const double tf = (double)cfg->tf, ts = (double)cfg->ts;
        pid->ad = (float)((2.0 * tf - ts) / (2.0 * tf + ts));
        pid->bd = (float)(2.0 / (2.0 * tf + ts));
    } else {
        pid->ad = 0.0f;
        pid->bd = 0.0f;
    }
    pid->aw = cfg->tt > 0.0f ? cfg->ts / cfg->tt : 0.0f;
    mc_pid_reset(pid);
    return MC_OK;
}

void mc_pid_reset(mc_pid_t *pid)
{
    pid->integ = 0.0f;
    pid->deriv = 0.0f;
    pid->e_prev = 0.0f;
    pid->xd_prev = 0.0f;
    pid->primed = false;
}

float mc_pid_update(mc_pid_t *pid, float r, float y)
{
    const mc_pid_config_t *c = &pid->cfg;
    const float e = r - y;
    const float xd = c->c * r - y;
    if (!pid->primed) {
        /* No history yet: no derivative kick, rectangular first integration step. */
        pid->e_prev = e;
        pid->xd_prev = xd;
        pid->primed = true;
    }
    pid->deriv = pid->ad * pid->deriv + pid->bd * (xd - pid->xd_prev);
    pid->integ += c->ki * c->ts * 0.5f * (e + pid->e_prev);
    const float v = unsaturated(pid, r, y);
    const float u = mc_clampf(v, c->u_min, c->u_max);
    if (c->ki != 0.0f) {
        pid->integ += pid->aw * (u - v);
    }
    pid->e_prev = e;
    pid->xd_prev = xd;
    return u;
}

mc_status_t mc_pid_set_gains(mc_pid_t *pid, float kp, float ki, float kd, float r, float y)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || (kd != 0.0f && !(pid->cfg.tf > 0.0f))) {
        return MC_EINVAL;
    }
    const float v_old = unsaturated(pid, r, y);
    pid->cfg.kp = kp;
    pid->cfg.ki = ki;
    pid->cfg.kd = kd;
    pid->integ = ki != 0.0f ? pid->integ + v_old - unsaturated(pid, r, y) : 0.0f;
    return MC_OK;
}

void mc_pid_bumpless_start(mc_pid_t *pid, float r, float y, float u_now)
{
    const mc_pid_config_t *c = &pid->cfg;
    pid->deriv = 0.0f;
    pid->e_prev = r - y;
    pid->xd_prev = c->c * r - y;
    pid->primed = true;
    pid->integ = c->ki != 0.0f ? u_now - c->kp * (c->b * r - y) : 0.0f;
}
