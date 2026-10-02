#include "motorctl/mc_ssc.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

static bool config_valid(const mc_ssc_config_t *c)
{
    if (!(c->ts > 0.0f) || !(c->u_min < c->u_max) || !(c->tt >= 0.0f)) {
        return false;
    }
    const float v[] = {c->phi[0][0], c->phi[0][1], c->phi[1][0], c->phi[1][1], c->gamma[0],
                       c->gamma[1], c->l[0], c->l[1], c->k[0], c->k[1], c->ki, c->kr};
    for (size_t i = 0; i < sizeof v / sizeof v[0]; ++i) {
        if (!isfinite(v[i])) {
            return false;
        }
    }
    return true;
}

static float unsaturated(const mc_ssc_t *s, float r)
{
    const mc_ssc_config_t *c = &s->cfg;
    return c->k[0] * (c->kr * r - s->xhat[0]) - c->k[1] * s->xhat[1] - c->ki * s->xi;
}

mc_status_t mc_ssc_init(mc_ssc_t *s, const mc_ssc_config_t *cfg)
{
    if (s == NULL || cfg == NULL || !config_valid(cfg)) {
        return MC_EINVAL;
    }
    s->cfg = *cfg;
    s->aw = (cfg->tt > 0.0f && cfg->ki != 0.0f)
                ? (float)((double)cfg->ts / ((double)cfg->tt * (double)cfg->ki))
                : 0.0f;
    mc_ssc_reset(s, 0.0f);
    return MC_OK;
}

void mc_ssc_reset(mc_ssc_t *s, float theta0)
{
    s->xhat[0] = theta0;
    s->xhat[1] = 0.0f;
    s->xi = 0.0f;
}

void mc_ssc_bumpless_start(mc_ssc_t *s, float r, float theta0, float omega0, float u_now)
{
    s->xhat[0] = theta0;
    s->xhat[1] = omega0;
    s->xi = 0.0f;
    if (s->cfg.ki != 0.0f) {
        s->xi = (unsaturated(s, r) - u_now) / s->cfg.ki;
    }
}

float mc_ssc_update(mc_ssc_t *s, float r, float y)
{
    const mc_ssc_config_t *c = &s->cfg;
    const float innov = y - s->xhat[0];
    const float v = unsaturated(s, r);
    const float u = mc_clampf(v, c->u_min, c->u_max);
    s->xi += c->ts * (y - r) + s->aw * (v - u);
    /* The observer gets the saturated u: the voltage the motor actually sees. */
    const float x0 = c->phi[0][0] * s->xhat[0] + c->phi[0][1] * s->xhat[1] + c->gamma[0] * u +
                     c->l[0] * innov;
    const float x1 = c->phi[1][0] * s->xhat[0] + c->phi[1][1] * s->xhat[1] + c->gamma[1] * u +
                     c->l[1] * innov;
    s->xhat[0] = x0;
    s->xhat[1] = x1;
    return u;
}
