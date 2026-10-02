#include "motorctl/mc_filter.h"

#include <math.h>
#include <stddef.h>

mc_status_t mc_lpf1_init(mc_lpf1_t *f, float tau, float ts, float y0)
{
    if (f == NULL || !(tau >= 0.0f) || !(ts > 0.0f)) {
        return MC_EINVAL;
    }
    /* Coefficient in double: computed once, and 1 - exp(-x) loses digits in
     * float when Ts << tau. */
    f->a = tau > 0.0f ? (float)(1.0 - exp(-(double)ts / (double)tau)) : 1.0f;
    f->y = y0;
    return MC_OK;
}

float mc_lpf1_update(mc_lpf1_t *f, float x)
{
    f->y += f->a * (x - f->y);
    return f->y;
}

mc_status_t mc_slew_init(mc_slew_t *s, float rate, float ts, float y0)
{
    if (s == NULL || !(ts > 0.0f)) {
        return MC_EINVAL;
    }
    s->ts = ts;
    mc_slew_set_rate(s, rate);
    s->y = y0;
    return MC_OK;
}

void mc_slew_set_rate(mc_slew_t *s, float rate)
{
    s->step_max = rate > 0.0f ? rate * s->ts : 0.0f;
}

float mc_slew_update(mc_slew_t *s, float x)
{
    if (s->step_max > 0.0f) {
        s->y += mc_clampf(x - s->y, -s->step_max, s->step_max);
    } else {
        s->y = x;
    }
    return s->y;
}
