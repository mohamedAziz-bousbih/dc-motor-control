#include "motorctl/mc_encoder.h"

#include <stddef.h>

mc_status_t mc_enc_ext_init(mc_enc_ext_t *e, int16_t limit, int16_t raw0)
{
    if (e == NULL || limit < 2 || raw0 <= -limit || raw0 >= limit) {
        return MC_EINVAL;
    }
    e->position = 0;
    e->last_raw = raw0;
    e->limit = limit;
    return MC_OK;
}

int32_t mc_enc_ext_update(mc_enc_ext_t *e, int16_t raw)
{
    const int32_t m = e->limit;
    /* raw and last_raw lie in (-m, m), so the difference lies in (-2m, 2m);
     * reduce it to [-m/2, m - m/2). */
    int32_t d = ((int32_t)raw - (int32_t)e->last_raw) % m; /* now in (-m, m) */
    if (d >= m - m / 2) {
        d -= m;
    } else if (d < -(m / 2)) {
        d += m;
    }
    e->last_raw = raw;
    e->position = (int32_t)((uint32_t)e->position + (uint32_t)d);
    return e->position;
}
